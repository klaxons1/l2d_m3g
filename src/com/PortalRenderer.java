package com;

import javax.microedition.m3g.Transform;

/**
 * Рендер видов через порталы.
 *
 * Основной путь (render-to-texture, JSR-184 позволяет биндить mutable Image2D
 * как цель рендера):
 *   1) до основного прохода для каждого видимого портала считается виртуальная
 *      камера (M = toWorld[dst] * FLIP * fromWorld[src]) и сцена из комнаты
 *      парного портала рендерится в Image2D. Проекция - суб-пирамида основной
 *      камеры по bbox окна портала, ближняя плоскость заменена на плоскость
 *      выходного портала (oblique clipping), поэтому стена за ним не мешает;
 *   2) в основном проходе окно портала рисуется обычной геометрией с этой
 *      текстурой. Texcoord'ы = экранные координаты вершин, перспективная
 *      коррекция выключена -> текстура ложится ровно в дырку, а буфер глубины
 *      сам разбирается с перекрытием портала стенами и объектами.
 *
 * Рекурсия: если внутри вида через портал виден другой портал, для него тем же
 * способом рендерится вид следующего уровня (число уровней задаётся в
 * настройках). На самом глубоком уровне окно просто заливается цветом портала.
 *
 * Запасной путь (если реализация M3G не умеет рендерить в Image2D): вид через
 * портал рисуется прямо в кадр в прямоугольнике bbox с очисткой глубины.
 */
public final class PortalRenderer {

	private static final int MODE_NONE = 0;
	private static final int MODE_TEXTURED = 1;
	private static final int MODE_FLAT = 2;
	private static final int MODE_DIRECT = 3;
	/** Вид сквозь портал уже нарисован в кадр, окно занимает своя полоса глубины. */
	private static final int MODE_BANDED = 4;

	/**
	 * Разбиение буфера глубины на полосы (window-координаты, 0 - у камеры).
	 * Ближняя половина достаётся миру вокруг игрока, две четверти - видам
	 * сквозь порталы. Ближний к камере портал получает полосу ближе, чтобы
	 * при перекрытии выигрывал именно он.
	 */
	private static final float MAIN_FAR = 0.5f;
	private static final float[] BAND_NEAR = {0.5f, 0.75f};
	private static final float[] BAND_FAR = {0.75f, 1.0f};

	/**
	 * Сдвиг маски окна к камере. Тест глубины в M3G - LEQUAL, поэтому стена,
	 * лежащая ровно в плоскости портала, без сдвига прошла бы по равенству
	 * и затёрла вид.
	 */
	private static final float MASK_BIAS = 0.0004f;

	/** Сдвиг окна портала по глубине (NDC), см. Renderer.setDepthBias. */
	private static final float DEPTH_BIAS = 0.00015f;

	private static final int MAX_LEVELS = 3;

	private final PortalManager pm;

	private final Transform mainCam = new Transform();
	private final Transform[] camStack = new Transform[MAX_LEVELS + 1];
	private final int[][] boxStack = new int[MAX_LEVELS + 1][4];
	private final int[] tmpBox = new int[4];

	private final float[] plane = new float[4];
	private final float[] camMat = new float[16];
	private final float[] forward = new float[3];

	private final int[][] bbox = new int[PortalManager.COUNT][4];
	private final int[] mode = new int[PortalManager.COUNT];

	/**
	 * Обновление текстур порталов размазано по кадрам: за кадр перерисовывается
	 * не больше одного портала. Вид сквозь портал отстаёт на кадр, зато
	 * тяжёлых RTT-проходов вдвое меньше.
	 */
	private static final boolean FRAME_SKIP = true;
	/** Насколько может сместиться камера, чтобы старая текстура ещё годилась. */
	private static final int CAM_JUMP = 500;

	private int frame;
	private final int[] lastMode = new int[PortalManager.COUNT];
	private final boolean[] reused = new boolean[PortalManager.COUNT];
	private final int[] capVersion = new int[PortalManager.COUNT];
	private final Vector3D[] capCam = new Vector3D[PortalManager.COUNT];

	private boolean rttChecked;
	private boolean rttSupported;
	/** Рисовать порталы полосами глубины вместо рендера в текстуру. */
	private boolean banded;
	private int levels = 1;

	public PortalRenderer(PortalManager pm) {
		this.pm = pm;
		for(int i = 0; i < camStack.length; i++) camStack[i] = new Transform();
		for(int i = 0; i < capCam.length; i++) {
			capCam[i] = new Vector3D();
			capVersion[i] = -1;
		}
	}

	public final boolean isTextureModeSupported() {
		return rttSupported;
	}

	/** Включить режим полос глубины (без render-to-texture). */
	public final void setBandedMode(boolean on) {
		banded = on;
	}

	public final boolean isBandedMode() {
		return banded;
	}

	/**
	 * Первый проход: рендер видов через порталы в текстуры.
	 * Вызывать ДО Graphics3D.bindTarget основного кадра (то есть до Scene.render).
	 */
	public final void renderTextures(Renderer g3d, House house) {
		frame++;

		lastMode[0] = mode[0];
		lastMode[1] = mode[1];
		mode[0] = MODE_NONE;
		mode[1] = MODE_NONE;
		reused[0] = false;
		reused[1] = false;

		if(!pm.isActive(0) && !pm.isActive(1)) return;

		if(!rttChecked && !banded) {
			rttChecked = true;
			pm.initResources();
			rttSupported = pm.hasImages() && g3d.checkTextureTargetSupport();
			levels = pm.getLevels();
			if(levels > MAX_LEVELS) levels = MAX_LEVELS;

			// нет поддержки рендера в текстуру - уходим на полосы глубины
			if(!rttSupported) banded = true;
		}

		boolean linked = pm.isLinked();

		g3d.getCameraTransform(mainCam);
		mainCam.get(camMat);
		// направление взгляда - минус третий столбец матрицы камеры
		forward[0] = -camMat[2];
		forward[1] = -camMat[6];
		forward[2] = -camMat[10];

		for(int i = 0; i < PortalManager.COUNT; i++) {
			// портал развёрнут от нас или не попадает в пирамиду видимости:
			// ни проекции 65 вершин, ни тем более рендера вида не нужно
			if(!pm.isVisible(i, g3d)) continue;

			int state = pm.projectQuad(i, g3d, bbox[i]);

			if(state == PortalManager.NOT_VISIBLE) continue;

			// одиночный портал (пары ещё нет) - просто цветное окно
			if(!linked) {
				mode[i] = MODE_FLAT;
				continue;
			}

			// вершины ушли за ближнюю плоскость - экранные координаты (а значит
			// и UV) посчитать нельзя
			if(state == PortalManager.NEAR_CLIPPED) {
				// игрок входит в портал: вид сквозь него рисуем прямо в кадр
				if(pm.isEyeAtOpening(i, g3d.camPos, PortalManager.HALF_H * 2, forward)) {
					bbox[i][0] = 0;
					bbox[i][1] = 0;
					bbox[i][2] = g3d.width;
					bbox[i][3] = g3d.height;
					mode[i] = MODE_DIRECT;
				} else {
					mode[i] = MODE_FLAT;
				}
				continue;
			}

			// режим полос: сам вид рисуется позже, уже в привязанный кадр
			if(banded) {
				mode[i] = MODE_BANDED;
				continue;
			}

			// кадр этого портала пропускаем - показываем текстуру прошлого кадра
			if(canReuse(i, g3d)) {
				mode[i] = MODE_TEXTURED;
				reused[i] = true;
				continue;
			}

			mode[i] = renderView(g3d, house, i, 0, mainCam, bbox[i])
					? MODE_TEXTURED : MODE_FLAT;

			if(mode[i] == MODE_TEXTURED) {
				capVersion[i] = pm.getVersion(i);
				capCam[i].set(g3d.camPos);
			}
		}

		// восстанавливаем основную камеру
		g3d.clearClipPlane();
		g3d.setCameraTransform(mainCam);
	}

	/**
	 * Можно ли в этом кадре не перерисовывать текстуру портала.
	 *
	 * Нельзя, если текстуры ещё нет, портал переставили, камера прыгнула
	 * (телепорт) или сейчас очередь именно этого портала.
	 */
	private boolean canReuse(int idx, Renderer g3d) {
		if(!FRAME_SKIP) return false;
		if(lastMode[idx] != MODE_TEXTURED) return false;
		if(capVersion[idx] != pm.getVersion(idx)) return false;

		// очередь обновления: порталы чередуются, за кадр не больше одного
		if(((frame + idx) & 1) == 0) return false;

		Vector3D c = capCam[idx];
		int dx = g3d.camPos.x - c.x;
		int dy = g3d.camPos.y - c.y;
		int dz = g3d.camPos.z - c.z;
		if((long) dx * dx + (long) dy * dy + (long) dz * dz > (long) CAM_JUMP * CAM_JUMP) {
			return false;
		}

		return true;
	}

	/**
	 * Рендерит вид через портал idx в его текстуру уровня level.
	 *
	 * @param cam камера, из которой смотрим (уровня level)
	 * @param box прямоугольник виртуального экрана, который занимает окно
	 * @return true, если текстура нарисована
	 */
	private boolean renderView(Renderer g3d, House house, int idx, int level, Transform cam, int[] box) {
		int dst = pm.getLinkedPortal(idx);
		int room = pm.getRoomId(dst);
		if(room < 0) return false;
		if(pm.getImage(idx, level) == null) return false;

		// виртуальная камера этого уровня
		Transform virtual = camStack[level];
		pm.getVirtualCamera(idx, cam, virtual);
		g3d.setCameraTransform(virtual);

		// --- ищем портал, видимый внутри этого вида ---
		int inner = -1;
		int[] innerBox = boxStack[level + 1];
		boolean innerTextured = false;
		long innerArea = 0;

		for(int j = 0; j < PortalManager.COUNT; j++) {
			if(!pm.isFrontFacing(j, g3d.camPos)) continue;
			if(pm.projectQuad(j, g3d, tmpBox) != PortalManager.VISIBLE) continue;

			int x1 = tmpBox[0] > box[0] ? tmpBox[0] : box[0];
			int y1 = tmpBox[1] > box[1] ? tmpBox[1] : box[1];
			int x2 = tmpBox[2] < box[2] ? tmpBox[2] : box[2];
			int y2 = tmpBox[3] < box[3] ? tmpBox[3] : box[3];
			if(x2 - x1 < 2 || y2 - y1 < 2) continue;

			long area = (long) (x2 - x1) * (y2 - y1);
			if(area <= innerArea) continue;

			inner = j;
			innerArea = area;
			innerBox[0] = x1;
			innerBox[1] = y1;
			innerBox[2] = x2;
			innerBox[3] = y2;
		}

		// --- сначала более глубокий уровень (он рендерится в свою текстуру) ---
		if(inner >= 0 && level + 1 < levels && pm.isLinked()) {
			innerTextured = renderView(g3d, house, inner, level + 1, virtual, innerBox);

			// после рекурсии камера уже восстановлена, но подстрахуемся
			g3d.setCameraTransform(virtual);
		}

		// --- этот уровень: комната парного портала в свою текстуру ---
		pm.getPlane(dst, plane);
		g3d.setClipPlane(plane[0], plane[1], plane[2], plane[3]);

		boolean ok = true;

		try {
			g3d.beginTextureTarget(pm.getImage(idx, level), box[0], box[1], box[2], box[3]);
			house.renderPortalView(g3d, room, box[0], box[1], box[2], box[3]);

			// окно вложенного портала - обычной геометрией в ту же текстуру
			if(inner >= 0) {
				int state = pm.projectQuad(inner, g3d, tmpBox);
				boolean textured = innerTextured && state == PortalManager.VISIBLE;

				if(textured) pm.updateQuadUV(inner, innerBox);
				pm.setWindow(inner, textured ? level + 1 : -1);

				g3d.setDepthBias(DEPTH_BIAS);
				g3d.setClip(box[0], box[1], box[2], box[3]);
				g3d.addMesh(pm.getQuad(inner), pm.getQuadTransform(inner));
				g3d.setDepthBias(0);
			}
		} catch (Throwable t) {
			System.out.println("PORTAL: ошибка рендера в текстуру: " + t);
			rttSupported = false;
			ok = false;
		} finally {
			g3d.endTextureTarget();
		}

		// ВАЖНО: возвращаем камеру вызывающего уровня. Иначе следующий портал
		// (и его bbox) считался бы виртуальной камерой предыдущего.
		g3d.clearClipPlane();
		g3d.setCameraTransform(cam);
		return ok;
	}

	/**
	 * Второй проход: окна порталов рисуются вместе с остальной геометрией
	 * кадра (после Scene.render, до flush).
	 */
	public final void renderQuads(Renderer g3d, House house) {
		if(mode[0] == MODE_NONE && mode[1] == MODE_NONE) return;

		boolean cameraDirty = false;

		for(int i = 0; i < PortalManager.COUNT; i++) {
			if(mode[i] == MODE_DIRECT) {
				renderDirect(g3d, house, i);
				cameraDirty = true;
			}
		}

		if(cameraDirty) {
			g3d.clearClipPlane();
			g3d.setCameraTransform(mainCam);
		}

		boolean any = false;
		for(int i = 0; i < PortalManager.COUNT; i++) {
			if(mode[i] != MODE_TEXTURED && mode[i] != MODE_FLAT
					&& mode[i] != MODE_BANDED) continue;

			if(!any) {
				// окно и обводка портала лежат почти в плоскости стены, поэтому
				// на расстоянии они начинают спорить с ней за глубину;
				// сдвиг в NDC одинаково эффективен на любой дистанции
				g3d.setDepthBias(DEPTH_BIAS);
				g3d.setClip(0, 0, g3d.width, g3d.height);
				any = true;
			}

			if(mode[i] == MODE_TEXTURED) {
				if(reused[i]) {
					// текстура снята в прошлом кадре: показываем её с теми же UV,
					// иначе вид сквозь портал поедет вслед за новым bbox
					pm.restoreQuadUV(i);
				} else if(pm.projectQuad(i, g3d, tmpBox) == PortalManager.VISIBLE) {
					// UV могли быть переписаны рендером вложенных уровней
					pm.updateQuadUV(i, bbox[i]);
					pm.saveQuadUV(i);
				}
			}

			if(mode[i] == MODE_BANDED) {
				// вид сквозь портал уже в кадре и защищён маской: рисуем
				// только неоновую обводку вокруг окна
				pm.setWindow(i, PortalManager.HIDDEN_WINDOW);
			} else {
				pm.setWindow(i, mode[i] == MODE_TEXTURED ? 0 : -1);
			}
			g3d.addMesh(pm.getQuad(i), pm.getQuadTransform(i));
		}

		if(any) {
			g3d.setDepthBias(0);
			g3d.setClip(0, 0, g3d.width, g3d.height);
		}
	}

	/**
	 * Проход полос глубины. Вызывать ПОСЛЕ привязки экрана и очистки буферов,
	 * но ДО рендера мира вокруг игрока.
	 *
	 * Порядок для каждого портала:
	 *  1) вьюпорт = экранный bbox окна, диапазон глубины = своя дальняя полоса;
	 *  2) рендер комнаты, видимой сквозь портал, виртуальной камерой;
	 *  3) диапазон глубины = полоса мира, в окно пишется невидимая маска -
	 *     она не даёт стене затереть уже нарисованный вид.
	 * После всех порталов диапазон возвращается на полосу мира.
	 */
	public final void renderBanded(Renderer g3d, House house) {
		if(mode[0] != MODE_BANDED && mode[1] != MODE_BANDED) return;

		g3d.getCameraTransform(mainCam);

		// ближний портал - в ближнюю полосу
		int first = 0, second = 1;
		if(distanceSq(1, g3d) < distanceSq(0, g3d)) {
			first = 1;
			second = 0;
		}

		int band = 0;
		if(mode[first] == MODE_BANDED) {
			renderBandedView(g3d, house, first, band++);
		}
		if(mode[second] == MODE_BANDED && band < BAND_NEAR.length) {
			renderBandedView(g3d, house, second, band);
		} else if(mode[second] == MODE_BANDED) {
			// полос не хватило - показываем плоскую заливку
			mode[second] = MODE_FLAT;
		}

		// мир вокруг игрока рисуется в ближнюю полосу
		g3d.clearClipPlane();
		g3d.setCameraTransform(mainCam);
		g3d.setDepthRange(0f, MAIN_FAR);
		g3d.setClip(0, 0, g3d.width, g3d.height);
	}

	private long distanceSq(int idx, Renderer g3d) {
		if(!pm.isActive(idx)) return Long.MAX_VALUE;

		Vector3D p = pm.getPosition(idx);
		long dx = p.x - g3d.camPos.x;
		long dy = p.y - g3d.camPos.y;
		long dz = p.z - g3d.camPos.z;
		return dx * dx + dy * dy + dz * dz;
	}

	private void renderBandedView(Renderer g3d, House house, int idx, int band) {
		int dst = pm.getLinkedPortal(idx);
		int room = pm.getRoomId(dst);

		if(room < 0) {
			mode[idx] = MODE_FLAT;
			return;
		}

		int[] b = bbox[idx];

		// 1-2) вид сквозь портал в свою полосу глубины
		g3d.setDepthRange(BAND_NEAR[band], BAND_FAR[band]);

		pm.getVirtualCamera(idx, mainCam, camStack[0]);
		g3d.setCameraTransform(camStack[0]);

		pm.getPlane(dst, plane);
		g3d.setClipPlane(plane[0], plane[1], plane[2], plane[3]);

		house.renderPortalView(g3d, room, b[0], b[1], b[2], b[3]);

		// 3) маска окна в полосе мира: защищает нарисованное от стены
		g3d.clearClipPlane();
		g3d.setCameraTransform(mainCam);
		g3d.setDepthRange(0f, MAIN_FAR);
		g3d.setDepthBias(MASK_BIAS);
		g3d.setClip(b[0], b[1], b[2], b[3]);

		g3d.addMesh(pm.getMaskQuad(idx), pm.getQuadTransform(idx));

		g3d.setDepthBias(0);
	}

	/** Запасной путь: вид через портал прямо в кадр, в прямоугольнике bbox. */
	private void renderDirect(Renderer g3d, House house, int idx) {
		int dst = pm.getLinkedPortal(idx);
		int room = pm.getRoomId(dst);
		if(room < 0) return;

		int[] b = bbox[idx];

		pm.getVirtualCamera(idx, mainCam, camStack[0]);
		g3d.setCameraTransform(camStack[0]);

		pm.getPlane(dst, plane);
		g3d.setClipPlane(plane[0], plane[1], plane[2], plane[3]);

		g3d.setClip(b[0], b[1], b[2], b[3]);
		g3d.clearDepth();

		house.renderPortalView(g3d, room, b[0], b[1], b[2], b[3]);
	}
}
