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

	private boolean rttChecked;
	private boolean rttSupported;
	private int levels = 1;

	public PortalRenderer(PortalManager pm) {
		this.pm = pm;
		for(int i = 0; i < camStack.length; i++) camStack[i] = new Transform();
	}

	public final boolean isTextureModeSupported() {
		return rttSupported;
	}

	/**
	 * Первый проход: рендер видов через порталы в текстуры.
	 * Вызывать ДО Graphics3D.bindTarget основного кадра (то есть до Scene.render).
	 */
	public final void renderTextures(Renderer g3d, House house) {
		mode[0] = MODE_NONE;
		mode[1] = MODE_NONE;

		if(!pm.isActive(0) && !pm.isActive(1)) return;

		if(!rttChecked) {
			rttChecked = true;
			pm.initResources();
			rttSupported = pm.hasImages() && g3d.checkTextureTargetSupport();
			levels = pm.getLevels();
			if(levels > MAX_LEVELS) levels = MAX_LEVELS;
		}

		boolean linked = pm.isLinked();

		g3d.getCameraTransform(mainCam);
		mainCam.get(camMat);
		// направление взгляда - минус третий столбец матрицы камеры
		forward[0] = -camMat[2];
		forward[1] = -camMat[6];
		forward[2] = -camMat[10];

		for(int i = 0; i < PortalManager.COUNT; i++) {
			if(!pm.isFrontFacing(i, g3d.camPos)) continue;

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

			if(!rttSupported) {
				mode[i] = MODE_DIRECT;
				continue;
			}

			mode[i] = renderView(g3d, house, i, 0, mainCam, bbox[i])
					? MODE_TEXTURED : MODE_FLAT;
		}

		// восстанавливаем основную камеру
		g3d.clearClipPlane();
		g3d.setCameraTransform(mainCam);
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

			// рекурсия сменила камеру - возвращаем свою
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

		g3d.clearClipPlane();
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
			if(mode[i] != MODE_TEXTURED && mode[i] != MODE_FLAT) continue;

			if(!any) {
				// окно и обводка портала лежат почти в плоскости стены, поэтому
				// на расстоянии они начинают спорить с ней за глубину;
				// сдвиг в NDC одинаково эффективен на любой дистанции
				g3d.setDepthBias(DEPTH_BIAS);
				g3d.setClip(0, 0, g3d.width, g3d.height);
				any = true;
			}

			// UV могли быть переписаны рендером вложенных уровней - пересчитываем
			if(mode[i] == MODE_TEXTURED) {
				if(pm.projectQuad(i, g3d, tmpBox) == PortalManager.VISIBLE) {
					pm.updateQuadUV(i, bbox[i]);
				}
			}

			pm.setWindow(i, mode[i] == MODE_TEXTURED ? 0 : -1);
			g3d.addMesh(pm.getQuad(i), pm.getQuadTransform(i));
		}

		if(any) {
			g3d.setDepthBias(0);
			g3d.setClip(0, 0, g3d.width, g3d.height);
		}
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
