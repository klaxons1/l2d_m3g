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
 *      камеры по bbox квада портала, ближняя плоскость заменена на плоскость
 *      выходного портала (oblique clipping), поэтому стена за ним не мешает;
 *   2) в основном проходе квад портала рисуется обычной геометрией с этой
 *      текстурой. Texcoord'ы = экранные координаты вершин, перспективная
 *      коррекция выключена -> текстура ложится ровно в дырку, а буфер глубины
 *      сам разбирается с перекрытием портала стенами и объектами.
 *
 * Запасной путь (если реализация M3G не умеет рендерить в Image2D): вид через
 * портал рисуется прямо в кадр в прямоугольнике bbox с очисткой глубины.
 */
public final class PortalRenderer {

	private static final int MODE_NONE = 0;
	private static final int MODE_TEXTURED = 1;
	private static final int MODE_FLAT = 2;
	private static final int MODE_DIRECT = 3;

	private final PortalManager pm;

	private final Transform mainCam = new Transform();
	private final Transform virtualCam = new Transform();
	private final float[] plane = new float[4];
	private final float[] camMat = new float[16];
	private final float[] forward = new float[3];

	private final int[][] bbox = new int[PortalManager.COUNT][4];
	private final int[] mode = new int[PortalManager.COUNT];

	private boolean rttChecked;
	private boolean rttSupported;

	public PortalRenderer(PortalManager pm) {
		this.pm = pm;
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

			pm.updateQuadUV(i, bbox[i]);
			renderPortalTexture(g3d, house, i);
			mode[i] = rttSupported ? MODE_TEXTURED : MODE_FLAT;
		}

		// восстанавливаем основную камеру
		g3d.clearClipPlane();
		g3d.setCameraTransform(mainCam);
	}

	private void renderPortalTexture(Renderer g3d, House house, int idx) {
		int dst = pm.getLinkedPortal(idx);
		int room = pm.getRoomId(dst);
		if(room < 0) return;

		int[] b = bbox[idx];

		pm.getVirtualCamera(idx, mainCam, virtualCam);
		g3d.setCameraTransform(virtualCam);

		// ближняя плоскость = плоскость выходного портала
		pm.getPlane(dst, plane);
		g3d.setClipPlane(plane[0], plane[1], plane[2], plane[3]);

		try {
			g3d.beginTextureTarget(pm.getImage(idx), b[0], b[1], b[2], b[3]);
			house.renderPortalView(g3d, room, b[0], b[1], b[2], b[3]);
		} catch (Throwable t) {
			System.out.println("PORTAL: ошибка рендера в текстуру: " + t);
			rttSupported = false;
		} finally {
			g3d.endTextureTarget();
		}

		// возвращаем основную камеру: она нужна для проекции следующего портала
		g3d.clearClipPlane();
		g3d.setCameraTransform(mainCam);
	}

	/**
	 * Второй проход: квады порталов рисуются вместе с остальной геометрией
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
				g3d.setClip(0, 0, g3d.width, g3d.height);
				any = true;
			}

			pm.setTextured(i, mode[i] == MODE_TEXTURED);
			g3d.addMesh(pm.getQuad(i), pm.getQuadTransform(i));
		}
	}

	/** Запасной путь: вид через портал прямо в кадр, в прямоугольнике bbox. */
	private void renderDirect(Renderer g3d, House house, int idx) {
		int dst = pm.getLinkedPortal(idx);
		int room = pm.getRoomId(dst);
		if(room < 0) return;

		int[] b = bbox[idx];

		pm.getVirtualCamera(idx, mainCam, virtualCam);
		g3d.setCameraTransform(virtualCam);

		pm.getPlane(dst, plane);
		g3d.setClipPlane(plane[0], plane[1], plane[2], plane[3]);

		g3d.setClip(b[0], b[1], b[2], b[3]);
		g3d.clearDepth();

		house.renderPortalView(g3d, room, b[0], b[1], b[2], b[3]);
	}
}
