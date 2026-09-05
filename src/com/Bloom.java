package com;

import javax.microedition.m3g.Appearance;
import javax.microedition.m3g.CompositingMode;
import javax.microedition.m3g.Image2D;
import javax.microedition.m3g.IndexBuffer;
import javax.microedition.m3g.Mesh;
import javax.microedition.m3g.PolygonMode;
import javax.microedition.m3g.Texture2D;
import javax.microedition.m3g.Transform;
import javax.microedition.m3g.TriangleStripArray;
import javax.microedition.m3g.VertexArray;
import javax.microedition.m3g.VertexBuffer;

/**
 * Bloom (свечение) на чистом JSR-184, без шейдеров.
 *
 * Идея: в M3G нельзя прочитать пиксели обратно, зато можно рендерить в
 * mutable Image2D и тут же использовать его как текстуру. Поэтому:
 *
 *  1) CAPTURE  - светящиеся объекты (окна и обводки порталов, искры, летящий
 *                сгусток) рисуются главной камерой в маленькую текстуру
 *                level[0] на чёрном фоне. Обычная геометрия не рисуется
 *                вообще, поэтому bright-pass не нужен: в буфере сразу только
 *                то, что должно светиться.
 *  2) BLUR     - цепочка уменьшений level[0] -> level[1] -> level[2].
 *                Каждый шаг - один полноэкранный квад с билинейной
 *                фильтрацией, то есть усреднение 2x2. Две ступени дают
 *                размытие радиусом ~4 текселя, а растягивание маленькой
 *                текстуры обратно на экран (тоже билинейно) добавляет
 *                ещё одно сглаживание.
 *  3) COMPOSITE- последний уровень накладывается на кадр одним квадом с
 *                ALPHA_ADD поверх готовой сцены.
 *
 * Полноэкранный квад - это billboard перед камерой: его размер точно равен
 * сечению фрустума на расстоянии d, поэтому он покрывает вьюпорт при любом
 * положении камеры и не требует отдельной проекции. Тест глубины у него
 * выключен, в буфер глубины он не пишет.
 *
 * Всё, что не получилось создать (нет поддержки Image2D-таргета, мало
 * памяти), выключает эффект целиком: игра просто рисуется без свечения.
 */
public final class Bloom {

	/** Сколько ступеней уменьшения (включая исходную). */
	private static final int LEVELS = 3;

	/** Яркость наложения, 0..255 (по каналу). */
	private static final int INTENSITY = 0xC0;

	/** Расстояние до полноэкранного квада в единицах nearPlane. */
	private static final float QUAD_DIST = 10f;

	private Image2D[] level;
	private Texture2D[] tex;

	private Mesh quad;
	private VertexBuffer quadVB;
	private Appearance apCopy;
	private Appearance apAdd;

	private boolean ready;
	private boolean captured;

	private final Transform tmp = new Transform();

	/**
	 * @param size сторона исходной текстуры свечения (степень двойки)
	 */
	public Bloom(int size) {
		try {
			if(size < (1 << (LEVELS - 1)) * 8) size = (1 << (LEVELS - 1)) * 8;

			level = new Image2D[LEVELS];
			tex = new Texture2D[LEVELS];

			int s = size;
			for(int i = 0; i < LEVELS; i++) {
				level[i] = new Image2D(Image2D.RGB, s, s);

				Texture2D t = new Texture2D(level[i]);
				t.setFiltering(Texture2D.FILTER_BASE_LEVEL, Texture2D.FILTER_LINEAR);
				t.setWrapping(Texture2D.WRAP_CLAMP, Texture2D.WRAP_CLAMP);
				t.setBlending(Texture2D.FUNC_MODULATE);
				tex[i] = t;

				s >>= 1;
			}

			createQuad();
			ready = true;
		} catch (Throwable t) {
			System.out.println("BLOOM: выключен - " + t);
			destroy();
		}
	}

	private void createQuad() {
		short[] pos = new short[]{
			-1, -1, 0,
			1, -1, 0,
			-1, 1, 0,
			1, 1, 0
		};
		// t = 0 - верх текстуры, значит верхним вершинам квада нужен v = 0
		short[] uv = new short[]{
			0, 1,
			1, 1,
			0, 0,
			1, 0
		};

		VertexArray vaPos = new VertexArray(4, 3, 2);
		vaPos.set(0, 4, pos);

		VertexArray vaUV = new VertexArray(4, 2, 2);
		vaUV.set(0, 4, uv);

		quadVB = new VertexBuffer();
		quadVB.setPositions(vaPos, 1.0f, null);
		quadVB.setTexCoords(0, vaUV, 1.0f, null);
		quadVB.setDefaultColor(0xffffffff);

		IndexBuffer ib = new TriangleStripArray(new int[]{0, 1, 2, 3}, new int[]{4});

		PolygonMode pm = new PolygonMode();
		pm.setCulling(PolygonMode.CULL_NONE);
		pm.setShading(PolygonMode.SHADE_FLAT);
		// квад строго параллелен экрану, коррекция перспективы не нужна
		pm.setPerspectiveCorrectionEnable(false);

		CompositingMode cmCopy = new CompositingMode();
		cmCopy.setDepthTestEnable(false);
		cmCopy.setDepthWriteEnable(false);
		cmCopy.setBlending(CompositingMode.REPLACE);

		CompositingMode cmAdd = new CompositingMode();
		cmAdd.setDepthTestEnable(false);
		cmAdd.setDepthWriteEnable(false);
		cmAdd.setBlending(CompositingMode.ALPHA_ADD);

		apCopy = new Appearance();
		apCopy.setPolygonMode(pm);
		apCopy.setCompositingMode(cmCopy);

		apAdd = new Appearance();
		apAdd.setPolygonMode(pm);
		apAdd.setCompositingMode(cmAdd);

		quad = new Mesh(quadVB, new IndexBuffer[]{ib}, new Appearance[]{apCopy});
	}

	public final boolean isReady() {
		return ready;
	}

	public final void destroy() {
		ready = false;
		captured = false;
		level = null;
		tex = null;
		quad = null;
		quadVB = null;
		apCopy = null;
		apAdd = null;
	}

	/**
	 * Рисует источники свечения в текстуру и размывает её.
	 * Вызывать ДО привязки экрана (bindTarget(Graphics)) и с уже
	 * установленной главной камерой.
	 */
	public final void capture(Renderer g3d, House house, PortalManager pm) {
		captured = false;
		if(!ready) return;

		try {
			if(!hasSources(g3d, house, pm)) return;

			int w = g3d.getWidth();
			int h = g3d.getHeight();

			// --- 1) источники света на чёрном фоне ---
			g3d.beginTextureTarget(level[0], 0, 0, w, h);
			g3d.setClip(0, 0, w, h);

			if(pm != null) {
				for(int i = 0; i < PortalManager.COUNT; i++) {
					if(!isPortalGlowing(g3d, house, pm, i)) continue;

					// плоская заливка: окно цветом портала, обводка ярче
					pm.setWindow(i, -1);
					g3d.addMesh(pm.getQuad(i), pm.getQuadTransform(i));
				}
			}

			PortalGun.renderGlow(g3d);

			g3d.endTextureTarget();

			// --- 2) размытие уменьшением ---
			for(int i = 1; i < LEVELS; i++) {
				g3d.beginTextureTarget(level[i], 0, 0, w, h);
				g3d.setClip(0, 0, w, h);
				drawQuad(g3d, apCopy, tex[i - 1], 0xffffff);
				g3d.endTextureTarget();
			}

			captured = true;
		} catch (Throwable t) {
			System.out.println("BLOOM: ошибка рендера, выключаю - " + t);
			try {
				g3d.endTextureTarget();
			} catch (Throwable t2) {
			}
			destroy();
		}
	}

	/**
	 * Накладывает свечение на готовый кадр.
	 * Вызывать после отрисовки сцены, пока экран ещё привязан.
	 */
	public final void composite(Renderer g3d) {
		if(!ready || !captured) return;

		try {
			g3d.setClip(0, 0, g3d.getWidth(), g3d.getHeight());
			drawQuad(g3d, apAdd, tex[LEVELS - 1],
					(INTENSITY << 16) | (INTENSITY << 8) | INTENSITY);
		} catch (Throwable t) {
			System.out.println("BLOOM: ошибка наложения, выключаю - " + t);
			destroy();
		}
	}

	/** Есть ли вообще что подсвечивать в этом кадре. */
	private boolean hasSources(Renderer g3d, House house, PortalManager pm) {
		if(PortalGun.hasGlow()) return true;

		if(pm != null) {
			for(int i = 0; i < PortalManager.COUNT; i++) {
				if(isPortalGlowing(g3d, house, pm, i)) return true;
			}
		}

		return false;
	}

	private boolean isPortalGlowing(Renderer g3d, House house, PortalManager pm, int idx) {
		if(!pm.isActive(idx)) return false;
		if(!pm.isFrontFacing(idx, g3d.camPos)) return false;

		// грубое отсечение по видимости комнаты (данные прошлого кадра):
		// свечение портала из другого крыла уровня не должно светить сквозь стены
		return house == null || house.isRoomVisible(pm.getRoomId(idx));
	}

	/**
	 * Полноэкранный квад: billboard перед камерой ровно по размеру фрустума.
	 */
	private void drawQuad(Renderer g3d, Appearance ap, Texture2D texture, int color) {
		ap.setTexture(0, texture);
		quad.setAppearance(0, ap);
		quadVB.setDefaultColor(0xff000000 | color);

		float yaw = g3d.camRot.y * 360f / (1 << 14);
		float pitch = g3d.camRot.x * 360f / (1 << 14);
		float roll = g3d.camRot.z * 360f / (1 << 14);

		double ry = Math.toRadians(yaw);
		double rp = Math.toRadians(pitch);

		float d = g3d.nearPlane * QUAD_DIST;
		float cp = (float) Math.cos(rp);

		float fx = (float) (-Math.sin(ry) * cp);
		float fy = (float) Math.sin(rp);
		float fz = (float) (-Math.cos(ry) * cp);

		// половина сечения фрустума на расстоянии d (+1% чтобы не было щелей по краям)
		float sx = g3d.viewportPhysW * d / g3d.nearPlane * 0.505f;
		float sy = g3d.viewportPhysH * d / g3d.nearPlane * 0.505f;

		tmp.setIdentity();
		tmp.postTranslate(g3d.camPos.x + fx * d, g3d.camPos.y + fy * d, g3d.camPos.z + fz * d);
		tmp.postRotate(yaw, 0, 1, 0);
		tmp.postRotate(pitch, 1, 0, 0);
		if(roll != 0) tmp.postRotate(roll, 0, 0, 1);
		tmp.postScale(sx, sy, 1);

		g3d.addMesh(quad, tmp);
	}
}
