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
 * Состояние двух порталов (синий / оранжевый): позиция, ортонормированный базис,
 * матрицы перехода, quad-меш с текстурой портала.
 *
 * Система координат портала: +X = right, +Y = up, +Z = нормаль стены (наружу, в комнату).
 * Матрица toWorld переводит координаты портала в мировые, fromWorld - наоборот.
 *
 * Матрица перехода между порталами (Portal-style):
 *   M = toWorld[dst] * FLIP * fromWorld[src],
 * где FLIP - поворот на 180 градусов вокруг локальной оси Y (diag(-1, 1, -1, 1)).
 * Ей же преобразуется камера при рендере вида через портал и игрок при телепортации.
 */
public final class PortalManager {

	public static final int COUNT = 2;
	public static final int BLUE = 0;
	public static final int ORANGE = 1;

	/** Полуразмеры окна портала в мировых единицах (радиус игрока ~800, рост ~1500). */
	public static final int HALF_W = 900;
	public static final int HALF_H = 1150;

	/** Отступ квада от стены, чтобы не было z-fighting со стеной. */
	private static final int WALL_OFFSET = 40;

	private static final float Q12 = 1.0f / 4096.0f;
	/** Фиксированная точка для texcoord'ов (VertexArray хранит short). */
	private static final int UV_ONE = 4096;

	/**
	 * Окно портала - эллипс. Он разбит на SEG секторов и RINGS колец
	 * (плюс внешнее кольцо-обводка). Texcoord'ы вершин - это их экранные
	 * координаты, а такое отображение линейно именно в пространстве экрана;
	 * мелкие треугольники делают ошибку интерполяции незаметной, даже если
	 * реализация игнорирует отключение перспективной коррекции.
	 */
	private static final int SEG = 16;
	private static final int RINGS = 3;
	/** Внешний радиус обводки в долях полуосей эллипса. */
	private static final float OUTLINE_OUTER = 1.14f;
	/** Центр + RINGS колец эллипса + кольцо обводки. */
	private static final int VERTS = 1 + (RINGS + 1) * SEG;

	/** Локальные координаты вершин в долях полуосей (единичный эллипс). */
	private static final float[] LOCX = new float[VERTS];
	private static final float[] LOCY = new float[VERTS];

	static {
		LOCX[0] = 0;
		LOCY[0] = 0;
		for(int k = 1; k <= RINGS + 1; k++) {
			float t = k <= RINGS ? (float) k / RINGS : OUTLINE_OUTER;
			for(int i = 0; i < SEG; i++) {
				double ang = 2.0 * Math.PI * i / SEG;
				int v = 1 + (k - 1) * SEG + i;
				LOCX[v] = t * (float) Math.cos(ang);
				LOCY[v] = t * (float) Math.sin(ang);
			}
		}
	}

	/** Индекс вершины кольца k (1..RINGS+1), сектор i. */
	private static int ringVert(int k, int i) {
		return 1 + (k - 1) * SEG + (i % SEG);
	}

	private static final int[] COLOR = {0x3366ff, 0xff6600};

	/** Результаты projectQuad. */
	public static final int NOT_VISIBLE = 0;
	public static final int VISIBLE = 1;
	public static final int NEAR_CLIPPED = 2;

	private static final Transform FLIP = createFlip();

	// ---- состояние порталов ----
	private final boolean[] active = new boolean[COUNT];
	private final int[] roomId = new int[COUNT];
	private final Vector3D[] pos = new Vector3D[COUNT];
	/** r(0..2), u(3..5), n(6..8), единичные векторы. */
	private final float[][] axis = new float[COUNT][9];

	private final Transform[] toWorld = new Transform[COUNT];
	private final Transform[] fromWorld = new Transform[COUNT];
	private final Transform[] quadTrans = new Transform[COUNT];

	// ---- ресурсы рендера ----
	private final Mesh[] quad = new Mesh[COUNT];
	private final VertexArray[] uvArray = new VertexArray[COUNT];
	/** Текстуры вида через портал: [портал][уровень вложенности]. */
	private final Image2D[][] image = new Image2D[COUNT][];
	private final Appearance[][] apTex = new Appearance[COUNT][];
	private final Appearance[] apFlat = new Appearance[COUNT];
	private final Appearance[] apOutline = new Appearance[COUNT];
	private int texSize;
	/** Сколько уровней вложенности порталов рисуем (1 = без рекурсии). */
	private int levels;

	// ---- временные буферы (без аллокаций в кадре) ----
	private final float[] mat = new float[16];
	private final float[] vec = new float[4];
	private final float[] vec2 = new float[4];
	private final float[] quadView = new float[VERTS * 4];
	private final float[] screen = new float[VERTS * 2];
	private final short[] uv = new short[VERTS * 2];
	private final Transform tmp = new Transform();
	private final float[] backupAxis = new float[9];
	private final Vector3D tmpDir = new Vector3D();

	public PortalManager(int texSize) {
		this(texSize, 1);
	}

	public PortalManager(int texSize, int levels) {
		this.texSize = texSize;
		this.levels = levels < 1 ? 1 : (levels > 3 ? 3 : levels);

		for(int i = 0; i < COUNT; i++) {
			pos[i] = new Vector3D();
			toWorld[i] = new Transform();
			fromWorld[i] = new Transform();
			quadTrans[i] = new Transform();
			active[i] = false;
			roomId[i] = -1;
		}
	}

	private static Transform createFlip() {
		Transform t = new Transform();
		t.set(new float[]{
			-1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, -1, 0,
			0, 0, 0, 1
		});
		return t;
	}

	/** Ленивая инициализация меша/текстуры портала. */
	public final void initResources() {
		for(int i = 0; i < COUNT; i++) {
			if(quad[i] != null) continue;

			// Перспективная коррекция ВЫКЛЮЧЕНА намеренно: texcoord'ы квада хранят
			// экранные координаты его вершин, поэтому нужна линейная в экранном
			// пространстве интерполяция - тогда текстура ложится пиксель-в-пиксель.
			PolygonMode pmode = new PolygonMode();
			pmode.setPerspectiveCorrectionEnable(false);
			// отсечение выключено намеренно: квад и так рисуется только когда
			// камера с лицевой стороны портала, а от ошибки в обходе вершин
			// (winding) портал просто исчез бы
			pmode.setCulling(PolygonMode.CULL_NONE);
			pmode.setShading(PolygonMode.SHADE_SMOOTH);
			pmode.setWinding(PolygonMode.WINDING_CCW);

			CompositingMode cm = new CompositingMode();
			cm.setDepthTestEnable(true);
			cm.setDepthWriteEnable(true);
			// подтягиваем квад к камере в буфере глубины, чтобы стена под ним
			// не пробивалась сквозь портал на больших расстояниях
			cm.setDepthOffset(-1.0f, -8.0f);

			apFlat[i] = new Appearance();
			apFlat[i].setPolygonMode(pmode);
			apFlat[i].setCompositingMode(cm);

			apTex[i] = new Appearance[levels];
			for(int l = 0; l < levels; l++) apTex[i][l] = apFlat[i];

			// обводка: тот же материал, но чуть сильнее подтянута к камере,
			// чтобы кольцо не спорило по глубине ни со стеной, ни с самим окном
			CompositingMode cmOut = new CompositingMode();
			cmOut.setDepthTestEnable(true);
			cmOut.setDepthWriteEnable(true);
			cmOut.setDepthOffset(-1.0f, -12.0f);

			apOutline[i] = new Appearance();
			apOutline[i].setPolygonMode(pmode);
			apOutline[i].setCompositingMode(cmOut);

			image[i] = new Image2D[levels];
			for(int l = 0; l < levels; l++) {
				try {
					image[i][l] = new Image2D(Image2D.RGB, texSize, texSize);

					Texture2D tex = new Texture2D(image[i][l]);
					tex.setBlending(Texture2D.FUNC_REPLACE);
					tex.setWrapping(Texture2D.WRAP_CLAMP, Texture2D.WRAP_CLAMP);
					tex.setFiltering(Texture2D.FILTER_BASE_LEVEL, Texture2D.FILTER_LINEAR);

					Appearance ap = new Appearance();
					ap.setPolygonMode(pmode);
					ap.setCompositingMode(cm);
					ap.setTexture(0, tex);
					apTex[i][l] = ap;
				} catch (Throwable t) {
					System.out.println("PORTAL: текстура портала не создана: " + t);
					image[i][l] = null;
					apTex[i][l] = apFlat[i];
					// глубже уже не рисуем
					if(l < levels) levels = l < 1 ? 1 : l;
				}
			}

			quad[i] = createQuadMesh(i);
		}
	}

	/** Есть ли текстуры для рендера видов через порталы. */
	public final boolean hasImages() {
		return image[0] != null && image[0][0] != null
				&& image[1] != null && image[1][0] != null;
	}

	/** Доступное число уровней вложенности (1 = без рекурсии). */
	public final int getLevels() {
		return levels;
	}

	/**
	 * Меш портала: эллиптическое окно (submesh 0) и кольцо-обводка (submesh 1).
	 * Обе части лежат в одном VertexBuffer, поэтому texcoord'ы пересчитываются
	 * одним проходом, а материалы у них разные.
	 */
	private Mesh createQuadMesh(int idx) {
		short[] positions = new short[VERTS * 3];
		byte[] colors = new byte[VERTS * 4];

		int col = COLOR[idx];
		int cr = (col >> 16) & 0xff, cg = (col >> 8) & 0xff, cb = col & 0xff;
		// обводка ярче самого окна
		int br = cr + (255 - cr) * 2 / 3;
		int bg = cg + (255 - cg) * 2 / 3;
		int bb = cb + (255 - cb) * 2 / 3;

		for(int v = 0; v < VERTS; v++) {
			positions[v * 3] = (short) (LOCX[v] * HALF_W);
			positions[v * 3 + 1] = (short) (LOCY[v] * HALF_H);
			positions[v * 3 + 2] = 0;

			boolean rim = v >= 1 + (RINGS - 1) * SEG;
			colors[v * 4] = (byte) (rim ? br : cr);
			colors[v * 4 + 1] = (byte) (rim ? bg : cg);
			colors[v * 4 + 2] = (byte) (rim ? bb : cb);
			colors[v * 4 + 3] = (byte) 255;
		}

		VertexArray posArray = new VertexArray(VERTS, 3, 2);
		posArray.set(0, VERTS, positions);

		VertexArray colArray = new VertexArray(VERTS, 4, 1);
		colArray.set(0, VERTS, colors);

		// стартовые texcoord'ы (до первого кадра); дальше их переписывает updateQuadUV
		uvArray[idx] = new VertexArray(VERTS, 2, 2);
		for(int v = 0; v < VERTS; v++) {
			uv[v * 2] = (short) (UV_ONE * (LOCX[v] + 1f) * 0.5f);
			uv[v * 2 + 1] = (short) (UV_ONE * (1f - LOCY[v]) * 0.5f);
		}
		uvArray[idx].set(0, VERTS, uv);

		VertexBuffer vb = new VertexBuffer();
		vb.setPositions(posArray, 1.0f, null);
		vb.setTexCoords(0, uvArray[idx], 1.0f / UV_ONE, null);
		vb.setColors(colArray);
		vb.setDefaultColor(0xff000000 | col);

		// --- окно: веер из центра + ленты между кольцами ---
		int perStrip = 2 * (SEG + 1);
		int[] discIdx = new int[RINGS * perStrip];
		int[] discLen = new int[RINGS];
		int k = 0;

		discLen[0] = perStrip;
		for(int i = 0; i <= SEG; i++) {
			discIdx[k++] = ringVert(1, i);
			discIdx[k++] = 0;
		}
		for(int r = 1; r < RINGS; r++) {
			discLen[r] = perStrip;
			for(int i = 0; i <= SEG; i++) {
				discIdx[k++] = ringVert(r + 1, i);
				discIdx[k++] = ringVert(r, i);
			}
		}

		// --- обводка: лента между внешним кольцом окна и кольцом обводки ---
		int[] ringIdx = new int[perStrip];
		k = 0;
		for(int i = 0; i <= SEG; i++) {
			ringIdx[k++] = ringVert(RINGS + 1, i);
			ringIdx[k++] = ringVert(RINGS, i);
		}

		IndexBuffer disc = new TriangleStripArray(discIdx, discLen);
		IndexBuffer ring = new TriangleStripArray(ringIdx, new int[]{perStrip});

		return new Mesh(vb,
				new IndexBuffer[]{disc, ring},
				new Appearance[]{apTex[idx][0], apOutline[idx]});
	}

	// ======================= размещение =======================

	/**
	 * Ставит портал на стену.
	 *
	 * @param idx    0 = синий, 1 = оранжевый
	 * @param point  точка попадания (мировые координаты)
	 * @param normal нормаль поверхности в Q12 (4096 = 1.0)
	 * @param room   id комнаты
	 */
	public final void placePortal(int idx, Vector3D point, Vector3D normal, int room) {
		float nx = normal.x * Q12;
		float ny = normal.y * Q12;
		float nz = normal.z * Q12;

		float len = (float) Math.sqrt(nx * nx + ny * ny + nz * nz);
		if(len < 0.0001f) return;
		nx /= len;
		ny /= len;
		nz /= len;

		// up: мировой Y, спроецированный на плоскость портала.
		// Для пола/потолка берём мировой -Z.
		float ux, uy, uz;
		if(ny > 0.9f || ny < -0.9f) {
			ux = 0;
			uy = 0;
			uz = -1;
		} else {
			ux = 0;
			uy = 1;
			uz = 0;
		}

		float d = nx * ux + ny * uy + nz * uz;
		ux -= nx * d;
		uy -= ny * d;
		uz -= nz * d;

		len = (float) Math.sqrt(ux * ux + uy * uy + uz * uz);
		if(len < 0.0001f) return;
		ux /= len;
		uy /= len;
		uz /= len;

		// right = up x normal (правая тройка: x = y * z)
		float rx = uy * nz - uz * ny;
		float ry = uz * nx - ux * nz;
		float rz = ux * ny - uy * nx;

		float[] a = axis[idx];
		a[0] = rx;
		a[1] = ry;
		a[2] = rz;
		a[3] = ux;
		a[4] = uy;
		a[5] = uz;
		a[6] = nx;
		a[7] = ny;
		a[8] = nz;

		pos[idx].set(point);
		roomId[idx] = room;
		active[idx] = true;

		updateMatrices(idx);
	}

	private void updateMatrices(int idx) {
		float[] a = axis[idx];
		Vector3D p = pos[idx];

		mat[0] = a[0];
		mat[1] = a[3];
		mat[2] = a[6];
		mat[3] = p.x;
		mat[4] = a[1];
		mat[5] = a[4];
		mat[6] = a[7];
		mat[7] = p.y;
		mat[8] = a[2];
		mat[9] = a[5];
		mat[10] = a[8];
		mat[11] = p.z;
		mat[12] = 0;
		mat[13] = 0;
		mat[14] = 0;
		mat[15] = 1;

		toWorld[idx].set(mat);
		fromWorld[idx].set(toWorld[idx]);
		fromWorld[idx].invert();

		// квад чуть отодвинут от стены
		mat[3] = p.x + a[6] * WALL_OFFSET;
		mat[7] = p.y + a[7] * WALL_OFFSET;
		mat[11] = p.z + a[8] * WALL_OFFSET;
		quadTrans[idx].set(mat);
	}

	/**
	 * Сдвигает центр портала так, чтобы окно целиком помещалось на поверхности:
	 * лучами вверх/вниз/вправо/влево ищем ближайшую геометрию и отодвигаемся от неё.
	 * Возвращает false, если места для портала нет.
	 */
	public final boolean fitOnSurface(int idx, House house, int room, Ray ray) {
		float[] a = axis[idx];
		Vector3D p = pos[idx];

		for(int pass = 0; pass < 2; pass++) {
			int dUp = freeSpace(house, room, ray, p, a, a[3], a[4], a[5], HALF_H);
			int dDown = freeSpace(house, room, ray, p, a, -a[3], -a[4], -a[5], HALF_H);
			int dRight = freeSpace(house, room, ray, p, a, a[0], a[1], a[2], HALF_W);
			int dLeft = freeSpace(house, room, ray, p, a, -a[0], -a[1], -a[2], HALF_W);

			int shiftV = 0, shiftH = 0;
			if(dUp < HALF_H) shiftV -= HALF_H - dUp;
			if(dDown < HALF_H) shiftV += HALF_H - dDown;
			if(dRight < HALF_W) shiftH -= HALF_W - dRight;
			if(dLeft < HALF_W) shiftH += HALF_W - dLeft;

			if(shiftV == 0 && shiftH == 0) break;

			// поверхность меньше окна портала - ставить некуда
			if(dUp + dDown < HALF_H * 17 / 10 || dLeft + dRight < HALF_W * 17 / 10) return false;

			p.x += (int) (a[3] * shiftV + a[0] * shiftH);
			p.y += (int) (a[4] * shiftV + a[1] * shiftH);
			p.z += (int) (a[5] * shiftV + a[2] * shiftH);
		}

		updateMatrices(idx);
		return true;
	}

	private int freeSpace(House house, int room, Ray ray, Vector3D p, float[] a,
			float dx, float dy, float dz, int limit) {
		ray.reset();
		ray.getStart().set(
			p.x + (int) (a[6] * WALL_OFFSET),
			p.y + (int) (a[7] * WALL_OFFSET),
			p.z + (int) (a[8] * WALL_OFFSET)
		);
		ray.getDir().set((int) (dx * limit), (int) (dy * limit), (int) (dz * limit));
		house.rayCast(room, ray);

		if(!ray.isCollision()) return limit;

		Vector3D c = ray.getCollisionPoint();
		int ddx = c.x - ray.getStart().x;
		int ddy = c.y - ray.getStart().y;
		int ddz = c.z - ray.getStart().z;
		int dist = (int) Math.sqrt((double) ddx * ddx + (double) ddy * ddy + (double) ddz * ddz);
		return dist < limit ? dist : limit;
	}

	public final void clear(int idx) {
		active[idx] = false;
		roomId[idx] = -1;
	}

	/**
	 * Пытается поставить портал: считает базис, подгоняет центр под поверхность
	 * и откатывает изменения, если окно портала не помещается.
	 *
	 * @return true, если портал поставлен
	 */
	public final boolean tryPlacePortal(int idx, Vector3D point, Vector3D normal, int room, House house, Ray ray) {
		boolean oldActive = active[idx];
		int oldRoom = roomId[idx];
		int oldX = pos[idx].x, oldY = pos[idx].y, oldZ = pos[idx].z;
		System.arraycopy(axis[idx], 0, backupAxis, 0, 9);

		active[idx] = false;
		placePortal(idx, point, normal, room);

		if(active[idx] && fitOnSurface(idx, house, room, ray)) return true;

		// откат
		active[idx] = oldActive;
		roomId[idx] = oldRoom;
		pos[idx].set(oldX, oldY, oldZ);
		System.arraycopy(backupAxis, 0, axis[idx], 0, 9);
		if(oldActive) updateMatrices(idx);

		return false;
	}

	// ======================= доступ =======================

	public final boolean isActive(int idx) {
		return active[idx];
	}

	public final boolean isLinked() {
		return active[0] && active[1];
	}

	public final int getRoomId(int idx) {
		return roomId[idx];
	}

	public final Vector3D getPosition(int idx) {
		return pos[idx];
	}

	public final int getLinkedPortal(int idx) {
		return idx == 0 ? 1 : 0;
	}

	public final int getColor(int idx) {
		return COLOR[idx];
	}

	public final Image2D getImage(int idx, int level) {
		if(image[idx] == null || level < 0 || level >= image[idx].length) return null;
		return image[idx][level];
	}

	public final Mesh getQuad(int idx) {
		return quad[idx];
	}

	public final Transform getQuadTransform(int idx) {
		return quadTrans[idx];
	}

	public final int getTexSize() {
		return texSize;
	}

	/**
	 * Чем заполнено окно портала при следующей отрисовке.
	 *
	 * @param level текстура нужного уровня вложенности, -1 = сплошная заливка
	 */
	/**
	 * Чем заполнено окно портала:
	 * level >= 0 - текстура вида соответствующего уровня,
	 * иначе - плоская заливка цветом портала.
	 */
	public final void setWindow(int idx, int level) {
		if(quad[idx] == null) return;

		Appearance ap = apFlat[idx];
		if(level >= 0 && apTex[idx] != null && level < apTex[idx].length) {
			ap = apTex[idx][level];
		}
		quad[idx].setAppearance(0, ap);
	}

	/** Нормаль портала в Q12. */
	public final void getNormal(int idx, Vector3D out) {
		float[] a = axis[idx];
		out.set((int) (a[6] * 4096), (int) (a[7] * 4096), (int) (a[8] * 4096));
	}

	/** Камера должна быть с лицевой стороны портала, иначе он не виден. */
	public final boolean isFrontFacing(int idx, Vector3D camPos) {
		if(!active[idx]) return false;
		float[] a = axis[idx];
		float dx = camPos.x - pos[idx].x;
		float dy = camPos.y - pos[idx].y;
		float dz = camPos.z - pos[idx].z;
		return dx * a[6] + dy * a[7] + dz * a[8] > 1.0f;
	}

	// ======================= матрицы перехода =======================

	/** out = toWorld[dst] * FLIP * fromWorld[src] - перенос из системы src в систему dst. */
	public final void getPortalTransform(int srcIdx, Transform out) {
		int dst = getLinkedPortal(srcIdx);
		out.set(toWorld[dst]);
		out.postMultiply(FLIP);
		out.postMultiply(fromWorld[srcIdx]);
	}

	/**
	 * Виртуальная камера для рендера вида через портал srcIdx.
	 * out = M * camToWorld, где M - матрица перехода src -> dst.
	 */
	public final void getVirtualCamera(int srcIdx, Transform camToWorld, Transform out) {
		getPortalTransform(srcIdx, out);
		out.postMultiply(camToWorld);
	}

	/**
	 * Плоскость парного портала в мировых координатах: (a, b, c, d),
	 * где a*x + b*y + c*z + d > 0 для точек перед порталом (внутри комнаты).
	 */
	public final void getPlane(int idx, float[] out) {
		float[] a = axis[idx];
		out[0] = a[6];
		out[1] = a[7];
		out[2] = a[8];
		out[3] = -(a[6] * pos[idx].x + a[7] * pos[idx].y + a[8] * pos[idx].z);
	}

	/** Локальные координаты точки в системе портала (x = right, y = up, z = нормаль). */
	private void toLocal(int idx, int x, int y, int z, float[] out) {
		out[0] = x;
		out[1] = y;
		out[2] = z;
		out[3] = 1;
		fromWorld[idx].transform(out);
	}

	/** Точка внутри эллипса окна (в локальных координатах портала). */
	private static boolean insideEllipse(float lx, float ly) {
		float ex = lx / HALF_W;
		float ey = ly / HALF_H;
		return ex * ex + ey * ey <= 1.0f;
	}

	/**
	 * Ищет портал, в проём которого попала точка и сквозь который она движется
	 * (или уже оказалась за плоскостью).
	 *
	 * @param range допуск по нормали; берётся с запасом, чтобы столкновения
	 *              выключались ДО того, как стена затормозит игрока
	 * @return индекс портала или -1
	 */
	private int openingIndex(int x, int y, int z, int radius, Vector3D speed) {
		if(!isLinked()) return -1;

		int range = radius * 2;

		for(int i = 0; i < COUNT; i++) {
			if(!active[i]) continue;

			toLocal(i, x, y, z, vec);
			if(vec[2] > range || vec[2] < -range) continue;
			if(!insideEllipse(vec[0], vec[1])) continue;

			// вплотную к плоскости (или уже за ней) - стены тут нет в любом
			// случае: так игрока не выталкивает ни на входе, ни сразу после
			// телепортации на выходе
			if(vec[2] < radius) return i;
			if(speed == null) return i;

			// движется в сторону портала?
			float[] a = axis[i];
			if(speed.x * a[6] + speed.y * a[7] + speed.z * a[8] < 0) return i;
		}
		return -1;
	}

	/**
	 * Точка входит в проём портала: столкновения со стенами надо отключить,
	 * иначе стена с порталом осталась бы твёрдой и игрок "спотыкался" бы о неё.
	 */
	public final boolean isInOpening(int x, int y, int z, int radius, Vector3D speed) {
		return openingIndex(x, y, z, radius, speed) >= 0;
	}

	/**
	 * Камера вплотную к проёму портала и смотрит в него: в этот момент окно
	 * занимает почти весь экран, а вершины уходят за ближнюю плоскость.
	 *
	 * @param fwd направление взгляда (единичное, мировые координаты)
	 */
	public final boolean isEyeAtOpening(int idx, Vector3D eye, int range, float[] fwd) {
		if(!active[idx]) return false;

		float[] a = axis[idx];
		if(fwd[0] * a[6] + fwd[1] * a[7] + fwd[2] * a[8] > -0.3f) return false;

		toLocal(idx, eye.x, eye.y, eye.z, vec);
		if(vec[2] > range || vec[2] < -range) return false;

		return insideEllipse(vec[0], vec[1]);
	}

	/** То же, но портал лежит в полу/потолке - тогда надо отключить и прилипание к полу. */
	public final boolean isInFloorOpening(int x, int y, int z, int radius, Vector3D speed) {
		int i = openingIndex(x, y, z, radius, speed);
		if(i < 0) return false;
		float ny = axis[i][7];
		return ny > 0.7f || ny < -0.7f;
	}

	/**
	 * Проверяет, пересёк ли отрезок from-to окно какого-либо портала спереди назад.
	 *
	 * @return индекс портала или -1
	 */
	public final int findCrossedPortal(int fx, int fy, int fz, int tx, int ty, int tz) {
		if(!isLinked()) return -1;

		for(int i = 0; i < COUNT; i++) {
			if(!active[i]) continue;

			toLocal(i, fx, fy, fz, vec);
			float z1 = vec[2];
			if(z1 < 0) continue;

			toLocal(i, tx, ty, tz, vec2);
			float z2 = vec2[2];
			if(z2 > 0) continue;

			float dz = z1 - z2;
			float t = dz > 0.0001f ? z1 / dz : 0f;

			float lx = vec[0] + (vec2[0] - vec[0]) * t;
			float ly = vec[1] + (vec2[1] - vec[1]) * t;

			if(!insideEllipse(lx, ly)) continue;

			return i;
		}

		return -1;
	}

	/**
	 * Телепортирует персонажа через портал srcIdx (позиция, скорость, углы).
	 *
	 * @param refOffsetY смещение по Y опорной точки, для которой зафиксировано
	 *                   пересечение (высота глаз или 0 для ног)
	 */
	public final void teleport(int srcIdx, Character ch, int refOffsetY) {
		getPortalTransform(srcIdx, tmp);

		Vector3D p = ch.getPosition();
		Vector3D speed = ch.getSpeed();
		Vector3D rot = ch.getRotation();

		// опорная точка
		vec[0] = p.x;
		vec[1] = p.y + refOffsetY;
		vec[2] = p.z;
		vec[3] = 1;
		tmp.transform(vec);

		// направление взгляда
		tmpDir.setFromRotation(rot.x, rot.y);
		vec2[0] = tmpDir.x;
		vec2[1] = tmpDir.y;
		vec2[2] = tmpDir.z;
		vec2[3] = 0;
		tmp.transform(vec2);

		float fx = vec2[0], fy = vec2[1], fz = vec2[2];
		float horiz = (float) Math.sqrt(fx * fx + fz * fz);

		float yawDeg = MathUtils.atan2(0, 0, -fx, -fz);
		rot.y = ((int) (yawDeg * (1 << 14) / 360f)) & ((1 << 14) - 1);
		if(horiz > 0.001f) {
			float pitchDeg = MathUtils.atan2(0, 0, fy, horiz);
			rot.x = (int) (pitchDeg * (1 << 14) / 360f);
		}

		// скорость
		vec2[0] = speed.x;
		vec2[1] = speed.y;
		vec2[2] = speed.z;
		vec2[3] = 0;
		tmp.transform(vec2);
		speed.set((int) vec2[0], (int) vec2[1], (int) vec2[2]);

		// выталкиваем игрока перед выходным порталом, чтобы он не застрял в стене
		int dst = getLinkedPortal(srcIdx);
		float[] a = axis[dst];
		int push = ch.getRadius() / 2;

		p.set(
			(int) (vec[0] + a[6] * push),
			(int) (vec[1] + a[7] * push) - refOffsetY,
			(int) (vec[2] + a[8] * push)
		);
	}

	// ======================= проекция на экран =======================

	/** Вершины меша портала в мировых координатах (x, y, z, 1). */
	private void getQuadWorldVerts(int idx, float[] out) {
		float[] a = axis[idx];
		Vector3D p = pos[idx];

		float cx = p.x + a[6] * WALL_OFFSET;
		float cy = p.y + a[7] * WALL_OFFSET;
		float cz = p.z + a[8] * WALL_OFFSET;

		for(int v = 0; v < VERTS; v++) {
			float lx = LOCX[v] * HALF_W;
			float ly = LOCY[v] * HALF_H;

			out[v * 4] = cx + a[0] * lx + a[3] * ly;
			out[v * 4 + 1] = cy + a[1] * lx + a[4] * ly;
			out[v * 4 + 2] = cz + a[2] * lx + a[5] * ly;
			out[v * 4 + 3] = 1;
		}
	}

	/**
	 * Проецирует квад портала на экран основной камеры.
	 * Заполняет bboxOut = {x1, y1, x2, y2} (обрезано по экрану).
	 *
	 * @return NOT_VISIBLE / VISIBLE / NEAR_CLIPPED
	 */
	public final int projectQuad(int idx, Renderer g3d, int[] bboxOut) {
		if(!active[idx] || quad[idx] == null) return NOT_VISIBLE;

		getQuadWorldVerts(idx, quadView);
		g3d.getInvCam().transform(quadView);

		float near = g3d.nearPlane;
		float w2 = g3d.width * 0.5f;
		float h2 = g3d.height * 0.5f;

		float minX = Float.MAX_VALUE, minY = Float.MAX_VALUE;
		float maxX = -Float.MAX_VALUE, maxY = -Float.MAX_VALUE;

		for(int i = 0; i < VERTS; i++) {
			float ax = quadView[i * 4];
			float ay = quadView[i * 4 + 1];
			float az = -quadView[i * 4 + 2];

			// вершина за ближней плоскостью: экранные координаты (а значит и UV)
			// посчитать нельзя, портал рисуем плоской заливкой
			if(az < near * 2f) return NEAR_CLIPPED;

			float w = near / az;
			float sx = ax * w * g3d.projXscale + w2;
			float sy = -ay * w * g3d.projYscale + h2;

			screen[i * 2] = sx;
			screen[i * 2 + 1] = sy;

			if(sx < minX) minX = sx;
			if(sx > maxX) maxX = sx;
			if(sy < minY) minY = sy;
			if(sy > maxY) maxY = sy;
		}

		int x1 = (int) Math.floor(minX);
		int y1 = (int) Math.floor(minY);
		int x2 = (int) Math.ceil(maxX) + 1;
		int y2 = (int) Math.ceil(maxY) + 1;

		if(x1 < 0) x1 = 0;
		if(y1 < 0) y1 = 0;
		if(x2 > g3d.width) x2 = g3d.width;
		if(y2 > g3d.height) y2 = g3d.height;

		if(x2 - x1 < 2 || y2 - y1 < 2) return NOT_VISIBLE;

		bboxOut[0] = x1;
		bboxOut[1] = y1;
		bboxOut[2] = x2;
		bboxOut[3] = y2;

		return VISIBLE;
	}

	/**
	 * Прописывает в texcoord'ы квада экранные координаты его вершин,
	 * нормированные на bbox (в него отрендерена текстура портала).
	 * Вместе с выключенной перспективной коррекцией это даёт точное
	 * попиксельное совпадение текстуры с тем, что видно "сквозь дырку".
	 */
	public final void updateQuadUV(int idx, int[] bbox) {
		float bx = bbox[0];
		float by = bbox[1];
		float bw = bbox[2] - bbox[0];
		float bh = bbox[3] - bbox[1];

		for(int i = 0; i < VERTS; i++) {
			float u = (screen[i * 2] - bx) / bw;
			float v = (screen[i * 2 + 1] - by) / bh;

			if(u < -4f) u = -4f;
			else if(u > 5f) u = 5f;
			if(v < -4f) v = -4f;
			else if(v > 5f) v = 5f;

			uv[i * 2] = (short) (u * UV_ONE);
			uv[i * 2 + 1] = (short) (v * UV_ONE);
		}

		uvArray[idx].set(0, VERTS, uv);
	}
}
