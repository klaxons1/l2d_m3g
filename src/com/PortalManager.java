package com;

import javax.microedition.m3g.Appearance;
import javax.microedition.m3g.CompositingMode;
import javax.microedition.m3g.IndexBuffer;
import javax.microedition.m3g.Mesh;
import javax.microedition.m3g.PolygonMode;
import javax.microedition.m3g.Transform;
import javax.microedition.m3g.TriangleStripArray;
import javax.microedition.m3g.VertexArray;
import javax.microedition.m3g.VertexBuffer;

/**
 * State of the two portals (blue / orange): position, orthonormal basis,
 * warp matrices, and the meshes that draw the elliptical window, its neon
 * outline and the invisible depth mask.
 *
 * Portal views are rendered straight into the frame with the depth buffer
 * split into bands (see PortalRenderer), so no render-to-texture resources
 * are used here.
 *
 * Portal coordinate frame: +X = right, +Y = up, +Z = wall normal (out of the
 * wall, into the room). The toWorld matrix maps portal coordinates to world
 * coordinates, fromWorld maps them back.
 *
 * The Portal-style warp transform is
 *   M = toWorld[dst] * FLIP * fromWorld[src],
 * where FLIP is a 180 degree rotation around the local Y axis
 * (diag(-1, 1, -1, 1)). It is applied both to the camera while rendering the
 * view through a portal and to the player during teleportation.
 */
public final class PortalManager {

	public static final int COUNT = 2;
	public static final int BLUE = 0;
	public static final int ORANGE = 1;

	/** Half extents of the portal opening in world units (player radius ~800, height ~1500). */
	public static final int HALF_W = 900;
	public static final int HALF_H = 1150;

	/** Offset of the quad from the wall to avoid z-fighting with it. */
	private static final int WALL_OFFSET = 40;

	private static final float Q12 = 1.0f / 4096.0f;

	/**
	 * The portal opening is an ellipse. It is split into SEG sectors and
	 * RINGS rings (plus the outer outline ring). Small triangles keep the
	 * ellipse smooth and let the depth mask hug the opening closely.
	 */
	private static final int SEG = 16;
	private static final int RINGS = 3;
	/** Outer radius of the outline as a fraction of the ellipse semiaxes. */
	private static final float OUTLINE_OUTER = 1.14f;
	/** Center + RINGS ellipse rings + the outline ring. */
	private static final int VERTS = 1 + (RINGS + 1) * SEG;

	/** Local vertex positions in fractions of the semiaxes (unit ellipse). */
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

	/** Vertex index of ring k (1..RINGS+1), sector i. */
	private static int ringVert(int k, int i) {
		return 1 + (k - 1) * SEG + (i % SEG);
	}

	private static final int[] COLOR = {0x3366ff, 0xff6600};

	/** Radius of the sphere bounding the portal mesh together with its outline. */
	private static final float BOUND_RADIUS =
			OUTLINE_OUTER * (float) Math.sqrt(HALF_W * HALF_W + HALF_H * HALF_H);

	/** setWindow level value: do not draw the window at all (the view is already in the frame). */
	public static final int HIDDEN_WINDOW = -2;

	public static final int NOT_VISIBLE = 0;
	public static final int VISIBLE = 1;
	public static final int NEAR_CLIPPED = 2;

	private static final Transform FLIP = createFlip();

	// ---- portal state ----
	private final boolean[] active = new boolean[COUNT];
	private final int[] roomId = new int[COUNT];
	private final Vector3D[] pos = new Vector3D[COUNT];
	/** r(0..2), u(3..5), n(6..8), unit vectors. */
	private final float[][] axis = new float[COUNT][9];

	private final Transform[] toWorld = new Transform[COUNT];
	private final Transform[] fromWorld = new Transform[COUNT];
	private final Transform[] quadTrans = new Transform[COUNT];

	// ---- render resources ----
	private final Mesh[] quad = new Mesh[COUNT];
	/** Same window geometry, but writes depth only (protects the view in band mode). */
	private final Mesh[] maskQuad = new Mesh[COUNT];
	private Appearance apMask;
	private Appearance apHidden;
	private final Appearance[] apFlat = new Appearance[COUNT];
	private final Appearance[] apOutline = new Appearance[COUNT];
	/** How many nested portal levels to render (1 = no recursion). */
	private int levels;

	// ---- scratch buffers (no per-frame allocations) ----
	private final float[] mat = new float[16];
	private final float[] vec = new float[4];
	private final float[] vec2 = new float[4];
	private final float[] quadView = new float[VERTS * 4];
	private final float[] screen = new float[VERTS * 2];
	// scratch for near-plane clipping of the outline ring
	private final float[] clipInX = new float[SEG + 1];
	private final float[] clipInY = new float[SEG + 1];
	private final float[] clipInZ = new float[SEG + 1];
	private final float[] clipOutX = new float[SEG + 2];
	private final float[] clipOutY = new float[SEG + 2];
	private final float[] clipOutZ = new float[SEG + 2];
	private final Transform tmp = new Transform();
	private final float[] backupAxis = new float[9];
	private final Vector3D tmpDir = new Vector3D();

	public PortalManager(int levels) {
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

	/** Lazy initialization of the portal meshes. */
	public final void initResources() {
		for(int i = 0; i < COUNT; i++) {
			if(quad[i] != null) continue;

			PolygonMode pmode = new PolygonMode();
			// Culling is deliberately off: the quad is only drawn while the
			// camera faces the portal, and a winding mistake would just make
			// the portal disappear.
			pmode.setCulling(PolygonMode.CULL_NONE);
			pmode.setShading(PolygonMode.SHADE_SMOOTH);
			pmode.setWinding(PolygonMode.WINDING_CCW);

			CompositingMode cm = new CompositingMode();
			cm.setDepthTestEnable(true);
			cm.setDepthWriteEnable(true);
			// Pull the quad toward the camera in the depth buffer, so the
			// wall behind it cannot poke through the portal at long range.
			cm.setDepthOffset(-1.0f, -8.0f);

			apFlat[i] = new Appearance();
			apFlat[i].setPolygonMode(pmode);
			apFlat[i].setCompositingMode(cm);

			// Outline: same material but pulled a little closer to the
			// camera, so the ring never fights the wall or the window over
			// depth.
			CompositingMode cmOut = new CompositingMode();
			cmOut.setDepthTestEnable(true);
			cmOut.setDepthWriteEnable(true);
			cmOut.setDepthOffset(-1.0f, -12.0f);

			apOutline[i] = new Appearance();
			apOutline[i].setPolygonMode(pmode);
			apOutline[i].setCompositingMode(cmOut);

			quad[i] = createQuadMesh(i);
		}
	}

	/** Available number of nested levels (1 = no recursion). */
	public final int getLevels() {
		return levels;
	}

	/**
	 * Portal mesh: elliptical window (submesh 0) and outline ring (submesh 1).
	 * Both parts share one VertexBuffer and differ only in material.
	 */
	private Mesh createQuadMesh(int idx) {
		short[] positions = new short[VERTS * 3];
		byte[] colors = new byte[VERTS * 4];

		int col = COLOR[idx];
		int cr = (col >> 16) & 0xff, cg = (col >> 8) & 0xff, cb = col & 0xff;
		// The outline is brighter than the window itself.
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

		VertexBuffer vb = new VertexBuffer();
		vb.setPositions(posArray, 1.0f, null);
		vb.setColors(colArray);
		vb.setDefaultColor(0xff000000 | col);

		// --- window: fan from the center + strips between the rings ---
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

		// --- outline: strip between the outer window ring and the outline ring ---
		int[] ringIdx = new int[perStrip];
		k = 0;
		for(int i = 0; i <= SEG; i++) {
			ringIdx[k++] = ringVert(RINGS + 1, i);
			ringIdx[k++] = ringVert(RINGS, i);
		}

		IndexBuffer disc = new TriangleStripArray(discIdx, discLen);
		IndexBuffer ring = new TriangleStripArray(ringIdx, new int[]{perStrip});

		// Mask: same window geometry, but only depth is written into the frame.
		maskQuad[idx] = new Mesh(vb, new IndexBuffer[]{disc},
				new Appearance[]{getMaskAppearance()});

		return new Mesh(vb,
				new IndexBuffer[]{disc, ring},
				new Appearance[]{apFlat[idx], apOutline[idx]});
	}

	/** Writes depth without touching color or alpha. */
	private Appearance getMaskAppearance() {
		if(apMask == null) {
			CompositingMode cm = new CompositingMode();
			cm.setColorWriteEnable(false);
			cm.setAlphaWriteEnable(false);
			cm.setDepthWriteEnable(true);
			cm.setDepthTestEnable(true);

			PolygonMode pmode = new PolygonMode();
			pmode.setCulling(PolygonMode.CULL_NONE);
			pmode.setPerspectiveCorrectionEnable(false);

			apMask = new Appearance();
			apMask.setCompositingMode(cm);
			apMask.setPolygonMode(pmode);
		}
		return apMask;
	}

	/** Draws nothing at all. */
	private Appearance getHiddenAppearance() {
		if(apHidden == null) {
			CompositingMode cm = new CompositingMode();
			cm.setColorWriteEnable(false);
			cm.setAlphaWriteEnable(false);
			cm.setDepthWriteEnable(false);

			apHidden = new Appearance();
			apHidden.setCompositingMode(cm);
		}
		return apHidden;
	}

	/** Window mask mesh (depth write only). */
	public final Mesh getMaskQuad(int idx) {
		return maskQuad[idx];
	}

	// ======================= placement =======================

	/**
	 * Places a portal on a wall.
	 *
	 * @param idx    0 = blue, 1 = orange
	 * @param point  hit point (world coordinates)
	 * @param normal surface normal in Q12 (4096 = 1.0)
	 * @param room   room id
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

		// Up is world Y projected onto the portal plane.
		// For floors/ceilings use world -Z instead.
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

		// right = up x normal (right-handed frame: x = y cross z)
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

		// The quad sits a little away from the wall.
		mat[3] = p.x + a[6] * WALL_OFFSET;
		mat[7] = p.y + a[7] * WALL_OFFSET;
		mat[11] = p.z + a[8] * WALL_OFFSET;
		quadTrans[idx].set(mat);
	}

	/**
	 * Shifts the portal center so the whole opening fits on the surface:
	 * casts rays up/down/left/right to find the nearest geometry and moves
	 * away from it. Returns false if there is not enough room for the portal.
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

			// Surface smaller than the opening: nowhere to place.
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
	 * Tries to place a portal: builds the basis, fits the center onto the
	 * surface and rolls everything back if the opening does not fit.
	 *
	 * @return true if the portal was placed
	 */
	public final boolean tryPlacePortal(int idx, Vector3D point, Vector3D normal, int room, House house, Ray ray) {
		boolean oldActive = active[idx];
		int oldRoom = roomId[idx];
		int oldX = pos[idx].x, oldY = pos[idx].y, oldZ = pos[idx].z;
		System.arraycopy(axis[idx], 0, backupAxis, 0, 9);

		active[idx] = false;
		placePortal(idx, point, normal, room);

		if(active[idx] && fitOnSurface(idx, house, room, ray)) return true;

		// Rollback.
		active[idx] = oldActive;
		roomId[idx] = oldRoom;
		pos[idx].set(oldX, oldY, oldZ);
		System.arraycopy(backupAxis, 0, axis[idx], 0, 9);
		if(oldActive) updateMatrices(idx);

		return false;
	}

	// ======================= accessors =======================

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

	/** Portal surface normal (out of the wall, unit vector, Q12). */
	public final void getNormal(int idx, Vector3D out) {
		out.x = (int) (axis[idx][6] * 4096f);
		out.y = (int) (axis[idx][7] * 4096f);
		out.z = (int) (axis[idx][8] * 4096f);
	}

	public final int getLinkedPortal(int idx) {
		return idx == 0 ? 1 : 0;
	}

	public final int getColor(int idx) {
		return COLOR[idx];
	}

	public final Mesh getQuad(int idx) {
		return quad[idx];
	}

	public final Transform getQuadTransform(int idx) {
		return quadTrans[idx];
	}

	/**
	 * Selects what fills the portal window on the next draw.
	 * HIDDEN_WINDOW draws nothing (the view is already in the frame),
	 * any other value draws a flat portal color fill.
	 */
	public final void setWindow(int idx, int level) {
		if(quad[idx] == null) return;

		Appearance ap = (level == HIDDEN_WINDOW) ? getHiddenAppearance() : apFlat[idx];
		quad[idx].setAppearance(0, ap);
	}

	/**
	 * Cheap but honest frustum test. The portal is treated as a bounding
	 * sphere: projecting all 65 vertices is pointless when it is entirely
	 * behind the camera or off-screen.
	 */
	public final boolean isInFrustum(int idx, Renderer g3d) {
		if(!active[idx] || quad[idx] == null) return false;

		Vector3D p = pos[idx];
		vec[0] = p.x;
		vec[1] = p.y;
		vec[2] = p.z;
		vec[3] = 1f;
		g3d.getInvCam().transform(vec);

		float x = vec[0], y = vec[1], z = vec[2];
		float near = g3d.nearPlane;

		// Entirely in front of the near plane (including behind the camera).
		if(-z + BOUND_RADIUS < near) return false;

		float hw = g3d.viewportPhysW * 0.5f;
		float lw = (float) Math.sqrt(near * near + hw * hw) * BOUND_RADIUS;
		if(near * x + hw * z > lw) return false;   // right of the right plane
		if(-near * x + hw * z > lw) return false;  // left of the left plane

		float hh = g3d.viewportPhysH * 0.5f;
		float lh = (float) Math.sqrt(near * near + hh * hh) * BOUND_RADIUS;
		if(near * y + hh * z > lh) return false;   // above the top plane
		if(-near * y + hh * z > lh) return false;  // below the bottom plane

		return true;
	}

	/** Active, facing the camera and inside the frustum. */
	public final boolean isVisible(int idx, Renderer g3d) {
		return active[idx] && isFrontFacing(idx, g3d.camPos) && isInFrustum(idx, g3d);
	}

	public final boolean isFrontFacing(int idx, Vector3D camPos) {
		if(!active[idx]) return false;
		float[] a = axis[idx];
		float dx = camPos.x - pos[idx].x;
		float dy = camPos.y - pos[idx].y;
		float dz = camPos.z - pos[idx].z;
		return dx * a[6] + dy * a[7] + dz * a[8] > 1.0f;
	}

	// ======================= warp matrices =======================

	/** out = toWorld[dst] * FLIP * fromWorld[src] - maps from the src frame to the dst frame. */
	public final void getPortalTransform(int srcIdx, Transform out) {
		int dst = getLinkedPortal(srcIdx);
		out.set(toWorld[dst]);
		out.postMultiply(FLIP);
		out.postMultiply(fromWorld[srcIdx]);
	}

	/**
	 * Virtual camera for the view through portal srcIdx.
	 * out = M * camToWorld, where M is the src -> dst warp transform.
	 */
	public final void getVirtualCamera(int srcIdx, Transform camToWorld, Transform out) {
		getPortalTransform(srcIdx, out);
		out.postMultiply(camToWorld);
	}

	/**
	 * Portal plane in world coordinates: (a, b, c, d), where
	 * a*x + b*y + c*z + d > 0 for points in front of the portal (in the room).
	 */
	public final void getPlane(int idx, float[] out) {
		float[] a = axis[idx];
		out[0] = a[6];
		out[1] = a[7];
		out[2] = a[8];
		out[3] = -(a[6] * pos[idx].x + a[7] * pos[idx].y + a[8] * pos[idx].z);
	}

	/** Signed local Z (distance along the portal normal) of a world point. */
	public final float getLocalZ(int idx, int x, int y, int z) {
		toLocal(idx, x, y, z, vec);
		return vec[2];
	}

	/** Local coordinates of a point in the portal frame (x = right, y = up, z = normal). */
	private void toLocal(int idx, int x, int y, int z, float[] out) {
		out[0] = x;
		out[1] = y;
		out[2] = z;
		out[3] = 1;
		fromWorld[idx].transform(out);
	}

	/** Whether a point (in portal local coordinates) is inside the elliptical opening. */
	private static boolean insideEllipse(float lx, float ly) {
		float ex = lx / HALF_W;
		float ey = ly / HALF_H;
		return ex * ex + ey * ey <= 1.0f;
	}

	/**
	 * Finds a portal whose opening contains the point and through which it
	 * moves (or behind whose plane it already is).
	 *
	 * @param range normal tolerance; kept generous so collisions are disabled
	 *              BEFORE the wall can stop the player
	 * @return portal index or -1
	 */
	private int openingIndex(int x, int y, int z, int radius, Vector3D speed) {
		if(!isLinked()) return -1;

		int range = radius * 2;

		for(int i = 0; i < COUNT; i++) {
			if(!active[i]) continue;

			toLocal(i, x, y, z, vec);
			if(vec[2] > range || vec[2] < -range) continue;
			if(!insideEllipse(vec[0], vec[1])) continue;

			// Right at the plane (or already past it) there is no wall
			// either way: this keeps the player from being pushed out both
			// on entry and right after teleport on exit.
			if(vec[2] < radius) return i;
			if(speed == null) return i;

			// Moving toward the portal?
			float[] a = axis[i];
			if(speed.x * a[6] + speed.y * a[7] + speed.z * a[8] < 0) return i;
		}
		return -1;
	}

	/**
	 * The point enters a portal opening: wall collisions must be disabled,
	 * otherwise the portal wall would stay solid and the player would
	 * stumble on it.
	 */
	public final boolean isInOpening(int x, int y, int z, int radius, Vector3D speed) {
		return openingIndex(x, y, z, radius, speed) >= 0;
	}

	/**
	 * The camera is right at the portal opening and looks into it: at that
	 * moment the window covers almost the whole screen and its vertices go
	 * beyond the near plane.
	 *
	 * @param fwd look direction (unit vector, world coordinates)
	 */
	public final boolean isEyeAtOpening(int idx, Vector3D eye, int range, float[] fwd) {
		if(!active[idx]) return false;

		float[] a = axis[idx];
		if(fwd[0] * a[6] + fwd[1] * a[7] + fwd[2] * a[8] > -0.3f) return false;

		toLocal(idx, eye.x, eye.y, eye.z, vec);
		if(vec[2] > range || vec[2] < -range) return false;

		return insideEllipse(vec[0], vec[1]);
	}

	/**
	 * Stricter than {@link #isEyeAtOpening}: the eye is actually entering
	 * the ellipse at the wall plane, which is the only situation where the
	 * linked room may paint the whole frame. Checking a wide Z band here
	 * used to flip the view full-screen at random while merely standing
	 * next to a portal.
	 */
	public final boolean isEyeEntering(int idx, Vector3D eye, float[] fwd) {
		if(!active[idx]) return false;

		float[] a = axis[idx];
		if(fwd[0] * a[6] + fwd[1] * a[7] + fwd[2] * a[8] > -0.3f) return false;

		toLocal(idx, eye.x, eye.y, eye.z, vec);
		// at or just crossing the plane, and not already far past it
		if(vec[2] > WALL_OFFSET + 60 || vec[2] < -HALF_H * 2) return false;

		return insideEllipse(vec[0], vec[1]);
	}

	/** Same check, but for a portal in the floor/ceiling: floor snapping must be disabled too. */
	public final boolean isInFloorOpening(int x, int y, int z, int radius, Vector3D speed) {
		int i = openingIndex(x, y, z, radius, speed);
		if(i < 0) return false;
		float ny = axis[i][7];
		return ny > 0.7f || ny < -0.7f;
	}

	/**
	 * Tests whether the segment from-to crosses any portal window from
	 * front to back.
	 *
	 * @return portal index or -1
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
	 * Teleports a character through portal srcIdx (position, velocity, angles).
	 *
	 * @param refOffsetY Y offset of the reference point whose crossing was
	 *                   detected (eye height, or 0 for the feet)
	 */
	public final void teleport(int srcIdx, Character ch, int refOffsetY) {
		getPortalTransform(srcIdx, tmp);

		Vector3D p = ch.getPosition();
		Vector3D speed = ch.getSpeed();
		Vector3D rot = ch.getRotation();

		// Reference point (the eye or feet point whose plane crossing was
		// detected).
		vec[0] = p.x;
		vec[1] = p.y + refOffsetY;
		vec[2] = p.z;
		vec[3] = 1;
		tmp.transform(vec);

		// Look direction.
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

		// Velocity.
		vec2[0] = speed.x;
		vec2[1] = speed.y;
		vec2[2] = speed.z;
		vec2[3] = 0;
		tmp.transform(vec2);
		speed.set((int) vec2[0], (int) vec2[1], (int) vec2[2]);

		// No fixed push-out: the warp mirrors the crossing point, so the
		// reference point already emerges exactly as far in front of the
		// destination portal as it had crossed past the source one. Adding
		// a constant forward offset here snapped the camera forward in a
		// single frame (the visible "jerk") and broke continuity with the
		// virtual view already rendered through the portal. While the body
		// is inside the opening band wall collisions stay disabled (see
		// isInOpening), so it coasts out of the wall on its own
		// transformed velocity.
		// The character is always an upright capsule (feet straight below
		// the eye in world Y), so after warping the crossed reference point
		// the feet follow by the eye height along world Y - not by rotating
		// that offset through the warp, which would lay a floor/ceiling
		// portal traveller horizontally into the floor.
		p.set((int) vec[0], (int) vec[1] - refOffsetY, (int) vec[2]);
	}

	// ======================= screen projection =======================

	/** Portal mesh vertices in world coordinates (x, y, z, 1). */
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
	 * Projects the portal quad onto the main camera screen.
	 * Fills bboxOut = {x1, y1, x2, y2} (clipped to the screen).
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

		boolean allAhead = true;
		for(int i = 0; i < VERTS; i++) {
			float ax = quadView[i * 4];
			float ay = quadView[i * 4 + 1];
			float az = -quadView[i * 4 + 2];

			if(az < near * 2f) {
				allAhead = false;
				continue;
			}

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

		if(!allAhead) {
			// Some outline vertices crossed the near plane while the eye is
			// still outside the window (walking up close at an angle): clip
			// the outline polygon against the near plane and project the
			// remainder. This keeps the portal view inside the window shape
			// instead of bailing to a full-screen or flat fallback.
			int clipped = clipOutlineToNear(g3d, near, bboxOut);
			if(clipped < 3) return NEAR_CLIPPED;
			return VISIBLE;
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
	 * Clips the outer outline ring against the camera near plane in camera
	 * space and projects the surviving polygon into a screen rectangle. The
	 * window may legitimately extend past the screen borders; the result is
	 * clamped to the viewport. Returns the number of surviving points.
	 */
	private int clipOutlineToNear(Renderer g3d, float near, int[] bboxOut) {
		final int first = 1 + RINGS * SEG;
		final float plane = near * 2f;

		for(int i = 0; i < SEG; i++) {
			int v = ringVert(RINGS + 1, i);
			clipInX[i] = quadView[v * 4];
			clipInY[i] = quadView[v * 4 + 1];
			clipInZ[i] = -quadView[v * 4 + 2];
		}

		int inN = SEG, outN = 0;

		// Single Sutherland-Hodgman pass against cz >= plane.
		for(int i = 0; i < inN; i++) {
			float ax = clipInX[i], ay = clipInY[i], az = clipInZ[i];
			float bx = clipInX[(i + 1) % inN], by = clipInY[(i + 1) % inN], bz = clipInZ[(i + 1) % inN];
			boolean aIn = az >= plane;
			boolean bIn = bz >= plane;

			if(aIn != bIn) {
				float t = (az - plane) / (az - bz);
				clipOutX[outN] = ax + (bx - ax) * t;
				clipOutY[outN] = ay + (by - ay) * t;
				clipOutZ[outN] = plane;
				if(++outN >= clipOutX.length) break;
			}
			if(bIn) {
				clipOutX[outN] = bx;
				clipOutY[outN] = by;
				clipOutZ[outN] = bz;
				if(++outN >= clipOutX.length) break;
			}
		}

		if(outN < 3) return outN;

		float w2 = g3d.width * 0.5f;
		float h2 = g3d.height * 0.5f;
		float minX = Float.MAX_VALUE, minY = Float.MAX_VALUE;
		float maxX = -Float.MAX_VALUE, maxY = -Float.MAX_VALUE;

		for(int i = 0; i < outN; i++) {
			float w = near / clipOutZ[i];
			float sx = clipOutX[i] * w * g3d.projXscale + w2;
			float sy = -clipOutY[i] * w * g3d.projYscale + h2;
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

		return outN;
	}
}
