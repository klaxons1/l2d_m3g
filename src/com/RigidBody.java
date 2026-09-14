package com;

/**
 * Oriented rigid box with impulse based contact physics.
 *
 * Contact generation is separating-axis-theorem (SAT) box vs mesh
 * triangle: 13 candidate axes (3 box face normals, 1 triangle face
 * normal, 9 edge-edge cross products), minimum-overlap axis gives the
 * MTV. This resolves box-vs-triangle exactly - including box-corner-
 * vs-mesh-edge, which the previous feature-sampling approach could only
 * approximate.
 *
 * All state is Q12 fixed point (4096 == 1.0). No per-frame allocations.
 *
 * Matrices are 3x3 row-major Q12; orientation matrix R maps local
 * vectors to world vectors (world = R * local). Columns of R are the
 * box's local axes expressed in world.
 */
public final class RigidBody {

	public static final int F = 4096;

	private static final int MAX_CONTACTS = 32;
	private static final int VERTICES = 8;

	/** Broadphase band around the box (units). */
	private static final int CONTACT_MARGIN = 64;
	/** Deeper penetration than this rolls the step back and halves dt. */
	private static final int PENETRATION_THRESHOLD = 64;
	/** Smallest substep as a fraction of a frame (1/256). */
	private static final int MIN_DT = 16;
	/** Gauss-Seidel sweeps over the contact list per frame. */
	private static final int IMPULSE_ITERATIONS = 8;
	/** Position projection sweeps. */
	private static final int POSITION_ITERATIONS = 4;
	/** Contacts within this many units are treated as resting. */
	private static final int POSITION_SLOP = 3;
	private static final int MAX_LINEAR = 2048 << 12;
	private static final int MAX_ANGULAR = 2 * F;
	private static final int RESTITUTION_SPEED = 80 << 12;

	private static final int GRAVITY = 20 << 12;
	private static final int LINEAR_DRAG = 25;
	private static final int ANGULAR_DRAG = 20;

	private static final int RESTITUTION = 819;
	private static final int FRICTION = 4096;

	private static final int SLEEP_LOW = 120000;
	private static final int SLEEP_HIGH = 400000;
	private static final int SLEEP_TIME = 16;

	// ---- body state ----
	private int hx, hy, hz;
	private int mass, invMass;
	private int il0, il4, il8;
	private int px, py, pz;
	private int vx, vy, vz;
	private int lx, ly, lz;
	private int wx, wy, wz;
	private final int[] r = new int[9];
	private final int[] invILocal = new int[9];
	private final int[] invIWorld = new int[9];

	// ---- contacts ----
	private final int[] cpx = new int[MAX_CONTACTS];
	private final int[] cpy = new int[MAX_CONTACTS];
	private final int[] cpz = new int[MAX_CONTACTS];
	private final int[] cnx = new int[MAX_CONTACTS];
	private final int[] cny = new int[MAX_CONTACTS];
	private final int[] cnz = new int[MAX_CONTACTS];
	private final int[] cpen = new int[MAX_CONTACTS];
	private int numContacts;
	private int maxPenetration;
	private boolean groundContact;

	// ---- world AABB (units) ----
	private int boxMinX, boxMinY, boxMinZ, boxMaxX, boxMaxY, boxMaxZ;
	private int cornerRadius;

	// ---- SAT scratch ----
	private int satBestOverlap;
	private int satBestNx, satBestNy, satBestNz;

	// ---- backup for substep rollback ----
	private int bpx, bpy, bpz, bvx, bvy, bvz, blx, bly, blz, bwx, bwy, bwz;
	private final int[] br = new int[9];
	private final int[] biw = new int[9];

	private boolean sleeping;
	private int sleepCounter;
	private int energy;
	public int lastSubsteps = 1;
	
	// Sutherland-Hodgman scratch for clipping the triangle to a box face.
	// Two buffers because each half-plane clip reads one and writes the
	// other. A triangle clipped by a rectangle has at most 7 vertices.
	private static final int CLIP_MAX = 12;
	private final int[] clipUa = new int[CLIP_MAX];
	private final int[] clipVa = new int[CLIP_MAX];
	private final int[] clipUb = new int[CLIP_MAX];
	private final int[] clipVb = new int[CLIP_MAX];
	
	// Warm-start state: last frame's contact impulses, used as the
	// initial guess for this frame's solver. Resting contacts then start
	// already converged instead of building up from zero every frame,
	// which is what leaves a few units/frame of residual velocity and
	// shows up as the resting-jitter ("slight jumping").
	private int prevNumContacts;
	private final int[] prevCpx = new int[MAX_CONTACTS];
	private final int[] prevCpy = new int[MAX_CONTACTS];
	private final int[] prevCpz = new int[MAX_CONTACTS];
	private final int[] prevCnx = new int[MAX_CONTACTS];
	private final int[] prevCny = new int[MAX_CONTACTS];
	private final int[] prevCnz = new int[MAX_CONTACTS];
	private final int[] prevAccN = new int[MAX_CONTACTS];
	private final int[] prevAccT = new int[MAX_CONTACTS];

	/**
	 * Triangle/quad mesh as the engine stores it (see MeshData and
	 * SphereCast): short indexed vertices/polygons, Q12 polygon normals
	 * and an fp8 scale with integer offsets applied to every vertex.
	 */
	public static final class Collider {
		public short[] verts;
		public short[] pols;
		public short[] norms;
		public int quads;
		public int tris;
		public int scale8;
		public int offX, offY, offZ;
	}

	public RigidBody(int halfExtentUnits) {
		this.hx = this.hy = this.hz = halfExtentUnits << 12;
		reset(0, 0, 0);
	}

	public void setMass(int massQ12) {
		this.mass = massQ12;
		this.invMass = divQ(F, massQ12);
		computeLocalInertia();
	}

	public void reset(int centerX, int centerY, int centerZ) {
		this.mass = F;
		this.invMass = F;
		this.px = centerX << 12;
		this.py = centerY << 12;
		this.pz = centerZ << 12;
		this.vx = this.vy = this.vz = 0;
		this.lx = this.ly = this.lz = 0;
		this.wx = this.wy = this.wz = 0;
		for(int i = 0; i < 9; i++) r[i] = 0;
		r[0] = r[4] = r[8] = F;
		computeLocalInertia();
		recomputeWorldInertia();
		this.sleeping = false;
		this.sleepCounter = 0;
		this.energy = 0;
		computeVertices();
	}

	public void wake() {
		this.sleeping = false;
		this.sleepCounter = 0;
	}

	public boolean isSleeping() {
		return sleeping;
	}

	public int getContactCount() {
		return numContacts;
	}

	public int getCenterX() { return px >> 12; }
	public int getCenterY() { return py >> 12; }
	public int getCenterZ() { return pz >> 12; }

	public int getVelocityX() { return vx >> 12; }
	public int getVelocityY() { return vy >> 12; }
	public int getVelocityZ() { return vz >> 12; }

	public int getHalfExtent() { return hx >> 12; }

	public int getOrientation(int i) { return r[i]; }

	public void setVelocity(int unitsX, int unitsY, int unitsZ) {
		this.vx = unitsX << 12;
		this.vy = unitsY << 12;
		this.vz = unitsZ << 12;
		wake();
	}

	public int getAngularX() { return wx; }
	public int getAngularY() { return wy; }
	public int getAngularZ() { return wz; }

	public void setAngularVelocity(int ax, int ay, int az) {
		this.wx = ax; this.wy = ay; this.wz = az;
		recomputeMomentum();
		wake();
	}

	public void nudge(int dxUnits, int dyUnits, int dzUnits) {
		this.px += dxUnits << 12;
		this.py += dyUnits << 12;
		this.pz += dzUnits << 12;
		this.vx += dxUnits << 12;
		this.vy += dyUnits << 12;
		this.vz += dzUnits << 12;
		wake();
	}

	public void moveKinematic(int centerX, int centerY, int centerZ) {
		int nx = centerX << 12, ny = centerY << 12, nz = centerZ << 12;
		this.vx = nx - px;
		this.vy = ny - py;
		this.vz = nz - pz;
		this.px = nx;
		this.py = ny;
		this.pz = nz;
		this.lx = this.ly = this.lz = 0;
		this.wx = this.wy = this.wz = 0;
		identity3(r);
		recomputeWorldInertia();
		wake();
		computeVertices();
	}

	public void warp(float[] m) {
		float x = px / (float) F, y = py / (float) F, z = pz / (float) F;
		px = q(m[0] * x + m[1] * y + m[2] * z + m[3]);
		py = q(m[4] * x + m[5] * y + m[6] * z + m[7]);
		pz = q(m[8] * x + m[9] * y + m[10] * z + m[11]);

		int[] wv = warpVector(m, vx, vy, vz);
		vx = wv[0]; vy = wv[1]; vz = wv[2];
		wv = warpVector(m, wx, wy, wz);
		wx = wv[0]; wy = wv[1]; wz = wv[2];

		for(int col = 0; col < 3; col++) {
			float ax = r[col] / (float) F;
			float ay = r[3 + col] / (float) F;
			float az = r[6 + col] / (float) F;
			r[col] = q(m[0] * ax + m[1] * ay + m[2] * az);
			r[3 + col] = q(m[4] * ax + m[5] * ay + m[6] * az);
			r[6 + col] = q(m[8] * ax + m[9] * ay + m[10] * az);
		}
		fixMatrix();
		recomputeWorldInertia();

		recomputeMomentum();
		wake();
		computeVertices();
	}

	private final int[] warpScratch = new int[3];
	private int[] warpVector(float[] m, int x, int y, int z) {
		float fx = x / (float) F, fy = y / (float) F, fz = z / (float) F;
		warpScratch[0] = q(m[0] * fx + m[1] * fy + m[2] * fz);
		warpScratch[1] = q(m[4] * fx + m[5] * fy + m[6] * fz);
		warpScratch[2] = q(m[8] * fx + m[9] * fy + m[10] * fz);
		return warpScratch;
	}
	private static int q(float v) {
		return (int) (v * F);
	}

	// ===================== simulation =====================

	public void step(Collider[] colliders, int count, boolean world) {
		if(sleeping) {
			vx = vy = vz = 0;
			wx = wy = wz = 0;
			lx = ly = lz = 0;
			computeVertices();
			this.energy = 0;
			return;
		}

		int fx = -vx / LINEAR_DRAG;
		int fy = -GRAVITY - vy / LINEAR_DRAG;
		int fz = -vz / LINEAR_DRAG;
		int mx = -wx / ANGULAR_DRAG;
		int my = -wy / ANGULAR_DRAG;
		int mz = -wz / ANGULAR_DRAG;

		int dt = F;
		int substeps = 1;
		while(true) {
			backup();
			integrate(dt, fx, fy, fz, mx, my, mz);
			collideWorld(colliders, count, world);

			if(maxPenetration > PENETRATION_THRESHOLD << 12 && dt > MIN_DT) {
				restore();
				dt >>= 1;
				++substeps;
				continue;
			}

			if(numContacts > 0) applyImpulses();
			break;
		}
		this.lastSubsteps = substeps;

		clampVelocity();

		energy = mul(vx, vx) + mul(vy, vy) + mul(vz, vz)
				+ mul(wx, wx) + mul(wy, wy) + mul(wz, wz);
		if(!groundContact) {
			sleepCounter = 0;
		} else if(energy >= SLEEP_HIGH) {
			sleeping = false;
			sleepCounter = 0;
		} else if(energy <= SLEEP_LOW) {
			if(++sleepCounter >= SLEEP_TIME) {
				sleeping = true;
				vx = vy = vz = 0;
				wx = wy = wz = 0;
				lx = ly = lz = 0;
			}
		} else {
			sleepCounter = 0;
		}

		computeVertices();
	}

	private void integrate(int dt, int fx, int fy, int fz, int mx, int my, int mz) {
		px += mul(vx, dt);
		py += mul(vy, dt);
		pz += mul(vz, dt);

		int wxd = mul(wx, dt), wyd = mul(wy, dt), wzd = mul(wz, dt);
		for(int row = 0; row < 3; row++) {
			int s0, s1, s2;
			if(row == 0) { s0 = 0; s1 = -wzd; s2 = wyd; }
			else if(row == 1) { s0 = wzd; s1 = 0; s2 = -wxd; }
			else { s0 = -wyd; s1 = wxd; s2 = 0; }
			for(int col = 0; col < 3; col++) {
				int v = mul(s0, r[col]) + mul(s1, r[3 + col]) + mul(s2, r[6 + col]);
				r[row * 3 + col] += v;
			}
		}

		vx += divQ(mul(fx, dt), mass);
		vy += divQ(mul(fy, dt), mass);
		vz += divQ(mul(fz, dt), mass);

		lx += mul(mx, dt);
		ly += mul(my, dt);
		lz += mul(mz, dt);

		fixMatrix();
		recomputeWorldInertia();
		wx = eval24X(invIWorld, lx, ly, lz);
		wy = eval24Y(invIWorld, lx, ly, lz);
		wz = eval24Z(invIWorld, lx, ly, lz);
	}

	private void computeLocalInertia() {
		identity3(invILocal);
		int x2 = mul(hx, hx), y2 = mul(hy, hy), z2 = mul(hz, hz);
		invILocal[0] = (int) (((long) (3 * F) << 24) / mul(mass, y2 + z2));
		invILocal[4] = (int) (((long) (3 * F) << 24) / mul(mass, x2 + z2));
		invILocal[8] = (int) (((long) (3 * F) << 24) / mul(mass, x2 + y2));
		il0 = mul(mass, y2 + z2) / 3;
		il4 = mul(mass, x2 + z2) / 3;
		il8 = mul(mass, x2 + y2) / 3;
	}

	private void recomputeWorldInertia() {
		for(int row = 0; row < 3; row++) {
			for(int col = 0; col < 3; col++) {
				int t0 = mul(r[row * 3], invILocal[0]);
				int t1 = mul(r[row * 3 + 1], invILocal[4]);
				int t2 = mul(r[row * 3 + 2], invILocal[8]);
				int v = mul(t0, r[col * 3]) + mul(t1, r[col * 3 + 1]) + mul(t2, r[col * 3 + 2]);
				invIWorld[row * 3 + col] = v;
			}
		}
	}

	private static int eval24X(int[] m, int x, int y, int z) {
		return (int) (((long) m[0] * x + (long) m[1] * y + (long) m[2] * z) >> 24);
	}
	private static int eval24Y(int[] m, int x, int y, int z) {
		return (int) (((long) m[3] * x + (long) m[4] * y + (long) m[5] * z) >> 24);
	}
	private static int eval24Z(int[] m, int x, int y, int z) {
		return (int) (((long) m[6] * x + (long) m[7] * y + (long) m[8] * z) >> 24);
	}

	private void recomputeMomentum() {
		int[] iw = tmpMatrix;
		for(int row = 0; row < 3; row++) {
			for(int col = 0; col < 3; col++) {
				int t0 = mul(r[row * 3], il0);
				int t1 = mul(r[row * 3 + 1], il4);
				int t2 = mul(r[row * 3 + 2], il8);
				iw[row * 3 + col] = mul(t0, r[col * 3]) + mul(t1, r[col * 3 + 1]) + mul(t2, r[col * 3 + 2]);
			}
		}
		lx = evalX(iw, wx, wy, wz);
		ly = evalY(iw, wx, wy, wz);
		lz = evalZ(iw, wx, wy, wz);
	}

	private void fixMatrix() {
		int xx = r[0], xy = r[3], xz = r[6];
		int yx = r[1], yy = r[4], yz = r[7];

		int mx = norm3(xx, xy, xz);
		if(mx == 0) { identity3(r); return; }
		xx = divQ(xx, mx); xy = divQ(xy, mx); xz = divQ(xz, mx);

		int d = mul(xx, yx) + mul(xy, yy) + mul(xz, yz);
		yx -= mul(xx, d); yy -= mul(xy, d); yz -= mul(xz, d);
		int my = norm3(yx, yy, yz);
		if(my == 0) { identity3(r); return; }
		yx = divQ(yx, my); yy = divQ(yy, my); yz = divQ(yz, my);

		int zx = mul(xy, yz) - mul(xz, yy);
		int zy = mul(xz, yx) - mul(xx, yz);
		int zz = mul(xx, yy) - mul(xy, yx);

		r[0] = xx; r[3] = xy; r[6] = xz;
		r[1] = yx; r[4] = yy; r[7] = yz;
		r[2] = zx; r[5] = zy; r[8] = zz;
	}

	// ===================== contact generation (SAT) =====================

	private void collideWorld(Collider[] colliders, int count, boolean world) {
		numContacts = 0;
		maxPenetration = 0;
		groundContact = false;
		computeVertices();
		if(!world) return;

		// Broadphase swept against pre-integration position, as before.
		int bcr = cornerRadius + CONTACT_MARGIN;
		int bX = bpx >> 12, bY = bpy >> 12, bZ = bpz >> 12;
		int bandMinX = Math.min(boxMinX, bX - bcr) - CONTACT_MARGIN;
		int bandMinY = Math.min(boxMinY, bY - bcr) - CONTACT_MARGIN;
		int bandMinZ = Math.min(boxMinZ, bZ - bcr) - CONTACT_MARGIN;
		int bandMaxX = Math.max(boxMaxX, bX + bcr) + CONTACT_MARGIN;
		int bandMaxY = Math.max(boxMaxY, bY + bcr) + CONTACT_MARGIN;
		int bandMaxZ = Math.max(boxMaxZ, bZ + bcr) + CONTACT_MARGIN;

		for(int ci = 0; ci < count; ci++) {
			Collider mesh = colliders[ci];
			short[] verts = mesh.verts;
			short[] pols = mesh.pols;
			int s8 = mesh.scale8;
			int ox = mesh.offX, oy = mesh.offY, oz = mesh.offZ;

			int x1 = ((bandMinX - ox) << 8) / s8 - 1;
			int y1 = ((bandMinY - oy) << 8) / s8 - 1;
			int z1 = ((bandMinZ - oz) << 8) / s8 - 1;
			int x2 = ((bandMaxX - ox) << 8) / s8 + 1;
			int y2 = ((bandMaxY - oy) << 8) / s8 + 1;
			int z2 = ((bandMaxZ - oz) << 8) / s8 + 1;

			for(int vpp = 4, pIdx = 0; vpp >= 3; vpp--) {
				int pEnd = vpp == 4 ? mesh.quads * 4 : pols.length;
				for(; pIdx < pEnd; pIdx += vpp) {
					int i1 = pols[pIdx] * 3, i2 = pols[pIdx + 1] * 3, i3 = pols[pIdx + 2] * 3;
					int sax = verts[i1], say = verts[i1 + 1], saz = verts[i1 + 2];
					int sbx = verts[i2], sby = verts[i2 + 1], sbz = verts[i2 + 2];
					int scx = verts[i3], scy = verts[i3 + 1], scz = verts[i3 + 2];

					int mnx = sax, mxx = sax, mny = say, mxy = say, mnz = saz, mxz = saz;
					if(sbx < mnx) mnx = sbx; if(sbx > mxx) mxx = sbx;
					if(sby < mny) mny = sby; if(sby > mxy) mxy = sby;
					if(sbz < mnz) mnz = sbz; if(sbz > mxz) mxz = sbz;
					if(scx < mnx) mnx = scx; if(scx > mxx) mxx = scx;
					if(scy < mny) mny = scy; if(scy > mxy) mxy = scy;
					if(scz < mnz) mnz = scz; if(scz > mxz) mxz = scz;
					int sdx = 0, sdy = 0, sdz = 0;
					if(vpp == 4) {
						int i4 = pols[pIdx + 3] * 3;
						sdx = verts[i4]; sdy = verts[i4 + 1]; sdz = verts[i4 + 2];
						if(sdx < mnx) mnx = sdx; if(sdx > mxx) mxx = sdx;
						if(sdy < mny) mny = sdy; if(sdy > mxy) mxy = sdy;
						if(sdz < mnz) mnz = sdz; if(sdz > mxz) mxz = sdz;
					}
					if(mxx < x1 || mnx > x2 || mxy < y1 || mny > y2 || mxz < z1 || mnz > z2) continue;

					// world space vertex coords (units)
					int ax = (sax * s8 >> 8) + ox, ay = (say * s8 >> 8) + oy, az = (saz * s8 >> 8) + oz;
					int bx = (sbx * s8 >> 8) + ox, by = (sby * s8 >> 8) + oy, bz = (sbz * s8 >> 8) + oz;
					int cx = (scx * s8 >> 8) + ox, cy = (scy * s8 >> 8) + oy, cz = (scz * s8 >> 8) + oz;
					int dx = 0, dy = 0, dz = 0;
					if(vpp == 4) {
						dx = (sdx * s8 >> 8) + ox; dy = (sdy * s8 >> 8) + oy; dz = (sdz * s8 >> 8) + oz;
					}

					// quads are split into two triangles for SAT; adjacent
					// coplanar tris give the same normal at different contact
					// points, which the solver distributes impulses over.
					boxTriangleSAT(ax, ay, az, bx, by, bz, cx, cy, cz);
					if(vpp == 4) {
						boxTriangleSAT(ax, ay, az, cx, cy, cz, dx, dy, dz);
					}
				}
			}
		}
	}

	/**
	 * SAT narrowphase: box vs one mesh triangle. All arguments are world
	 * units (int). Emits a contact if the shapes overlap on every
	 * candidate axis.
	 *
	 * The 13 axes are built in BOX LOCAL SPACE, where the box is an
	 * axis-aligned box centred at the origin. This makes the box face
	 * normals and box edge directions trivially (1,0,0)/(0,1,0)/(0,0,1),
	 * so the only real work is the triangle normal (1 axis) and the 9
	 * cross products of box edge directions with the 3 triangle edges.
	 * The chosen MTV axis is rotated back to world for the contact
	 * normal.
	 *
	 * This resolves all three contact regimes (box vertex vs tri face,
	 * tri vertex vs box face, box edge vs tri edge) uniformly and
	 * exactly. No sampling, no perpendicularity guards, no per-vertex
	 * corner slots, no edge-contact dedup - none of that is needed
	 * because SAT returns the true MTV.
	 */
	private void boxTriangleSAT(int awx, int awy, int awz,
			int bwx, int bwy, int bwz,
			int cwx, int cwy, int cwz) {

		int pwx = px >> 12, pwy = py >> 12, pwz = pz >> 12;
		int dax = awx - pwx, day = awy - pwy, daz = awz - pwz;
		int dbx = bwx - pwx, dby = bwy - pwy, dbz = bwz - pwz;
		int dcx = cwx - pwx, dcy = cwy - pwy, dcz = cwz - pwz;

		// triangle into box local space (units)
		int lax = mul(r[0], dax) + mul(r[3], day) + mul(r[6], daz);
		int lay = mul(r[1], dax) + mul(r[4], day) + mul(r[7], daz);
		int laz = mul(r[2], dax) + mul(r[5], day) + mul(r[8], daz);
		int lbx = mul(r[0], dbx) + mul(r[3], dby) + mul(r[6], dbz);
		int lby = mul(r[1], dbx) + mul(r[4], dby) + mul(r[7], dbz);
		int lbz = mul(r[2], dbx) + mul(r[5], dby) + mul(r[8], dbz);
		int lcx = mul(r[0], dcx) + mul(r[3], dcy) + mul(r[6], dcz);
		int lcy = mul(r[1], dcx) + mul(r[4], dcy) + mul(r[7], dcz);
		int lcz = mul(r[2], dcx) + mul(r[5], dcy) + mul(r[8], dcz);

		// local coords in Q12 for the projections
		int aX = lax << 12, aY = lay << 12, aZ = laz << 12;
		int bX = lbx << 12, bY = lby << 12, bZ = lbz << 12;
		int cX = lcx << 12, cY = lcy << 12, cZ = lcz << 12;

		// triangle edges in units (for the tri normal and edge-edge axes)
		int e0x = lbx - lax, e0y = lby - lay, e0z = lbz - laz;
		int e1x = lcx - lbx, e1y = lcy - lby, e1z = lcz - lbz;
		int e2x = lax - lcx, e2y = lay - lcy, e2z = laz - lcz;

		satBestOverlap = Integer.MAX_VALUE;
		satBestNx = 0; satBestNy = 0; satBestNz = 0;

		// 3 box face normals (unit Q12)
		if(!satTestAxis(F, 0, 0, aX,aY,aZ, bX,bY,bZ, cX,cY,cZ)) return;
		if(!satTestAxis(0, F, 0, aX,aY,aZ, bX,bY,bZ, cX,cY,cZ)) return;
		if(!satTestAxis(0, 0, F, aX,aY,aZ, bX,bY,bZ, cX,cY,cZ)) return;

		// triangle face normal
		long tnx = (long) e0y * e1z - (long) e0z * e1y;
		long tny = (long) e0z * e1x - (long) e0x * e1z;
		long tnz = (long) e0x * e1y - (long) e0y * e1x;
		if(normalizeToQ12(tnx, tny, tnz)) {
			if(!satTestAxis(normX, normY, normZ, aX,aY,aZ, bX,bY,bZ, cX,cY,cZ)) return;
		}

		// 9 edge-edge axes: box edge direction (one of ±X, ±Y, ±Z in local)
		// cross each triangle edge. We use +X/+Y/+Z only; the negative
		// directions give the same axis up to sign and SAT is sign
		// agnostic (sign is chosen from the projections).
		if(!satTestEdgeEdge(1, 0, 0, e0x, e0y, e0z, aX,aY,aZ, bX,bY,bZ, cX,cY,cZ)) return;
		if(!satTestEdgeEdge(0, 1, 0, e0x, e0y, e0z, aX,aY,aZ, bX,bY,bZ, cX,cY,cZ)) return;
		if(!satTestEdgeEdge(0, 0, 1, e0x, e0y, e0z, aX,aY,aZ, bX,bY,bZ, cX,cY,cZ)) return;
		if(!satTestEdgeEdge(1, 0, 0, e1x, e1y, e1z, aX,aY,aZ, bX,bY,bZ, cX,cY,cZ)) return;
		if(!satTestEdgeEdge(0, 1, 0, e1x, e1y, e1z, aX,aY,aZ, bX,bY,bZ, cX,cY,cZ)) return;
		if(!satTestEdgeEdge(0, 0, 1, e1x, e1y, e1z, aX,aY,aZ, bX,bY,bZ, cX,cY,cZ)) return;
		if(!satTestEdgeEdge(1, 0, 0, e2x, e2y, e2z, aX,aY,aZ, bX,bY,bZ, cX,cY,cZ)) return;
		if(!satTestEdgeEdge(0, 1, 0, e2x, e2y, e2z, aX,aY,aZ, bX,bY,bZ, cX,cY,cZ)) return;
		if(!satTestEdgeEdge(0, 0, 1, e2x, e2y, e2z, aX,aY,aZ, bX,bY,bZ, cX,cY,cZ)) return;

		// Overlap on every axis: satBestOverlap is the penetration (Q12),
		// satBestN* is the local-space escape direction.

		// world-space contact normal = R * local normal
		int wnx = mul(r[0], satBestNx) + mul(r[1], satBestNy) + mul(r[2], satBestNz);
		int wny = mul(r[3], satBestNx) + mul(r[4], satBestNy) + mul(r[5], satBestNz);
		int wnz = mul(r[6], satBestNx) + mul(r[7], satBestNy) + mul(r[8], satBestNz);

		// If the winning axis is one of the box's own face normals, the
		// triangle is resting against a box face and we need MULTIPLE
		// contact points spread over that face - not just the single
		// deepest triangle vertex. A single contact lets the cube tip;
		// next frame the opposite corner is deepest; it jitters
		// indefinitely. Clip the triangle against the box face rectangle
		// (Sutherland-Hodgman) and emit every clipped vertex.
		//
		// For the triangle-face axis (box vertex vs triangle face) and
		// for edge-edge axes a single contact point is sufficient - those
		// are genuinely point contacts, not face contacts.
		boolean boxFace =
				((satBestNx == F || satBestNx == -F) && satBestNy == 0 && satBestNz == 0) ||
				((satBestNy == F || satBestNy == -F) && satBestNx == 0 && satBestNz == 0) ||
				((satBestNz == F || satBestNz == -F) && satBestNx == 0 && satBestNy == 0);

		if(boxFace) {
			int axis, sign;
			if(satBestNx != 0) { axis = 0; sign = satBestNx > 0 ? 1 : -1; }
			else if(satBestNy != 0) { axis = 1; sign = satBestNy > 0 ? 1 : -1; }
			else { axis = 2; sign = satBestNz > 0 ? 1 : -1; }
			clipTriangleToBoxFace(aX, aY, aZ, bX, bY, bZ, cX, cY, cZ, axis, sign,
					wnx, wny, wnz);
			return;
		}

		// Single-point path (unchanged): deepest triangle vertex along
		// the escape direction, clamped to the box, rotated to world.
		int dA = mul(aX, satBestNx) + mul(aY, satBestNy) + mul(aZ, satBestNz);
		int dB = mul(bX, satBestNx) + mul(bY, satBestNy) + mul(bZ, satBestNz);
		int dC = mul(cX, satBestNx) + mul(cY, satBestNy) + mul(cZ, satBestNz);
		int dpx, dpy, dpz;
		if(dA >= dB && dA >= dC) { dpx = aX; dpy = aY; dpz = aZ; }
		else if(dB >= dC) { dpx = bX; dpy = bY; dpz = bZ; }
		else { dpx = cX; dpy = cY; dpz = cZ; }

		if(dpx > hx) dpx = hx; else if(dpx < -hx) dpx = -hx;
		if(dpy > hy) dpy = hy; else if(dpy < -hy) dpy = -hy;
		if(dpz > hz) dpz = hz; else if(dpz < -hz) dpz = -hz;

		int wcx = mul(r[0], dpx) + mul(r[1], dpy) + mul(r[2], dpz) + px;
		int wcy = mul(r[3], dpx) + mul(r[4], dpy) + mul(r[5], dpz) + py;
		int wcz = mul(r[6], dpx) + mul(r[7], dpy) + mul(r[8], dpz) + pz;

		addContact(wcx, wcy, wcz, wnx, wny, wnz, satBestOverlap);
	}
	
	/**
	 * Clips the triangle against the box's face perpendicular to `axis`
	 * at the side the box is escaping toward (i.e. the contact face),
	 * then emits one contact per clipped vertex.
	 *
	 * Axis 0/1/2 selects the box's local X/Y/Z face; `sign` is the sign
	 * of the escape direction along that axis (satBestN component). The
	 * contact face's outward normal is -satBestN, so its plane sits at
	 * local coord -sign * halfExtent along the axis.
	 *
	 * The triangle is projected onto the two perpendicular local axes
	 * (U, V), clipped to the box face rectangle [-uHalf, uHalf] x
	 * [-vHalf, vHalf] by Sutherland-Hodgman, then each surviving vertex
	 * is placed back on the contact face plane and rotated to world.
	 */
	private void clipTriangleToBoxFace(
			int aX, int aY, int aZ,
			int bX, int bY, int bZ,
			int cX, int cY, int cZ,
			int axis, int sign,
			int wnx, int wny, int wnz) {

		int uHalf, vHalf, uIdx, vIdx;
		if(axis == 0)      { uIdx = 1; vIdx = 2; uHalf = hy; vHalf = hz; }
		else if(axis == 1) { uIdx = 0; vIdx = 2; uHalf = hx; vHalf = hz; }
		else               { uIdx = 0; vIdx = 1; uHalf = hx; vHalf = hy; }

		// initial polygon = triangle, in (u, v) local coords, Q12
		int[] uu = clipUa, vv = clipVa;
		uu[0] = coordAt(aX, aY, aZ, uIdx); vv[0] = coordAt(aX, aY, aZ, vIdx);
		uu[1] = coordAt(bX, bY, bZ, uIdx); vv[1] = coordAt(bX, bY, bZ, vIdx);
		uu[2] = coordAt(cX, cY, cZ, uIdx); vv[2] = coordAt(cX, cY, cZ, vIdx);
		int n = 3;

		// clip against the four edges of the face rectangle, alternating
		// buffers each time
		n = clipHalfPlane(n, uu, vv, clipUb, clipVb, 0, +1, uHalf);
		if(n == 0) return;
		n = clipHalfPlane(n, clipUb, clipVb, clipUa, clipVa, 0, -1, uHalf);
		if(n == 0) return;
		n = clipHalfPlane(n, clipUa, clipVa, clipUb, clipVb, 1, +1, vHalf);
		if(n == 0) return;
		n = clipHalfPlane(n, clipUb, clipVb, clipUa, clipVa, 1, -1, vHalf);
		if(n == 0) return;
		// result is in clipUa / clipVa

		// Contact face plane sits at -sign * halfExtent along the normal
		// axis. Reason: satBestN points AWAY from the obstacle (the
		// escape direction), and the contact face's outward normal is
		// its opposite, so the face is on the opposite side of the box
		// from satBestN. For a resting cube, satBestN is +Y (escape up),
		// faceHalf = hy, plane sits at -hy (box bottom face) - correct.
		int faceHalf = axis == 0 ? hx : axis == 1 ? hy : hz;
		int faceN = -sign * faceHalf;

		for(int i = 0; i < n; i++) {
			int lu = clipUa[i], lv = clipVa[i];
			int lX, lY, lZ;
			if(axis == 0)      { lX = faceN; lY = lu;    lZ = lv;    }
			else if(axis == 1) { lX = lu;    lY = faceN; lZ = lv;    }
			else               { lX = lu;    lY = lv;    lZ = faceN; }

			int wcx = mul(r[0], lX) + mul(r[1], lY) + mul(r[2], lZ) + px;
			int wcy = mul(r[3], lX) + mul(r[4], lY) + mul(r[5], lZ) + py;
			int wcz = mul(r[6], lX) + mul(r[7], lY) + mul(r[8], lZ) + pz;

			addContact(wcx, wcy, wcz, wnx, wny, wnz, satBestOverlap);
		}
	}

	private static int coordAt(int x, int y, int z, int idx) {
		if(idx == 0) return x;
		if(idx == 1) return y;
		return z;
	}

	/**
	 * Sutherland-Hodgman: clip a closed polygon (uIn, vIn) of size n
	 * against the half-plane sign * coord <= limit, where coordAxis 0
	 * selects U and 1 selects V. Result written to (uOut, vOut); returns
	 * new size. All coords Q12.
	 */
	private static int clipHalfPlane(int n,
			int[] uIn, int[] vIn, int[] uOut, int[] vOut,
			int coordAxis, int sign, int limit) {
		int out = 0;
		for(int i = 0; i < n; i++) {
			int j = (i + 1) % n;
			int ci = coordAxis == 0 ? uIn[i] : vIn[i];
			int cj = coordAxis == 0 ? uIn[j] : vIn[j];
			int si = sign * ci, sj = sign * cj;
			boolean inI = si <= limit;
			boolean inJ = sj <= limit;
			if(inI) {
				uOut[out] = uIn[i]; vOut[out] = vIn[i]; out++;
			}
			if(inI != inJ) {
				// intersection at sign * coord = limit, interpolate
				long dc = (long) sj - si;
				long t = (((long) (limit - si)) << 16) / dc;   // Q16
				uOut[out] = uIn[i] + (int) (((long) (uIn[j] - uIn[i]) * t) >> 16);
				vOut[out] = vIn[i] + (int) (((long) (vIn[j] - vIn[i]) * t) >> 16);
				out++;
			}
		}
		return out;
	}

	/**
	 * Tests one candidate SAT axis. Returns false if the projections of
	 * the box and triangle are disjoint on this axis (which by SAT means
	 * the whole pair is disjoint, so the caller can bail immediately).
	 * Otherwise updates the running best-overlap axis.
	 *
	 * The axis is a Q12 unit vector in box local space. The box interval
	 * on it is [-rBox, rBox] with rBox = hx|ux| + hy|uy| + hz|uz|; the
	 * triangle interval is [tMin, tMax]. The per-axis escape distance is
	 * the smaller of rBox - tMin (push box along -axis) and tMax + rBox
	 * (push box along +axis). Positive means overlapping, negative means
	 * separated on this axis.
	 */
	private boolean satTestAxis(int ux, int uy, int uz,
			int aX, int aY, int aZ,
			int bX, int bY, int bZ,
			int cX, int cY, int cZ) {
		int rBox = mul(hx, abs(ux)) + mul(hy, abs(uy)) + mul(hz, abs(uz));
		int pA = mul(aX, ux) + mul(aY, uy) + mul(aZ, uz);
		int pB = mul(bX, ux) + mul(bY, uy) + mul(bZ, uz);
		int pC = mul(cX, ux) + mul(cY, uy) + mul(cZ, uz);
		int tMin = pA, tMax = pA;
		if(pB < tMin) tMin = pB; if(pB > tMax) tMax = pB;
		if(pC < tMin) tMin = pC; if(pC > tMax) tMax = pC;

		int oPlus = rBox - tMin;    // push box along -axis
		int oMinus = tMax + rBox;   // push box along +axis
		if(oPlus < 0 || oMinus < 0) return false;
		int o = oPlus < oMinus ? oPlus : oMinus;
		if(o < satBestOverlap) {
			satBestOverlap = o;
			if(oPlus < oMinus) {
				satBestNx = -ux; satBestNy = -uy; satBestNz = -uz;
			} else {
				satBestNx = ux; satBestNy = uy; satBestNz = uz;
			}
		}
		return true;
	}

	private boolean satTestEdgeEdge(int bx, int by, int bz,
			int ex, int ey, int ez,
			int aX, int aY, int aZ,
			int bX, int bY, int bZ,
			int cX, int cY, int cZ) {
		// cross product in units^2 (long to avoid overflow)
		long nx = (long) by * ez - (long) bz * ey;
		long ny = (long) bz * ex - (long) bx * ez;
		long nz = (long) bx * ey - (long) by * ex;
		if(!normalizeToQ12(nx, ny, nz)) return true;  // degenerate, skip axis
		return satTestAxis(normX, normY, normZ, aX, aY, aZ, bX, bY, bZ, cX, cY, cZ);
	}

	// scratch for normalizeToQ12
	private static int normX, normY, normZ;

	/** Normalizes a (long) vector in units^2 to a Q12 unit vector. */
	private static boolean normalizeToQ12(long nx, long ny, long nz) {
		long len2 = nx * nx + ny * ny + nz * nz;
		if(len2 == 0) return false;
		long len = isqrt(len2);
		if(len == 0) return false;
		normX = (int) ((nx * F) / len);
		normY = (int) ((ny * F) / len);
		normZ = (int) ((nz * F) / len);
		return true;
	}

	private void addContact(int x, int y, int z, int nx, int ny, int nz, int pen) {
		// Deduplicate: adjacent triangles / the two halves of a split quad
		// produce overlapping clipped polygons at their shared edge, which
		// would double the effective support there and inject energy.
		// Skip a contact if a nearly-identical one (same position within
		// a small tolerance, same normal) is already in the list.
		for(int i = 0; i < numContacts; i++) {
			long dx = (long) cpx[i] - x;
			long dy = (long) cpy[i] - y;
			long dz = (long) cpz[i] - z;
			// 2 units tolerance, Q12: (2 << 12)^2 == 67108864
			if(dx * dx + dy * dy + dz * dz < 67108864L) {
				int d = mul(cnx[i], nx) + mul(cny[i], ny) + mul(cnz[i], nz);
				if(d > F - 64) return;   // ~0.985 dot, same direction
			}
		}
		if(numContacts >= MAX_CONTACTS) return;
		cpx[numContacts] = x; cpy[numContacts] = y; cpz[numContacts] = z;
		cnx[numContacts] = nx; cny[numContacts] = ny; cnz[numContacts] = nz;
		cpen[numContacts] = pen;
		if(pen > maxPenetration) maxPenetration = pen;
		if(ny > (F * 7 / 10)) groundContact = true;
		++numContacts;
	}

	// ===================== impulses =====================

	private final int[] accN = new int[MAX_CONTACTS];
	private final int[] accT = new int[MAX_CONTACTS];
	private final int[] vbias = new int[MAX_CONTACTS];

		private void applyImpulses() {
		for(int i = 0; i < numContacts; i++) {
			accN[i] = 0; accT[i] = 0; vbias[i] = 0;
		}

		// Warm start: for each current contact, look for a matching
		// contact from the previous frame (same position within a few
		// units, same normal direction). If found, seed accN/accT from
		// that contact's final impulse, and apply those impulses
		// immediately. Then the iteration loop below only has to add the
		// small delta needed to account for the way the body moved since
		// last frame - it does not have to rediscover the resting
		// support impulse from scratch, which is what makes a resting
		// body jitter in a from-zero solver.
		for(int i = 0; i < numContacts; i++) {
			for(int j = 0; j < prevNumContacts; j++) {
				long dx = (long) prevCpx[j] - cpx[i];
				long dy = (long) prevCpy[j] - cpy[i];
				long dz = (long) prevCpz[j] - cpz[i];
				// 4 unit position tolerance in Q12: (4 << 12)^2
				if(dx * dx + dy * dy + dz * dz > 268435456L) continue;
				int d = mul(prevCnx[j], cnx[i]) + mul(prevCny[j], cny[i])
						+ mul(prevCnz[j], cnz[i]);
				if(d < F - 64) continue;   // require same normal
				accN[i] = prevAccN[j];
				accT[i] = prevAccT[j];
				break;
			}
		}

		// Apply the warm-start impulses (normal only; the friction
		// tangent direction is derived from the current relative
		// velocity during the iteration loop, so warming it up is not
		// meaningful here and it starts from 0 like before).
		for(int i = 0; i < numContacts; i++) {
			if(accN[i] == 0) continue;
			int rx = cpx[i] - px, ry = cpy[i] - py, rz = cpz[i] - pz;
			int dN = accN[i];
			int imp = mul(dN, invMass);
			vx += mul(cnx[i], imp);
			vy += mul(cny[i], imp);
			vz += mul(cnz[i], imp);
			lx += mul(ry, mul(cnz[i], dN)) - mul(rz, mul(cny[i], dN));
			ly += mul(rz, mul(cnx[i], dN)) - mul(rx, mul(cnz[i], dN));
			lz += mul(rx, mul(cny[i], dN)) - mul(ry, mul(cnx[i], dN));
			wx = eval24X(invIWorld, lx, ly, lz);
			wy = eval24Y(invIWorld, lx, ly, lz);
			wz = eval24Z(invIWorld, lx, ly, lz);
		}

		// Restitution targets, fixed once from the approach velocities
		// (this now runs AFTER warm starting, so vbias reflects the
		// warm-started contact velocity; that is the correct approach
		// velocity for the restitution decision).
		for(int i = 0; i < numContacts; i++) {
			int rx = cpx[i] - px, ry = cpy[i] - py, rz = cpz[i] - pz;
			int crx = mul(wy, rz) - mul(wz, ry);
			int cry = mul(wz, rx) - mul(wx, rz);
			int crz = mul(wx, ry) - mul(wy, rx);
			int vn = mul(vx + crx, cnx[i]) + mul(vy + cry, cny[i])
					+ mul(vz + crz, cnz[i]);
			vbias[i] = -vn > RESTITUTION_SPEED ? -mul(RESTITUTION, vn) : 0;
		}

		for(int iter = 0; iter < IMPULSE_ITERATIONS; iter++) {
			for(int i = 0; i < numContacts; i++) {
				int rx = cpx[i] - px, ry = cpy[i] - py, rz = cpz[i] - pz;
				int nx = cnx[i], ny = cny[i], nz = cnz[i];

				int crx = mul(wy, rz) - mul(wz, ry);
				int cry = mul(wz, rx) - mul(wx, rz);
				int crz = mul(wx, ry) - mul(wy, rx);
				int rvx = vx + crx, rvy = vy + cry, rvz = vz + crz;
				int vn = mul(rvx, nx) + mul(rvy, ny) + mul(rvz, nz);

				int rnx = mul(ry, nz) - mul(rz, ny);
				int rny = mul(rz, nx) - mul(rx, nz);
				int rnz = mul(rx, ny) - mul(ry, nx);
				int irx = eval24X(invIWorld, rnx, rny, rnz);
				int iry = eval24Y(invIWorld, rnx, rny, rnz);
				int irz = eval24Z(invIWorld, rnx, rny, rnz);
				int krx = mul(iry, rz) - mul(irz, ry);
				int kry = mul(irz, rx) - mul(irx, rz);
				int krz = mul(irx, ry) - mul(iry, rx);
				int kn = invMass + mul(krx, nx) + mul(kry, ny) + mul(krz, nz);

				if(kn > 0) {
					int dN = divQ(vbias[i] - vn, kn);
					int newAcc = accN[i] + dN;
					if(newAcc < 0) newAcc = 0;
					dN = newAcc - accN[i];
					accN[i] = newAcc;
					if(dN != 0) {
						int imp = mul(dN, invMass);
						vx += mul(nx, imp);
						vy += mul(ny, imp);
						vz += mul(nz, imp);
						lx += mul(ry, mul(nz, dN)) - mul(rz, mul(ny, dN));
						ly += mul(rz, mul(nx, dN)) - mul(rx, mul(nz, dN));
						lz += mul(rx, mul(ny, dN)) - mul(ry, mul(nx, dN));
						wx = eval24X(invIWorld, lx, ly, lz);
						wy = eval24Y(invIWorld, lx, ly, lz);
						wz = eval24Z(invIWorld, lx, ly, lz);
					}
				}

				if(accN[i] <= 0) continue;

				crx = mul(wy, rz) - mul(wz, ry);
				cry = mul(wz, rx) - mul(wx, rz);
				crz = mul(wx, ry) - mul(wy, rx);
				rvx = vx + crx; rvy = vy + cry; rvz = vz + crz;
				int vnn = mul(rvx, nx) + mul(rvy, ny) + mul(rvz, nz);
				int tx = rvx - mul(nx, vnn);
				int ty = rvy - mul(ny, vnn);
				int tz = rvz - mul(nz, vnn);
				int tl = norm3(tx, ty, tz);
				if(tl >= 1) {
					tx = divQ(tx, tl); ty = divQ(ty, tl); tz = divQ(tz, tl);

					int rtx = mul(ry, tz) - mul(rz, ty);
					int rty = mul(rz, tx) - mul(rx, tz);
					int rtz = mul(rx, ty) - mul(ry, tx);
					int itx = eval24X(invIWorld, rtx, rty, rtz);
					int ity = eval24Y(invIWorld, rtx, rty, rtz);
					int itz = eval24Z(invIWorld, rtx, rty, rtz);
					int ktx = mul(ity, rz) - mul(itz, ry);
					int kty = mul(itz, rx) - mul(itx, rz);
					int ktz = mul(itx, ry) - mul(ity, rx);
					int kt = invMass + mul(ktx, tx) + mul(kty, ty) + mul(ktz, tz);

					int vt = mul(rvx, tx) + mul(rvy, ty) + mul(rvz, tz);
					if(kt > 0) {
						int dT = -divQ(vt, kt);
						int maxFric = abs(mul(FRICTION, accN[i]));
						int newAcc = accT[i] + dT;
						if(newAcc > maxFric) newAcc = maxFric;
						else if(newAcc < -maxFric) newAcc = -maxFric;
						dT = newAcc - accT[i];
						accT[i] = newAcc;
						if(dT != 0) {
							int imp = mul(dT, invMass);
							vx += mul(tx, imp);
							vy += mul(ty, imp);
							vz += mul(tz, imp);
							lx += mul(ry, mul(tz, dT)) - mul(rz, mul(ty, dT));
							ly += mul(rz, mul(tx, dT)) - mul(rx, mul(tz, dT));
							lz += mul(rx, mul(ty, dT)) - mul(ry, mul(tx, dT));
							wx = eval24X(invIWorld, lx, ly, lz);
							wy = eval24Y(invIWorld, lx, ly, lz);
							wz = eval24Z(invIWorld, lx, ly, lz);
						}
					}
				}
			}
		}

		// Save this frame's contacts and impulses for next frame's warm
		// start. Done before correctPositions because correctPositions
		// does not change the impulses (it is position-only).
		prevNumContacts = numContacts;
		for(int i = 0; i < numContacts; i++) {
			prevCpx[i] = cpx[i]; prevCpy[i] = cpy[i]; prevCpz[i] = cpz[i];
			prevCnx[i] = cnx[i]; prevCny[i] = cny[i]; prevCnz[i] = cnz[i];
			prevAccN[i] = accN[i]; prevAccT[i] = accT[i];
		}

		correctPositions();
	}

	private void correctPositions() {
		final int beta = (int) ((long) F / POSITION_ITERATIONS);
		for(int iter = 0; iter < POSITION_ITERATIONS; iter++) {
			for(int i = 0; i < numContacts; i++) {
				int pen = cpen[i];
				if(pen <= POSITION_SLOP << 12) continue;
				int rx = cpx[i] - px, ry = cpy[i] - py, rz = cpz[i] - pz;
				int nx = cnx[i], ny = cny[i], nz = cnz[i];

				int rnx = mul(ry, nz) - mul(rz, ny);
				int rny = mul(rz, nx) - mul(rx, nz);
				int rnz = mul(rx, ny) - mul(ry, nx);
				int irx = eval24X(invIWorld, rnx, rny, rnz);
				int iry = eval24Y(invIWorld, rnx, rny, rnz);
				int irz = eval24Z(invIWorld, rnx, rny, rnz);
				int krx = mul(iry, rz) - mul(irz, ry);
				int kry = mul(irz, rx) - mul(irx, rz);
				int krz = mul(irx, ry) - mul(iry, rx);
				int k = invMass + mul(krx, nx) + mul(kry, ny) + mul(krz, nz);
				if(k <= 0) continue;

				int target = mul(pen - (POSITION_SLOP << 12), beta);
				int dp = divQ(target, k);
				px += mul(nx, mul(dp, invMass));
				py += mul(ny, mul(dp, invMass));
				pz += mul(nz, mul(dp, invMass));

				int qx = eval24X(invIWorld,
						mul(ry, mul(nz, dp)) - mul(rz, mul(ny, dp)),
						mul(rz, mul(nx, dp)) - mul(rx, mul(nz, dp)),
						mul(rx, mul(ny, dp)) - mul(ry, mul(nx, dp)));
				int qy = eval24Y(invIWorld,
						mul(ry, mul(nz, dp)) - mul(rz, mul(ny, dp)),
						mul(rz, mul(nx, dp)) - mul(rx, mul(nz, dp)),
						mul(rx, mul(ny, dp)) - mul(ry, mul(nx, dp)));
				int qz = eval24Z(invIWorld,
						mul(ry, mul(nz, dp)) - mul(rz, mul(ny, dp)),
						mul(rz, mul(nx, dp)) - mul(rx, mul(nz, dp)),
						mul(rx, mul(ny, dp)) - mul(ry, mul(nx, dp)));
				rotateMatrix(qx, qy, qz);
				recomputeWorldInertia();
			}
		}
		fixMatrix();
		recomputeWorldInertia();
	}

	private void rotateMatrix(int qx, int qy, int qz) {
		for(int row = 0; row < 3; row++) {
			int s0, s1, s2;
			if(row == 0) { s0 = 0; s1 = -qz; s2 = qy; }
			else if(row == 1) { s0 = qz; s1 = 0; s2 = -qx; }
			else { s0 = -qy; s1 = qx; s2 = 0; }
			for(int col = 0; col < 3; col++) {
				int v = mul(s0, r[col]) + mul(s1, r[3 + col]) + mul(s2, r[6 + col]);
				r[row * 3 + col] += v;
			}
		}
	}

	private void clampVelocity() {
		if(vx > MAX_LINEAR) vx = MAX_LINEAR; else if(vx < -MAX_LINEAR) vx = -MAX_LINEAR;
		if(vy > MAX_LINEAR) vy = MAX_LINEAR; else if(vy < -MAX_LINEAR) vy = -MAX_LINEAR;
		if(vz > MAX_LINEAR) vz = MAX_LINEAR; else if(vz < -MAX_LINEAR) vz = -MAX_LINEAR;
		if(wx > MAX_ANGULAR) wx = MAX_ANGULAR; else if(wx < -MAX_ANGULAR) wx = -MAX_ANGULAR;
		if(wy > MAX_ANGULAR) wy = MAX_ANGULAR; else if(wy < -MAX_ANGULAR) wy = -MAX_ANGULAR;
		if(wz > MAX_ANGULAR) wz = MAX_ANGULAR; else if(wz < -MAX_ANGULAR) wz = -MAX_ANGULAR;
		recomputeMomentum();
	}

	// ===================== AABB =====================

	private void computeVertices() {
		int c0x = r[0], c0y = r[3], c0z = r[6];
		int c1x = r[1], c1y = r[4], c1z = r[7];
		int c2x = r[2], c2y = r[5], c2z = r[8];
		int ex = hx, ey = hy, ez = hz;

		int mnx = Integer.MAX_VALUE, mny = Integer.MAX_VALUE, mnz = Integer.MAX_VALUE;
		int mxx = Integer.MIN_VALUE, mxy = Integer.MIN_VALUE, mxz = Integer.MIN_VALUE;

		for(int k = 0; k < VERTICES; k++) {
			int sx = (k == 1 || k == 2 || k == 6 || k == 7) ? 1 : -1;
			int sy = (k == 4 || k == 5 || k == 6 || k == 7) ? 1 : -1;
			int sz = (k == 2 || k == 3 || k == 4 || k == 7) ? 1 : -1;
			int ox3 = sx * mul(c0x, ex) + sy * mul(c1x, ey) + sz * mul(c2x, ez);
			int oy3 = sx * mul(c0y, ex) + sy * mul(c1y, ey) + sz * mul(c2y, ez);
			int oz3 = sx * mul(c0z, ex) + sy * mul(c1z, ey) + sz * mul(c2z, ez);
			int wx = (px + ox3) >> 12, wy = (py + oy3) >> 12, wz = (pz + oz3) >> 12;
			if(wx < mnx) mnx = wx; if(wx > mxx) mxx = wx;
			if(wy < mny) mny = wy; if(wy > mxy) mxy = wy;
			if(wz < mnz) mnz = wz; if(wz > mxz) mxz = wz;
		}
		boxMinX = mnx; boxMinY = mny; boxMinZ = mnz;
		boxMaxX = mxx; boxMaxY = mxy; boxMaxZ = mxz;

		int rxr = abs(c0x) * (ex >> 12) + abs(c1x) * (ey >> 12) + abs(c2x) * (ez >> 12);
		int ryr = abs(c0y) * (ex >> 12) + abs(c1y) * (ey >> 12) + abs(c2y) * (ez >> 12);
		int rzr = abs(c0z) * (ex >> 12) + abs(c1z) * (ey >> 12) + abs(c2z) * (ez >> 12);
		int cr = (rxr >> 12);
		if((ryr >> 12) > cr) cr = ryr >> 12;
		if((rzr >> 12) > cr) cr = rzr >> 12;
		cornerRadius = cr;
	}

	// ===================== rollback =====================

	private void backup() {
		bpx = px; bpy = py; bpz = pz;
		bvx = vx; bvy = vy; bvz = vz;
		blx = lx; bly = ly; blz = lz;
		bwx = wx; bwy = wy; bwz = wz;
		System.arraycopy(r, 0, br, 0, 9);
		System.arraycopy(invIWorld, 0, biw, 0, 9);
	}

	private void restore() {
		px = bpx; py = bpy; pz = bpz;
		vx = bvx; vy = bvy; vz = bvz;
		lx = blx; ly = bly; lz = blz;
		wx = bwx; wy = bwy; wz = bwz;
		System.arraycopy(br, 0, r, 0, 9);
		System.arraycopy(biw, 0, invIWorld, 0, 9);
	}

	// ===================== static math =====================

	public static int mul(int a, int b) {
		return (int) ((long) a * b >> 12);
	}

	public static int divQ(int a, int b) {
		return (int) (((long) a << 12) / b);
	}

	private static int evalX(int[] m, int x, int y, int z) {
		return mul(m[0], x) + mul(m[1], y) + mul(m[2], z);
	}
	private static int evalY(int[] m, int x, int y, int z) {
		return mul(m[3], x) + mul(m[4], y) + mul(m[5], z);
	}
	private static int evalZ(int[] m, int x, int y, int z) {
		return mul(m[6], x) + mul(m[7], y) + mul(m[8], z);
	}

	private static void identity3(int[] m) {
		m[0] = F; m[1] = 0; m[2] = 0;
		m[3] = 0; m[4] = F; m[5] = 0;
		m[6] = 0; m[7] = 0; m[8] = F;
	}

	private final int[] tmpMatrix = new int[9];

	private static int norm3(int x, int y, int z) {
		return isqrt((long) x * x + (long) y * y + (long) z * z);
	}

	public static int isqrt(long x) {
		if(x <= 0) return 0;
		long rem = 0, root = 0;
		for(int i = 31; i >= 0; i--) {
			rem = (rem << 2) | ((x >>> (i * 2)) & 3);
			long t = (root << 2) | 1;
			if(rem >= t) { rem -= t; root = (root << 1) | 1; }
			else root <<= 1;
		}
		return (int) root;
	}

	private static int abs(int v) { return v < 0 ? -v : v; }
}