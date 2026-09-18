package com;

// Oriented rigid box with impulse based contact physics: a J2ME style port of
// the OBB solver from portalDS (arm7/source/OBB.c + AAR.c) onto this engine's
// triangle soup. Q12 throughout (4096 == 1.0), no floats and no per frame
// allocation; semi-implicit Euler plus a Gram-Schmidt re-orthonormalization;
// contacts at the 8 box vertices; sequential impulses with restitution and
// Coulomb friction; adaptive substeps on deep penetration; sleep once resting on
// an upward surface. BodyPair does body against body, SolverMath the arithmetic.
//
// Coordinates are plain engine units, one step per rendered frame. Mesh normals
// point INTO the solid, so an outward contact normal is the negated mesh normal.
// Matrices are 3x3 row-major Q12, R maps local to world. State is package
// private so BodyPair needs no accessor per component per contact.
public final class RigidBody extends SolverMath {

	// Vertex contacts (up to VERTICES * CORNER_SLOTS == 24) and edge contacts (up
	// to EDGE_CONTACT_SLOTS == 8) are two capped budgets feeding one list, so
	// MAX_CONTACTS covers both and neither can starve the other out.
	private static final int MAX_CONTACTS = 32;
	private static final int VERTICES = 8;
	// Max simultaneous, direction-distinct contacts kept for one box vertex
	// (a vertex wedged into a corner can touch more than one wall at once).
	private static final int CORNER_SLOTS = 3;
	// Max direction-distinct mesh-edge-vs-box-face contacts (addEdgeFaceContact),
	// capped and deduplicated like CORNER_SLOTS: polygons have seams.
	private static final int EDGE_CONTACT_SLOTS = 8;
	// Same-normal edge-face candidates closer than this (squared, Q12) are
	// the same contact (wall seam duplicates) and get merged.
	private static final long EDGE_MERGE_DIST2 = (long) (8 << 12) * (8 << 12);
	// Two candidate normals for one vertex are the same surface (merge, keep the
	// deeper) once their dot reaches this. Q12, ~3072 == cos(41 deg): a real
	// corner is dot ~0, two coplanar polygons dot ~F.
	private static final int DUPLICATE_NORMAL_DOT = F * 3 / 4;
	// A vertex-face penetration is only trusted while that face is nearly the
	// closest feature, or a far face claims it and spins the box. Units.
	private static final int CLOSEST_FEATURE_TOL = 16;

	// Contacts are generated within this band around a surface (units).
	static final int CONTACT_MARGIN = 64;
	// A vertex within this distance on the free side still touches a face.
	static final int SURFACE_TOUCH = 6;
	// Convex polygon edges only catch vertices this close (tighter than faces).
	private static final int EDGE_MARGIN = 24;
	// Deeper penetration than this rolls the step back and halves dt.
	private static final int PENETRATION_THRESHOLD = 64;
	// Smallest substep as a fraction of a frame (1/256).
	private static final int MIN_DT = 16;
	// Gauss-Seidel sweeps over the contact list per frame.
	private static final int IMPULSE_ITERATIONS = 8;
	// Position projection sweeps; every contact is de-penetrated, not just
	// the deepest one, so a cube wedged in a corner cannot be repeatedly
	// snapped and gain energy.
	private static final int POSITION_ITERATIONS = 4;
	// Contacts within this many units are treated as resting (no projection).
	static final int POSITION_SLOP = 3;
	// Hard safety clamps: pathological single-vertex corner contacts must
	// never launch or spin the cube beyond gameplay-scale velocities.
	private static final int MAX_LINEAR = 2048 << 12;
	private static final int MAX_ANGULAR = 2 * F;
	// Impacts slower than this (Q12 units/frame) are treated as inelastic,
	// otherwise micro rocking on a resting contact never damps out.
	static final int RESTITUTION_SPEED = 30 << 12;

	// Gravity, units per frame (matches Character: speed.y -= 20).
	private static final int GRAVITY = 20 << 12;
	// Linear velocity damping divisor (v -= v / 25 per frame).
	private static final int LINEAR_DRAG = 25;
	// Angular velocity damping divisor.
	private static final int ANGULAR_DRAG = 20;

	// Contact material against world geometry (portalDS plane values).
	private static final int RESTITUTION = 819;   // 0.2
	private static final int FRICTION = 4096;    // 1.0

	// Sleep thresholds are energy values (Q12 of units/frame, squared).
	private static final int SLEEP_LOW = 120000;
	private static final int SLEEP_HIGH = 400000;
	private static final int SLEEP_TIME = 16;

	// ---- body state (all Q12 unless noted) ----
	// Package-private: BodyPair reads and writes this directly, an accessor per
	// component per contact being a call in the hottest loop there is.
	int hx, hy, hz;          // half extents
	int skipGuard;           // travel beyond which no surface can be straddled
	int mass, invMass;

	// Mass follows volume, and a box of this half extent weighs one, so the
	// shipped cube keeps the mass it always had.
	private static final int UNIT_HALF = 500;
	private static final long UNIT_VOLUME = (long) UNIT_HALF * UNIT_HALF * UNIT_HALF;
	private long il0, il4, il8;      // local inertia (inverse of invILocal)
	int px, py, pz;          // center position
	int vx, vy, vz;          // linear velocity, units/frame
	long lx, ly, lz;         // angular momentum
	int wx, wy, wz;          // angular velocity
	final int[] r = new int[9];   // orientation, row-major
	private final int[] invILocal = new int[9];
	final int[] invIWorld = new int[9];

	// ---- contacts ----
	private final int[] cpx = new int[MAX_CONTACTS];
	private final int[] cpy = new int[MAX_CONTACTS];
	private final int[] cpz = new int[MAX_CONTACTS];
	final int[] cnx = new int[MAX_CONTACTS];
	final int[] cny = new int[MAX_CONTACTS];
	final int[] cnz = new int[MAX_CONTACTS];
	private final int[] cpen = new int[MAX_CONTACTS];
	int numContacts;

	// Recent world surface normals, each with the center it was seen at, kept
	// until the body has moved a contact margin away. A vertex only reaches a
	// face within SURFACE_TOUCH, so a cube pressed against a wall rides up out
	// of that reach and reports floor only for frames at a time - which the
	// pair pass, blind to the world, reads as free space to shove into.
	static final int MEM_SLOTS = 4;
	int memCount;
	final int[] memNX = new int[MEM_SLOTS];
	final int[] memNY = new int[MEM_SLOTS];
	final int[] memNZ = new int[MEM_SLOTS];
	final int[] memX = new int[MEM_SLOTS];
	final int[] memY = new int[MEM_SLOTS];
	final int[] memZ = new int[MEM_SLOTS];

	// Deepest overlap this kinematic body was pressed into another body this
	// pair pass, and the direction it was pressing in, cleared at the start of
	// every pass. A carried cube displaces whatever it touches and is out of it
	// again a frame or two later; still pressed in past Cube.JAM_PEN, the thing
	// in front is not giving way and the hand stops advancing - but only while
	// it advances, since backing out is what frees the two.
	int pressPen, pressNX, pressNY, pressNZ;
	private int maxPenetration;
	private boolean groundContact;

	// ---- closest feature(s) per box vertex while scanning a mesh ----
	// CORNER_SLOTS direction-distinct contacts per vertex, not just the deepest: a
	// vertex wedged in a corner is close to several walls, and one contact there
	// ping-pongs between them and launches the box (addCandidate).
	private final int[] bestGap = new int[VERTICES * CORNER_SLOTS];
	private final int[] bestNX = new int[VERTICES * CORNER_SLOTS];
	private final int[] bestNY = new int[VERTICES * CORNER_SLOTS];
	private final int[] bestNZ = new int[VERTICES * CORNER_SLOTS];
	private final int[] bestPen = new int[VERTICES * CORNER_SLOTS];
	private final int[] bestCount = new int[VERTICES];
	// Closest face-plane distance per box vertex (absolute, units): the
	// distance to the nearest face whose footprint contains the vertex.
	// Used to reject ghost contacts, see CLOSEST_FEATURE_TOL.
	private final int[] minAbsGap = new int[VERTICES];

	// ---- direction-distinct mesh-edge-vs-box-face candidates (addEdgeCandidate) ----
	// Global for the box, not per-vertex, so they keep their own world point.
	private final int[] edgeGap = new int[EDGE_CONTACT_SLOTS];
	private final int[] edgeNX = new int[EDGE_CONTACT_SLOTS];
	private final int[] edgeNY = new int[EDGE_CONTACT_SLOTS];
	private final int[] edgeNZ = new int[EDGE_CONTACT_SLOTS];
	private final int[] edgePen = new int[EDGE_CONTACT_SLOTS];
	private final int[] edgeCX = new int[EDGE_CONTACT_SLOTS];
	private final int[] edgeCY = new int[EDGE_CONTACT_SLOTS];
	private final int[] edgeCZ = new int[EDGE_CONTACT_SLOTS];
	private int edgeCount;

	// ---- world vertices (Q12 and plain units) and world AABB ----
	private final int[] vq = new int[VERTICES * 3];
	private final int[] vu = new int[VERTICES * 3];
	int boxMinX, boxMinY, boxMinZ, boxMaxX, boxMaxY, boxMaxZ;
	int cornerRadius;

	// ---- backup for substep rollback ----
	private int bpx, bpy, bpz, bvx, bvy, bvz, bwx, bwy, bwz;
	private long blx, bly, blz;
	private final int[] br = new int[9];
	private final int[] biw = new int[9];

	boolean sleeping;
	private int sleepCounter;
	private int stepDt = F;
	private int energy;

	// ---- external forces (applyForceAt) ----
	// Accumulated for the next step() and consumed there. Torque is r x force.
	private int extFX, extFY, extFZ;
	private int extMX, extMY, extMZ;

	// ---- body vs body state (see collideBodies) ----
	// A carried cube is placed kinematically: it joins pair contacts as an
	// immovable obstacle that still lends its hand velocity.
	boolean kinematic;
	// A walker's push box: allowed to drag what it presses along the surface
	// that holds it (BodyPair.velocitySweep). A carried cube is not, or the
	// obstacle yields sideways and the hand loses the jam it stops on.
	boolean drags;
	// Hand velocity of a kinematic body (Q12 units/frame): its own velocity
	// stays zero because the hand target is re-derived every frame.
	int kvx, kvy, kvz;
	// Held up by another body instead of by the world; counts as ground for
	// the sleep bookkeeping, so a stack of cubes can come to rest.
	boolean bodySupport;
	RigidBody supportBody;
	// Support normal of the contact that holds this body up (Q12, pointing
	// out of the support into the body): the direction a body resting on
	// another body cannot be pushed into.
	int supportNX, supportNY, supportNZ;
	RigidBody prevSupport;
	boolean pairTouched;

	// Number of integration substeps used by the last frame (for tests).
	public int lastSubsteps = 1;

	// Triangle/quad mesh as the engine stores it (see MeshData and
	// SphereCast): short indexed vertices/polygons, Q12 polygon normals and
	// an fp8 scale with integer offsets applied to every vertex.
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
		this(halfExtentUnits, halfExtentUnits, halfExtentUnits);
	}

	public RigidBody(int halfX, int halfY, int halfZ) {
		this.hx = halfX << 12;
		this.hy = halfY << 12;
		this.hz = halfZ << 12;
		// Stepping from one side of a surface to the other takes the body's
		// whole width along that axis. The circumradius is the width that holds
		// however the body is turned, since a corner reaches further than a face.
		this.skipGuard = isqrt((long) hx * hx + (long) hy * hy + (long) hz * hz) << 1;
		reset(0, 0, 0);
	}

	// Places the body at a center position given in engine units.
	public void reset(int centerX, int centerY, int centerZ) {
		// Plain units: the Q12 volume does not fit a long.
		long volume = (long) (hx >> 12) * (hy >> 12) * (hz >> 12);
		this.mass = (int) (((long) F * volume) / UNIT_VOLUME);
		if(mass <= 0) mass = 1;   // a sliver still has to be movable
		this.invMass = (int) (((long) F * F) / mass);
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
		clearForces();
		this.kinematic = false;
		this.kvx = this.kvy = this.kvz = 0;
		this.bodySupport = false;
		this.supportBody = null;
		this.supportNX = this.supportNY = this.supportNZ = 0;
		this.prevSupport = null;
		this.pairTouched = false;
		computeVertices();
	}

	// Marks the body as carried: other bodies collide with it and are pushed
	// aside, it never reacts to them. The hand velocity is kept separately (see
	// setKinematicPose) so a swipe still knocks other cubes away.
	public void setKinematic(boolean carried) {
		this.kinematic = carried;
		if(!carried) this.kvx = this.kvy = this.kvz = 0;
		wake();
	}

	public boolean isKinematic() {
		return kinematic;
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

	// ---- accessors (engine units) ----

	public int getCenterX() { return px >> 12; }
	public int getCenterY() { return py >> 12; }
	public int getCenterZ() { return pz >> 12; }

	public int getVelocityX() { return vx >> 12; }
	public int getVelocityY() { return vy >> 12; }
	public int getVelocityZ() { return vz >> 12; }

	public int getHalfX() { return hx >> 12; }
	public int getHalfY() { return hy >> 12; }
	public int getHalfZ() { return hz >> 12; }

	// Half extent along box axis i, Q12.
	int halfExtent(int i) { return i == 0 ? hx : (i == 1 ? hy : hz); }

	// True when the world geometry or the body this one rests on holds it
	// against a move along (dx, dy, dz). Surfaces it has left still count until
	// it has moved a contact margin off them.
	boolean heldAgainst(int dx, int dy, int dz) {
		for(int i = 0; i < numContacts; i++) {
			if(opposes(cnx[i], cny[i], cnz[i], dx, dy, dz)) return true;
		}
		int m = CONTACT_MARGIN << 12;
		for(int i = 0; i < memCount; i++) {
			if(abs(px - memX[i]) > m || abs(py - memY[i]) > m
					|| abs(pz - memZ[i]) > m) continue;
			if(opposes(memNX[i], memNY[i], memNZ[i], dx, dy, dz)) return true;
		}
		return bodySupport && opposes(supportNX, supportNY, supportNZ, dx, dy, dz);
	}

	// Broadly into the surface: the same 3/4 of a cosine two normals are
	// duplicates at. Wedged in a corner, a cube collects diagonal contacts with
	// a component along every axis, and counting those as holds freezes a push
	// that only glances off the wall. Squared: the normal is not always unit.
	private static boolean opposes(int nx, int ny, int nz, int dx, int dy, int dz) {
		int dot = mul(nx, dx) + mul(ny, dy) + mul(nz, dz);
		if(dot >= 0) return false;
		long n2 = (long) nx * nx + (long) ny * ny + (long) nz * nz;
		long d2 = (long) dx * dx + (long) dy * dy + (long) dz * dz;
		return 16 * (long) dot * dot * ((long) F * F) > 9 * n2 * d2;
	}

	// Orientation entry, row-major (0..8), Q12.
	public int getOrientation(int i) { return r[i]; }

	public void setVelocity(int unitsX, int unitsY, int unitsZ) {
		this.vx = unitsX << 12;
		this.vy = unitsY << 12;
		this.vz = unitsZ << 12;
		wake();
	}

	public int getAngularZ() { return wz; }

	// Sets angular velocity (Q12 radians/frame) and derives momentum from it.
	public void setAngularVelocity(int ax, int ay, int az) {
		this.wx = ax; this.wy = ay; this.wz = az;
		recomputeMomentum();
		wake();
	}

	// The one way to push a body from the game: a force of magnitude (Q12,
	// units/frame^2) along a direction, at a world point. Gravity is 20 << 12.
	// It lasts one step(): held across frames it is wind or a driven wheel, once
	// it is a blast, where a frame of force lands as an impulse of the same
	// number. Off centre it also spins the body. The direction is normalised
	// here, so Q14 Vector3D components, a raw delta or a unit Q12 vector all
	// work alike. A carried body ignores it: the hand places those.
	public void applyForceAt(int worldX, int worldY, int worldZ,
			int dirX, int dirY, int dirZ, int magnitude) {
		if(kinematic) return;
		int len = norm3(dirX, dirY, dirZ);
		if(len < 1 || magnitude == 0) return;
		// step() throws away the velocity of a sleeping body, so it has to wake
		// first or the push would never be seen.
		wake();
		int fx = mul(divQ(dirX, len), magnitude);
		int fy = mul(divQ(dirY, len), magnitude);
		int fz = mul(divQ(dirZ, len), magnitude);
		int rx = (worldX << 12) - px, ry = (worldY << 12) - py, rz = (worldZ << 12) - pz;
		extFX += fx; extFY += fy; extFZ += fz;
		extMX += mul(ry, fz) - mul(rz, fy);
		extMY += mul(rz, fx) - mul(rx, fz);
		extMZ += mul(rx, fy) - mul(ry, fx);
	}

	// Forces live for one step(): spent here so a caller that stops pushing
	// does not leave a shove queued for some later frame.
	private void clearForces() {
		extFX = extFY = extFZ = 0;
		extMX = extMY = extMZ = 0;
	}

	// Kinematic placement while held: follows a target center point and keeps an
	// axis aligned orientation. The delta over frameDt becomes the hand velocity,
	// so a throw inherits the carry motion.
	public void moveKinematic(int centerX, int centerY, int centerZ) {
		moveKinematic(centerX, centerY, centerZ, F);
	}

	public void moveKinematic(int centerX, int centerY, int centerZ, int frameDt) {
		int nx = centerX << 12, ny = centerY << 12, nz = centerZ << 12;
		this.vx = divQ(nx - px, frameDt);
		this.vy = divQ(ny - py, frameDt);
		this.vz = divQ(nz - pz, frameDt);
		// the hand velocity other cubes are pushed with (see velX)
		this.kvx = this.vx;
		this.kvy = this.vy;
		this.kvz = this.vz;
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

	// Kinematic placement while held, with a camera relative orientation (row-major
	// 4x4 as Transform produces): the carried cube turns with the camera.
	// Velocities stay zero; the hand target is re-derived every frame.
	public void setKinematicPose(int centerX, int centerY, int centerZ, float[] m) {
		setKinematicPose(centerX, centerY, centerZ, m, F);
	}

	public void setKinematicPose(int centerX, int centerY, int centerZ, float[] m, int frameDt) {
		// The hand velocity is kept apart from the simulated one: a carried cube
		// is immovable, but a swipe must still knock other cubes away.
		this.kvx = divQ((centerX << 12) - this.px, frameDt);
		this.kvy = divQ((centerY << 12) - this.py, frameDt);
		this.kvz = divQ((centerZ << 12) - this.pz, frameDt);
		this.px = centerX << 12;
		this.py = centerY << 12;
		this.pz = centerZ << 12;
		this.vx = this.vy = this.vz = 0;
		this.wx = this.wy = this.wz = 0;
		this.lx = this.ly = this.lz = 0;
		for(int row = 0; row < 3; row++) {
			for(int col = 0; col < 3; col++) {
				r[row * 3 + col] = q(m[row * 4 + col]);
			}
		}
		memCount = 0;
		fixMatrix();
		recomputeWorldInertia();
		recomputeMomentum();
		wake();
		computeVertices();
	}

	// Applies a portal warp matrix (row-major float[16], as produced by
	// PortalManager.getPortalTransform) to position, velocities and the
	// orientation frame.
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

		// angular momentum consistent with the rotated angular velocity
		recomputeMomentum();
		memCount = 0;
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

	// Advances the body by one frame. colliders are the meshes of the current and
	// neighbouring rooms, count how many of them are valid, and world is false
	// while the body is crossing a portal opening.
	public void step(Collider[] colliders, int count, boolean world) {
		step(colliders, count, world, F);
	}

	// frameDt is this step's length in Q12 nominal frames, one nominal frame
	// being F. Velocities stay units per nominal frame, so only the integration,
	// the forces and the sleep timer see it.
	public void step(Collider[] colliders, int count, boolean world, int frameDt) {
		this.stepDt = frameDt;
		// A cube at rest over a portal opening has lost the floor that put it to
		// sleep, and the world pass is skipped while it crosses, so it has to
		// wake and fall instead of hanging asleep in mid air.
		if(sleeping && !world) wake();

		if(sleeping) {
			vx = vy = vz = 0;
			wx = wy = wz = 0;
			lx = ly = lz = 0;
			computeVertices();
			this.energy = 0;
			clearForces();
			return;
		}

		// Gravity plus portalDS style velocity damping, expressed as forces,
		// plus whatever the game pushed with this frame (see applyForceAt).
		int fx = extFX;
		int fy = extFY;
		int fz = extFZ;
		int mx = -wx / ANGULAR_DRAG + extMX;
		int my = -wy / ANGULAR_DRAG + extMY;
		int mz = -wz / ANGULAR_DRAG + extMZ;

		int dt = frameDt;
		int substeps = 1;
		// Penetration cannot refine a step that skipped the geometry outright,
		// so the travel bound is what decides the substep count up front.
		int speed = isqrt((long) vx * vx + (long) vy * vy + (long) vz * vz);
		while(mul(speed, dt) > skipGuard && dt > MIN_DT) {
			dt >>= 1;
			++substeps;
		}
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
		clearForces();

		// Safety clamps run on every frame (not only contact frames): a
		// pathological impact must never leave a runaway spin behind.
		clampVelocity();

		// A body held up by another body is only at rest once the pair pass has
		// cancelled the gravity this step could not see, so its sleep bookkeeping
		// is redone by collideBodies. Every other body is settled by now.
		if(!bodySupport) updateSleep();

		computeVertices();
	}

	// Rest detection: low kinetic energy for SLEEP_TIME consecutive frames, and
	// only when supported from below - by the world or by another body
	// (BodyPair.recordSupport), or a stack could never come to rest.
	void updateSleep() {
		energy = mul(vx, vx) + mul(vy, vy) + mul(vz, vz)
				+ mul(wx, wx) + mul(wy, wy) + mul(wz, wz);
		if(!groundContact && !bodySupport) {
			sleepCounter = 0;
		} else if(energy >= SLEEP_HIGH) {
			sleeping = false;
			sleepCounter = 0;
		} else if(energy <= SLEEP_LOW) {
			sleepCounter += stepDt;
			if(sleepCounter >= SLEEP_TIME << 12) {
				sleeping = true;
				vx = vy = vz = 0;
				wx = wy = wz = 0;
				lx = ly = lz = 0;
			}
		} else {
			sleepCounter = 0;
		}
	}

	private void integrate(int dt, int fx, int fy, int fz, int mx, int my, int mz) {
		px += mul(vx, dt);
		py += mul(vy, dt);
		pz += mul(vz, dt);

		// R += skew(w * dt) * R, first order rotation update
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

		// Gravity and drag are accelerations and skip the mass. Routed through it
		// they only came out right for the unit cube: a 300 unit box fell 4.6
		// times as fast and a thin tile 8 times, which is why light boxes bounced
		// off the floor forever while the cube settled.
		vx += mul(divQ(fx, mass) - vx / LINEAR_DRAG, dt);
		vy += mul(divQ(fy, mass) - GRAVITY - vy / LINEAR_DRAG, dt);
		vz += mul(divQ(fz, mass) - vz / LINEAR_DRAG, dt);

		lx += mulL(mx, dt);
		ly += mulL(my, dt);
		lz += mulL(mz, dt);

		fixMatrix();
		recomputeWorldInertia();
		wx = eval24X(invIWorld, lx, ly, lz) >> iShift;
		wy = eval24Y(invIWorld, lx, ly, lz) >> iShift;
		wz = eval24Z(invIWorld, lx, ly, lz) >> iShift;
	}

	// Local inertia of a box, I = m/3(h2+h3). Extents in plain units: the Q12
	// square of anything past ~700 units does not fit an int.
	//
	// The inverse is Q24, stored 2^iShift times its value, and every reader
	// shifts back down. Past the unit cube the plain Q24 value rounds to almost
	// nothing, and a coarse tensor rounds the spin up or down every frame until
	// the box walks over and tips. The shipped cube keeps shift 0.
	private static final int SPIN_PRECISION = 64;
	private static final int SPIN_SHIFT_MAX = 20;
	private static final long SPIN_NUM = (long) 3 * F << 24;
	int iShift;

	private void computeLocalInertia() {
		identity3(invILocal);
		long x2 = (long) (hx >> 12) * (hx >> 12);
		long y2 = (long) (hy >> 12) * (hy >> 12);
		long z2 = (long) (hz >> 12) * (hz >> 12);
		long sx = Math.max(1, mass * (y2 + z2));
		long sy = Math.max(1, mass * (x2 + z2));
		long sz = Math.max(1, mass * (x2 + y2));
		long coarsest = Math.max(Math.max(sx, sy), sz);
		iShift = 0;
		while(iShift < SPIN_SHIFT_MAX && (SPIN_NUM << iShift) / coarsest < SPIN_PRECISION) {
			iShift++;
		}
		long num = SPIN_NUM << iShift;
		invILocal[0] = (int) (num / sx);
		invILocal[4] = (int) (num / sy);
		invILocal[8] = (int) (num / sz);
		il0 = sx / 3;
		il4 = sy / 3;
		il8 = sz / 3;
	}

	// invIWorld (Q24) = R * invILocal * R^T
	void recomputeWorldInertia() {
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


	// L = Iworld (Q12, built from il0..il8) * w
	private void recomputeMomentum() {
		long[] iw = tmpMatrix;
		for(int row = 0; row < 3; row++) {
			for(int col = 0; col < 3; col++) {
				long t0 = mulL(r[row * 3], il0);
				long t1 = mulL(r[row * 3 + 1], il4);
				long t2 = mulL(r[row * 3 + 2], il8);
				iw[row * 3 + col] = mulL(t0, r[col * 3]) + mulL(t1, r[col * 3 + 1])
						+ mulL(t2, r[col * 3 + 2]);
			}
		}
		lx = mulL(iw[0], wx) + mulL(iw[1], wy) + mulL(iw[2], wz);
		ly = mulL(iw[3], wx) + mulL(iw[4], wy) + mulL(iw[5], wz);
		lz = mulL(iw[6], wx) + mulL(iw[7], wy) + mulL(iw[8], wz);
	}

	// Gram-Schmidt re-orthonormalization of the three axis columns.
	void fixMatrix() {
		int xx = r[0], xy = r[3], xz = r[6];
		int yx = r[1], yy = r[4], yz = r[7];

		// Normalize x, project y onto it, rebuild z as x cross y: the cross keeps the
		// frame exact when a large spin drives two columns nearly parallel, where the
		// three-projection form degenerated.
		int mx = norm3(xx, xy, xz);
		if(mx == 0) { identity3(r); return; }
		xx = divQ(xx, mx); xy = divQ(xy, mx); xz = divQ(xz, mx);

		int d = mul(xx, yx) + mul(xy, yy) + mul(xz, yz);
		yx -= mul(xx, d); yy -= mul(xy, d); yz -= mul(xz, d);
		int my = norm3(yx, yy, yz);
		if(my == 0) { identity3(r); return; }
		yx = divQ(yx, my); yy = divQ(yy, my); yz = divQ(yz, my);

		// z = x cross y (columns are x=(r0,r3,r6), y=(r1,r4,r7))
		int zx = mul(xy, yz) - mul(xz, yy);
		int zy = mul(xz, yx) - mul(xx, yz);
		int zz = mul(xx, yy) - mul(xy, yx);

		r[0] = xx; r[3] = xy; r[6] = xz;
		r[1] = yx; r[4] = yy; r[7] = yz;
		r[2] = zx; r[5] = zy; r[8] = zz;
	}

	// ===================== contact generation =====================

	private void collideWorld(Collider[] colliders, int count, boolean world) {
		numContacts = 0;
		maxPenetration = 0;
		groundContact = false;
		computeVertices();
		if(!world) return;

		for(int i = 0; i < VERTICES; i++) {
			bestCount[i] = 0;
			minAbsGap[i] = Integer.MAX_VALUE;
		}
		edgeCount = 0;

		// Broadphase is swept against the pre-integration position: after a
		// deep (to be rolled back) step the box may sit beyond the very
		// surface it should collide with and would otherwise cull it.
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
			short[] norms = mesh.norms;
			int s8 = mesh.scale8;
			int ox = mesh.offX, oy = mesh.offY, oz = mesh.offZ;

			int x1 = ((bandMinX - ox) << 8) / s8 - 1;
			int y1 = ((bandMinY - oy) << 8) / s8 - 1;
			int z1 = ((bandMinZ - oz) << 8) / s8 - 1;
			int x2 = ((bandMaxX - ox) << 8) / s8 + 1;
			int y2 = ((bandMaxY - oy) << 8) / s8 + 1;
			int z2 = ((bandMaxZ - oz) << 8) / s8 + 1;

			for(int vpp = 4, pIdx = 0, nIdx = 0; vpp >= 3; vpp--) {
				int pEnd = vpp == 4 ? mesh.quads * 4 : pols.length;
				for(; pIdx < pEnd; pIdx += vpp, nIdx++) {
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

					int ax = (sax * s8 >> 8) + ox, ay = (say * s8 >> 8) + oy, az = (saz * s8 >> 8) + oz;
					int bx = (sbx * s8 >> 8) + ox, by = (sby * s8 >> 8) + oy, bz = (sbz * s8 >> 8) + oz;
					int cx = (scx * s8 >> 8) + ox, cy = (scy * s8 >> 8) + oy, cz = (scz * s8 >> 8) + oz;
					int dx = 0, dy = 0, dz = 0;
					if(vpp == 4) {
						dx = (sdx * s8 >> 8) + ox; dy = (sdy * s8 >> 8) + oy; dz = (sdz * s8 >> 8) + oz;
					}

					// mesh normal points into the solid (see SphereCast); the
					// point-in-poly test wants the mesh normal, the contact
					// normal is its negation (pointing out of the solid)
					int snx = norms[nIdx * 3], sny = norms[nIdx * 3 + 1], snz = norms[nIdx * 3 + 2];
					if(snx == 0 && sny == 0 && snz == 0) continue;
					int nx = -snx, ny = -sny, nz = -snz;

					// project the polygon onto its dominant axis plane once
					int au, av, bu, bv, cu, cv, du, dv;
					boolean flip;
					int axn = abs(snx), ayn = abs(sny), azn = abs(snz);
					if(axn >= ayn && axn >= azn) {
						au = az; av = ay; bu = bz; bv = by; cu = cz; cv = cy; du = dz; dv = dy;
						flip = snx < 0;
					} else if(ayn >= axn && ayn >= azn) {
						au = ax; av = az; bu = bx; bv = bz; cu = cx; cv = cz; du = dx; dv = dz;
						flip = sny < 0;
					} else {
						au = ax; av = ay; bu = bx; bv = by; cu = cx; cv = cy; du = dx; dv = dy;
						flip = snz > 0;
					}

					// A mesh edge can poke into the middle of a box face with no
					// box vertex anywhere near it (see addEdgeFaceContact), so it
					// is checked explicitly, once per polygon edge.
					addEdgeFaceContact(ax, ay, az, bx, by, bz);
					addEdgeFaceContact(bx, by, bz, cx, cy, cz);
					if(vpp == 4) {
						addEdgeFaceContact(cx, cy, cz, dx, dy, dz);
						addEdgeFaceContact(dx, dy, dz, ax, ay, az);
					} else {
						addEdgeFaceContact(cx, cy, cz, ax, ay, az);
					}

					for(int k = 0; k < VERTICES; k++) {
						int qx = vu[k * 3], qy = vu[k * 3 + 1], qz = vu[k * 3 + 2];

						// signed plane distance along the mesh normal;
						// positive == inside the solid, negative == free
						int d = ((qx - ax) * snx + (qy - ay) * sny + (qz - az) * snz) >> 12;

						// foot point projection on the plane
						int fx2 = qx - (snx * d >> 12);
						int fy2 = qy - (sny * d >> 12);
						int fz2 = qz - (snz * d >> 12);

						boolean inside;
						if(axn >= ayn && axn >= azn) {
							inside = pointInPoly(fz2, fy2, au, av, bu, bv, cu, cv, du, dv, vpp == 4, flip);
						} else if(ayn >= axn && ayn >= azn) {
							inside = pointInPoly(fx2, fz2, au, av, bu, bv, cu, cv, du, dv, vpp == 4, flip);
						} else {
							inside = pointInPoly(fx2, fy2, au, av, bu, bv, cu, cv, du, dv, vpp == 4, flip);
						}

						if(inside) {
							// record how far this face is from the vertex;
							// used to reject ghost contacts (see emit loop)
							int ag = abs(d);
							if(ag < minAbsGap[k]) minAbsGap[k] = ag;

							// vertices only touch a face once they reach the plane
							// (a wide free-side band would act like an invisible
							// shell); a deep penetration is kept on purpose so the
							// substep rollback can recover
							if(d < -SURFACE_TOUCH) continue;
							int gap = -d;
							if(gap <= CONTACT_MARGIN) {
								addCandidate(k, gap, nx, ny, nz, d > 0 ? d << 12 : 0);
							}
						} else {
							// nearest polygon edge
							int ex, ey, ez, s2;
							int bestS2 = Integer.MAX_VALUE, bx2 = 0, by2 = 0, bz2 = 0;

							s2 = edgeClosest(qx, qy, qz, ax, ay, az, bx, by, bz);
							if(s2 < bestS2) { bestS2 = s2; bx2 = ecx; by2 = ecy; bz2 = ecz; }
							s2 = edgeClosest(qx, qy, qz, bx, by, bz, cx, cy, cz);
							if(s2 < bestS2) { bestS2 = s2; bx2 = ecx; by2 = ecy; bz2 = ecz; }
							if(vpp == 4) {
								s2 = edgeClosest(qx, qy, qz, cx, cy, cz, dx, dy, dz);
								if(s2 < bestS2) { bestS2 = s2; bx2 = ecx; by2 = ecy; bz2 = ecz; }
								s2 = edgeClosest(qx, qy, qz, dx, dy, dz, ax, ay, az);
								if(s2 < bestS2) { bestS2 = s2; bx2 = ecx; by2 = ecy; bz2 = ecz; }
							} else {
								s2 = edgeClosest(qx, qy, qz, cx, cy, cz, ax, ay, az);
								if(s2 < bestS2) { bestS2 = s2; bx2 = ecx; by2 = ecy; bz2 = ecz; }
							}

							int s = isqrt(bestS2);
							if(s <= EDGE_MARGIN) {
								// Normalized in Q12, not by the integer
								// distance: one unit off a seam, isqrt(2) and
								// isqrt(3) both truncate to 1 and the corner
								// normal comes out sqrt(2)/sqrt(3) long.
								long edx = ((long) (qx - bx2)) << 12;
								long edy = ((long) (qy - by2)) << 12;
								long edz = ((long) (qz - bz2)) << 12;
								int edgeLen = isqrt(edx * edx + edy * edy + edz * edz);
								int enx, eny, enz;
								if(edgeLen > 0) {
									enx = (int) ((edx << 12) / edgeLen);
									eny = (int) ((edy << 12) / edgeLen);
									enz = (int) ((edz << 12) / edgeLen);
								} else {
									enx = nx; eny = ny; enz = nz;
								}
								addCandidate(k, s, enx, eny, enz, 0);
							}
						}
					}
				}
			}
		}

		for(int k = 0; k < VERTICES; k++) {
			int base = k * CORNER_SLOTS;
			int mg = minAbsGap[k] + CLOSEST_FEATURE_TOL;
			for(int s = 0; s < bestCount[k]; s++) {
				int idx = base + s;
				// Reject a "penetrating" candidate whose face is much farther than the
				// closest one: the vertex lies past the solid and would spin it wrong.
				if(bestGap[idx] < 0 && -bestGap[idx] > mg) continue;
				addContact(vq[k * 3], vq[k * 3 + 1], vq[k * 3 + 2],
						bestNX[idx], bestNY[idx], bestNZ[idx], bestPen[idx]);
			}
		}

		// emitted after the vertex contacts on purpose: those are the
		// reliable ones and must not be starved out of the shared budget
		for(int s = 0; s < edgeCount; s++) {
			addContact(edgeCX[s], edgeCY[s], edgeCZ[s], edgeNX[s], edgeNY[s], edgeNZ[s], edgePen[s]);
		}

		rememberSurfaces();
	}

	// Folds this step's contact normals into the recent surface memory, merging
	// a normal into the slot that already holds that surface.
	private void rememberSurfaces() {
		for(int i = 0; i < numContacts; i++) {
			int nx = cnx[i], ny = cny[i], nz = cnz[i];
			int slot = -1;
			for(int s = 0; s < memCount; s++) {
				if(mul(nx, memNX[s]) + mul(ny, memNY[s]) + mul(nz, memNZ[s]) >= DUPLICATE_NORMAL_DOT) {
					slot = s;
					break;
				}
			}
			if(slot < 0) {
				if(memCount < MEM_SLOTS) {
					slot = memCount++;
				} else {
					slot = 0;
					int farthest = -1;
					for(int s = 0; s < memCount; s++) {
						int d = abs(px - memX[s]) + abs(py - memY[s]) + abs(pz - memZ[s]);
						if(d > farthest) { farthest = d; slot = s; }
					}
				}
			}
			memNX[slot] = nx; memNY[slot] = ny; memNZ[slot] = nz;
			memX[slot] = px; memY[slot] = py; memZ[slot] = pz;
		}
	}

	// Up to CORNER_SLOTS contacts for one vertex, not just the deepest. A candidate
	// whose normal matches one already kept (dot >= DUPLICATE_NORMAL_DOT) is the
	// same surface and only the deeper survives; a different normal (a real
	// corner) stays independent, so both constraints hold in one frame instead of
	// alternating - which was the corner jitter and launch.
	private void addCandidate(int k, int gap, int nx, int ny, int nz, int pen) {
		int base = k * CORNER_SLOTS;
		int count = bestCount[k];
		for(int s = 0; s < count; s++) {
			int idx = base + s;
			int dot = mul(nx, bestNX[idx]) + mul(ny, bestNY[idx]) + mul(nz, bestNZ[idx]);
			if(dot >= DUPLICATE_NORMAL_DOT) {
				if(gap < bestGap[idx]) {
					bestGap[idx] = gap;
					bestNX[idx] = nx; bestNY[idx] = ny; bestNZ[idx] = nz;
					bestPen[idx] = pen;
				}
				return;
			}
		}
		if(count < CORNER_SLOTS) {
			int idx = base + count;
			bestGap[idx] = gap;
			bestNX[idx] = nx; bestNY[idx] = ny; bestNZ[idx] = nz;
			bestPen[idx] = pen;
			bestCount[k] = count + 1;
			return;
		}
		// slots full: keep the closest features. Evict the candidate with the
		// largest |gap| (farthest feature) so a ghost contact (huge depth on
		// a far face) can never push a real, close contact out of the slots.
		int worst = base, worstGap = abs(bestGap[base]);
		for(int s = 1; s < count; s++) {
			int idx = base + s;
			int ag = abs(bestGap[idx]);
			if(ag > worstGap) { worst = idx; worstGap = ag; }
		}
		if(abs(gap) < worstGap) {
			bestGap[worst] = gap;
			bestNX[worst] = nx; bestNY[worst] = ny; bestNZ[worst] = nz;
			bestPen[worst] = pen;
		}
	}

	// One mesh polygon edge against the box's six faces in its local frame: the
	// mirror of the vertex test, and the only thing that sees a wall corner
	// buried in a flat box face. Missed, that penetration grows past
	// PENETRATION_THRESHOLD with no contact, and the one deep correction is the
	// random launch at corners. Sampled at both endpoints, at every crossing of
	// the six face planes, and at the point closest to the box centre.
	private void addEdgeFaceContact(int ax, int ay, int az, int bx, int by, int bz) {
		int pcx = px >> 12, pcy = py >> 12, pcz = pz >> 12;
		int phx = hx >> 12, phy = hy >> 12, phz = hz >> 12;

		int rax = ax - pcx, ray = ay - pcy, raz = az - pcz;
		int rbx = bx - pcx, rby = by - pcy, rbz = bz - pcz;
		int l0x = mul(r[0], rax) + mul(r[3], ray) + mul(r[6], raz);
		int l0y = mul(r[1], rax) + mul(r[4], ray) + mul(r[7], raz);
		int l0z = mul(r[2], rax) + mul(r[5], ray) + mul(r[8], raz);
		int l1x = mul(r[0], rbx) + mul(r[3], rby) + mul(r[6], rbz);
		int l1y = mul(r[1], rbx) + mul(r[4], rby) + mul(r[7], rbz);
		int l1z = mul(r[2], rbx) + mul(r[5], rby) + mul(r[8], rbz);

		// cheap reject: does the edge's local bounding box reach the inflated box?
		if(Math.max(l0x, l1x) < -phx - CONTACT_MARGIN || Math.min(l0x, l1x) > phx + CONTACT_MARGIN) return;
		if(Math.max(l0y, l1y) < -phy - CONTACT_MARGIN || Math.min(l0y, l1y) > phy + CONTACT_MARGIN) return;
		if(Math.max(l0z, l1z) < -phz - CONTACT_MARGIN || Math.min(l0z, l1z) > phz + CONTACT_MARGIN) return;

		int dLX = l1x - l0x, dLY = l1y - l0y, dLZ = l1z - l0z;

		// Parameter (Q14, 0..16384 spans the segment) of the point closest to the
		// box center: for a segment passing through the box this is also the
		// deepest point of the overlap.
		int tCenter = 0;
		long dd = (long) dLX * dLX + (long) dLY * dLY + (long) dLZ * dLZ;
		if(dd != 0) {
			long num = -((long) l0x * dLX + (long) l0y * dLY + (long) l0z * dLZ);
			tCenter = (int) ((num << 14) / dd);
			if(tCenter < 0) tCenter = 0;
			else if(tCenter > 16384) tCenter = 16384;
		}

		// Candidate parameters, Q14 (0..16384 spans the segment), sampled in the
		// order addEdgeFaceContact's javadoc describes; -1 means "doesn't cross".
		int[] ts = { 0, 16384, tCenter,
				dLX != 0 ? (int) (((long) (phx - l0x) << 14) / dLX) : -1,
				dLX != 0 ? (int) (((long) (-phx - l0x) << 14) / dLX) : -1,
				dLY != 0 ? (int) (((long) (phy - l0y) << 14) / dLY) : -1,
				dLY != 0 ? (int) (((long) (-phy - l0y) << 14) / dLY) : -1,
				dLZ != 0 ? (int) (((long) (phz - l0z) << 14) / dLZ) : -1,
				dLZ != 0 ? (int) (((long) (-phz - l0z) << 14) / dLZ) : -1 };

		int bestT = -1, bestAxis = -1, bestD = Integer.MIN_VALUE;
		int bestLx = 0, bestLy = 0, bestLz = 0;
		for(int i = 0; i < ts.length; i++) {
			int t = ts[i];
			if(t < 0 || t > 16384) continue;
			int lx = l0x + (int) (((long) dLX * t) >> 14);
			int ly = l0y + (int) (((long) dLY * t) >> 14);
			int lz = l0z + (int) (((long) dLZ * t) >> 14);
			int dX = phx - abs(lx), dY = phy - abs(ly), dZ = phz - abs(lz);

			// an axis only counts as the exit face if the point falls within
			// the box's footprint on the OTHER two, else it is near an edge
			int axis = -1, d = Integer.MAX_VALUE;
			if(dY >= 0 && dZ >= 0 && dX < d) { axis = 0; d = dX; }
			if(dX >= 0 && dZ >= 0 && dY < d) { axis = 1; d = dY; }
			if(dX >= 0 && dY >= 0 && dZ < d) { axis = 2; d = dZ; }

			if(axis != -1 && d > bestD) {
				bestD = d; bestAxis = axis; bestT = t;
				bestLx = lx; bestLy = ly; bestLz = lz;
			}
		}
		if(bestAxis == -1) return;
		// same acceptance band as the box-vertex-vs-mesh-face test: a deep
		// penetration is fine (kept for the substep rollback), a wide
		// free-side gap is not
		if(bestD < -SURFACE_TOUCH || -bestD > CONTACT_MARGIN) return;

		// The normal is the direction the BOX must move to stop containing the mesh
		// point: the OPPOSITE of the point's own local sign.
		int sign, nx, ny, nz;
		if(bestAxis == 0) {
			sign = bestLx >= 0 ? -1 : 1;
			nx = sign * r[0]; ny = sign * r[3]; nz = sign * r[6];
		} else if(bestAxis == 1) {
			sign = bestLy >= 0 ? -1 : 1;
			nx = sign * r[1]; ny = sign * r[4]; nz = sign * r[7];
		} else {
			sign = bestLz >= 0 ? -1 : 1;
			nx = sign * r[2]; ny = sign * r[5]; nz = sign * r[8];
		}

		int cx = ax + (int) (((long) (bx - ax) * bestT) >> 14);
		int cy = ay + (int) (((long) (by - ay) * bestT) >> 14);
		int cz = az + (int) (((long) (bz - az) * bestT) >> 14);

		addEdgeCandidate(cx << 12, cy << 12, cz << 12, nx, ny, nz,
				-bestD, bestD > 0 ? bestD << 12 : 0);
	}

	// Up to EDGE_CONTACT_SLOTS direction-distinct edge-vs-face candidates for the
	// whole box, the way addCandidate does per vertex: without it every seam
	// between two polygons of one wall registers a near-duplicate, and these are
	// emitted before the reliable vertex contacts fill the shared budget.
	private void addEdgeCandidate(int cx, int cy, int cz, int nx, int ny, int nz, int gap, int pen) {
		for(int s = 0; s < edgeCount; s++) {
			int dot = mul(nx, edgeNX[s]) + mul(ny, edgeNY[s]) + mul(nz, edgeNZ[s]);
			// Only merge same-normal candidates that are the SAME contact (the seam of two
			// coplanar polygons, tested once per polygon). Two at different positions are
			// a genuine manifold: collapsing them moved the impulse off the centroid and
			// spun the box on a straight-on hit.
			if(dot >= DUPLICATE_NORMAL_DOT) {
				long dx = (long) cx - edgeCX[s];
				long dy = (long) cy - edgeCY[s];
				long dz = (long) cz - edgeCZ[s];
				long d2 = dx * dx + dy * dy + dz * dz;
				if(d2 <= EDGE_MERGE_DIST2) {
					if(gap < edgeGap[s]) {
						edgeGap[s] = gap; edgeNX[s] = nx; edgeNY[s] = ny; edgeNZ[s] = nz; edgePen[s] = pen;
						edgeCX[s] = cx; edgeCY[s] = cy; edgeCZ[s] = cz;
					}
					return;
				}
			}
		}
		if(edgeCount < EDGE_CONTACT_SLOTS) {
			int s = edgeCount++;
			edgeGap[s] = gap; edgeNX[s] = nx; edgeNY[s] = ny; edgeNZ[s] = nz; edgePen[s] = pen;
			edgeCX[s] = cx; edgeCY[s] = cy; edgeCZ[s] = cz;
			return;
		}
		int worst = 0, worstGap = edgeGap[0];
		for(int s = 1; s < edgeCount; s++) {
			if(edgeGap[s] > worstGap) { worst = s; worstGap = edgeGap[s]; }
		}
		if(gap < worstGap) {
			edgeGap[worst] = gap; edgeNX[worst] = nx; edgeNY[worst] = ny; edgeNZ[worst] = nz; edgePen[worst] = pen;
			edgeCX[worst] = cx; edgeCY[worst] = cy; edgeCZ[worst] = cz;
		}
	}

	private void addContact(int x, int y, int z, int nx, int ny, int nz, int pen) {
		if(numContacts >= MAX_CONTACTS) return;
		cpx[numContacts] = x; cpy[numContacts] = y; cpz[numContacts] = z;
		cnx[numContacts] = nx; cny[numContacts] = ny; cnz[numContacts] = nz;
		cpen[numContacts] = pen;
		if(pen > maxPenetration) maxPenetration = pen;
		// outward normal pointing mostly up == supported by the ground
		if(ny > (F * 7 / 10)) groundContact = true;
		++numContacts;
	}

	// ===================== impulses =====================

	private final int[] accN = new int[MAX_CONTACTS];
	private final int[] accT = new int[MAX_CONTACTS];
	private final int[] vbias = new int[MAX_CONTACTS];

	// The micro-ops shared with the pair pass are in SolverMath; what is left here
	// is specific to one body against static geometry.

	// v + w x r at a contact offset r from this body's centre, into
	// cvx/cvy/cvz.
	private int cvx, cvy, cvz;
	private void contactVelocity(int rx, int ry, int rz) {
		cvx = vx + mul(wy, rz) - mul(wz, ry);
		cvy = vy + mul(wz, rx) - mul(wx, rz);
		cvz = vz + mul(wx, ry) - mul(wy, rx);
	}

	// This body taking an impulse of magnitude j along d at offset r: j/m into
	// the linear velocity, r x (d j) into the angular momentum, and then w
	// re-derived from l.
	private void applyContactImpulse(int rx, int ry, int rz,
			int dx, int dy, int dz, int j) {
		int imp = mul(j, invMass);
		vx += mul(dx, imp);
		vy += mul(dy, imp);
		vz += mul(dz, imp);
		lx += mulL(ry, mulL(dz, j)) - mulL(rz, mulL(dy, j));
		ly += mulL(rz, mulL(dx, j)) - mulL(rx, mulL(dz, j));
		lz += mulL(rx, mulL(dy, j)) - mulL(ry, mulL(dx, j));
		wx = eval24X(invIWorld, lx, ly, lz) >> iShift;
		wy = eval24Y(invIWorld, lx, ly, lz) >> iShift;
		wz = eval24Z(invIWorld, lx, ly, lz) >> iShift;
	}

	private void applyImpulses() {
		for(int i = 0; i < numContacts; i++) {
			accN[i] = 0; accT[i] = 0; vbias[i] = 0;
		}

		// Sequential impulses with accumulated magnitudes: several Gauss-Seidel sweeps
		// let a multi-point contact converge instead of handing a resting box four
		// independent impulses that make it tumble. Restitution targets are fixed once
		// from the approach velocities, or mid-sweep targets disagree.
		// A resting contact closes at one step of gravity, so this is a threshold
		// on the step, not on the second.
		int bounceSpeed = mul(RESTITUTION_SPEED, stepDt);
		for(int i = 0; i < numContacts; i++) {
			contactVelocity(cpx[i] - px, cpy[i] - py, cpz[i] - pz);
			int vn = mul(cvx, cnx[i]) + mul(cvy, cny[i]) + mul(cvz, cnz[i]);
			vbias[i] = -vn > bounceSpeed ? -mul(RESTITUTION, vn) : 0;
		}

		for(int iter = 0; iter < IMPULSE_ITERATIONS; iter++) {
			for(int i = 0; i < numContacts; i++) {
				int rx = cpx[i] - px, ry = cpy[i] - py, rz = cpz[i] - pz;
				int nx = cnx[i], ny = cny[i], nz = cnz[i];

				contactVelocity(rx, ry, rz);
				int vn = mul(cvx, nx) + mul(cvy, ny) + mul(cvz, nz);
				int kn = invMass
						+ angularEffectiveMass(rx, ry, rz, invIWorld, iShift, nx, ny, nz);

				// The restitution target comes from the prepass and is enforced by
				// every sweep, so later sweeps do not eat the bounce.
				if(kn > 0) {
					int dN = accumulate(accN, i, divQ(vbias[i] - vn, kn),
							0, Integer.MAX_VALUE);
					if(dN != 0) applyContactImpulse(rx, ry, rz, nx, ny, nz, dN);
				}

				if(accN[i] <= 0) continue;

				// Friction resists the sliding component of the contact velocity,
				// up to mu times the normal impulse accumulated so far.
				contactVelocity(rx, ry, rz);
				if(!contactTangent(cvx, cvy, cvz, nx, ny, nz)) continue;
				int kt = invMass + angularEffectiveMass(rx, ry, rz, invIWorld,
						iShift, tanX, tanY, tanZ);
				if(kt <= 0) continue;
				int vt = mul(cvx, tanX) + mul(cvy, tanY) + mul(cvz, tanZ);
				int maxFric = abs(mul(FRICTION, accN[i]));
				int dT = accumulate(accT, i, -divQ(vt, kt), -maxFric, maxFric);
				if(dT != 0) applyContactImpulse(rx, ry, rz, tanX, tanY, tanZ, dT);
			}
		}

		correctPositions();
	}

	// Sequential position projection over every contact: moves and rotates the body
	// like the velocity impulses but touches positions only, so it never injects
	// momentum. Split across all contacts, not one snap on the deepest point,
	// which was the corner-jam energy pump.
	private void correctPositions() {
		final int beta = (int) ((long) F / POSITION_ITERATIONS);
		for(int iter = 0; iter < POSITION_ITERATIONS; iter++) {
			for(int i = 0; i < numContacts; i++) {
				int pen = cpen[i];
				if(pen <= POSITION_SLOP << 12) continue;
				int rx = cpx[i] - px, ry = cpy[i] - py, rz = cpz[i] - pz;
				int nx = cnx[i], ny = cny[i], nz = cnz[i];

				int k = invMass
						+ angularEffectiveMass(rx, ry, rz, invIWorld, iShift, nx, ny, nz);
				if(k <= 0) continue;

				int target = mul(pen - (POSITION_SLOP << 12), beta);
				int dp = divQ(target, k);
				px += mul(nx, mul(dp, invMass));
				py += mul(ny, mul(dp, invMass));
				pz += mul(nz, mul(dp, invMass));

				// angular position step dq = I^-1 (r x n * dp)
				int mx = mul(ry, mul(nz, dp)) - mul(rz, mul(ny, dp));
				int my = mul(rz, mul(nx, dp)) - mul(rx, mul(nz, dp));
				int mz = mul(rx, mul(ny, dp)) - mul(ry, mul(nx, dp));
				rotateMatrix(eval24X(invIWorld, mx, my, mz) >> iShift,
						eval24Y(invIWorld, mx, my, mz) >> iShift,
						eval24Z(invIWorld, mx, my, mz) >> iShift);
				recomputeWorldInertia();
			}
		}
		fixMatrix();
		recomputeWorldInertia();
	}

	// R += skew(q) * R in place (same first order update as integration).
	void rotateMatrix(int qx, int qy, int qz) {
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

	void clampVelocity() {
		if(vx > MAX_LINEAR) vx = MAX_LINEAR; else if(vx < -MAX_LINEAR) vx = -MAX_LINEAR;
		if(vy > MAX_LINEAR) vy = MAX_LINEAR; else if(vy < -MAX_LINEAR) vy = -MAX_LINEAR;
		if(vz > MAX_LINEAR) vz = MAX_LINEAR; else if(vz < -MAX_LINEAR) vz = -MAX_LINEAR;
		if(wx > MAX_ANGULAR) wx = MAX_ANGULAR; else if(wx < -MAX_ANGULAR) wx = -MAX_ANGULAR;
		if(wy > MAX_ANGULAR) wy = MAX_ANGULAR; else if(wy < -MAX_ANGULAR) wy = -MAX_ANGULAR;
		if(wz > MAX_ANGULAR) wz = MAX_ANGULAR; else if(wz < -MAX_ANGULAR) wz = -MAX_ANGULAR;
		// momentum must stay consistent with the clamped velocities
		recomputeMomentum();
	}

	// ================== body vs body: see BodyPair ==================

	// Cube against cube: the separating axis test, manifold generation and the
	// pair solver live in BodyPair. This stays the entry point, because stepping
	// a group of bodies reads as something a RigidBody does.
	public static void collideBodies(RigidBody[] bodies, int count) {
		BodyPair.collide(bodies, count);
	}

	// ===================== vertices / AABB =====================

	// Recomputes world vertices (Q12 and units) and the world AABB.
	void computeVertices() {
		int c0x = r[0], c0y = r[3], c0z = r[6];
		int c1x = r[1], c1y = r[4], c1z = r[7];
		int c2x = r[2], c2y = r[5], c2z = r[8];
		int ex = hx, ey = hy, ez = hz;

		int mnx = Integer.MAX_VALUE, mny = Integer.MAX_VALUE, mnz = Integer.MAX_VALUE;
		int mxx = Integer.MIN_VALUE, mxy = Integer.MIN_VALUE, mxz = Integer.MIN_VALUE;
		int cr = 0;

		// 0:--- 1:+-- 2:+-+ 3:--+ 4:-++ 5:-+- 6:++- 7:+++
		for(int k = 0; k < VERTICES; k++) {
			int sx = (k == 1 || k == 2 || k == 6 || k == 7) ? 1 : -1;
			int sy = (k == 4 || k == 5 || k == 6 || k == 7) ? 1 : -1;
			int sz = (k == 2 || k == 3 || k == 4 || k == 7) ? 1 : -1;
			int ox3 = sx * mul(c0x, ex) + sy * mul(c1x, ey) + sz * mul(c2x, ez);
			int oy3 = sx * mul(c0y, ex) + sy * mul(c1y, ey) + sz * mul(c2y, ez);
			int oz3 = sx * mul(c0z, ex) + sy * mul(c1z, ey) + sz * mul(c2z, ez);
			int wx = px + ox3, wy = py + oy3, wz = pz + oz3;
			vq[k * 3] = wx; vq[k * 3 + 1] = wy; vq[k * 3 + 2] = wz;
			int ux = wx >> 12, uy = wy >> 12, uz = wz >> 12;
			vu[k * 3] = ux; vu[k * 3 + 1] = uy; vu[k * 3 + 2] = uz;
			if(ux < mnx) mnx = ux; if(ux > mxx) mxx = ux;
			if(uy < mny) mny = uy; if(uy > mxy) mxy = uy;
			if(uz < mnz) mnz = uz; if(uz > mxz) mxz = uz;
		}
		boxMinX = mnx; boxMinY = mny; boxMinZ = mnz;
		boxMaxX = mxx; boxMaxY = mxy; boxMaxZ = mxz;

		// largest center-to-corner reach, used to reject ambiguous planes
		int rxr = abs(c0x) * (ex >> 12) + abs(c1x) * (ey >> 12) + abs(c2x) * (ez >> 12);
		int ryr = abs(c0y) * (ex >> 12) + abs(c1y) * (ey >> 12) + abs(c2y) * (ez >> 12);
		int rzr = abs(c0z) * (ex >> 12) + abs(c1z) * (ey >> 12) + abs(c2z) * (ez >> 12);
		cr = (rxr >> 12); if((ryr >> 12) > cr) cr = ryr >> 12; if((rzr >> 12) > cr) cr = rzr >> 12;
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


	private static void identity3(int[] m) {
		m[0] = F; m[1] = 0; m[2] = 0;
		m[3] = 0; m[4] = F; m[5] = 0;
		m[6] = 0; m[7] = 0; m[8] = F;
	}

	private final long[] tmpMatrix = new long[9];


	// closest point on an edge, scratch return via ecx/ecy/ecz, distance squared
	private static int ecx, ecy, ecz;
	static int edgeClosest(int px, int py, int pz,
			int ax, int ay, int az, int bx, int by, int bz) {
		int dx = bx - ax, dy = by - ay, dz = bz - az;
		long lenSq = (long) dx * dx + (long) dy * dy + (long) dz * dz;
		long t = 0;
		if(lenSq != 0) t = (((long) (px - ax) * dx + (long) (py - ay) * dy + (long) (pz - az) * dz) << 14) / lenSq;
		if(t < 0) t = 0;
		else if(t > 16384) t = 16384;
		ecx = ax + (int) ((long) dx * t >> 14);
		ecy = ay + (int) ((long) dy * t >> 14);
		ecz = az + (int) ((long) dz * t >> 14);
		int ex2 = px - ecx, ey2 = py - ecy, ez2 = pz - ecz;
		return (int) ((long) ex2 * ex2 + (long) ey2 * ey2 + (long) ez2 * ez2);
	}

	// 2D half plane test for one polygon edge
	private static boolean halfPlane(int pu, int pv, int au, int av, int bu, int bv) {
		return (long) (bu - au) * (pv - av) <= (long) (pu - au) * (bv - av);
	}

	// Point in triangle/quad projected on a dominant axis plane; flip reverses winding.
	private static boolean pointInPoly(int pu, int pv,
			int au, int av, int bu, int bv, int cu, int cv, int du, int dv,
			boolean quad, boolean flip) {
		if(!flip) {
			if(!halfPlane(pu, pv, au, av, bu, bv)) return false;
			if(!halfPlane(pu, pv, bu, bv, cu, cv)) return false;
			if(quad) {
				if(!halfPlane(pu, pv, cu, cv, du, dv)) return false;
				if(!halfPlane(pu, pv, du, dv, au, av)) return false;
			} else {
				if(!halfPlane(pu, pv, cu, cv, au, av)) return false;
			}
		} else {
			if(!halfPlane(pu, pv, cu, cv, bu, bv)) return false;
			if(!halfPlane(pu, pv, bu, bv, au, av)) return false;
			if(quad) {
				if(!halfPlane(pu, pv, au, av, du, dv)) return false;
				if(!halfPlane(pu, pv, du, dv, cu, cv)) return false;
			} else {
				if(!halfPlane(pu, pv, au, av, cu, cv)) return false;
			}
		}
		return true;
	}
}
