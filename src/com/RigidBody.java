package com;

/**
 * Oriented rigid box with impulse based contact physics.
 *
 * Direct J2ME style port of the OBB solver from portalDS
 * (arm7/source/OBB.c + arm7/source/AAR.c), adapted to this engine:
 *
 *  - all state is Q12 fixed point (4096 == 1.0): no floating point and no
 *    per frame allocations anywhere in the solver,
 *  - semi-implicit Euler integration with a first order rotation update
 *    followed by a Gram-Schmidt matrix re-orthonormalization,
 *  - contacts are generated at the 8 box vertices against the engine's
 *    triangle/quad meshes (portalDS collided against axis aligned planes;
 *    here the room geometry is an arbitrary triangle soup),
 *  - sequential contact impulses with restitution and Coulomb friction,
 *  - adaptive substeps: the integrated state is backed up and the step is
 *    halved whenever a contact penetrates too deep,
 *  - the body falls asleep once it rests on an upward facing surface,
 *  - boxes also collide with each other: {@link #collideBodies} runs a
 *    separating axis test plus face clipping over every pair once per frame
 *    and solves the contacts against both bodies at once, so cubes stack on,
 *    bounce off and push each other (see the body vs body section).
 *
 * World coordinates are plain engine units (one solver step per rendered
 * frame). Mesh face normals follow the SphereCast convention and point INTO
 * the solid, so an outward contact normal is the negated mesh normal.
 *
 * Matrices are 3x3 row-major Q12; orientation matrix R maps local vectors
 * to world vectors (world = R * local).
 */
public final class RigidBody {

	/** Q12 scale. */
	public static final int F = 4096;

	// Vertex-based contacts (up to VERTICES * CORNER_SLOTS == 24) and
	// edge-based contacts (up to EDGE_CONTACT_SLOTS == 8) are two
	// independent, capped budgets that both feed the same contact list;
	// MAX_CONTACTS covers both so neither can starve the other out.
	private static final int MAX_CONTACTS = 32;
	private static final int VERTICES = 8;
	/** Max simultaneous, direction-distinct contacts kept for one box vertex
	 *  (a vertex wedged into a corner can touch more than one wall at once). */
	private static final int CORNER_SLOTS = 3;
	/** Max simultaneous, direction-distinct mesh-edge-vs-box-face contacts
	 *  (see addEdgeFaceContact). Capped and deduplicated the same way as
	 *  CORNER_SLOTS, and for the same reason: a wall or floor built from
	 *  more than one polygon has seams, and every seam under the box would
	 *  otherwise register its own near-duplicate contact. */
	private static final int EDGE_CONTACT_SLOTS = 8;
	/** Same-normal edge-face candidates closer than this (squared, Q12) are
	 *  the same contact (wall seam duplicates) and get merged. */
	private static final long EDGE_MERGE_DIST2 = (long) (8 << 12) * (8 << 12);
	/** Two candidate normals for the same vertex are treated as the same
	 *  surface (merge, keep the deeper one) once their dot product reaches
	 *  this; below it they are kept as separate simultaneous contacts. Q12;
	 *  ~3072 == cos(41 deg), comfortably separating a real corner (normals
	 *  near perpendicular, dot ~0) from two coplanar polygons of one wall
	 *  (dot ~F). */
	private static final int DUPLICATE_NORMAL_DOT = F * 3 / 4;
	/** A vertex-face penetration is only trusted while the face is (nearly)
	 *  the closest feature to the vertex. A vertex past a closed solid (e.g.
	 *  beyond the far side of a square column) would otherwise be claimed by
	 *  the far face as a deep penetration with a perpendicular normal, which
	 *  is what spun the box about the wrong axis at corners. Units. */
	private static final int CLOSEST_FEATURE_TOL = 16;

	/** Contacts are generated within this band around a surface (units). */
	private static final int CONTACT_MARGIN = 64;
	/** A vertex within this distance on the free side still touches a face. */
	private static final int SURFACE_TOUCH = 6;
	/** Convex polygon edges only catch vertices this close (tighter than faces). */
	private static final int EDGE_MARGIN = 24;
	/** Deeper penetration than this rolls the step back and halves dt. */
	private static final int PENETRATION_THRESHOLD = 64;
	/** Smallest substep as a fraction of a frame (1/256). */
	private static final int MIN_DT = 16;
	/** Gauss-Seidel sweeps over the contact list per frame. */
	private static final int IMPULSE_ITERATIONS = 8;
	/** Position projection sweeps; every contact is de-penetrated, not just
	 *  the deepest one, so a cube wedged in a corner cannot be repeatedly
	 *  snapped and gain energy. */
	private static final int POSITION_ITERATIONS = 4;
	/** Contacts within this many units are treated as resting (no projection). */
	private static final int POSITION_SLOP = 3;
	/** Hard safety clamps: pathological single-vertex corner contacts must
	 *  never launch or spin the cube beyond gameplay-scale velocities. */
	private static final int MAX_LINEAR = 2048 << 12;
	private static final int MAX_ANGULAR = 2 * F;
	/** Impacts slower than this (Q12 units/frame) are treated as inelastic,
	 *  otherwise micro rocking on a resting contact never damps out. */
	private static final int RESTITUTION_SPEED = 30 << 12;

	/** Gravity, units per frame (matches Character: speed.y -= 20). */
	private static final int GRAVITY = 20 << 12;
	/** Linear velocity damping divisor (v -= v / 25 per frame). */
	private static final int LINEAR_DRAG = 25;
	/** Angular velocity damping divisor. */
	private static final int ANGULAR_DRAG = 20;

	/** Contact material against world geometry (portalDS plane values). */
	private static final int RESTITUTION = 819;   // 0.2
	private static final int FRICTION = 4096;    // 1.0

	// Sleep thresholds are energy values (Q12 of units/frame, squared).
	private static final int SLEEP_LOW = 120000;
	private static final int SLEEP_HIGH = 400000;
	private static final int SLEEP_TIME = 16;

	// ---- body state (all Q12 unless noted) ----
	private int hx, hy, hz;          // half extents
	private int mass, invMass;
	private int il0, il4, il8;       // local inertia (inverse of invILocal)
	private int px, py, pz;          // center position
	private int vx, vy, vz;          // linear velocity, units/frame
	private int lx, ly, lz;          // angular momentum
	private int wx, wy, wz;          // angular velocity
	private final int[] r = new int[9];   // orientation, row-major
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

	// ---- closest feature(s) per box vertex while scanning a mesh ----
	// Up to CORNER_SLOTS direction-distinct contacts are kept per vertex, not
	// just the single deepest one: a vertex wedged into a corner is close to
	// more than one wall at once, and collapsing that down to one contact is
	// what let the box ping-pong between walls and launch itself (see
	// addCandidate).
	private final int[] bestGap = new int[VERTICES * CORNER_SLOTS];
	private final int[] bestNX = new int[VERTICES * CORNER_SLOTS];
	private final int[] bestNY = new int[VERTICES * CORNER_SLOTS];
	private final int[] bestNZ = new int[VERTICES * CORNER_SLOTS];
	private final int[] bestPen = new int[VERTICES * CORNER_SLOTS];
	private final int[] bestCount = new int[VERTICES];
	/** Closest face-plane distance per box vertex (absolute, units): the
	 *  distance to the nearest face whose footprint contains the vertex.
	 *  Used to reject ghost contacts, see CLOSEST_FEATURE_TOL. */
	private final int[] minAbsGap = new int[VERTICES];

	// ---- direction-distinct mesh-edge-vs-box-face candidates, see
	// addEdgeFaceContact / addEdgeCandidate. Global for the box (not
	// per-vertex): these contacts aren't tied to any of the 8 vertices, so
	// they also need their own world-space contact point (Q12), unlike the
	// vertex ones which just reuse vq[k].
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
	private int boxMinX, boxMinY, boxMinZ, boxMaxX, boxMaxY, boxMaxZ;
	private int cornerRadius;

	// ---- backup for substep rollback ----
	private int bpx, bpy, bpz, bvx, bvy, bvz, blx, bly, blz, bwx, bwy, bwz;
	private final int[] br = new int[9];
	private final int[] biw = new int[9];

	private boolean sleeping;
	private int sleepCounter;
	private int energy;

	// ---- body vs body state (see collideBodies) ----
	/** A carried cube is placed kinematically: it joins pair contacts as an
	 *  immovable obstacle that still lends its hand velocity. */
	private boolean kinematic;
	/** Hand velocity of a kinematic body (Q12 units/frame): its own velocity
	 *  stays zero because the hand target is re-derived every frame. */
	private int kvx, kvy, kvz;
	/** Held up by another body instead of by the world; counts as ground for
	 *  the sleep bookkeeping, so a stack of cubes can come to rest. */
	private boolean bodySupport;
	private RigidBody supportBody;
	/** Support normal of the contact that holds this body up (Q12, pointing
	 *  out of the support into the body): the direction a body resting on
	 *  another body cannot be pushed into. */
	private int supportNX, supportNY, supportNZ;
	private RigidBody prevSupport;
	private boolean pairTouched;

	/** Number of integration substeps used by the last frame (for tests). */
	public int lastSubsteps = 1;

	/**
	 * Triangle/quad mesh as the engine stores it (see MeshData and
	 * SphereCast): short indexed vertices/polygons, Q12 polygon normals and
	 * an fp8 scale with integer offsets applied to every vertex.
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

	/** Mass is 1.0 Q12 by default; heavier bodies move less eagerly. */
	public void setMass(int massQ12) {
		this.mass = massQ12;
		this.invMass = divQ(F, massQ12);
		computeLocalInertia();
	}

	/** Places the body at a center position given in engine units. */
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
		this.kinematic = false;
		this.kvx = this.kvy = this.kvz = 0;
		this.bodySupport = false;
		this.supportBody = null;
		this.supportNX = this.supportNY = this.supportNZ = 0;
		this.prevSupport = null;
		this.pairTouched = false;
		computeVertices();
	}

	/**
	 * Marks the body as carried: other bodies collide with it and are pushed
	 * aside, it never reacts to them. The hand velocity is kept separately
	 * (see setKinematicPose) so a swipe still knocks other cubes away.
	 */
	public void setKinematic(boolean carried) {
		this.kinematic = carried;
		if(!carried) this.kvx = this.kvy = this.kvz = 0;
		wake();
	}

	public boolean isKinematic() {
		return kinematic;
	}

	/** True while another body holds this one up (see recordSupport). */
	public boolean isSupportedByBody() {
		return bodySupport;
	}

	public void wake() {
		this.sleeping = false;
		this.sleepCounter = 0;
	}

	public boolean isSleeping() {
		return sleeping;
	}

	/** Number of contacts generated for the last solved frame. */
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

	public int getHalfExtent() { return hx >> 12; }

	/** Orientation entry, row-major (0..8), Q12. */
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

	/** Sets angular velocity (Q12 radians/frame) and derives momentum from it. */
	public void setAngularVelocity(int ax, int ay, int az) {
		this.wx = ax; this.wy = ay; this.wz = az;
		recomputeMomentum();
		wake();
	}

	/** Instant positional nudge (used for kinematic character pushes). */
	public void nudge(int dxUnits, int dyUnits, int dzUnits) {
		this.px += dxUnits << 12;
		this.py += dyUnits << 12;
		this.pz += dzUnits << 12;
		// a push also becomes velocity, otherwise the cube would not slide
		this.vx += dxUnits << 12;
		this.vy += dyUnits << 12;
		this.vz += dzUnits << 12;
		wake();
	}

	/**
	 * Kinematic placement while held: follows a target center point and
	 * keeps an axis aligned orientation. The frame to frame delta becomes
	 * the body velocity so a throw inherits the carry motion.
	 */
	public void moveKinematic(int centerX, int centerY, int centerZ) {
		int nx = centerX << 12, ny = centerY << 12, nz = centerZ << 12;
		this.vx = nx - px;
		this.vy = ny - py;
		this.vz = nz - pz;
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

	/**
	 * Kinematic placement while held with an explicit camera relative
	 * orientation (row-major 4x4 as produced by Transform). The carried
	 * cube turns with the camera; velocities stay zero because the hand
	 * target is re-derived every frame and the throw adds its own speed.
	 */
	public void setKinematicPose(int centerX, int centerY, int centerZ, float[] m) {
		// The frame to frame hand motion, kept apart from the simulated
		// velocity: a carried cube is an immovable obstacle for other cubes,
		// but a swipe must still knock them away (see velX/bodySpeed).
		this.kvx = (centerX << 12) - this.px;
		this.kvy = (centerY << 12) - this.py;
		this.kvz = (centerZ << 12) - this.pz;
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
		fixMatrix();
		recomputeWorldInertia();
		recomputeMomentum();
		wake();
		computeVertices();
	}

	/**
	 * Applies a portal warp matrix (row-major float[16], as produced by
	 * PortalManager.getPortalTransform) to position, velocities and the
	 * orientation frame.
	 */
	public void warp(float[] m) {
		float x = px / (float) F, y = py / (float) F, z = pz / (float) F;
		px = q(m[0] * x + m[1] * y + m[2] * z + m[3]);
		py = q(m[4] * x + m[5] * y + m[6] * z + m[7]);
		pz = q(m[8] * x + m[9] * y + m[10] * z + m[11]);

		int[] wv = warpVector(m, vx, vy, vz);
		vx = wv[0]; vy = wv[1]; vz = wv[2];
		wv = warpVector(m, wx, wy, wz);
		wx = wv[0]; wy = wv[1]; wz = wv[2];

		// Rotate the three orientation columns (axes).
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

	/**
	 * Advances the body by one frame.
	 *
	 * @param colliders meshes of the current and neighbouring rooms
	 * @param count     number of valid entries
	 * @param world     test world collisions (false while crossing a portal
	 *                  opening)
	 */
	public void step(Collider[] colliders, int count, boolean world) {
		if(sleeping) {
			vx = vy = vz = 0;
			wx = wy = wz = 0;
			lx = ly = lz = 0;
			computeVertices();
			this.energy = 0;
			return;
		}

		// Gravity plus portalDS style velocity damping, expressed as forces.
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

		// Safety clamps run on every frame (not only contact frames): a
		// pathological impact must never leave a runaway spin behind.
		clampVelocity();

		// A body held up by another body is only at rest once the pair pass
		// has cancelled the gravity this step could not see, so its sleep
		// bookkeeping is skipped here and redone by collideBodies (which reads
		// bodySupport from the pass that just ran). Every other body is
		// settled by now: its world contacts were solved inside this step.
		if(!bodySupport) updateSleep();

		computeVertices();
	}

	/**
	 * Rest detection for a supported body: kinetic energy that stays low for
	 * SLEEP_TIME consecutive frames puts it to sleep. Only a body supported
	 * from below may rest - a body held up by another body counts as supported
	 * (see recordSupport), otherwise a stack of cubes could never come to rest
	 * and would keep re-resolving its own weight every frame.
	 */
	private void updateSleep() {
		energy = mul(vx, vx) + mul(vy, vy) + mul(vz, vz)
				+ mul(wx, wx) + mul(wy, wy) + mul(wz, wz);
		if(!groundContact && !bodySupport) {
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
	}

	private void integrate(int dt, int fx, int fy, int fz, int mx, int my, int mz) {
		// position += v * dt
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

		// v += f * dt / mass
		vx += divQ(mul(fx, dt), mass);
		vy += divQ(mul(fy, dt), mass);
		vz += divQ(mul(fz, dt), mass);

		// L += moment * dt
		lx += mul(mx, dt);
		ly += mul(my, dt);
		lz += mul(mz, dt);

		fixMatrix();
		recomputeWorldInertia();
		wx = eval24X(invIWorld, lx, ly, lz);
		wy = eval24Y(invIWorld, lx, ly, lz);
		wz = eval24Z(invIWorld, lx, ly, lz);
	}

	/**
	 * Local inertia of a box with half extents h: I = m/3(h2+h3).
	 * The inverse tensor is stored in Q24 because at world cube scale (half
	 * extent ~500 units) its Q12 value is below the fixed point resolution;
	 * the positive tensor used for angular momentum stays Q12.
	 */
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

	/** invIWorld (Q24) = R * invILocal * R^T */
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

	/** Evaluates a Q24 3x3 matrix against a Q12 vector, result Q12. */
	private static int eval24X(int[] m, int x, int y, int z) {
		return (int) (((long) m[0] * x + (long) m[1] * y + (long) m[2] * z) >> 24);
	}
	private static int eval24Y(int[] m, int x, int y, int z) {
		return (int) (((long) m[3] * x + (long) m[4] * y + (long) m[5] * z) >> 24);
	}
	private static int eval24Z(int[] m, int x, int y, int z) {
		return (int) (((long) m[6] * x + (long) m[7] * y + (long) m[8] * z) >> 24);
	}

	/** L = Iworld (Q12, built from il0..il8) * w */
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

	/** Gram-Schmidt re-orthonormalization of the three axis columns. */
	private void fixMatrix() {
		int xx = r[0], xy = r[3], xz = r[6];
		int yx = r[1], yy = r[4], yz = r[7];

		// Stable Gram-Schmidt: normalize the x column, project the y column
		// onto it, then rebuild z as x cross y. The cross product makes the
		// frame exact even when the first order rotation update has driven
		// two raw columns nearly parallel (large per-frame spins after a
		// corner impact), where the old three-projection form degenerated.
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

					// world space polygon
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

					// A mesh edge (most often a wall corner) can poke straight
					// into the middle of a box face without any box vertex
					// being anywhere near it - typical whenever the box is
					// rotated relative to the corner it hits. The vertex-based
					// tests below can't see that relationship at all, so check
					// it explicitly, once per polygon edge, against the box's
					// own faces.
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

							// vertices only touch a face once they reach the
							// plane (a wide free-side band would act like an
							// invisible shell); a deep penetration is kept
							// on purpose so the substep rollback can recover
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
								int enx, eny, enz;
								if(s > 0) {
									enx = ((qx - bx2) << 12) / s;
									eny = ((qy - by2) << 12) / s;
									enz = ((qz - bz2) << 12) / s;
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
				// Reject a "penetrating" candidate whose face is much farther
				// from the vertex than the closest face: the vertex then lies
				// past the solid (e.g. beyond the far side of a column), and
				// this contact would push/spin the box about the wrong axis.
				if(bestGap[idx] < 0 && -bestGap[idx] > mg) continue;
				addContact(vq[k * 3], vq[k * 3 + 1], vq[k * 3 + 2],
						bestNX[idx], bestNY[idx], bestNZ[idx], bestPen[idx]);
			}
		}

		// emitted after the vertex contacts on purpose: those are the
		// reliable, well-established contacts (actual box vertices), and
		// should never be starved out of the shared contact budget by the
		// supplementary edge-vs-face contacts below
		for(int s = 0; s < edgeCount; s++) {
			addContact(edgeCX[s], edgeCY[s], edgeCZ[s], edgeNX[s], edgeNY[s], edgeNZ[s], edgePen[s]);
		}
	}

	/**
	 * Keeps up to CORNER_SLOTS simultaneous contacts for one box vertex,
	 * instead of only the single deepest feature. A candidate whose normal
	 * points in essentially the same direction as one already kept (dot
	 * product at or above DUPLICATE_NORMAL_DOT) is treated as the same
	 * surface: only the deeper of the two survives. A candidate whose normal
	 * is meaningfully different (a real corner: two near-perpendicular
	 * walls touching the same vertex) is kept as an additional, independent
	 * contact, so the solver enforces both constraints in the same frame
	 * instead of alternating between them frame to frame - which is what
	 * was producing the corner jitter/launch.
	 */
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

	/**
	 * Tests one mesh polygon edge against the box's own six faces, in the
	 * box's local axis-aligned frame. This is the mirror image of the
	 * box-vertex-vs-mesh-face test above: it catches a mesh edge (most
	 * often a wall or column corner) poking into the middle of a flat box
	 * face, which the vertex-based tests never see because none of the
	 * box's 8 vertices need be anywhere near the mesh for that to happen -
	 * it only takes the box being rotated relative to the corner it hits.
	 * Left undetected, that penetration can grow well past
	 * PENETRATION_THRESHOLD before any contact exists at all, and the
	 * eventual single, deep correction is what shows up as a random
	 * "launch" at corners.
	 *
	 * The segment is sampled at the two endpoints, at every point where it
	 * crosses one of the box's six face planes (at most 6 more points), and
	 * at the point on the segment closest to the box center. The last one
	 * is what catches the common "column edge buried in the middle of a
	 * box face" case: that deepest point is neither an endpoint nor a
	 * face-plane crossing, so without it a rotated box could sit on a
	 * column corner with no contact at all until the overlap was already
	 * far beyond what the substep rollback can fix, and the resulting one
	 * shot correction was a launch.
	 */
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

		// cheap reject: does the edge's local bounding box even reach the
		// margin-inflated box on every axis?
		if(Math.max(l0x, l1x) < -phx - CONTACT_MARGIN || Math.min(l0x, l1x) > phx + CONTACT_MARGIN) return;
		if(Math.max(l0y, l1y) < -phy - CONTACT_MARGIN || Math.min(l0y, l1y) > phy + CONTACT_MARGIN) return;
		if(Math.max(l0z, l1z) < -phz - CONTACT_MARGIN || Math.min(l0z, l1z) > phz + CONTACT_MARGIN) return;

		int dLX = l1x - l0x, dLY = l1y - l0y, dLZ = l1z - l0z;

		// Parameter (Q14, 0..16384 spanning the whole segment) of the point
		// on the segment closest to the box center. For a segment passing
		// through the box this is also the deepest point of the overlap -
		// the one that matters most for detecting a column edge buried in
		// the middle of a box face.
		int tCenter = 0;
		long dd = (long) dLX * dLX + (long) dLY * dLY + (long) dLZ * dLZ;
		if(dd != 0) {
			long num = -((long) l0x * dLX + (long) l0y * dLY + (long) l0z * dLZ);
			tCenter = (int) ((num << 14) / dd);
			if(tCenter < 0) tCenter = 0;
			else if(tCenter > 16384) tCenter = 16384;
		}

		// candidate parameters, Q14 (0..16384 spans the whole segment):
		// both endpoints, the closest point to the box center, and every
		// point where the edge crosses one of the box's six face planes;
		// -1 marks "doesn't cross" (parallel)
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

			// an axis only counts as the exit face if the point actually
			// falls within the box's footprint on the OTHER two axes;
			// otherwise this point is near a box edge/corner rather than
			// cleanly on one face, and is left to the vertex-based tests
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

		// The normal is the direction the BOX must move to stop containing
		// this mesh point - i.e. away from it, not toward it. If the point
		// sits on the box's local +axis side, the box has to retreat toward
		// -axis to uncover it (moving further +axis only buries it deeper),
		// so the sign is the OPPOSITE of the point's own local sign.
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

	/**
	 * Keeps up to EDGE_CONTACT_SLOTS simultaneous, direction-distinct
	 * mesh-edge-vs-box-face candidates for the whole box, exactly the way
	 * addCandidate does per vertex, and for the same reason: without this,
	 * every seam edge between adjacent polygons of the same wall or floor
	 * would register its own near-duplicate contact and, since
	 * addEdgeFaceContact is called for every edge of every nearby polygon
	 * while the box's own (reliable) vertex contacts aren't emitted until
	 * the very end of collideWorld, those duplicates could fill the shared
	 * contact budget before the real support contacts ever get a slot.
	 */
	private void addEdgeCandidate(int cx, int cy, int cz, int nx, int ny, int nz, int gap, int pen) {
		for(int s = 0; s < edgeCount; s++) {
			int dot = mul(nx, edgeNX[s]) + mul(ny, edgeNY[s]) + mul(nz, edgeNZ[s]);
			// Only merge same-normal candidates that are the SAME contact
			// (e.g. the seam edge of two coplanar wall polygons, which is
			// tested once per polygon). Two same-normal contacts at different
			// positions are a genuine two-point manifold (e.g. the two corner
			// edges of a column against one box face): collapsing them into
			// one point moved the impulse off the contact centroid and spun
			// the box about the wrong axis on a straight-on hit.
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

	private void applyImpulses() {
		for(int i = 0; i < numContacts; i++) {
			accN[i] = 0; accT[i] = 0; vbias[i] = 0;
		}

		// Sequential impulses with accumulated magnitudes: several Gauss-Seidel
		// sweeps let a multi-point face contact converge; without them a resting
		// box gets four independent impulses and tumbles/jitters.
		// Prepass: restitution targets are fixed once from the approach
		// velocities before any impulse is applied, otherwise targets
		// computed mid-sweep are mutually inconsistent across contacts.
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

				// relative velocity at the contact point: v + w x r
				int crx = mul(wy, rz) - mul(wz, ry);
				int cry = mul(wz, rx) - mul(wx, rz);
				int crz = mul(wx, ry) - mul(wy, rx);
				int rvx = vx + crx, rvy = vy + cry, rvz = vz + crz;
				int vn = mul(rvx, nx) + mul(rvy, ny) + mul(rvz, nz);

				// K_n = 1/m + ((I^-1 (r x n)) x r) . n
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

				// Normal impulse; the restitution target comes from the
				// prepass and is enforced by every sweep so later sweeps do
				// not eat the bounce.
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

				// Friction: tangent relative velocity, Coulomb clamp mu * jn
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

		// De-penetration runs as its own position-only pass.
		correctPositions();
	}

	/**
	 * Sequential position projection over every contact. Each iteration
	 * moves the center and rotates the body just like the velocity impulses
	 * do, but touches positions only, so it never injects momentum. Splitting
	 * the correction across all contacts (instead of one big snap on the
	 * deepest point) prevents the corner-jam energy pump.
	 */
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

				// angular position step dq = I^-1 (r x n * dp)
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

	/** R += skew(q) * R in place (same first order update as integration). */
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
		// momentum must stay consistent with the clamped velocities
		recomputeMomentum();
	}

	// ===================== body vs body =====================

	/**
	 * Cube against cube.
	 *
	 * The world pass above only ever sees one box against the static triangle
	 * soup, so two dynamic boxes are resolved here, in a pass over every pair
	 * that runs once per frame after all bodies have stepped:
	 *
	 *  - a separating axis test over the 15 box-box axes (6 face normals plus
	 *    9 edge cross products) rejects non-touching pairs and picks the axis
	 *    of least penetration,
	 *  - a face axis resolves by clipping the incident face against the
	 *    reference face (Sutherland-Hodgman, deepest PAIR_MAX_CONTACTS kept),
	 *  - an edge axis resolves to the single closest point pair of the two
	 *    extreme edges,
	 *  - contacts are solved with sequential impulses applied to BOTH bodies
	 *    (equal and opposite, so linear and angular momentum are conserved)
	 *    plus a position projection that re-derives the penetration from local
	 *    contact anchors every iteration, so a stack converges instead of
	 *    settling at a fixed residual overlap.
	 *
	 * A carried (kinematic) body and a sleeping body take part with zero
	 * inverse mass: they act as immovable obstacles that still lend their own
	 * velocity to the contact. A sleeper is woken by a hard impact or by a
	 * moving neighbour, and a body resting on another body may sleep as well
	 * (bodySupport), otherwise a stack could never come to rest.
	 */

	/** Deepest contacts kept for one pair: a clipped face manifold has at
	 *  most four meaningful support points, and every pair is solved several
	 *  times per frame. */
	private static final int PAIR_MAX_CONTACTS = 4;
	/** Clipping keeps a point this far outside the reference face footprint
	 *  (Q12 units) so a contact exactly on a face edge is not lost to
	 *  truncation. */
	private static final int CLIP_SLOP = 4 << 12;
	/** An edge-edge axis wins over the best face axis only when it is this
	 *  much shallower (Q12 units): a face manifold is far more stable, so
	 *  near ties go to the face case. */
	private static final int EDGE_AXIS_BIAS = 8 << 12;
	private static final int PAIR_IMPULSE_ITERATIONS = 8;
	private static final int PAIR_POSITION_ITERATIONS = 6;
	/** Walks over the whole pair list per frame, see collideBodies. */
	private static final int PAIR_ROUNDS = 3;
	/** Cube against cube: less bouncy than the world material (a stack must
	 *  not ping-pong) but just as grippy, so cubes can rest on each other. */
	private static final int BODY_RESTITUTION = 614;   // 0.15
	private static final int BODY_FRICTION = 4096;     // 1.0
	/** A sleeping body is woken by a contact closing faster than this. */
	private static final int WAKE_SPEED = 60 << 12;
	/** A sleeping body is also woken when the body it leans on moves faster
	 *  than this, otherwise it would hang in the air while its support
	 *  slides away. */
	private static final int WAKE_NEIGHBOUR_SPEED = 48 << 12;
	/** Cross product length (Q24) below which two box axes count as parallel
	 *  and their edge-edge axis is skipped (~0.9 degrees): the face axes
	 *  already separate boxes aligned that closely. */
	private static final int PARALLEL_EPS = 1 << 18;
	/** Same-normal pair contacts closer than this (squared, Q12) are one
	 *  contact; clipping can emit a corner twice when an incident edge lies
	 *  exactly on a clip plane. */
	private static final long PAIR_MERGE_DIST2 = (long) (8 << 12) * (8 << 12);
	private static final int PAIR_MERGE_DOT = F * 3 / 4;
	/** Upward component a pair contact needs to count as support. Wider than
	 *  the world pass' ground cone (F * 7 / 10): a cube balanced on the seam
	 *  between two cubes is held up by contacts whose normals are tilted well
	 *  past 45 degrees, and a body that is not recognised as supported may
	 *  never sleep - it would keep re-resolving its own weight forever. */
	private static final int SUPPORT_UP = F / 2;
	/** Position projection strength by manifold size. The penetration is
	 *  re-derived from the local anchors on every iteration, so a single
	 *  contact needs a much bigger step than a four point face manifold to
	 *  converge in the same number of sweeps; without this an edge-edge
	 *  impact keeps a third of its depth. */
	private static final int[] PAIR_BETA = {F / 2, F * 3 / 8, F * 5 / 16, F * 3 / 16};
	/** Zero inverse inertia, for bodies that must not rotate (carried,
	 *  asleep). */
	private static final int[] ZERO_I = new int[9];
	/** Clipping polygon buffers: 4 incident corners plus at most one point
	 *  per clip plane. */
	private static final int CLIP_MAX = 10;

	// ---- pair scratch: exactly one pair is generated and solved at a time ----
	private static int pairContacts;
	private static final int[] ppx = new int[PAIR_MAX_CONTACTS];
	private static final int[] ppy = new int[PAIR_MAX_CONTACTS];
	private static final int[] ppz = new int[PAIR_MAX_CONTACTS];
	private static final int[] pnx = new int[PAIR_MAX_CONTACTS];
	private static final int[] pny = new int[PAIR_MAX_CONTACTS];
	private static final int[] pnz = new int[PAIR_MAX_CONTACTS];
	private static final int[] ppen = new int[PAIR_MAX_CONTACTS];
	/** Contact anchors in each body's local frame, see addPairContact. */
	private static final int[] pral = new int[PAIR_MAX_CONTACTS * 3];
	private static final int[] prbl = new int[PAIR_MAX_CONTACTS * 3];
	private static final int[] paccN = new int[PAIR_MAX_CONTACTS];
	private static final int[] paccT = new int[PAIR_MAX_CONTACTS];
	private static final int[] pbias = new int[PAIR_MAX_CONTACTS];
	private static final int[] clipX = new int[CLIP_MAX];
	private static final int[] clipY = new int[CLIP_MAX];
	private static final int[] clipZ = new int[CLIP_MAX];
	private static final int[] clipOX = new int[CLIP_MAX];
	private static final int[] clipOY = new int[CLIP_MAX];
	private static final int[] clipOZ = new int[CLIP_MAX];
	// closest point pair scratch, see closestPairPoints
	private static int cp1x, cp1y, cp1z, cp2x, cp2y, cp2z;

	/**
	 * Resolves every box-box pair for one frame. Called after all bodies
	 * stepped against the world, so a cube lands on, slides off, pushes and
	 * stacks on another cube instead of passing through it.
	 *
	 * The whole pair list is walked PAIR_ROUNDS times: contacts are generated
	 * from scratch each round, so a round sees what the previous one moved and
	 * support propagates from the ground up. Within a pair, a body the world or
	 * its own support holds in place gives up its share of the response (see
	 * pairHeld), which is what lets a stack come to rest.
	 *
	 * @param bodies every simulated box in the level (null entries allowed)
	 * @param count  number of valid entries
	 */
	public static void collideBodies(RigidBody[] bodies, int count) {
		for(int i = 0; i < count; i++) {
			RigidBody x = bodies[i];
			if(x == null) continue;
			x.prevSupport = x.supportBody;
			x.supportBody = null;
			x.bodySupport = false;
			x.supportNX = x.supportNY = x.supportNZ = 0;
			x.pairTouched = false;
		}

		for(int round = 0; round < PAIR_ROUNDS; round++) {
			for(int i = 0; i < count; i++) {
				RigidBody a = bodies[i];
				if(a == null) continue;
				for(int j = i + 1; j < count; j++) {
					RigidBody b = bodies[j];
					if(b == null) continue;
					collidePair(a, b);
				}
			}
		}

		for(int i = 0; i < count; i++) {
			RigidBody x = bodies[i];
			if(x == null) continue;
			// support slid away while it slept: fall again instead of floating
			if(x.sleeping && x.prevSupport != null && x.supportBody == null) x.wake();
			if(x.pairTouched) {
				x.fixMatrix();
				x.recomputeWorldInertia();
				x.clampVelocity();
				x.computeVertices();
			}
			// Rest detection for a cube held up by another cube has to happen
			// here: its own step() still saw the gravity this pass cancelled,
			// so a stack would look permanently restless and never sleep.
			if(x.bodySupport && !x.kinematic) x.updateSleep();
		}
	}

	private static void collidePair(RigidBody a, RigidBody b) {
		int count = generateContacts(a, b);
		if(count == 0) return;

		recordSupport(a, b, count);

		// Wake a sleeper before deciding who is static, otherwise a carried
		// cube would sweep straight through a resting one (both would count as
		// immovable and the pair would be skipped).
		if(a.sleeping && !b.sleeping && shouldWake(a, b, count)) a.wake();
		if(b.sleeping && !a.sleeping && shouldWake(b, a, count)) b.wake();

		boolean aStatic = a.kinematic || a.sleeping;
		boolean bStatic = b.kinematic || b.sleeping;
		if(aStatic && bStatic) return;

		solvePair(a, b, aStatic, bStatic, count);
		a.pairTouched = !aStatic;
		b.pairTouched = !bStatic;
	}

	// ---- box accessors, so the axis code below stays readable ----

	/** World space box axis i (column i of the row-major orientation). */
	private static int axisX(RigidBody b, int i) { return b.r[i]; }
	private static int axisY(RigidBody b, int i) { return b.r[3 + i]; }
	private static int axisZ(RigidBody b, int i) { return b.r[6 + i]; }
	private static int halfExt(RigidBody b, int i) {
		return i == 0 ? b.hx : (i == 1 ? b.hy : b.hz);
	}

	/** Velocity a body lends to a contact: a carried cube moves with the hand
	 *  even though its own simulated velocity is kept at zero. */
	private static int velX(RigidBody x) { return x.kinematic ? x.kvx : x.vx; }
	private static int velY(RigidBody x) { return x.kinematic ? x.kvy : x.vy; }
	private static int velZ(RigidBody x) { return x.kinematic ? x.kvz : x.vz; }

	/** Fastest point speed of a body (translation plus spin at the corner). */
	private static int bodySpeed(RigidBody x) {
		return norm3(velX(x), velY(x), velZ(x))
				+ mul(norm3(x.wx, x.wy, x.wz), x.cornerRadius);
	}

	private static int clamp14(int t) {
		if(t < 0) return 0;
		if(t > 16384) return 16384;
		return t;
	}

	/** Overlap of the two boxes along a unit axis (Q12); <= 0 separates. */
	private static int overlapOnAxis(RigidBody a, RigidBody b, int lx, int ly, int lz) {
		int pa = 0, pb = 0;
		for(int i = 0; i < 3; i++) {
			int d = mul(axisX(a, i), lx) + mul(axisY(a, i), ly) + mul(axisZ(a, i), lz);
			pa += mul(halfExt(a, i), d < 0 ? -d : d);
			d = mul(axisX(b, i), lx) + mul(axisY(b, i), ly) + mul(axisZ(b, i), lz);
			pb += mul(halfExt(b, i), d < 0 ? -d : d);
		}
		int d = mul(b.px - a.px, lx) + mul(b.py - a.py, ly) + mul(b.pz - a.pz, lz);
		return pa + pb - (d < 0 ? -d : d);
	}

	/**
	 * Separating axis test plus manifold generation for one pair of boxes.
	 * Fills the pair scratch and returns the number of contacts.
	 */
	private static int generateContacts(RigidBody a, RigidBody b) {
		pairContacts = 0;

		int m = CONTACT_MARGIN;
		if(a.boxMaxX + m < b.boxMinX || b.boxMaxX + m < a.boxMinX) return 0;
		if(a.boxMaxY + m < b.boxMinY || b.boxMaxY + m < a.boxMinY) return 0;
		if(a.boxMaxZ + m < b.boxMinZ || b.boxMaxZ + m < a.boxMinZ) return 0;

		int dx = b.px - a.px, dy = b.py - a.py, dz = b.pz - a.pz;

		int best = Integer.MAX_VALUE;      // least penetration over the face axes
		int bestEdge = Integer.MAX_VALUE;  // least penetration over the edge axes
		boolean faceIsA = true;
		int faceK = 0, faceS = 1;
		int edgeI = -1, edgeJ = -1, edgeNX = 0, edgeNY = 0, edgeNZ = 0;

		// six face axes; every axis normal is turned to point from a to b
		for(int k = 0; k < 3; k++) {
			for(int which = 0; which < 2; which++) {
				RigidBody ref = which == 0 ? a : b;
				int nx = axisX(ref, k), ny = axisY(ref, k), nz = axisZ(ref, k);
				if(mul(dx, nx) + mul(dy, ny) + mul(dz, nz) < 0) {
					nx = -nx; ny = -ny; nz = -nz;
				}
				int ov = overlapOnAxis(a, b, nx, ny, nz);
				if(ov <= 0) return 0;
				if(ov < best) {
					best = ov;
					faceIsA = which == 0;
					faceK = k;
					// the reference face normal must point from the reference
					// box toward the other box: a's own normal, or b's negated
					int tx = faceIsA ? nx : -nx;
					int ty = faceIsA ? ny : -ny;
					int tz = faceIsA ? nz : -nz;
					int dot = mul(tx, axisX(ref, k)) + mul(ty, axisY(ref, k))
							+ mul(tz, axisZ(ref, k));
					faceS = dot >= 0 ? 1 : -1;
				}
			}
		}

		// nine edge cross product axes
		for(int i = 0; i < 3; i++) {
			int pax = axisX(a, i), pay = axisY(a, i), paz = axisZ(a, i);
			for(int j = 0; j < 3; j++) {
				int pbx = axisX(b, j), pby = axisY(b, j), pbz = axisZ(b, j);
				long cx = (long) pay * pbz - (long) paz * pby;
				long cy = (long) paz * pbx - (long) pax * pbz;
				long cz = (long) pax * pby - (long) pay * pbx;
				int ln = isqrt(cx * cx + cy * cy + cz * cz);
				if(ln < PARALLEL_EPS) continue;
				int nx = (int) ((cx << 12) / ln);
				int ny = (int) ((cy << 12) / ln);
				int nz = (int) ((cz << 12) / ln);
				if(mul(dx, nx) + mul(dy, ny) + mul(dz, nz) < 0) {
					nx = -nx; ny = -ny; nz = -nz;
				}
				int ov = overlapOnAxis(a, b, nx, ny, nz);
				if(ov <= 0) return 0;
				if(ov < bestEdge) {
					bestEdge = ov;
					edgeI = i; edgeJ = j;
					edgeNX = nx; edgeNY = ny; edgeNZ = nz;
				}
			}
		}

		if(edgeI >= 0 && bestEdge < best - EDGE_AXIS_BIAS) {
			addEdgeEdgeContact(a, b, edgeI, edgeJ, edgeNX, edgeNY, edgeNZ, bestEdge);
		} else {
			RigidBody ref = faceIsA ? a : b;
			RigidBody inc = faceIsA ? b : a;
			clipFacePair(a, b, ref, inc, faceK, faceS, faceIsA);
		}
		return pairContacts;
	}

	/**
	 * Face manifold: clip the incident face of inc against the face of ref
	 * whose outward normal is s * ref.axis[k] (that normal points from ref
	 * toward inc). The stored contact normals always point from a to b.
	 */
	private static void clipFacePair(RigidBody a, RigidBody b,
			RigidBody ref, RigidBody inc, int k, int s, boolean refIsA) {
		int nx = s * axisX(ref, k), ny = s * axisY(ref, k), nz = s * axisZ(ref, k);
		int u = (k + 1) % 3, v = (k + 2) % 3;
		int ux = axisX(ref, u), uy = axisY(ref, u), uz = axisZ(ref, u);
		int vx = axisX(ref, v), vy = axisY(ref, v), vz = axisZ(ref, v);
		int hn = halfExt(ref, k), hu = halfExt(ref, u), hv = halfExt(ref, v);

		// incident face: the face of inc most anti-parallel to the reference
		// normal
		int bestJ = 0, bestS = -1, bestDot = Integer.MAX_VALUE;
		for(int j = 0; j < 3; j++) {
			int d = mul(axisX(inc, j), nx) + mul(axisY(inc, j), ny)
					+ mul(axisZ(inc, j), nz);
			if(-d < bestDot) { bestDot = -d; bestJ = j; bestS = -1; }
			if(d < bestDot) { bestDot = d; bestJ = j; bestS = 1; }
		}
		int iu = (bestJ + 1) % 3, iv = (bestJ + 2) % 3;
		int hj = halfExt(inc, bestJ), hu2 = halfExt(inc, iu), hv2 = halfExt(inc, iv);
		int icx = inc.px + mul(bestS * hj, axisX(inc, bestJ));
		int icy = inc.py + mul(bestS * hj, axisY(inc, bestJ));
		int icz = inc.pz + mul(bestS * hj, axisZ(inc, bestJ));

		// the four incident face corners, in cyclic order
		int cnt = 0;
		for(int c = 0; c < 4; c++) {
			int su = (c == 0 || c == 3) ? 1 : -1;
			int sv = (c == 0 || c == 1) ? 1 : -1;
			clipX[cnt] = icx + mul(su * hu2, axisX(inc, iu)) + mul(sv * hv2, axisX(inc, iv));
			clipY[cnt] = icy + mul(su * hu2, axisY(inc, iu)) + mul(sv * hv2, axisY(inc, iv));
			clipZ[cnt] = icz + mul(su * hu2, axisZ(inc, iu)) + mul(sv * hv2, axisZ(inc, iv));
			cnt++;
		}

		cnt = clipPolygon(ref, ux, uy, uz, hu, cnt);
		if(cnt == 0) return;
		cnt = clipPolygon(ref, -ux, -uy, -uz, hu, cnt);
		if(cnt == 0) return;
		cnt = clipPolygon(ref, vx, vy, vz, hv, cnt);
		if(cnt == 0) return;
		cnt = clipPolygon(ref, -vx, -vy, -vz, hv, cnt);
		if(cnt == 0) return;

		int cnx = refIsA ? nx : -nx;
		int cny = refIsA ? ny : -ny;
		int cnz = refIsA ? nz : -nz;
		for(int i = 0; i < cnt; i++) {
			// depth below the reference face plane
			int d = mul(clipX[i] - ref.px, nx) + mul(clipY[i] - ref.py, ny)
					+ mul(clipZ[i] - ref.pz, nz);
			int pen = hn - d;
			if(pen < -(SURFACE_TOUCH << 12)) continue;
			// contact halfway between the incident point and the reference
			// plane, so neither body owns the whole lever arm
			addPairContact(a, b,
					clipX[i] + mul(nx, pen >> 1),
					clipY[i] + mul(ny, pen >> 1),
					clipZ[i] + mul(nz, pen >> 1),
					cnx, cny, cnz, pen);
		}
	}

	/**
	 * Sutherland-Hodgman clip of the scratch polygon against one side plane of
	 * the reference face: keeps the points with
	 * (p - ref.center) . axis <= limit (+ slop).
	 */
	private static int clipPolygon(RigidBody ref, int ax, int ay, int az, int limit, int cnt) {
		int out = 0;
		for(int i = 0; i < cnt && out < CLIP_MAX; i++) {
			int j = i + 1 < cnt ? i + 1 : 0;
			int ds = mul(clipX[i] - ref.px, ax) + mul(clipY[i] - ref.py, ay)
					+ mul(clipZ[i] - ref.pz, az) - limit;
			int de = mul(clipX[j] - ref.px, ax) + mul(clipY[j] - ref.py, ay)
					+ mul(clipZ[j] - ref.pz, az) - limit;
			boolean sIn = ds <= CLIP_SLOP;
			boolean eIn = de <= CLIP_SLOP;
			if(eIn) {
				if(!sIn && out < CLIP_MAX) {
					ds -= CLIP_SLOP; de -= CLIP_SLOP;
					int t = divQ(ds, ds - de);
					clipOX[out] = clipX[i] + mul(clipX[j] - clipX[i], t);
					clipOY[out] = clipY[i] + mul(clipY[j] - clipY[i], t);
					clipOZ[out] = clipZ[i] + mul(clipZ[j] - clipZ[i], t);
					out++;
				}
				if(out < CLIP_MAX) {
					clipOX[out] = clipX[j]; clipOY[out] = clipY[j]; clipOZ[out] = clipZ[j];
					out++;
				}
			} else if(sIn && out < CLIP_MAX) {
				ds -= CLIP_SLOP; de -= CLIP_SLOP;
				int t = divQ(ds, ds - de);
				clipOX[out] = clipX[i] + mul(clipX[j] - clipX[i], t);
				clipOY[out] = clipY[i] + mul(clipY[j] - clipY[i], t);
				clipOZ[out] = clipZ[i] + mul(clipZ[j] - clipZ[i], t);
				out++;
			}
		}
		for(int i = 0; i < out; i++) {
			clipX[i] = clipOX[i]; clipY[i] = clipOY[i]; clipZ[i] = clipOZ[i];
		}
		return out;
	}

	/**
	 * Edge manifold: the extreme edge of a along the separation axis against
	 * the extreme edge of b along its opposite. The closest point pair of the
	 * two segments is the contact.
	 */
	private static void addEdgeEdgeContact(RigidBody a, RigidBody b,
			int i, int j, int nx, int ny, int nz, int pen) {
		int u = (i + 1) % 3, v = (i + 2) % 3;
		int du = mul(axisX(a, u), nx) + mul(axisY(a, u), ny) + mul(axisZ(a, u), nz);
		int dv = mul(axisX(a, v), nx) + mul(axisY(a, v), ny) + mul(axisZ(a, v), nz);
		int su = du >= 0 ? 1 : -1, sv = dv >= 0 ? 1 : -1;
		int ax = a.px + mul(su * halfExt(a, u), axisX(a, u)) + mul(sv * halfExt(a, v), axisX(a, v));
		int ay = a.py + mul(su * halfExt(a, u), axisY(a, u)) + mul(sv * halfExt(a, v), axisY(a, v));
		int az = a.pz + mul(su * halfExt(a, u), axisZ(a, u)) + mul(sv * halfExt(a, v), axisZ(a, v));
		int ex = mul(halfExt(a, i), axisX(a, i));
		int ey = mul(halfExt(a, i), axisY(a, i));
		int ez = mul(halfExt(a, i), axisZ(a, i));

		u = (j + 1) % 3; v = (j + 2) % 3;
		du = mul(axisX(b, u), nx) + mul(axisY(b, u), ny) + mul(axisZ(b, u), nz);
		dv = mul(axisX(b, v), nx) + mul(axisY(b, v), ny) + mul(axisZ(b, v), nz);
		su = du <= 0 ? 1 : -1; sv = dv <= 0 ? 1 : -1;
		int bx = b.px + mul(su * halfExt(b, u), axisX(b, u)) + mul(sv * halfExt(b, v), axisX(b, v));
		int by = b.py + mul(su * halfExt(b, u), axisY(b, u)) + mul(sv * halfExt(b, v), axisY(b, v));
		int bz = b.pz + mul(su * halfExt(b, u), axisZ(b, u)) + mul(sv * halfExt(b, v), axisZ(b, v));
		ex = mul(halfExt(b, j), axisX(b, j));
		ey = mul(halfExt(b, j), axisY(b, j));
		ez = mul(halfExt(b, j), axisZ(b, j));

		closestPairPoints(ax - ex, ay - ey, az - ez, ax + ex, ay + ey, az + ez,
				bx - ex, by - ey, bz - ez, bx + ex, by + ey, bz + ez);

		addPairContact(a, b, (cp1x + cp2x) >> 1, (cp1y + cp2y) >> 1, (cp1z + cp2z) >> 1,
				nx, ny, nz, pen);
	}

	/**
	 * Closest points of two segments (all Q12), Ericson's segment/segment test
	 * evaluated in plain units with Q14 parameters: the Q12 products of a
	 * world-scale distance would overflow 64 bits otherwise. Results land in
	 * cp1* and cp2*.
	 */
	private static void closestPairPoints(int p1x, int p1y, int p1z,
			int p2x, int p2y, int p2z, int q1x, int q1y, int q1z,
			int q2x, int q2y, int q2z) {
		int d1x = (p2x - p1x) >> 12, d1y = (p2y - p1y) >> 12, d1z = (p2z - p1z) >> 12;
		int d2x = (q2x - q1x) >> 12, d2y = (q2y - q1y) >> 12, d2z = (q2z - q1z) >> 12;
		int rx = (p1x - q1x) >> 12, ry = (p1y - q1y) >> 12, rz = (p1z - q1z) >> 12;

		long a = (long) d1x * d1x + (long) d1y * d1y + (long) d1z * d1z;
		long e = (long) d2x * d2x + (long) d2y * d2y + (long) d2z * d2z;
		long f = (long) d2x * rx + (long) d2y * ry + (long) d2z * rz;
		int s = 0, t = 0;

		if(a <= 0 && e <= 0) {
			s = 0; t = 0;
		} else if(a <= 0) {
			s = 0;
			t = clamp14((int) ((f << 14) / e));
		} else {
			long c = (long) d1x * rx + (long) d1y * ry + (long) d1z * rz;
			if(e <= 0) {
				t = 0;
				s = clamp14((int) (-(c << 14) / a));
			} else {
				long bb = (long) d1x * d2x + (long) d1y * d2y + (long) d1z * d2z;
				long denom = a * e - bb * bb;
				s = denom != 0 ? clamp14((int) (((bb * f - c * e) << 14) / denom)) : 0;
				t = clamp14((int) ((bb * s + (f << 14)) / e));
				if(t == 0) {
					s = clamp14((int) (-(c << 14) / a));
				} else if(t == 16384) {
					s = clamp14((int) (((bb - c) << 14) / a));
				}
			}
		}

		cp1x = p1x + (int) (((long) (p2x - p1x) * s) >> 14);
		cp1y = p1y + (int) (((long) (p2y - p1y) * s) >> 14);
		cp1z = p1z + (int) (((long) (p2z - p1z) * s) >> 14);
		cp2x = q1x + (int) (((long) (q2x - q1x) * t) >> 14);
		cp2y = q1y + (int) (((long) (q2y - q1y) * t) >> 14);
		cp2z = q1z + (int) (((long) (q2z - q1z) * t) >> 14);
	}

	/** Stores one contact whose normal points from a to b, keeping the
	 *  PAIR_MAX_CONTACTS deepest points. */
	private static void addPairContact(RigidBody a, RigidBody b, int x, int y, int z,
			int nx, int ny, int nz, int pen) {
		for(int i = 0; i < pairContacts; i++) {
			int dot = mul(nx, pnx[i]) + mul(ny, pny[i]) + mul(nz, pnz[i]);
			if(dot < PAIR_MERGE_DOT) continue;
			long dx = (long) x - ppx[i];
			long dy = (long) y - ppy[i];
			long dz = (long) z - ppz[i];
			if(dx * dx + dy * dy + dz * dz <= PAIR_MERGE_DIST2) {
				if(pen > ppen[i]) ppen[i] = pen;
				return;
			}
		}

		int slot;
		if(pairContacts < PAIR_MAX_CONTACTS) {
			slot = pairContacts++;
		} else {
			slot = 0;
			for(int i = 1; i < PAIR_MAX_CONTACTS; i++) {
				if(ppen[i] < ppen[slot]) slot = i;
			}
			if(pen <= ppen[slot]) return;
		}
		ppx[slot] = x; ppy[slot] = y; ppz[slot] = z;
		pnx[slot] = nx; pny[slot] = ny; pnz[slot] = nz;
		ppen[slot] = pen;

		// Anchors in each body's local frame (local = R^T * world offset). The
		// position projection re-derives the current penetration from them, so
		// it converges while the bodies move instead of pushing out a stale
		// depth once per iteration.
		int wx = x - a.px, wy = y - a.py, wz = z - a.pz;
		pral[slot * 3] = mul(wx, a.r[0]) + mul(wy, a.r[3]) + mul(wz, a.r[6]);
		pral[slot * 3 + 1] = mul(wx, a.r[1]) + mul(wy, a.r[4]) + mul(wz, a.r[7]);
		pral[slot * 3 + 2] = mul(wx, a.r[2]) + mul(wy, a.r[5]) + mul(wz, a.r[8]);
		wx = x - b.px; wy = y - b.py; wz = z - b.pz;
		prbl[slot * 3] = mul(wx, b.r[0]) + mul(wy, b.r[3]) + mul(wz, b.r[6]);
		prbl[slot * 3 + 1] = mul(wx, b.r[1]) + mul(wy, b.r[4]) + mul(wz, b.r[7]);
		prbl[slot * 3 + 2] = mul(wx, b.r[2]) + mul(wy, b.r[5]) + mul(wz, b.r[8]);
	}

	/** Notes which body is held up by the other, so a stacked cube may sleep
	 *  exactly like one resting on the floor. */
	private static void recordSupport(RigidBody a, RigidBody b, int count) {
		for(int i = 0; i < count; i++) {
			if(pny[i] > SUPPORT_UP) {
				b.bodySupport = true;
				b.supportBody = a;
				// the support normal points out of the support into the body
				b.supportNX = pnx[i]; b.supportNY = pny[i]; b.supportNZ = pnz[i];
			} else if(pny[i] < -SUPPORT_UP) {
				a.bodySupport = true;
				a.supportBody = b;
				a.supportNX = -pnx[i]; a.supportNY = -pny[i]; a.supportNZ = -pnz[i];
			}
		}
	}

	/** True when the world geometry holds body x against a move along
	 *  (dx, dy, dz): one of the contacts its last step produced pushes back
	 *  the other way. */
	private static boolean worldBlocked(RigidBody x, int dx, int dy, int dz) {
		for(int i = 0; i < x.numContacts; i++) {
			if(mul(x.cnx[i], dx) + mul(x.cny[i], dy) + mul(x.cnz[i], dz) < 0) {
				return true;
			}
		}
		return false;
	}

	/** True when another body holds x up and the move would push x into that
	 *  support. Together with worldBlocked this keeps a stack from paying for
	 *  the pair above it by sinking into the pair below: two corrections that
	 *  each push a shared body the other way never converge. */
	private static boolean bodyBlocked(RigidBody x, int dx, int dy, int dz) {
		return x.bodySupport
				&& mul(x.supportNX, dx) + mul(x.supportNY, dy)
						+ mul(x.supportNZ, dz) < 0;
	}

	/** True when x cannot be moved along (dx, dy, dz) at all. */
	private static boolean blocked(RigidBody x, int dx, int dy, int dz) {
		return worldBlocked(x, dx, dy, dz) || bodyBlocked(x, dx, dy, dz);
	}

	/** Which of the two bodies must not take contact i: whoever the world or
	 *  its own support holds in place gives up its share, so the whole
	 *  response goes to the body that can actually move. A cube resting on the
	 *  floor then carries a stack instead of being squashed into the floor,
	 *  and the stack comes to rest instead of keeping the residual downward
	 *  velocity that the floor only answers on the next step.
	 *
	 *  When neither body could move at all - a carried cube pressing a cube
	 *  onto the floor, say - the hold is released again, because a pair of
	 *  immovable bodies has no solution and would stay interpenetrated.
	 *  The result lands in heldA/heldB. */
	private static boolean heldA, heldB;
	private static void pairHeld(RigidBody a, RigidBody b,
			boolean aStatic, boolean bStatic, int i) {
		heldA = !aStatic && blocked(a, -pnx[i], -pny[i], -pnz[i]);
		heldB = !bStatic && blocked(b, pnx[i], pny[i], pnz[i]);
		if((aStatic || heldA) && (bStatic || heldB)) {
			heldA = false;
			heldB = false;
		}
	}

	/** True when the sleeping body a must join the pair solve. */
	private static boolean shouldWake(RigidBody a, RigidBody b, int count) {
		int avx = velX(a), avy = velY(a), avz = velZ(a);
		int bvx = velX(b), bvy = velY(b), bvz = velZ(b);
		for(int i = 0; i < count; i++) {
			int rax = ppx[i] - a.px, ray = ppy[i] - a.py, raz = ppz[i] - a.pz;
			int rbx = ppx[i] - b.px, rby = ppy[i] - b.py, rbz = ppz[i] - b.pz;
			int vax = avx + mul(a.wy, raz) - mul(a.wz, ray);
			int vay = avy + mul(a.wz, rax) - mul(a.wx, raz);
			int vaz = avz + mul(a.wx, ray) - mul(a.wy, rax);
			int vbx = bvx + mul(b.wy, rbz) - mul(b.wz, rby);
			int vby = bvy + mul(b.wz, rbx) - mul(b.wx, rbz);
			int vbz = bvz + mul(b.wx, rby) - mul(b.wy, rbx);
			int vn = mul(vbx - vax, pnx[i]) + mul(vby - vay, pny[i]) + mul(vbz - vaz, pnz[i]);
			if(-vn > WAKE_SPEED) return true;
		}
		// the body it rests on is moving: hanging around would leave the
		// sleeper floating once its support slid away
		return bodySpeed(b) > WAKE_NEIGHBOUR_SPEED;
	}

	/** Penetration of contact i now, re-derived from the local anchors. */
	private static int currentPen(RigidBody a, RigidBody b, int i) {
		int ax = pral[i * 3], ay = pral[i * 3 + 1], az = pral[i * 3 + 2];
		int awx = a.px + mul(ax, a.r[0]) + mul(ay, a.r[1]) + mul(az, a.r[2]);
		int awy = a.py + mul(ax, a.r[3]) + mul(ay, a.r[4]) + mul(az, a.r[5]);
		int awz = a.pz + mul(ax, a.r[6]) + mul(ay, a.r[7]) + mul(az, a.r[8]);
		int bx = prbl[i * 3], by = prbl[i * 3 + 1], bz = prbl[i * 3 + 2];
		int bwx = b.px + mul(bx, b.r[0]) + mul(by, b.r[1]) + mul(bz, b.r[2]);
		int bwy = b.py + mul(bx, b.r[3]) + mul(by, b.r[4]) + mul(bz, b.r[5]);
		int bwz = b.pz + mul(bx, b.r[6]) + mul(by, b.r[7]) + mul(bz, b.r[8]);
		int sep = mul(bwx - awx, pnx[i]) + mul(bwy - awy, pny[i]) + mul(bwz - awz, pnz[i]);
		return ppen[i] - sep;
	}

	/** n . ((I^-1 (r x n)) x r) for one body: the angular part of the
	 *  effective mass K along a unit direction (invI is that body's world
	 *  inverse inertia tensor, zero for a body that must not rotate). */
	private static int pairEffectiveMass(int rx, int ry, int rz, int[] invI,
			int nx, int ny, int nz) {
		int rnx = mul(ry, nz) - mul(rz, ny);
		int rny = mul(rz, nx) - mul(rx, nz);
		int rnz = mul(rx, ny) - mul(ry, nx);
		int wx = eval24X(invI, rnx, rny, rnz);
		int wy = eval24Y(invI, rnx, rny, rnz);
		int wz = eval24Z(invI, rnx, rny, rnz);
		return mul(mul(wy, rz) - mul(wz, ry), nx)
				+ mul(mul(wz, rx) - mul(wx, rz), ny)
				+ mul(mul(wx, ry) - mul(wy, rx), nz);
	}

	/** World space anchor of contact i on body a (scratch: anchorX/Y/Z). */
	private static int anchorX, anchorY, anchorZ;
	private static void anchorOnA(RigidBody a, int i) {
		int lx = pral[i * 3], ly = pral[i * 3 + 1], lz = pral[i * 3 + 2];
		anchorX = a.px + mul(lx, a.r[0]) + mul(ly, a.r[1]) + mul(lz, a.r[2]);
		anchorY = a.py + mul(lx, a.r[3]) + mul(ly, a.r[4]) + mul(lz, a.r[5]);
		anchorZ = a.pz + mul(lx, a.r[6]) + mul(ly, a.r[7]) + mul(lz, a.r[8]);
	}

	/** World space anchor of contact i on body b (scratch: anchorX/Y/Z). */
	private static void anchorOnB(RigidBody b, int i) {
		int lx = prbl[i * 3], ly = prbl[i * 3 + 1], lz = prbl[i * 3 + 2];
		anchorX = b.px + mul(lx, b.r[0]) + mul(ly, b.r[1]) + mul(lz, b.r[2]);
		anchorY = b.py + mul(lx, b.r[3]) + mul(ly, b.r[4]) + mul(lz, b.r[5]);
		anchorZ = b.pz + mul(lx, b.r[6]) + mul(ly, b.r[7]) + mul(lz, b.r[8]);
	}

	/**
	 * Sequential impulses for one pair: equal and opposite on both bodies,
	 * with restitution, Coulomb friction and a converging position projection.
	 * A static (carried or sleeping) body has zero inverse mass and inertia, so
	 * it absorbs nothing and only lends its velocity.
	 */
	private static void solvePair(RigidBody a, RigidBody b,
			boolean aStatic, boolean bStatic, int count) {
		int imA = aStatic ? 0 : a.invMass;
		int imB = bStatic ? 0 : b.invMass;
		int[] iiA = aStatic ? ZERO_I : a.invIWorld;
		int[] iiB = bStatic ? ZERO_I : b.invIWorld;

		// Bit i marks the contacts a body must not answer for (see pairHeld).
		int aBlock = 0, bBlock = 0;
		for(int i = 0; i < count; i++) {
			pairHeld(a, b, aStatic, bStatic, i);
			if(heldA) aBlock |= 1 << i;
			if(heldB) bBlock |= 1 << i;
		}

		int avx = velX(a), avy = velY(a), avz = velZ(a);
		int bvx = velX(b), bvy = velY(b), bvz = velZ(b);
		int alx = a.lx, aly = a.ly, alz = a.lz;
		int blx = b.lx, bly = b.ly, blz = b.lz;
		int awx = a.wx, awy = a.wy, awz = a.wz;
		int bwx = b.wx, bwy = b.wy, bwz = b.wz;

		for(int i = 0; i < count; i++) {
			paccN[i] = 0; paccT[i] = 0; pbias[i] = 0;
		}

		// Restitution targets are taken once from the approach velocities,
		// before any impulse is applied, so the sweeps stay consistent.
		for(int i = 0; i < count; i++) {
			int rax = ppx[i] - a.px, ray = ppy[i] - a.py, raz = ppz[i] - a.pz;
			int rbx = ppx[i] - b.px, rby = ppy[i] - b.py, rbz = ppz[i] - b.pz;
			int vax = avx + mul(awy, raz) - mul(awz, ray);
			int vay = avy + mul(awz, rax) - mul(awx, raz);
			int vaz = avz + mul(awx, ray) - mul(awy, rax);
			int vbx = bvx + mul(bwy, rbz) - mul(bwz, rby);
			int vby = bvy + mul(bwz, rbx) - mul(bwx, rbz);
			int vbz = bvz + mul(bwx, rby) - mul(bwy, rbx);
			int vn = mul(vbx - vax, pnx[i]) + mul(vby - vay, pny[i]) + mul(vbz - vaz, pnz[i]);
			pbias[i] = -vn > RESTITUTION_SPEED ? -mul(BODY_RESTITUTION, vn) : 0;
		}

		for(int iter = 0; iter < PAIR_IMPULSE_ITERATIONS; iter++) {
			for(int i = 0; i < count; i++) {
				int nx = pnx[i], ny = pny[i], nz = pnz[i];
				boolean aHeld = ((aBlock >> i) & 1) != 0;
				boolean bHeld = ((bBlock >> i) & 1) != 0;
				int imAc = aHeld ? 0 : imA;
				int imBc = bHeld ? 0 : imB;
				int[] iiAc = aHeld ? ZERO_I : iiA;
				int[] iiBc = bHeld ? ZERO_I : iiB;
				int rax = ppx[i] - a.px, ray = ppy[i] - a.py, raz = ppz[i] - a.pz;
				int rbx = ppx[i] - b.px, rby = ppy[i] - b.py, rbz = ppz[i] - b.pz;

				// relative velocity at the contact point: (v + w x r)_b - _a
				int vax = avx + mul(awy, raz) - mul(awz, ray);
				int vay = avy + mul(awz, rax) - mul(awx, raz);
				int vaz = avz + mul(awx, ray) - mul(awy, rax);
				int vbx = bvx + mul(bwy, rbz) - mul(bwz, rby);
				int vby = bvy + mul(bwz, rbx) - mul(bwx, rbz);
				int vbz = bvz + mul(bwx, rby) - mul(bwy, rbx);
				int rvx = vbx - vax, rvy = vby - vay, rvz = vbz - vaz;
				int vn = mul(rvx, nx) + mul(rvy, ny) + mul(rvz, nz);

				// K_n = 1/ma + 1/mb + ((I^-1 (r x n)) x r) . n for both bodies
				int kn = imAc + imBc
						+ pairEffectiveMass(rax, ray, raz, iiAc, nx, ny, nz)
						+ pairEffectiveMass(rbx, rby, rbz, iiBc, nx, ny, nz);
				if(kn > 0) {
					int dN = divQ(pbias[i] - vn, kn);
					int newAcc = paccN[i] + dN;
					if(newAcc < 0) newAcc = 0;
					dN = newAcc - paccN[i];
					paccN[i] = newAcc;
					if(dN != 0) {
						// b takes +j n, a takes -j n
						int imp = mul(dN, imBc);
						bvx += mul(nx, imp); bvy += mul(ny, imp); bvz += mul(nz, imp);
						imp = mul(dN, imAc);
						avx -= mul(nx, imp); avy -= mul(ny, imp); avz -= mul(nz, imp);
						blx += mul(rby, mul(nz, dN)) - mul(rbz, mul(ny, dN));
						bly += mul(rbz, mul(nx, dN)) - mul(rbx, mul(nz, dN));
						blz += mul(rbx, mul(ny, dN)) - mul(rby, mul(nx, dN));
						alx -= mul(ray, mul(nz, dN)) - mul(raz, mul(ny, dN));
						aly -= mul(raz, mul(nx, dN)) - mul(rax, mul(nz, dN));
						alz -= mul(rax, mul(ny, dN)) - mul(ray, mul(nx, dN));
						bwx = eval24X(iiBc, blx, bly, blz);
						bwy = eval24Y(iiBc, blx, bly, blz);
						bwz = eval24Z(iiBc, blx, bly, blz);
						awx = eval24X(iiAc, alx, aly, alz);
						awy = eval24Y(iiAc, alx, aly, alz);
						awz = eval24Z(iiAc, alx, aly, alz);
					}
				}

				if(paccN[i] <= 0) continue;

				// Friction along the tangent of the relative contact velocity,
				// clamped to mu * accumulated normal impulse.
				vax = avx + mul(awy, raz) - mul(awz, ray);
				vay = avy + mul(awz, rax) - mul(awx, raz);
				vaz = avz + mul(awx, ray) - mul(awy, rax);
				vbx = bvx + mul(bwy, rbz) - mul(bwz, rby);
				vby = bvy + mul(bwz, rbx) - mul(bwx, rbz);
				vbz = bvz + mul(bwx, rby) - mul(bwy, rbx);
				rvx = vbx - vax; rvy = vby - vay; rvz = vbz - vaz;
				int vnn = mul(rvx, nx) + mul(rvy, ny) + mul(rvz, nz);
				int tx = rvx - mul(nx, vnn);
				int ty = rvy - mul(ny, vnn);
				int tz = rvz - mul(nz, vnn);
				int tl = norm3(tx, ty, tz);
				if(tl < 1) continue;
				tx = divQ(tx, tl); ty = divQ(ty, tl); tz = divQ(tz, tl);

				int kt = imAc + imBc
						+ pairEffectiveMass(rax, ray, raz, iiAc, tx, ty, tz)
						+ pairEffectiveMass(rbx, rby, rbz, iiBc, tx, ty, tz);
				if(kt <= 0) continue;
				int vt = mul(rvx, tx) + mul(rvy, ty) + mul(rvz, tz);
				int dT = -divQ(vt, kt);
				int maxFric = abs(mul(BODY_FRICTION, paccN[i]));
				int newAccT = paccT[i] + dT;
				if(newAccT > maxFric) newAccT = maxFric;
				else if(newAccT < -maxFric) newAccT = -maxFric;
				dT = newAccT - paccT[i];
				paccT[i] = newAccT;
				if(dT == 0) continue;

				int imp = mul(dT, imBc);
				bvx += mul(tx, imp); bvy += mul(ty, imp); bvz += mul(tz, imp);
				imp = mul(dT, imAc);
				avx -= mul(tx, imp); avy -= mul(ty, imp); avz -= mul(tz, imp);
				blx += mul(rby, mul(tz, dT)) - mul(rbz, mul(ty, dT));
				bly += mul(rbz, mul(tx, dT)) - mul(rbx, mul(tz, dT));
				blz += mul(rbx, mul(ty, dT)) - mul(rby, mul(tx, dT));
				alx -= mul(ray, mul(tz, dT)) - mul(raz, mul(ty, dT));
				aly -= mul(raz, mul(tx, dT)) - mul(rax, mul(tz, dT));
				alz -= mul(rax, mul(ty, dT)) - mul(ray, mul(tx, dT));
				bwx = eval24X(iiBc, blx, bly, blz);
				bwy = eval24Y(iiBc, blx, bly, blz);
				bwz = eval24Z(iiBc, blx, bly, blz);
				awx = eval24X(iiAc, alx, aly, alz);
				awy = eval24Y(iiAc, alx, aly, alz);
				awz = eval24Z(iiAc, alx, aly, alz);
			}
		}

		if(!aStatic) {
			a.vx = avx; a.vy = avy; a.vz = avz;
			a.lx = alx; a.ly = aly; a.lz = alz;
			a.wx = awx; a.wy = awy; a.wz = awz;
		}
		if(!bStatic) {
			b.vx = bvx; b.vy = bvy; b.vz = bvz;
			b.lx = blx; b.ly = bly; b.lz = blz;
			b.wx = bwx; b.wy = bwy; b.wz = bwz;
		}

		correctPairPositions(a, b, aStatic, bStatic, imA, imB, iiA, iiB, count);
	}

	/**
	 * Position only de-penetration for one pair. Unlike the world pass this
	 * re-derives the penetration from the local anchors every iteration, so
	 * the sweep converges on the slop instead of leaving a fixed fraction of
	 * the overlap behind (a stack would otherwise sink visibly into itself).
	 */
	private static void correctPairPositions(RigidBody a, RigidBody b,
			boolean aStatic, boolean bStatic, int imA, int imB,
			int[] iiA, int[] iiB, int count) {
		int beta = count <= PAIR_BETA.length ? PAIR_BETA[count - 1] : PAIR_BETA[PAIR_BETA.length - 1];
		for(int iter = 0; iter < PAIR_POSITION_ITERATIONS; iter++) {
			for(int i = 0; i < count; i++) {
				int pen = currentPen(a, b, i);
				if(pen <= POSITION_SLOP << 12) continue;
				int nx = pnx[i], ny = pny[i], nz = pnz[i];
				pairHeld(a, b, aStatic, bStatic, i);
				boolean aMove = !aStatic && !heldA;
				boolean bMove = !bStatic && !heldB;
				int imAc = aMove ? imA : 0, imBc = bMove ? imB : 0;
				int[] iiAc = aMove ? iiA : ZERO_I;
				int[] iiBc = bMove ? iiB : ZERO_I;

				// lever arms around the current midpoint of the two anchors
				anchorOnA(a, i);
				int awx = anchorX, awy = anchorY, awz = anchorZ;
				anchorOnB(b, i);
				int cx = (awx + anchorX) >> 1, cy = (awy + anchorY) >> 1, cz = (awz + anchorZ) >> 1;
				int rax = cx - a.px, ray = cy - a.py, raz = cz - a.pz;
				int rbx = cx - b.px, rby = cy - b.py, rbz = cz - b.pz;

				int k = imAc + imBc
						+ pairEffectiveMass(rax, ray, raz, iiAc, nx, ny, nz)
						+ pairEffectiveMass(rbx, rby, rbz, iiBc, nx, ny, nz);
				if(k <= 0) continue;
				int dp = divQ(mul(pen - (POSITION_SLOP << 12), beta), k);

				if(bMove) {
					b.px += mul(nx, mul(dp, imBc));
					b.py += mul(ny, mul(dp, imBc));
					b.pz += mul(nz, mul(dp, imBc));
					int mx = mul(rby, mul(nz, dp)) - mul(rbz, mul(ny, dp));
					int my = mul(rbz, mul(nx, dp)) - mul(rbx, mul(nz, dp));
					int mz = mul(rbx, mul(ny, dp)) - mul(rby, mul(nx, dp));
					b.rotateMatrix(eval24X(iiBc, mx, my, mz),
							eval24Y(iiBc, mx, my, mz), eval24Z(iiBc, mx, my, mz));
					b.recomputeWorldInertia();
				}
				if(aMove) {
					a.px -= mul(nx, mul(dp, imAc));
					a.py -= mul(ny, mul(dp, imAc));
					a.pz -= mul(nz, mul(dp, imAc));
					int mx = mul(ray, mul(nz, dp)) - mul(raz, mul(ny, dp));
					int my = mul(raz, mul(nx, dp)) - mul(rax, mul(nz, dp));
					int mz = mul(rax, mul(ny, dp)) - mul(ray, mul(nx, dp));
					a.rotateMatrix(-eval24X(iiAc, mx, my, mz),
							-eval24Y(iiAc, mx, my, mz), -eval24Z(iiAc, mx, my, mz));
					a.recomputeWorldInertia();
				}
			}
		}
		if(!aStatic) {
			a.fixMatrix();
			a.recomputeWorldInertia();
		}
		if(!bStatic) {
			b.fixMatrix();
			b.recomputeWorldInertia();
		}
	}

	// ===================== vertices / AABB =====================

	/** Recomputes world vertices (Q12 and units) and the world AABB. */
	private void computeVertices() {
		// axis columns
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

	/** Q12 multiply (arithmetic shift, matches the C fixed point code). */
	public static int mul(int a, int b) {
		return (int) ((long) a * b >> 12);
	}

	/** Q12 division: a / b with both values Q12, result Q12. */
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

	/** Q12 magnitude of a Q12 vector. */
	private static int norm3(int x, int y, int z) {
		return isqrt((long) x * x + (long) y * y + (long) z * z);
	}

	/** Integer square root (plain integer domain, floor). */
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

	// closest point on an edge, scratch return via ecx/ecy/ecz, distance squared
	private static int ecx, ecy, ecz;
	private static int edgeClosest(int px, int py, int pz,
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

	/** Point in triangle/quad projected on a dominant axis plane; flip reverses winding. */
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
