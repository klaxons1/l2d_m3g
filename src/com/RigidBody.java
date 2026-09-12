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
 *  - the body falls asleep once it rests on an upward facing surface.
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

	private static final int MAX_CONTACTS = 24;
	private static final int VERTICES = 8;
	private static final int EDGES = 12;

	/**
	 * The 12 cube edges as vertex index pairs (see computeVertices),
	 * grouped by the axis they run along:
	 *   x edges (0,1)(2,3)(4,7)(5,6)
	 *   y edges (0,5)(1,6)(2,7)(3,4)
	 *   z edges (0,3)(1,2)(4,5)(6,7)
	 */
	private static final int[] EDGE_A = {
		0, 2, 4, 5,  0, 1, 2, 3,  0, 1, 4, 6
	};
	private static final int[] EDGE_B = {
		1, 3, 7, 6,  5, 6, 7, 4,  3, 2, 5, 7
	};

	/** Contacts are generated within this band around a surface (units). */
	private static final int CONTACT_MARGIN = 64;
	/** A vertex within this distance on the free side still touches a face. */
	private static final int SURFACE_TOUCH = 6;
	/** Convex polygon edges only catch vertices this close (tighter than faces). */
	private static final int EDGE_MARGIN = 24;
	/** Cube edge vs polygon edge contacts register within this gap (units). */
	private static final int EDGE_EDGE_MARGIN = 64;
	/** Edge contacts carry this much synthetic resting depth (units). */
	private static final int EDGE_SLOP = 16;
	/** Deeper penetration than this rolls the step back and halves dt. */
	private static final int PENETRATION_THRESHOLD = 64;
	/** Smallest substep as a fraction of a frame (1/256). */
	private static final int MIN_DT = 16;
	/** Largest rotation (Q12 radians) per microstep; beyond it the
	 *  first-order matrix update shears the basis, so the step halves. */
	private static final int MAX_MICRO_ROT = 320;
	/** Residual depth (units) at which the end-of-frame safety rewind
	 *  returns the body to its last verified collision-free pose. */
	private static final int WEDGE_PENETRATION = 200;
	/** A frame ending no deeper than this refreshes the safe-pose history. */
	private static final int SAFE_POSE_PEN = 40;
	/** Number of recent collision-free poses kept for the safety rewind. */
	private static final int SAFE_HISTORY = 12;
	/** Rest damping gate (see applyImpulses): fastest vertex speed below
	 *  this (Q12 units/frame) is damped while the support is stable. */
	private static final int REST_SPEED = 14 * F;
	private static final int REST_DAMP = F / 4;
	/** Support polygon test: contacts must bracket the center projection
	 *  within this slop (units) and not sit above this height (units). */
	private static final int SUPPORT_MARGIN = 120;
	private static final int SUPPORT_HIGH = 150;
	/** Calm wedge-rewind frames without ground support after which a body
	 *  trapped in an inescapable inter-room seam is put to sleep. */
	private static final int WEDGE_STREAK = 6;
	/** The hover cycle before each rewind only regains a few frames of
	 *  gravity; above this speed the body is still genuinely in flight. */
	private static final int WEDGE_SLEEP_SPEED = 64 * F;
	/** Pose based sleep: when every vertex stays within this many units
	 *  of where it was that many supported frames ago, the body is settled
	 *  even if a degenerate single-contact seam keeps residual energy just
	 *  above the energy threshold. */
	private static final int REST_POSE_SLOP = 8;
	private static final int REST_POSE_FRAMES = 12;
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

	// ---- closest feature per box vertex while scanning a mesh ----
	private final int[] bestGap = new int[VERTICES];
	private final int[] bestNX = new int[VERTICES];
	private final int[] bestNY = new int[VERTICES];
	private final int[] bestNZ = new int[VERTICES];
	private final int[] bestPen = new int[VERTICES];

	// ---- closest cube edge per polygon edge while scanning a mesh ----
	private final int[] bestEdgeGap = new int[EDGES];
	private final int[] bestEPX = new int[EDGES];
	private final int[] bestEPY = new int[EDGES];
	private final int[] bestEPZ = new int[EDGES];
	private final int[] bestENX = new int[EDGES];
	private final int[] bestENY = new int[EDGES];
	private final int[] bestENZ = new int[EDGES];

	/** Fastest point speed at the start of the current frame (Q12). */
	private int preFrameSpeed;
	/** Closest surface feature found while sweeping the current pose. */
	private int minSeparation;
	/** Length of the current microstep, Q12 fraction of a frame. */
	private int microDt = F;

	// Restitution fires only on the first frame of a new contact; without
	// cross-frame warm starting a continuing contact would otherwise gain
	// a fresh bounce every frame.
	private boolean contactLastFrame;
	private boolean restitutionOpen;
	/** Consecutive calm frames finished by a wedge rewind with no ground
	 *  support (see step): an inescapable inter-room seam traps the body. */
	private int wedgeStreak;
	// Previous end-of-frame pose for position based sleeping
	private final int[] restVQ = new int[VERTICES * 3];
	private int restPX, restPY, restPZ;
	private int restFrames;
	private boolean restPoseValid;

	// Last verified collision-free poses (position + orientation), newest
	// last. Fixed backing arrays: no per-frame allocation on the handset.
	private final int[] safePX = new int[SAFE_HISTORY];
	private final int[] safePY = new int[SAFE_HISTORY];
	private final int[] safePZ = new int[SAFE_HISTORY];
	private final int[] safeR = new int[SAFE_HISTORY * 9];
	private int safeCount;
	private final boolean[] contactAlive = new boolean[MAX_CONTACTS];
	private final int[] contactKeep = new int[MAX_CONTACTS];

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
		this.numContacts = 0;
		this.maxPenetration = 0;
		this.minSeparation = Integer.MAX_VALUE;
		this.microDt = F;
		this.groundContact = false;
		this.contactLastFrame = false;
		this.restitutionOpen = true;
		this.wedgeStreak = 0;
		this.restFrames = 0;
		this.restPoseValid = false;
		resetSafeHistory();
		computeVertices();
	}

	/** Owns the current pose as the only collision-free pose so far. */
	private void resetSafeHistory() {
		safeCount = 1;
		safePX[0] = px; safePY[0] = py; safePZ[0] = pz;
		System.arraycopy(r, 0, safeR, 0, 9);
	}

	private void rememberSafe() {
		if(safeCount > 0 && safePX[safeCount - 1] == px
				&& safePY[safeCount - 1] == py
				&& safePZ[safeCount - 1] == pz) {
			return;
		}
		if(safeCount == SAFE_HISTORY) {
			System.arraycopy(safePX, 1, safePX, 0, SAFE_HISTORY - 1);
			System.arraycopy(safePY, 1, safePY, 0, SAFE_HISTORY - 1);
			System.arraycopy(safePZ, 1, safePZ, 0, SAFE_HISTORY - 1);
			System.arraycopy(safeR, 9, safeR, 0, (SAFE_HISTORY - 1) * 9);
			safeCount = SAFE_HISTORY - 1;
		}
		safePX[safeCount] = px; safePY[safeCount] = py; safePZ[safeCount] = pz;
		System.arraycopy(r, 0, safeR, safeCount * 9, 9);
		++safeCount;
	}

	public void wake() {
		this.sleeping = false;
		this.sleepCounter = 0;
		this.wedgeStreak = 0;
		this.restFrames = 0;
		this.restPoseValid = false;
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
		this.px = nx;
		this.py = ny;
		this.pz = nz;
		this.lx = this.ly = this.lz = 0;
		this.wx = this.wy = this.wz = 0;
		identity3(r);
		recomputeWorldInertia();
		wake();
		computeVertices();
		// sphereCast keeps the held cube out of walls: own the placement
		resetSafeHistory();
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
		// the portal destination is an intentional placement: own it so a
		// later wedge cannot rewind the cube back through the portal
		resetSafeHistory();
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

		// Fastest point speed at the frame start: a body already resting
		// when a wedge rewind fires was parked in a concave seam (its
		// contacts flip on a one-unit drift), so the rewind must not keep
		// it awake forever.
		preFrameSpeed = norm3(vx, vy, vz)
				+ mul(norm3(wx, wy, wz), cornerRadius);

		// Gravity plus portalDS style velocity damping, expressed as forces.
		int fx = -vx / LINEAR_DRAG;
		int fy = -GRAVITY - vy / LINEAR_DRAG;
		int fz = -vz / LINEAR_DRAG;
		int mx = -wx / ANGULAR_DRAG;
		int my = -wy / ANGULAR_DRAG;
		int mz = -wz / ANGULAR_DRAG;

		// The frame is advanced in microsteps. A microstep that ends too
		// deep or that would leap past a nearby feature in one move is
		// rolled back and retried at half the length. Every accepted
		// microstep actually advances time, so the frame always covers
		// one full dt = F.
		int micro = F, remaining = F, depth = 0, prevDeep = 0;
		boolean contactedThisFrame = false;
		// bounce only on the first frame of a new contact
		restitutionOpen = !contactLastFrame;
		while(remaining > 0) {
			if(micro > remaining) micro = remaining;
			while(true) {
				backup();
				integrate(micro, fx, fy, fz, mx, my, mz);
				microDt = micro;
				collideWorld(colliders, count, world);

				// Subdivide for a feature this microstep would leap past
				// without a single contact to block it (wall-end spears).
				boolean leap = numContacts == 0
						&& minSeparation != Integer.MAX_VALUE
						&& minSeparation < microTravel() + 2;
				// Subdivide for freshly gained depth. If halving the step
				// barely changes the depth, the body was already embedded
				// before this frame; position projection resolves it.
				int deepUnits = maxPenetration >> 12;
				boolean deep = deepUnits > PENETRATION_THRESHOLD
						&& (prevDeep == 0 || deepUnits * 4 < prevDeep * 3);
				// Subdivide a large rotation: the first-order matrix update
				// cannot take more than ~MAX_MICRO_ROT radians without
				// shearing the basis and amplifying spin via the tensor.
				int rotStep = mul(norm3(wx, wy, wz), microDt);
				boolean spin = rotStep > MAX_MICRO_ROT;

				if((deep || leap || spin) && micro > MIN_DT) {
					restore();
					prevDeep = deepUnits;
					micro >>= 1;
					++depth;
					continue;
				}

				if(numContacts > 0) {
					applyImpulses();
					contactedThisFrame = true;
					// later microsteps/frames resolve without restitution
					restitutionOpen = false;
				}
				// Bound velocity before the next microstep integrates it,
				// so a pathological manifold can never launch the body
				// across the world inside a single frame.
				clampVelocity();
				break;
			}
			remaining -= micro;
			prevDeep = 0;
		}
		this.lastSubsteps = 1 << depth;
		contactLastFrame = contactedThisFrame;

		// Cross-frame safety net: a fast throw can plant the box across a
		// thin double-shell wall at an open end even after every microstep
		// was accepted, because the within-frame rollback only knows the
		// start of the current frame. Rewind a deeply wedged body to its
		// newest verified-clean pose; a shallow frame refreshes history.
		boolean rewound = false;
		if(maxPenetration > WEDGE_PENETRATION << 12) {
			safetyRewind(colliders, count, world);
			rewound = true;
		} else if(maxPenetration <= SAFE_POSE_PEN << 12) {
			rememberSafe();
		}

		// A calm, groundless body repeatedly rewound into the same
		// collision-free pose is caught in an inescapable inter-room seam
		// (the level has no valid surface beneath it): park it instead
		// of replaying the same fall-and-wedge forever. A later push or
		// throw wakes it again.
		if(rewound && preFrameSpeed <= WEDGE_SLEEP_SPEED && !groundContact) {
			++wedgeStreak;
			if(wedgeStreak >= WEDGE_STREAK) {
				sleeping = true;
				vx = vy = vz = 0;
				wx = wy = wz = 0;
				lx = ly = lz = 0;
			}
		} else if(groundContact || preFrameSpeed > WEDGE_SLEEP_SPEED) {
			// supported, or still in genuine free flight: clear streak
			wedgeStreak = 0;
		}

		// Safety clamps run on every frame (after any rewind): a
		// pathological impact must never leave a runaway spin behind.
		clampVelocity();

		// Sleep bookkeeping: only a body supported from below may rest.
		// The counter accumulates on calm frames and merely drains on a
		// restless one, so an occasional one-frame positional nudge on a
		// seam between coplanar floors cannot keep a parked cube awake.
		energy = mul(vx, vx) + mul(vy, vy) + mul(vz, vz)
				+ mul(wx, wx) + mul(wy, wy) + mul(wz, wz);
		if(!groundContact) {
			sleepCounter = 0;
		} else if(energy <= SLEEP_LOW) {
			if(++sleepCounter >= SLEEP_TIME) {
				sleeping = true;
				vx = vy = vz = 0;
				wx = wy = wz = 0;
				lx = ly = lz = 0;
			}
		} else if(energy >= SLEEP_HIGH) {
			sleeping = false;
			sleepCounter -= 2;
			if(sleepCounter < 0) sleepCounter = 0;
		} else {
			--sleepCounter;
			if(sleepCounter < 0) sleepCounter = 0;
		}

		computeVertices();
		updatePoseSleep();
	}

	/**
	 * Position based sleep: a supported body whose every vertex moves less
	 * than REST_POSE_SLOP for REST_POSE_FRAMES consecutive frames is parked
	 * even when a degenerate single-contact seam keeps residual energy
	 * just above the energy sleep threshold.
	 */
	private void updatePoseSleep() {
		if(!groundContact || sleeping) {
			restFrames = 0;
			restPoseValid = false;
			return;
		}
		final int slop = REST_POSE_SLOP << 12;
		if(restPoseValid) {
			boolean moved = abs(px - restPX) + abs(py - restPY)
					+ abs(pz - restPZ) > slop;
			if(!moved) {
				for(int k = 0; k < VERTICES * 3; k++) {
					if(abs(vq[k] - restVQ[k]) > slop) { moved = true; break; }
				}
			}
			if(moved) {
				restFrames = 0;
			} else if(++restFrames >= REST_POSE_FRAMES) {
				sleeping = true;
				vx = vy = vz = 0;
				wx = wy = wz = 0;
				lx = ly = lz = 0;
				restFrames = 0;
			}
		} else {
			restFrames = 0;
		}
		System.arraycopy(vq, 0, restVQ, 0, VERTICES * 3);
		restPX = px; restPY = py; restPZ = pz;
		restPoseValid = true;
	}

	/** Max distance any box point covers in the current microstep. */
	private int microTravel() {
		int lin = norm3(vx, vy, vz);
		int ang = mul(norm3(wx, wy, wz), cornerRadius);
		// result is plain engine units (compared against minSeparation)
		return mul(lin + ang, microDt) >> 12;
	}

	/**
	 * Rewinds a deeply wedged body to the newest remembered pose that is
	 * verified collision free at shallow depth, dropping the velocity that
	 * drove it in. If no remembered pose is clean, the oldest is restored
	 * and position projection climbs out of it.
	 */
	private void safetyRewind(Collider[] colliders, int count, boolean world) {
		vx = vy = vz = 0;
		wx = wy = wz = 0;
		lx = ly = lz = 0;
		boolean accepted = false;
		for(int h = safeCount - 1; h >= 0; h--) {
			px = safePX[h]; py = safePY[h]; pz = safePZ[h];
			System.arraycopy(safeR, h * 9, r, 0, 9);
			recomputeWorldInertia();
			computeVertices();
			// broadphase and backside cull are swept against the backup
			// pose; refresh it so they test the restored pose
			backup();
			collideWorld(colliders, count, world);
			if(maxPenetration <= SAFE_POSE_PEN << 12) {
				accepted = true;
				break;
			}
		}
		if(!accepted && safeCount > 0) {
			px = safePX[0]; py = safePY[0]; pz = safePZ[0];
			System.arraycopy(safeR, 0, r, 0, 9);
			fixMatrix();
			recomputeWorldInertia();
			computeVertices();
			backup();
			collideWorld(colliders, count, world);
			if(numContacts > 0) correctPositions();
		}
		if(preFrameSpeed >= REST_SPEED) {
			sleeping = false;
			sleepCounter = 0;
		}
		// a calm body parked in a geometrically tight seam keeps its sleep
		// counter, so the frame-end sleep bookkeeping can put it to sleep
		// instead of oscillating forever
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
		wx = evalIX(invIWorld, lx, ly, lz);
		wy = evalIY(invIWorld, lx, ly, lz);
		wz = evalIZ(invIWorld, lx, ly, lz);
	}

	/**
	 * Local inertia of a box with half extents h: I = m/3(h2+h3).
	 * The inverse tensor is stored in Q28 because at world cube scale
	 * (half extent ~500 units) its Q12 value is below the fixed point
	 * resolution and Q24 still showed 20% orientation dependent
	 * quantization; the positive tensor used for angular momentum stays
	 * Q12.
	 */
	private void computeLocalInertia() {
		identity3(invILocal);
		int x2 = mul(hx, hx), y2 = mul(hy, hy), z2 = mul(hz, hz);
		invILocal[0] = (int) (((long) (3 * F) << 28) / mul(mass, y2 + z2));
		invILocal[4] = (int) (((long) (3 * F) << 28) / mul(mass, x2 + z2));
		invILocal[8] = (int) (((long) (3 * F) << 28) / mul(mass, x2 + y2));
		il0 = mul(mass, y2 + z2) / 3;
		il4 = mul(mass, x2 + z2) / 3;
		il8 = mul(mass, x2 + y2) / 3;
	}

	/**
	 * invIWorld (Q28) = R * invILocal * R^T. The orientation matrix is
	 * stored column-major (element R[row][col] is r[col*3+row]); using
	 * row-major indexing here silently produced an anisotropic tensor for
	 * any tilted orientation and fed angular-energy pumps.
	 */
	private void recomputeWorldInertia() {
		for(int row = 0; row < 3; row++) {
			for(int col = 0; col < 3; col++) {
				int t0 = mul(r[row], invILocal[0]);
				int t1 = mul(r[row + 3], invILocal[4]);
				int t2 = mul(r[row + 6], invILocal[8]);
				int v = mul(t0, r[col]) + mul(t1, r[col + 3]) + mul(t2, r[col + 6]);
				invIWorld[row * 3 + col] = v;
			}
		}
	}

	/** Evaluates a Q28 3x3 matrix against a Q12 vector, result Q12. */
	private static int evalIX(int[] m, int x, int y, int z) {
		return (int) (((long) m[0] * x + (long) m[1] * y + (long) m[2] * z) >> 28);
	}
	private static int evalIY(int[] m, int x, int y, int z) {
		return (int) (((long) m[3] * x + (long) m[4] * y + (long) m[5] * z) >> 28);
	}
	private static int evalIZ(int[] m, int x, int y, int z) {
		return (int) (((long) m[6] * x + (long) m[7] * y + (long) m[8] * z) >> 28);
	}

	/** L = Iworld (Q12, built from il0..il8 with column-major R) * w */
	private void recomputeMomentum() {
		int[] iw = tmpMatrix;
		for(int row = 0; row < 3; row++) {
			for(int col = 0; col < 3; col++) {
				int t0 = mul(r[row], il0);
				int t1 = mul(r[row + 3], il4);
				int t2 = mul(r[row + 6], il8);
				iw[row * 3 + col] = mul(t0, r[col]) + mul(t1, r[col + 3]) + mul(t2, r[col + 6]);
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
		minSeparation = Integer.MAX_VALUE;
		computeVertices();
		if(!world) return;

		for(int i = 0; i < VERTICES; i++) bestGap[i] = Integer.MAX_VALUE;
		for(int i = 0; i < EDGES; i++) {
			bestEdgeGap[i] = Integer.MAX_VALUE;
		}

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

					// Backside cull: shared walls between rooms carry two
					// coincident faces with opposite normals. A face whose
					// plane puts the pre-step cube center deeper than the
					// box's maximal reach along the normal is on the far
					// side of that wall; its "contacts" are false positives
					// that would crush the cube between opposing normals.
					// The pre-integration position is used so a face the
					// body just tunnelled through cannot be hidden from the
					// rollback.
					int ccx = bpx >> 12, ccy = bpy >> 12, ccz = bpz >> 12;
					int dc = ((ccx - ax) * snx + (ccy - ay) * sny + (ccz - az) * snz) >> 12;
					int reach;
					if(axn >= ayn && axn >= azn) {
						reach = (mul(abs(r[0]), hx) + mul(abs(r[3]), hy) + mul(abs(r[6]), hz)) >> 12;
					} else if(ayn >= axn && ayn >= azn) {
						reach = (mul(abs(r[1]), hx) + mul(abs(r[4]), hy) + mul(abs(r[7]), hz)) >> 12;
					} else {
						reach = (mul(abs(r[2]), hx) + mul(abs(r[5]), hy) + mul(abs(r[8]), hz)) >> 12;
					}
					if(dc > reach + CONTACT_MARGIN) continue;

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
							// vertices only touch a face once they reach the
							// plane (a wide free-side band would act like an
							// invisible shell); a deep penetration is kept
							// on purpose so the substep rollback can recover
							if(d < -SURFACE_TOUCH) continue;
							if(d < 0 && -d < minSeparation) minSeparation = -d;
							if(-d < bestGap[k]) {
								bestGap[k] = -d;
								bestNX[k] = nx; bestNY[k] = ny; bestNZ[k] = nz;
								bestPen[k] = d > 0 ? d << 12 : 0;
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
							if(s < minSeparation) minSeparation = s;
							if(s <= EDGE_MARGIN && s < bestGap[k]) {
								bestGap[k] = s;
								if(s > 0) {
									bestNX[k] = ((qx - bx2) << 12) / s;
									bestNY[k] = ((qy - by2) << 12) / s;
									bestNZ[k] = ((qz - bz2) << 12) / s;
								} else {
									bestNX[k] = nx; bestNY[k] = ny; bestNZ[k] = nz;
								}
								bestPen[k] = 0;
							}
						}
					}

					// Edge vs edge: the open end of a thin wall can spear a
					// cube face while every cube vertex sits outside every
					// polygon; vertex-vs-face tests alone miss that.
					edgeVsCubeEdges(ax, ay, az, bx, by, bz, nx, ny, nz);
					edgeVsCubeEdges(bx, by, bz, cx, cy, cz, nx, ny, nz);
					if(vpp == 4) {
						edgeVsCubeEdges(cx, cy, cz, dx, dy, dz, nx, ny, nz);
						edgeVsCubeEdges(dx, dy, dz, ax, ay, az, nx, ny, nz);
					} else {
						edgeVsCubeEdges(cx, cy, cz, ax, ay, az, nx, ny, nz);
					}
				}
			}
		}

		for(int k = 0; k < VERTICES; k++) {
			if(bestGap[k] <= CONTACT_MARGIN) {
				addContact(vq[k * 3], vq[k * 3 + 1], vq[k * 3 + 2],
						bestNX[k], bestNY[k], bestNZ[k], bestPen[k]);
			}
		}
		for(int e = 0; e < EDGES; e++) {
			if(bestEdgeGap[e] <= EDGE_EDGE_MARGIN) {
				int pen = Math.max(0, EDGE_SLOP - bestEdgeGap[e]) << 12;
				addContact(bestEPX[e], bestEPY[e], bestEPZ[e],
						bestENX[e], bestENY[e], bestENZ[e], pen);
			}
		}

		// Collapse coincident opposed contacts. Shared walls carry two
		// coincident faces with opposite normals; when the box straddles such
		// a wall both faces report deep contacts, the corrections cancel
		// (leaving the box embedded frame after frame) and the two impulse
		// channels pump angular energy. Opposed contacts close together are
		// the same double-shell face: keep the deeper one. Genuine
		// floor/ceiling or corridor sandwiches have points a body width
		// apart and survive.
		final int dupDist = 300 << 12;
		for(int i = 0; i < numContacts; i++) contactAlive[i] = true;
		for(int i = 0; i < numContacts; i++) {
			for(int j = 0; j < i; j++) {
				if(!contactAlive[i] || !contactAlive[j]) continue;
				int dot = mul(cnx[i], cnx[j]) + mul(cny[i], cny[j]) + mul(cnz[i], cnz[j]);
				if(dot > -3 * F / 4) continue;
				long dx = cpx[i] - cpx[j], dy = cpy[i] - cpy[j], dz = cpz[i] - cpz[j];
				if(dx * dx + dy * dy + dz * dz > (long) dupDist * dupDist) continue;
				if(cpen[i] >= cpen[j]) contactAlive[j] = false;
				else { contactAlive[i] = false; break; }
			}
		}
		int kept = 0;
		for(int i = 0; i < numContacts; i++) {
			if(contactAlive[i]) contactKeep[kept++] = i;
		}
		if(kept < numContacts) compactContacts(contactKeep, kept);
	}

	/** Rewrites the contact arrays keeping only the indices in keep. */
	private void compactContacts(int[] keep, int n) {
		for(int slot = 0; slot < n; slot++) {
			int i = keep[slot];
			cpx[slot] = cpx[i]; cpy[slot] = cpy[i]; cpz[slot] = cpz[i];
			cnx[slot] = cnx[i]; cny[slot] = cny[i]; cnz[slot] = cnz[i];
			cpen[slot] = cpen[i];
		}
		for(int slot = n; slot < numContacts; slot++) cpen[slot] = 0;
		numContacts = n;
		maxPenetration = 0;
		groundContact = false;
		for(int slot = 0; slot < n; slot++) {
			if(cpen[slot] > maxPenetration) maxPenetration = cpen[slot];
			if(cny[slot] > F * 7 / 10) groundContact = true;
		}
	}

	private static int clampParam(long v) {
		if(v < 0) return 0;
		if(v > F) return F;
		return (int) v;
	}

	/**
	 * Evaluates (n * F) / d clamped to [0, F], truncating toward zero like
	 * Java integer division. The edge-edge parameter products can reach
	 * roughly 2e17, so shifting n by 12 may overflow long. When the result
	 * fits in range (n small) exact 64 bit division is kept; for extreme
	 * values, which always clamp, double is far more than precise enough to
	 * decide the clamp side. d must be positive.
	 */
	private static int sParam(long n, long d) {
		if(n == 0) return 0;
		if(n > 0) {
			if(n >>> 52 == 0) {
				long q = (n << 12) / d;
				return q > F ? F : (int) q;
			}
			double q = (double) n * (double) F / (double) d;
			return q > F ? F : (q < 0 ? 0 : (int) q);
		}
		long a = -n;
		if(a >>> 52 == 0) {
			long q = -((a << 12) / d);
			return q < 0 ? 0 : (int) q;
		}
		double q = (double) n * (double) F / (double) d;
		return q < 0 ? 0 : (q > F ? F : (int) q);
	}

	/**
	 * Segment/segment closest point test between one polygon edge
	 * (q0..q1, plain engine units) and the 12 cube edges, tracking the
	 * nearest per-cube-edge result in the best edge arrays.
	 */
	private void edgeVsCubeEdges(int q0x, int q0y, int q0z,
			int q1x, int q1y, int q1z, int fnx, int fny, int fnz) {
		int d2x = q1x - q0x, d2y = q1y - q0y, d2z = q1z - q0z;
		long c = (long) d2x * d2x + (long) d2y * d2y + (long) d2z * d2z;
		if(c == 0) return;
		for(int e = 0; e < EDGES; e++) {
			int a0 = EDGE_A[e] * 3, a1 = EDGE_B[e] * 3;
			int a0x = vu[a0], a0y = vu[a0 + 1], a0z = vu[a0 + 2];
			int a1x = vu[a1], a1y = vu[a1 + 1], a1z = vu[a1 + 2];
			int d1x = a1x - a0x, d1y = a1y - a0y, d1z = a1z - a0z;
			int rx = a0x - q0x, ry = a0y - q0y, rz = a0z - q0z;

			long a = (long) d1x * d1x + (long) d1y * d1y + (long) d1z * d1z;
			long bb = (long) d1x * d2x + (long) d1y * d2y + (long) d1z * d2z;
			long dd = (long) d1x * rx + (long) d1y * ry + (long) d1z * rz;
			long ee = (long) d2x * rx + (long) d2y * ry + (long) d2z * rz;
			long denom = a * c - bb * bb;  // |d1 x d2|^2 (Lagrange identity)
			if(a == 0 || denom <= (a * c >> 4)) continue;  // near parallel

			long n = bb * ee - c * dd;
			long s = sParam(n, denom);
			long t = clampParam((bb * s - ee * F) / c);
			s = clampParam((bb * t - dd * F) / a);
			t = clampParam((bb * s - ee * F) / c);

			int px = a0x + (int) (d1x * s >> 12);
			int py = a0y + (int) (d1y * s >> 12);
			int pz = a0z + (int) (d1z * s >> 12);
			int qx = q0x + (int) (d2x * t >> 12);
			int qy = q0y + (int) (d2y * t >> 12);
			int qz = q0z + (int) (d2z * t >> 12);

			int dx = px - qx, dy = py - qy, dz = pz - qz;
			int dist = isqrt((long) dx * dx + (long) dy * dy + (long) dz * dz);
			if(dist < minSeparation) minSeparation = dist;
			if(dist > EDGE_EDGE_MARGIN || dist >= bestEdgeGap[e]) continue;

			int exn, eyn, ezn;
			if(dist > 0) {
				// cube edge point minus polygon edge point points out
				exn = (dx << 12) / dist;
				eyn = (dy << 12) / dist;
				ezn = (dz << 12) / dist;
			} else {
				// segments intersect: d1 x d2, flipped to the face's outward
				// side
				long cx2 = (long) d1y * d2z - (long) d1z * d2y;
				long cy2 = (long) d1z * d2x - (long) d1x * d2z;
				long cz2 = (long) d1x * d2y - (long) d1y * d2x;
				long nl = isqrt(cx2 * cx2 + cy2 * cy2 + cz2 * cz2);
				if(nl == 0) continue;
				exn = (int) ((cx2 << 12) / nl);
				eyn = (int) ((cy2 << 12) / nl);
				ezn = (int) ((cz2 << 12) / nl);
				if((long) exn * fnx + (long) eyn * fny + (long) ezn * fnz < 0) {
					exn = -exn; eyn = -eyn; ezn = -ezn;
				}
			}

			bestEdgeGap[e] = dist;
			bestEPX[e] = px << 12; bestEPY[e] = py << 12; bestEPZ[e] = pz << 12;
			bestENX[e] = exn; bestENY[e] = eyn; bestENZ[e] = ezn;
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
			// Bounce only on the first frame of a new contact: with no
			// cross-frame warm starting, an ongoing sliding/spinning
			// contact would otherwise gain a fresh bounce every frame.
			vbias[i] = (-vn > RESTITUTION_SPEED && restitutionOpen)
					? -mul(RESTITUTION, vn) : 0;
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
				int irx = evalIX(invIWorld, rnx, rny, rnz);
				int iry = evalIY(invIWorld, rnx, rny, rnz);
				int irz = evalIZ(invIWorld, rnx, rny, rnz);
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
						wx = evalIX(invIWorld, lx, ly, lz);
						wy = evalIY(invIWorld, lx, ly, lz);
						wz = evalIZ(invIWorld, lx, ly, lz);
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
					int itx = evalIX(invIWorld, rtx, rty, rtz);
					int ity = evalIY(invIWorld, rtx, rty, rtz);
					int itz = evalIZ(invIWorld, rtx, rty, rtz);
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
							wx = evalIX(invIWorld, lx, ly, lz);
							wy = evalIY(invIWorld, lx, ly, lz);
							wz = evalIZ(invIWorld, lx, ly, lz);
						}
					}
				}
			}
		}

		// De-penetration runs as its own position-only pass.
		correctPositions();

		// Rest damping: a supported, barely moving body has its velocities
		// damped each solve, killing multi-frame micro-hop cycles on a
		// triangulated/seamed floor that otherwise reset the sleep
		// counter. It only applies when the contact points bracket the
		// center (supportIsStable): a cube balanced on one corner or edge
		// still has to tip over onto a face before motion is damped.
		if(groundContact && supportIsStable()) {
			int speed = norm3(vx, vy, vz) + mul(norm3(wx, wy, wz), cornerRadius);
			if(speed < REST_SPEED) {
				vx = mul(vx, REST_DAMP); vy = mul(vy, REST_DAMP); vz = mul(vz, REST_DAMP);
				wx = mul(wx, REST_DAMP); wy = mul(wy, REST_DAMP); wz = mul(wz, REST_DAMP);
				recomputeMomentum();
			}
		}
	}

	/**
	 * True when contact points (at or below center height) bracket the
	 * horizontal center projection on both sides along both axes. A face
	 * rest (four corners) or a stable floor/wall nest passes; a single
	 * corner or edge balance has all points to one side and fails, so the
	 * cube is free to tip over.
	 */
	private boolean supportIsStable() {
		final int margin = SUPPORT_MARGIN << 12;
		final int high = SUPPORT_HIGH << 12;
		int minX = Integer.MAX_VALUE, minZ = Integer.MAX_VALUE;
		int maxX = Integer.MIN_VALUE, maxZ = Integer.MIN_VALUE;
		int n = 0;
		for(int i = 0; i < numContacts; i++) {
			int oy = cpy[i] - py;
			if(oy > high) continue;
			++n;
			int ox = cpx[i] - px;
			int oz = cpz[i] - pz;
			if(ox < minX) minX = ox;
			if(ox > maxX) maxX = ox;
			if(oz < minZ) minZ = oz;
			if(oz > maxZ) maxZ = oz;
		}
		return n >= 2 && minX <= margin && maxX >= -margin
				&& minZ <= margin && maxZ >= -margin;
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
				int irx = evalIX(invIWorld, rnx, rny, rnz);
				int iry = evalIY(invIWorld, rnx, rny, rnz);
				int irz = evalIZ(invIWorld, rnx, rny, rnz);
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
				int qx = evalIX(invIWorld,
						mul(ry, mul(nz, dp)) - mul(rz, mul(ny, dp)),
						mul(rz, mul(nx, dp)) - mul(rx, mul(nz, dp)),
						mul(rx, mul(ny, dp)) - mul(ry, mul(nx, dp)));
				int qy = evalIY(invIWorld,
						mul(ry, mul(nz, dp)) - mul(rz, mul(ny, dp)),
						mul(rz, mul(nx, dp)) - mul(rx, mul(nz, dp)),
						mul(rx, mul(ny, dp)) - mul(ry, mul(nx, dp)));
				int qz = evalIZ(invIWorld,
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
