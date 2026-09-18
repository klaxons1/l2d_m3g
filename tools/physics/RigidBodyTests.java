package com;

/**
 * Self checking tests for the Q12 rigid body solver in src/com/RigidBody.java.
 * 27 scenarios: 13 single body against hand made world geometry, 13 cube vs
 * cube driven the way GameScreen drives them, and a randomized pile fuzz.
 *
 * The solver is autonomous - it has no imports at all and only uses
 * java.lang - so this file plus RigidBody.java are everything needed to build
 * and run the physics tests, with no M3G, no MIDP and no other game class:
 *
 *     tools/physics/run_tests.sh
 *
 * That script compiles the solver alone at -source 1.3 -target 1.3 against
 * the CLDC/MIDP/M3G bootclasspath, which is the phone compatibility gate,
 * then compiles these tests against it at the compiler's default level and
 * runs them. The exit code is 1 when any check fails, so it can gate a
 * build. Nothing here runs in CI: build.yml compiles the solver at 1.3 for
 * the game on the way to main, but only this suite checks what it does.
 *
 * There is no JUnit on CLDC and -source 1.3 has no assert keyword, hence the
 * small check framework at the bottom. Checks only print when they fail.
 *
 * Everything the tests touch is public API (getCenterX, getOrientation,
 * isSleeping, lastSubsteps, collideBodies, ...), so adding a test never needs
 * a solver change. RigidBodyHarness.java is the companion tool for dumping
 * raw state traces when one of these fails and the numbers need eyeballing.
 */
public final class RigidBodyTests {

	private static final int HALF = 500;
	/** The solver's own near-parallel edge axis cutoff, duplicated here
	 *  because the constant is private. */
	private static final int PARALLEL_EPS = 1 << 18;
	// Cube.JAM_PEN, duplicated for the same reason.
	private static final int CUBE_JAM_PEN = 100 << 12;
	private static final int F = RigidBody.F;

	// ------------------------------------------------------------- fixtures

	private static RigidBody.Collider quad(int[] a, int[] b, int[] c, int[] d) {
		RigidBody.Collider col = new RigidBody.Collider();
		short[] v = new short[12];
		for(int i = 0; i < 3; i++) {
			v[i] = (short) a[i];
			v[3 + i] = (short) b[i];
			v[6 + i] = (short) c[i];
			v[9 + i] = (short) d[i];
		}
		col.verts = v;
		col.pols = new short[]{0, 1, 2, 3};
		// normal from winding, same construction as MathUtils.createNormal:
		// n = (a - b) x (a - c), normalized in fixed point Q12
		long abx = a[0] - b[0], aby = a[1] - b[1], abz = a[2] - b[2];
		long acx = a[0] - c[0], acy = a[1] - c[1], acz = a[2] - c[2];
		long nx = aby * acz - abz * acy;
		long ny = abz * acx - abx * acz;
		long nz = abx * acy - aby * acx;
		int len = RigidBody.isqrt(nx * nx + ny * ny + nz * nz);
		col.norms = new short[]{
				(short) ((nx * 4096) / len),
				(short) ((ny * 4096) / len),
				(short) ((nz * 4096) / len)};
		col.quads = 1;
		col.tris = 0;
		col.scale8 = 256;
		return col;
	}

	private static RigidBody.Collider floor() {
		final int S = 20000;
		// winding gives a downward normal (into the solid), as in engine rooms
		return quad(
				new int[]{-S, 0, -S}, new int[]{S, 0, -S},
				new int[]{S, 0, S}, new int[]{-S, 0, S});
	}

	private static RigidBody.Collider wall() {
		final int S = 20000, H = 30000, W = 1800;
		// solid x > W, normal +x
		return quad(
				new int[]{W, 0, -S}, new int[]{W, H, -S},
				new int[]{W, H, S}, new int[]{W, 0, S});
	}

	private static RigidBody.Collider wallZ() {
		final int S = 20000, H = 30000, W = 1800;
		// solid z > W, normal +z
		return quad(
				new int[]{-S, 0, W}, new int[]{S, 0, W},
				new int[]{S, H, W}, new int[]{-S, H, W});
	}

	/** Solid x < -WEST, so the winding gives a normal pointing -x into it. The
	 *  fixture normals all point into the solid: the floor's points down. */
	private static RigidBody.Collider wallWest() {
		final int S = 20000, H = 30000, WEST = -8000;
		return quad(
				new int[]{WEST, 0, -S}, new int[]{WEST, 0, S},
				new int[]{WEST, H, S}, new int[]{WEST, H, -S});
	}

	private static RigidBody.Collider wallSouth() {
		final int S = 20000, H = 30000, SOUTH = -8000;
		// solid z < SOUTH, normal -z
		return quad(
				new int[]{S, 0, SOUTH}, new int[]{-S, 0, SOUTH},
				new int[]{-S, H, SOUTH}, new int[]{S, H, SOUTH});
	}

	/** The closed arena the fuzz throws piles into: a floor and four walls, so
	 *  a cube can never drift off the edge of a finite floor and fall out of
	 *  the world, which would look exactly like tunnelling. */
	private static RigidBody.Collider[] arena() {
		return new RigidBody.Collider[]{floor(), wall(), wallZ(), wallWest(),
				wallSouth()};
	}

	private static RigidBody.Collider ramp() {
		// ~20 degree slope rising in -x (plane y = -0.364x), solid below
		return quad(
				new int[]{-4000, 1456, 20000}, new int[]{-4000, 1456, -20000},
				new int[]{0, 0, -20000}, new int[]{0, 0, 20000});
	}

	// ------------------------------------------------------- measurement

	/** World space box axis i, from the public orientation accessor. */
	private static int axisX(RigidBody b, int i) { return b.getOrientation(i); }
	private static int axisY(RigidBody b, int i) { return b.getOrientation(3 + i); }
	private static int axisZ(RigidBody b, int i) { return b.getOrientation(6 + i); }

	/** Overlap of two boxes along a unit axis (Q12); <= 0 means separated. */
	private static int overlapOnAxis(RigidBody a, RigidBody b,
			int lx, int ly, int lz) {
		int pa = 0, pb = 0;
		for(int i = 0; i < 3; i++) {
			int d = RigidBody.mul(axisX(a, i), lx) + RigidBody.mul(axisY(a, i), ly)
					+ RigidBody.mul(axisZ(a, i), lz);
			pa += RigidBody.mul(a.halfExtent(i), d < 0 ? -d : d);
			d = RigidBody.mul(axisX(b, i), lx) + RigidBody.mul(axisY(b, i), ly)
					+ RigidBody.mul(axisZ(b, i), lz);
			pb += RigidBody.mul(b.halfExtent(i), d < 0 ? -d : d);
		}
		int dx = (b.getCenterX() - a.getCenterX()) << 12;
		int dy = (b.getCenterY() - a.getCenterY()) << 12;
		int dz = (b.getCenterZ() - a.getCenterZ()) << 12;
		int d = RigidBody.mul(dx, lx) + RigidBody.mul(dy, ly) + RigidBody.mul(dz, lz);
		return pa + pb - (d < 0 ? -d : d);
	}

	/**
	 * True oriented box overlap in units: the least penetration over the 15
	 * separating axes, 0 when the boxes are apart. Axis aligned bounds would
	 * overestimate this badly for tilted cubes, so the pair tests measure
	 * overlap with the same SAT the solver uses.
	 */
	private static int satPen(RigidBody a, RigidBody b) {
		int best = Integer.MAX_VALUE;
		for(int k = 0; k < 3; k++) {
			for(int which = 0; which < 2; which++) {
				RigidBody ref = which == 0 ? a : b;
				int ov = overlapOnAxis(a, b,
						axisX(ref, k), axisY(ref, k), axisZ(ref, k));
				if(ov <= 0) return 0;
				if(ov < best) best = ov;
			}
		}
		for(int i = 0; i < 3; i++) {
			for(int j = 0; j < 3; j++) {
				long cx = (long) axisY(a, i) * axisZ(b, j)
						- (long) axisZ(a, i) * axisY(b, j);
				long cy = (long) axisZ(a, i) * axisX(b, j)
						- (long) axisX(a, i) * axisZ(b, j);
				long cz = (long) axisX(a, i) * axisY(b, j)
						- (long) axisY(a, i) * axisX(b, j);
				int ln = RigidBody.isqrt(cx * cx + cy * cy + cz * cz);
				if(ln < PARALLEL_EPS) continue;
				int ov = overlapOnAxis(a, b,
						(int) ((cx << 12) / ln), (int) ((cy << 12) / ln),
						(int) ((cz << 12) / ln));
				if(ov <= 0) return 0;
				if(ov < best) best = ov;
			}
		}
		return best == Integer.MAX_VALUE ? 0 : best >> 12;
	}

	/** Every column of the orientation matrix must stay unit length. */
	private static void unitColumns(RigidBody b, String what) {
		long want = (long) F * F;
		long tol = want / 100;
		for(int c = 0; c < 3; c++) {
			int x = b.getOrientation(c), y = b.getOrientation(3 + c),
					z = b.getOrientation(6 + c);
			long len2 = (long) x * x + (long) y * y + (long) z * z;
			check(len2 - want < tol && want - len2 < tol,
					what + ": column " + c + " keeps unit length (len2=" + len2
							+ ", want " + want + ")");
		}
	}

	/** The columns must stay mutually perpendicular. */
	private static void orthogonalColumns(RigidBody b, String what) {
		int dot = RigidBody.mul(axisX(b, 0), axisX(b, 1))
				+ RigidBody.mul(axisY(b, 0), axisY(b, 1))
				+ RigidBody.mul(axisZ(b, 0), axisZ(b, 1));
		check(Math.abs(dot) < F * 16, what + ": columns stay orthogonal (dot=" + dot + ")");
	}

	/** Off diagonal orientation entries: how far the cube is from axis aligned. */
	// How far the body's up axis leans, in Q12. r[2] and r[6] carry yaw about
	// that axis, which is not a lean, so they stay out.
	private static int tilt(RigidBody b) {
		int worst = 0;
		int[] at = {1, 3, 5, 7};
		for(int i = 0; i < at.length; i++) {
			worst = Math.max(worst, Math.abs(b.getOrientation(at[i])));
		}
		return worst;
	}

	// ------------------------------------------------------ world scenarios

	private static void restOnFloor() {
		test("a dropped cube rests on the floor");
		RigidBody b = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		b.reset(0, 3000, 0);
		int lowest = Integer.MAX_VALUE;
		boolean bounced = false;
		int prevVy = 0;
		for(int f = 0; f < 120; f++) {
			b.step(cols, 1, true);
			lowest = Math.min(lowest, b.getCenterY());
			if(prevVy < 0 && b.getVelocityY() > 30) bounced = true;
			prevVy = b.getVelocityY();
		}
		atLeast(lowest, HALF - 72, "never tunnels deeper than the rollback threshold");
		check(bounced, "the first impact bounces (restitution)");
		near(b.getCenterY(), HALF, 6, "comes to rest one half extent up");
		eq(b.getVelocityY(), 0, "resting velocity is zero");
		check(b.isSleeping(), "falls asleep");
	}

	private static void frictionStopsSlide() {
		test("friction stops a sliding cube");
		RigidBody b = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		b.reset(0, 501, 0);
		b.setVelocity(180, 0, 0);
		int lowestVx = Integer.MAX_VALUE;
		boolean stopped = false;
		for(int f = 0; f < 120; f++) {
			b.step(cols, 1, true);
			lowestVx = Math.min(lowestVx, b.getVelocityX());
			if(b.getVelocityX() == 0) stopped = true;
		}
		check(stopped, "comes to a full stop");
		eq(b.getVelocityX(), 0, "stays stopped");
		atLeast(lowestVx, -512, "friction never kicks it hard backwards");
		check(b.isSleeping(), "falls asleep");
	}

	private static void wallStops() {
		test("a wall stops a sliding cube");
		RigidBody b = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor(), wall()};
		b.reset(0, 501, 0);
		b.setVelocity(250, 0, 0);
		int furthest = Integer.MIN_VALUE;
		for(int f = 0; f < 120; f++) {
			b.step(cols, 2, true);
			furthest = Math.max(furthest, b.getCenterX());
		}
		atMost(furthest, 1800 - HALF + 6, "keeps its half extent away from the wall face");
		eq(b.getVelocityX(), 0, "stops against the wall");
		check(b.isSleeping(), "falls asleep");
	}

	private static void deepPenetrationRollback() {
		test("a fast fall is rolled back and subdivided");
		RigidBody b = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		b.reset(0, 501, 0);
		b.setVelocity(0, -1500, 0);
		int lowest = Integer.MAX_VALUE;
		for(int f = 0; f < 200; f++) {
			b.step(cols, 1, true);
			lowest = Math.min(lowest, b.getCenterY());
			if(f == 0) {
				atLeast(b.lastSubsteps, 2, "the impact frame subdivides");
				atLeast(b.getVelocityY(), 200, "restitution returns ~0.2 of the impact speed");
			}
		}
		atLeast(lowest, HALF - 72, "never crosses the floor");
		check(b.isSleeping(), "settles and sleeps");
		near(b.getCenterY(), HALF, 10, "rests on the floor");
		atMost(Math.abs(b.getVelocityX()), 3, "no residual sideways drift");
		atMost(Math.abs(b.getVelocityZ()), 3, "no residual sideways drift");
	}

	private static void spinTumblesAndStaysOrthonormal() {
		test("a spinning cube tumbles without distorting");
		RigidBody b = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		b.reset(0, 1500, 0);
		b.setAngularVelocity(0, 0, 200);
		boolean tumbled = false;
		for(int f = 0; f < 120; f++) {
			b.step(cols, 1, true);
			unitColumns(b, "frame " + f);
			orthogonalColumns(b, "frame " + f);
			if(tilt(b) > 100) tumbled = true;
		}
		check(tumbled, "the spin really turned the cube");
		check(b.isSleeping(), "comes to rest");
		near(b.getCenterY(), HALF, 12, "rests on the floor, possibly on a side");
	}

	private static void slopeDoesNotSink() {
		test("a cube holds on a 20 degree slope");
		RigidBody b = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{ramp(), floor()};
		b.reset(-1281, 1152, 0);
		int lowest = Integer.MAX_VALUE;
		for(int f = 0; f < 120; f++) {
			b.step(cols, 2, true);
			// signed distance of the center above the ramp plane y = -0.364x
			lowest = Math.min(lowest, distAboveRamp(b));
		}
		atLeast(lowest, HALF - 30, "never sinks into the slope");
		check(b.isSleeping(), "held by static friction (mu = 1 > tan 20)");
		near(distAboveRamp(b), HALF, 12, "ends flush on the slope");
		check(b.getCenterX() > -1300 && b.getCenterX() < -900,
				"only slips a little, ends at x=" + b.getCenterX());
		eq(b.getVelocityX(), 0, "stops sliding");
	}

	/** 0.342*x + 0.940*y, scaled by 1000 to stay in integers. */
	private static int distAboveRamp(RigidBody b) {
		return (int) ((342L * b.getCenterX() + 940L * b.getCenterY()) / 1000);
	}

	private static void portalWarp() {
		test("a portal warp flips position and velocity");
		RigidBody b = new RigidBody(HALF);
		b.reset(0, 500, 0);
		b.setVelocity(100, 0, 0);
		float[] m = {-1, 0, 0, 5000, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1};
		int bx = 0, bz = 0, bvx = 0, bvz = 0;
		for(int f = 0; f < 40; f++) {
			b.step(null, 0, false);
			if(f == 9) {
				bx = b.getCenterX(); bz = b.getCenterZ();
				bvx = b.getVelocityX(); bvz = b.getVelocityZ();
			}
			if(f == 10) {
				b.warp(m);
				near(b.getCenterX(), 5000 - bx, 200, "180 degree flip plus translation");
				eq(b.getCenterZ(), -bz, "z mirrors");
				near(b.getVelocityX(), -bvx, 12, "velocity flips with the portal");
				near(b.getVelocityZ(), -bvz, 12, "velocity flips with the portal");
			}
		}
	}

	// ------------------------------------------------------- applied forces

	private static void oneFrameOfForceIsAnImpulse() {
		test("one frame of force is an impulse, held it reaches terminal speed");
		RigidBody b = new RigidBody(HALF);
		b.reset(0, 3000, 0);
		// the magnitude of gravity pushed along +x through the centre, with no
		// colliders so nothing answers it but the drag
		final int push = 20 << 12;
		b.applyForceAt(0, 3000, 0, F, 0, 0, push);
		b.step(null, 0, true);
		eq(b.getVelocityX(), 20, "one frame of force is an impulse of the same number");
		eq(b.getVelocityY(), -20, "gravity is integrated by that same step");
		eq(b.getAngularZ(), 0, "through the centre it does not spin");

		int v = b.getVelocityX();
		b.step(null, 0, true);
		check(b.getVelocityX() < v, "the force is spent: it does not linger a frame");

		// Held across frames it converges on LINEAR_DRAG x magnitude, which for
		// a push the size of gravity is the 500 units/frame a falling cube tops
		// out at. Re-aimed at the body's own centre every frame, so the lever
		// arm stays zero and this measures the linear half alone.
		for(int f = 0; f < 200; f++) {
			b.applyForceAt(b.getCenterX(), b.getCenterY(), b.getCenterZ(),
					1 << 14, 0, 0, push);
			b.step(null, 0, true);
		}
		near(b.getVelocityX(), 500, 1, "a held force settles at the drag terminal speed");
		eq(b.getAngularZ(), 0, "and a Q14 direction is normalised just like a Q12 one");

		// Any scale of direction works, because it is normalised: a raw 3-4-5
		// delta between two points is as good as a unit vector.
		RigidBody c = new RigidBody(HALF);
		c.reset(0, 3000, 0);
		c.applyForceAt(0, 3000, 0, 3, 0, 4, push);
		c.step(null, 0, true);
		near(c.getVelocityX(), 12, 1, "a 3-4-5 delta puts 0.6 of the force along x");
		near(c.getVelocityZ(), 16, 1, "and 0.8 of it along z");
	}

	private static void offCentreForceSpinsTheBody() {
		test("a force off centre spins the body, through the centre it does not");
		RigidBody mid = new RigidBody(HALF);
		RigidBody top = new RigidBody(HALF);
		RigidBody low = new RigidBody(HALF);
		mid.reset(0, 3000, 0);
		top.reset(0, 3000, 0);
		low.reset(0, 3000, 0);
		// The same shove along +x three times: through the centre, at the top
		// face and at the bottom face. r x f decides which way it tumbles.
		final int push = 400 << 12;
		mid.applyForceAt(0, 3000, 0, F, 0, 0, push);
		top.applyForceAt(0, 3500, 0, F, 0, 0, push);
		low.applyForceAt(0, 2500, 0, F, 0, 0, push);
		mid.step(null, 0, true);
		top.step(null, 0, true);
		low.step(null, 0, true);

		eq(mid.getVelocityX(), 400, "the centre push translates");
		eq(top.getVelocityX(), 400, "the lever arm takes nothing from the translation");
		eq(low.getVelocityX(), 400, "all three move alike");
		eq(mid.getAngularZ(), 0, "through the centre there is no spin");
		// w = 3 |r x f| / (2 m h^2), so 3 x 500 x 400 / (2 x 500^2) = 1.2
		// rad/frame here. The tolerance covers the inverse inertia being Q24.
		final int spin = 1200 * F / 1000;
		near(top.getAngularZ(), -spin, 60, "pushed above the centre it tumbles backwards");
		near(low.getAngularZ(), spin, 60, "and below the centre it tumbles forwards");
	}

	private static void sleepingCubeFallsThroughAPortalOpening() {
		test("a sleeping cube falls through a portal opened under it");
		RigidBody b = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		b.reset(0, 3000, 0);
		for(int f = 0; f < 120; f++) b.step(cols, 1, true);
		check(b.isSleeping(), "it comes to rest on the floor and falls asleep");
		near(b.getCenterY(), HALF, 6, "resting one half extent up");

		// The portal opens underneath. Cube.update passes world == false while
		// the body is inside the opening and that skips the whole world pass, so
		// for this body the floor is gone - but one that is still asleep would
		// hang in the air over the hole forever.
		int yBefore = b.getCenterY();
		b.step(cols, 1, false);
		check(!b.isSleeping(), "wakes instead of staying asleep over the opening");
		for(int f = 0; f < 60; f++) b.step(cols, 1, false);
		atMost(b.getCenterY(), yBefore - 1000, "and falls, floor collider passed or not");
		atLeast(b.getVelocityY(), -501, "no faster than the drag terminal speed");
	}

	private static void carriedCubeIgnoresAForce() {
		test("a carried cube ignores a force instead of queueing it up");
		RigidBody b = new RigidBody(HALF);
		b.reset(0, 3000, 0);
		b.setKinematic(true);
		b.applyForceAt(0, 3000, 0, F, 0, 0, 400 << 12);
		// Released again: had the push been queued while carried it would land now
		b.setKinematic(false);
		b.step(null, 0, true);
		eq(b.getVelocityX(), 0, "the hand places a carried body, a push does not move it");
		eq(b.getVelocityY(), -20, "gravity was the only thing that step applied");
	}

	private static void cornerThrowSettles() {
		test("a fast spinning throw into a corner settles");
		RigidBody b = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor(), wall(), wallZ()};
		b.reset(0, 502, 0);
		b.setVelocity(1000, 0, 1000);
		b.setAngularVelocity(120, 60, 90);
		final int W = 1800;
		int maxX = Integer.MIN_VALUE, maxZ = Integer.MIN_VALUE;
		int minY = Integer.MAX_VALUE, maxSpeed = 0;
		for(int f = 0; f < 120; f++) {
			b.step(cols, 3, true);
			maxX = Math.max(maxX, b.getCenterX());
			maxZ = Math.max(maxZ, b.getCenterZ());
			minY = Math.min(minY, b.getCenterY());
			maxSpeed = Math.max(maxSpeed, Math.abs(b.getVelocityX()));
			maxSpeed = Math.max(maxSpeed, Math.abs(b.getVelocityY()));
			maxSpeed = Math.max(maxSpeed, Math.abs(b.getVelocityZ()));
			unitColumns(b, "frame " + f);
		}
		atMost(maxX, W - HALF + 70, "never enters the x wall");
		atMost(maxZ, W - HALF + 70, "never enters the z wall");
		atLeast(minY, HALF - 70, "never crosses the floor");
		atMost(maxSpeed, 2059, "the velocity clamp holds");
		check(b.isSleeping(), "settles instead of being launched");
		eq(b.getVelocityX(), 0, "at rest");
		near(b.getCenterY(), HALF, 10, "rests on the floor");
		check(b.getCenterX() < W && b.getCenterZ() < W, "stays inside the corner");
	}

	private static void deterministic() {
		test("two runs of the same scenario are identical");
		int[] a = trace60();
		int[] c = trace60();
		boolean same = true;
		for(int i = 0; i < a.length; i++) {
			if(a[i] != c[i]) same = false;
		}
		check(same, "fixed point math is deterministic");
	}

	private static int[] trace60() {
		RigidBody b = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		b.reset(0, 3000, 0);
		int[] out = new int[60 * 6];
		for(int f = 0; f < 60; f++) {
			b.step(cols, 1, true);
			out[f * 6] = b.getCenterX();
			out[f * 6 + 1] = b.getCenterY();
			out[f * 6 + 2] = b.getCenterZ();
			out[f * 6 + 3] = b.getVelocityX();
			out[f * 6 + 4] = b.getVelocityY();
			out[f * 6 + 5] = b.getVelocityZ();
		}
		return out;
	}

	// ------------------------------------------------------- pair scenarios

	/** Steps a group of cubes the way GameScreen does: held cubes are posed,
	 *  free cubes step against the world, then every cube meets every other. */
	private static void stepGroup(RigidBody[] group, RigidBody.Collider[] cols,
			int count) {
		for(int i = 0; i < group.length; i++) {
			if(!group[i].isKinematic()) group[i].step(cols, count, true);
		}
		RigidBody.collideBodies(group, group.length);
	}

	private static void twoCubesStack() {
		test("two cubes stack and sleep");
		RigidBody lo = new RigidBody(HALF), hi = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		RigidBody[] group = new RigidBody[]{lo, hi};
		lo.reset(0, 500, 0);
		hi.reset(0, 1520, 0);
		int worst = 0, lowestGap = Integer.MAX_VALUE;
		for(int f = 0; f < 240; f++) {
			stepGroup(group, cols, 1);
			worst = Math.max(worst, satPen(lo, hi));
			lowestGap = Math.min(lowestGap, hi.getCenterY() - lo.getCenterY());
		}
		atMost(worst, 24, "transient overlap stays a small fraction of a cube");
		atLeast(lowestGap, HALF - 72, "the upper cube never sinks through the lower one");
		near(lo.getCenterY(), HALF, 8, "the lower cube still rests on the floor");
		near(hi.getCenterY() - lo.getCenterY(), 2 * HALF, 12, "a stack, not a squash");
		check(lo.isSleeping() && hi.isSleeping(), "the stack comes to rest");
		eq(hi.getVelocityX(), 0, "no residual drift");
		eq(hi.getVelocityZ(), 0, "no residual drift");
	}

	private static void threeCubesStack() {
		test("three dropped cubes settle into a stack");
		RigidBody a = new RigidBody(HALF), b = new RigidBody(HALF),
				c = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		RigidBody[] group = new RigidBody[]{a, b, c};
		a.reset(0, 500, 0);
		b.reset(0, 1600, 0);
		c.reset(0, 2700, 0);
		int worst = 0;
		for(int f = 0; f < 300; f++) {
			stepGroup(group, cols, 1);
			worst = Math.max(worst, Math.max(satPen(a, b),
					Math.max(satPen(b, c), satPen(a, c))));
		}
		atMost(worst, 24, "transient overlap stays a small fraction of a cube");
		check(a.isSleeping() && b.isSleeping() && c.isSleeping(), "all three come to rest");
		// sorted heights must be one cube apart: a column, not a pile
		int y0 = a.getCenterY(), y1 = b.getCenterY(), y2 = c.getCenterY();
		int lo = Math.min(y0, Math.min(y1, y2));
		int hi = Math.max(y0, Math.max(y1, y2));
		int mid = y0 + y1 + y2 - lo - hi;
		near(lo, HALF, 8, "the bottom cube rests on the floor");
		near(mid - lo, 2 * HALF, 16, "one cube size apart");
		near(hi - mid, 2 * HALF, 16, "one cube size apart");
		atMost(Math.abs(a.getCenterX()), 300, "the column stays a column");
		atMost(Math.abs(b.getCenterX()), 300, "the column stays a column");
		atMost(Math.abs(c.getCenterX()), 300, "the column stays a column");
	}

	private static void gravityIsTheSameForEveryMass() {
		test("a light box falls as fast as the unit cube");
		RigidBody chip = new RigidBody(150, 150, 150);
		RigidBody cube = new RigidBody(HALF);
		check(chip.mass != cube.mass, "the two really do weigh different amounts");
		chip.reset(0, 3000, 0);
		cube.reset(0, 3000, 0);
		for(int f = 0; f < 20; f++) {
			chip.step(null, 0, true);
			cube.step(null, 0, true);
		}
		eq(chip.getVelocityY(), cube.getVelocityY(), "same speed after the same fall");
		eq(chip.getCenterY(), cube.getCenterY(), "same height after the same fall");
	}

	private static void bigBoxesStillTumble() {
		test("a box past the unit cube still answers a shove off centre");
		RigidBody cube = new RigidBody(HALF);
		RigidBody b = new RigidBody(1000, 1000, 1000);
		eq(cube.iShift, 0, "the shipped cube keeps its unshifted inertia");
		check(b.iShift > 0, "a bigger box shifts its inverse inertia up to stay resolvable");
		b.reset(0, 3000, 0);
		b.applyForceAt(0, 4000, 0, F, 0, 0, 400 << 12);
		b.step(null, 0, true);
		// w = 3 f / (2 m h), and this box is eight times the cube's mass at twice
		// its size: 3 x 400 / (2 x 8 x 1000) = 0.075 rad/frame.
		near(b.getAngularZ(), -(75 * F / 1000), 4, "it tumbles like a cube, only slower");
		unitColumns(b, "big box");
	}

	private static void nonCubicBoxes() {
		test("boxes keep their own shape, mass and inertia");
		RigidBody slab = new RigidBody(1000, 250, 1000);
		RigidBody crate = new RigidBody(750, 500, 1000);
		RigidBody chip = new RigidBody(20, 20, 20);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		RigidBody[] group = new RigidBody[]{slab, crate};

		eq(slab.getHalfX(), 1000, "half extents are reported per axis");
		eq(slab.getHalfY(), 250, "half extents are reported per axis");
		eq(slab.getHalfZ(), 1000, "half extents are reported per axis");
		eq(slab.mass, 2 * F, "mass follows volume");
		eq(crate.mass, 3 * F, "mass follows volume");
		check(chip.mass > 0 && chip.invMass > 0, "even a sliver stays movable");
		check(slab.invIWorld[4] < slab.invIWorld[0],
				"a slab resists spinning about its short axis more");

		slab.reset(0, 250, 0);
		crate.reset(0, 1900, 0);
		int worst = 0;
		for(int f = 0; f < 300; f++) {
			stepGroup(group, cols, 1);
			worst = Math.max(worst, satPen(slab, crate));
		}
		atMost(worst, 48, "a dropped crate overlaps by less than a tenth of itself");
		near(slab.getCenterY(), 250, 8, "the slab rests on its own half height");
		near(crate.getCenterY(), 2 * 250 + 500, 8, "the crate lands square on the slab");
		atMost(tilt(crate), 60, "the crate stays flat");
		check(slab.isSleeping() && crate.isSleeping(), "the pair comes to rest");
		unitColumns(crate, "crate");
		orthogonalColumns(crate, "crate");
	}

	private static void bigBoxesRest() {
		test("boxes past the unit cube rest on their own height");
		int[][] shapes = {{800, 500, 800}, {1000, 1000, 1000},
				{1500, 200, 1500}, {300, 1200, 300}};
		for(int s = 0; s < shapes.length; s++) {
			RigidBody b = new RigidBody(shapes[s][0], shapes[s][1], shapes[s][2]);
			RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
			b.reset(0, shapes[s][1] + 800, 0);
			int sunk = 0;
			for(int f = 0; f < 300; f++) {
				b.step(cols, 1, true);
				sunk = Math.max(sunk, shapes[s][1] - b.getCenterY());
			}
			String what = shapes[s][0] + "x" + shapes[s][1] + "x" + shapes[s][2];
			near(b.getCenterY(), shapes[s][1], 40, what + " rests on its own height");
			atMost(sunk, 40, what + " never sinks in");
			atMost(tilt(b), 200, what + " stays flat");
			check(b.isSleeping(), what + " comes to rest");
		}
	}

	private static void slidingCubeKnocksRestingOne() {
		test("a sliding cube knocks a resting one along");
		RigidBody rest = new RigidBody(HALF), slider = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		RigidBody[] group = new RigidBody[]{rest, slider};
		rest.reset(0, 500, 0);
		slider.reset(-3000, 500, 0);
		slider.setVelocity(1200, 0, 0);
		int minY = Integer.MAX_VALUE;
		for(int f = 0; f < 200; f++) {
			stepGroup(group, cols, 1);
			minY = Math.min(minY, Math.min(rest.getCenterY(), slider.getCenterY()));
		}
		atLeast(minY, HALF - 72, "momentum never drives a cube through the floor");
		atLeast(rest.getCenterX(), 1000, "the resting cube was shoved along");
		check(slider.getCenterX() < rest.getCenterX(), "the slider stops behind it");
		check(rest.isSleeping() && slider.isSleeping(), "both come to rest");
	}

	/** Held cube pose: ploughs along +x at 60 units per frame, then lifts. */
	private static void carryPose(int frame, RigidBody held) {
		held.moveKinematic(carryX(frame), carryY(frame), 0);
	}

	private static int carryX(int frame) {
		return frame < 60 ? -2400 + frame * 60 : -2400 + 59 * 60;
	}

	private static int carryY(int frame) {
		return frame < 60 ? 700 : 700 + (frame - 59) * 80;
	}

	private static void carriedCubeShovesAndLeaves() {
		test("a carried cube shoves a resting one out of the way");
		RigidBody held = new RigidBody(HALF), free = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		RigidBody[] group = new RigidBody[]{held, free};
		held.reset(-2400, 700, 0);
		held.setKinematic(true);
		free.reset(0, 500, 0);
		int worst = 0, pushed = Integer.MIN_VALUE;
		for(int f = 0; f < 200; f++) {
			carryPose(f, held);
			stepGroup(group, cols, 1);
			worst = Math.max(worst, satPen(held, free));
			pushed = Math.max(pushed, free.getCenterX());
			eq(held.getCenterX(), carryX(f), "frame " + f + ": the hand pose is imposed");
			eq(held.getCenterY(), carryY(f), "frame " + f + ": the hand pose is imposed");
		}
		atMost(worst, 24, "the carried cube does not drive through the other one");
		atLeast(pushed, HALF, "the cube in the way was pushed clear");
		check(free.isSleeping(), "and settled back on the floor");
		near(free.getCenterY(), HALF, 10, "rests on the floor");
	}

	private static void carriedCubeIsNeverMoved() {
		test("no pair ever moves a carried cube");
		RigidBody held = new RigidBody(HALF), free = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		RigidBody[] group = new RigidBody[]{held, free};
		held.reset(-2400, 700, 0);
		held.setKinematic(true);
		free.reset(0, 500, 0);
		boolean moved = false;
		for(int f = 0; f < 200; f++) {
			carryPose(f, held);
			int x = held.getCenterX(), y = held.getCenterY(), z = held.getCenterZ();
			stepGroup(group, cols, 1);
			if(held.getCenterX() != x || held.getCenterY() != y
					|| held.getCenterZ() != z) moved = true;
		}
		check(!moved, "the solver treats a carried cube as immovable");
	}

	// Regression for a kinematic pusher driving a resting cube into the wall
	// behind it. The pair pass cannot see the world, so the world's own contact
	// set is what has to hold the cube (README, Cube vs cube).
	private static void pusherCannotBuryACubeInAWall() {
		test("walking a cube into a wall stops at the wall");
		RigidBody pusher = new RigidBody(300), cube = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor(), wall()};
		RigidBody[] group = new RigidBody[]{pusher, cube};
		pusher.reset(-2000, 300, 0);
		pusher.setKinematic(true);
		cube.reset(600, HALF, 0);
		int worst = Integer.MIN_VALUE, fastest = 0, px = -2000;
		for(int f = 0; f < 120; f++) {
			px += 150;                  // Player.moveZ(-150) every frame
			pusher.moveKinematic(px, 300, 0);
			stepGroup(group, cols, cols.length);
			worst = Math.max(worst, cube.getCenterX());
			fastest = Math.max(fastest, Math.abs(cube.getVelocityX()));
		}
		// 1300 is flush against the wall; the same 40 the pile fuzz allows
		atMost(worst, 1340, "the cube is never driven into the wall");
		atMost(fastest, 400, "nor is it launched back off it");
		near(cube.getCenterY(), HALF, 100, "and stays on the floor");
	}

	// The carried version. Nothing stops an imposed hand from advancing into a
	// cube that cannot move, so Cube.updateHeld reads pressPen and holds still
	// past a jam - which is what keeps the overlap, and so the shove, bounded.
	private static void carriedCubeCannotBuryACubeInAWall() {
		test("a carried cube pressed into a wall cube stops and reports the jam");
		RigidBody held = new RigidBody(HALF), cube = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor(), wall()};
		RigidBody[] group = new RigidBody[]{held, cube};
		held.reset(-2000, HALF, 0);
		held.setKinematic(true);
		cube.reset(600, HALF, 0);
		int worst = Integer.MIN_VALUE, jam = 0, overlap = 0, px = -2000;
		for(int f = 0; f < 120; f++) {
			// Cube.updateHeld: jammed, the hand stops advancing
			if(held.pressPen <= CUBE_JAM_PEN) px += 150;
			held.moveKinematic(px, HALF, 0);
			stepGroup(group, cols, cols.length);
			worst = Math.max(worst, cube.getCenterX());
			jam = Math.max(jam, held.pressPen >> 12);
			overlap = Math.max(overlap, satPen(held, cube));
		}
		atMost(worst, 1340, "the cube in front is not driven into the wall");
		atLeast(jam, 100, "the hand is told it is jammed, so the carry lets go");
		atMost(overlap, 200, "and the carried cube does not sink into it");
		// Cube.updateHeld holds the hand still only while it is still pressing
		// in, so the direction has to be the one the press goes along: from the
		// carried cube towards the cube in front, which is +x here.
		check(held.pressNX > 0 && Math.abs(held.pressNY) < held.pressNX
				&& Math.abs(held.pressNZ) < held.pressNX,
				"and it reports pressing along +x, into the wall");
	}

	// A parked hand inside a sleeping cube used to be ignored whole: the pair is
	// skipped when both bodies count as immovable, and a sleeper with a cube
	// inside it has nothing to wake it - no closing speed, no moving neighbour.
	// Nothing then reports how deep the hand is pressed, so Cube.updateHeld
	// takes another step and the overlap ratchets up a step at a time.
	private static void aSleeperAnswersACubeInsideIt() {
		test("a sleeping cube answers a carried cube parked inside it");
		RigidBody held = new RigidBody(HALF), rest = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		RigidBody[] group = new RigidBody[]{held, rest};
		rest.reset(0, HALF, 0);
		held.reset(0, 2000, 0);
		held.setKinematic(true);
		for(int f = 0; f < 240; f++) {
			held.moveKinematic(0, 2000, 0);
			stepGroup(group, cols, cols.length);
		}
		check(rest.isSleeping(), "the cube on the floor has fallen asleep");
		// lowered into it over four frames, then the hand parks
		for(int f = 0; f < 64; f++) {
			int y = f < 4 ? 2000 - 300 * (f + 1) : 800;
			held.moveKinematic(0, y, f < 4 ? 75 * (f + 1) : 300);
			stepGroup(group, cols, cols.length);
		}
		check(!rest.isSleeping(), "and a cube inside it wakes it");
		atLeast(satPen(held, rest), 400, "which it is, this deep");
		atLeast(held.pressPen, CUBE_JAM_PEN, "so the hand is told it is jammed");
	}

	private static void supportLossWakesTheCubeAbove() {
		test("a cube wakes when its support is teleported away");
		RigidBody shelf = new RigidBody(HALF), top = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		RigidBody[] group = new RigidBody[]{shelf, top};
		shelf.reset(0, 1500, 0);
		shelf.setKinematic(true);
		top.reset(0, 2500, 0);
		boolean sleptOnShelf = false, fell = false;
		int yBefore = 0;
		for(int f = 0; f < 200; f++) {
			shelf.moveKinematic(0, 1500, f < 40 ? 0 : 20000);
			stepGroup(group, cols, 1);
			if(f < 39) {
				near(top.getCenterY(), 2500, 24, "frame " + f + ": rests on the shelf");
				if(top.isSleeping()) sleptOnShelf = true;
			}
			if(f == 39) yBefore = top.getCenterY();
			if(f > 41 && top.getCenterY() < yBefore - 100) fell = true;
		}
		check(sleptOnShelf, "falls asleep while the shelf holds it");
		check(fell, "wakes and falls instead of floating where the shelf was");
		check(top.isSleeping(), "comes to rest on the floor");
		near(top.getCenterY(), HALF, 10, "ends on the floor");
	}

	private static void releasedCubeSettlesInsteadOfExploding() {
		test("a released cube eases out of a deep overlap");
		RigidBody held = new RigidBody(HALF), rest = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		RigidBody[] group = new RigidBody[]{held, rest};
		held.reset(0, 2000, 0);
		held.setKinematic(true);
		rest.reset(0, 500, 0);
		int maxSpeed = 0, minY = Integer.MAX_VALUE;
		for(int f = 0; f < 200; f++) {
			if(f < 20) {
				// the hand presses the carried cube into the other one
				held.moveKinematic(0, 2000 - f * 40, 0);
			} else if(f == 20) {
				held.setKinematic(false);
				held.setVelocity(0, 0, 0);
			}
			stepGroup(group, cols, 1);
			maxSpeed = Math.max(maxSpeed, Math.abs(held.getVelocityX()));
			maxSpeed = Math.max(maxSpeed, Math.abs(held.getVelocityY()));
			maxSpeed = Math.max(maxSpeed, Math.abs(rest.getVelocityX()));
			maxSpeed = Math.max(maxSpeed, Math.abs(rest.getVelocityY()));
			minY = Math.min(minY, Math.min(held.getCenterY(), rest.getCenterY()));
		}
		atMost(maxSpeed, 899, "deep overlap never launches a cube");
		// While the hand jams a cube into the floor the trapped cube has
		// nowhere to go and dips. It stays bounded (measured 267 units of a 1000
		// unit cube, deepest at the peak of the press) and heals within a few
		// frames of the release, which beats the old behaviour where the carried
		// cube slid straight through it. Elsewhere in this suite the floor bound
		// is 72 units.
		atLeast(minY, HALF - 300, "neither cube is driven through the floor");
		check(held.isSleeping() && rest.isSleeping(), "both come to rest");
		near(held.getCenterY(), 3 * HALF, 24, "it ends up stacked on the other cube");
		near(rest.getCenterY(), HALF, 12, "the other cube stayed on the floor");
	}

	private static void cubeRidesOnACarriedCube() {
		test("a cube resting on a carried cube rides along");
		RigidBody held = new RigidBody(HALF), rider = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		RigidBody[] group = new RigidBody[]{held, rider};
		held.reset(0, 500, 0);
		rider.reset(0, 1500, 0);
		for(int f = 0; f < 120; f++) stepGroup(group, cols, 1);
		check(rider.getCenterY() > 1400, "they settle into a stack first");
		held.setKinematic(true);
		int worst = 0;
		for(int f = 0; f < 80; f++) {
			held.moveKinematic(f * 40, 500 + f * 10, 0);
			stepGroup(group, cols, 1);
			worst = Math.max(worst, satPen(held, rider));
		}
		atMost(worst, 24, "the rider is not driven through the carried cube");
		atLeast(rider.getCenterX(), 2000, "friction drags the rider along");
		near(rider.getCenterY() - held.getCenterY(), 2 * HALF, 200,
				"and stays on top of it");
	}

	/** Regression: the wake test used to need a neighbour faster than 48 units
	 *  per frame, so at a slow walk the hand velocity stayed under it and a
	 *  carried cube slid straight through a sleeping one - a full cube of
	 *  overlap, with the resting cube never budging. A moving carried cube now
	 *  always wakes what it touches. */
	private static void slowCarriedCubePushesASleepingOne() {
		test("a slowly carried cube pushes a sleeping one aside");
		RigidBody held = new RigidBody(HALF), rest = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		RigidBody[] group = new RigidBody[]{held, rest};
		held.reset(-3000, 500, 0);
		held.setKinematic(true);
		rest.reset(0, 500, 0);
		boolean slept = false;
		for(int f = 0; f < 120; f++) {
			held.moveKinematic(-3000, 500, 0);
			stepGroup(group, cols, 1);
			if(rest.isSleeping()) slept = true;
		}
		check(slept, "the cube in the way falls asleep while the hand is parked");
		int worst = 0;
		for(int f = 0; f < 120; f++) {
			held.moveKinematic(-3000 + (f + 1) * 30, 500, 0);
			stepGroup(group, cols, 1);
			worst = Math.max(worst, satPen(held, rest));
		}
		atMost(worst, 24, "the carried cube never passes through the sleeper");
		atLeast(rest.getCenterX(), HALF, "it is shoved clear instead");
		near(rest.getCenterY(), HALF, 24, "and stays on the floor");
	}

	/** Cube.THROW_SPEED is 500 units/frame. At the old 320 a released cube died
	 *  inside two cube widths - the floor bleeds ~20 units/frame off anything
	 *  sliding, FRICTION being 1.0 - arrived doing 307 and handed the target a
	 *  third of a cube width of movement, which read as the target not moving.
	 *  This pins the release speed to something that visibly shoves. */
	private static void gameThrowSpeedMovesTheTarget() {
		test("a cube released at the game's throw speed shoves the target");
		RigidBody thrown = new RigidBody(HALF), target = new RigidBody(HALF);
		RigidBody.Collider[] cols = new RigidBody.Collider[]{floor()};
		RigidBody[] group = new RigidBody[]{thrown, target};
		target.reset(0, 500, 0);
		thrown.reset(-2000, 500, 0);
		for(int f = 0; f < 150; f++) stepGroup(group, cols, 1);
		check(target.isSleeping(), "the target is at rest before it is hit");
		// Cube.drop(): THROW_SPEED along the look direction, plus a 40 lift
		thrown.setVelocity(500, 40, 0);
		int peak = 0;
		for(int f = 0; f < 300; f++) {
			stepGroup(group, cols, 1);
			peak = Math.max(peak, target.getVelocityX());
		}
		atLeast(peak, 200, "the target is shoved along");
		atLeast(target.getCenterX(), 1500, "and travels at least a cube and a half");
		check(target.isSleeping(), "it settles again afterwards");
		near(target.getCenterY(), HALF, 24, "on the floor, not launched");
	}

	// --------------------------------------------------------------- fuzzing

	/** Deterministic LCG. Every case is reproducible from its number, because
	 *  the seed depends only on where the pass is in the sequence. */
	private static int fuzzSeed;

	private static int rnd(int bound) {
		fuzzSeed = fuzzSeed * 1103515245 + 12345;
		if(bound <= 0) return 0;
		return ((fuzzSeed >>> 16) & 0x7fff) % bound;
	}

	private static int rndFrom(int lo, int hi) { return lo + rnd(hi - lo + 1); }

	/** True while the orientation is still a usable rotation: unit length
	 *  columns that stay mutually perpendicular, at the tolerances
	 *  unitColumns and orthogonalColumns use. */
	private static boolean matrixSane(RigidBody b) {
		long want = (long) F * F, tol = want / 100;
		for(int c = 0; c < 3; c++) {
			int x = b.getOrientation(c), y = b.getOrientation(3 + c),
					z = b.getOrientation(6 + c);
			long len2 = (long) x * x + (long) y * y + (long) z * z;
			if(len2 - want >= tol || want - len2 >= tol) return false;
		}
		int dot = RigidBody.mul(axisX(b, 0), axisX(b, 1))
				+ RigidBody.mul(axisY(b, 0), axisY(b, 1))
				+ RigidBody.mul(axisZ(b, 0), axisZ(b, 1));
		return Math.abs(dot) < F * 16;
	}

	private static final int FUZZ_CASES = 60;
	private static final int FUZZ_FRAMES = 240;
	private static final int FUZZ_SEED = 20260915;
	/** Flip this to see every case's numbers while tuning the bounds below. */
	private static final boolean FUZZ_STATS = false;

	// worst cases seen by fuzzPass, cleared by randomPiles
	private static int fWorstPen, fPenCase;
	private static int fLowestY, fLowCase;
	private static int fFastest, fFastCase;
	private static int fMaxX, fMaxZ, fMinX, fMinZ, fWallCase;
	private static int fBadMatrix, fMatrixCase;
	private static int fAwakeCases, fAwakeCase, fAwakeSpeed;

	/** Throws 2 to 5 cubes into the floor/wall/wallZ corner from random
	 *  heights with random velocities and spins, and hammers the invariants
	 *  every frame. Returns a checksum of the final state so two passes from
	 *  the same seed can be compared. */
	private static long fuzzPass() {
		long sum = 0;
		fuzzSeed = FUZZ_SEED;
		for(int c = 0; c < FUZZ_CASES; c++) {
			int n = rndFrom(2, 5);
			RigidBody.Collider[] cols = arena();
			RigidBody[] group = new RigidBody[n];
			for(int i = 0; i < n; i++) {
				RigidBody b = new RigidBody(HALF);
				b.reset(rndFrom(-7000, 800), rndFrom(1500, 9500), rndFrom(-7000, 800));
				b.setVelocity(rndFrom(-700, 700), rndFrom(-200, 400), rndFrom(-700, 700));
				b.setAngularVelocity(rndFrom(-300, 300), rndFrom(-300, 300),
						rndFrom(-300, 300));
				group[i] = b;
			}
			boolean badMatrix = false;
			int caseMaxX = Integer.MIN_VALUE, caseMaxZ = Integer.MIN_VALUE;
			for(int f = 0; f < FUZZ_FRAMES; f++) {
				stepGroup(group, cols, cols.length);
				for(int i = 0; i < n; i++) {
					RigidBody b = group[i];
					if(!matrixSane(b)) badMatrix = true;
					if(b.getCenterY() < fLowestY) {
						fLowestY = b.getCenterY();
						fLowCase = c;
					}
					int sp = Math.max(Math.abs(b.getVelocityX()),
							Math.max(Math.abs(b.getVelocityY()),
									Math.abs(b.getVelocityZ())));
					if(sp > fFastest) { fFastest = sp; fFastCase = c; }
					if(b.getCenterX() > fMaxX) { fMaxX = b.getCenterX(); fWallCase = c; }
					if(b.getCenterZ() > fMaxZ) { fMaxZ = b.getCenterZ(); fWallCase = c; }
					if(b.getCenterX() < fMinX) { fMinX = b.getCenterX(); fWallCase = c; }
					if(b.getCenterZ() < fMinZ) { fMinZ = b.getCenterZ(); fWallCase = c; }
					caseMaxX = Math.max(caseMaxX, b.getCenterX());
					caseMaxZ = Math.max(caseMaxZ, b.getCenterZ());
					for(int j = i + 1; j < n; j++) {
						int pen = satPen(b, group[j]);
						if(pen > fWorstPen) { fWorstPen = pen; fPenCase = c; }
					}
				}
			}
			if(badMatrix) { fBadMatrix++; fMatrixCase = c; }
			int awake = 0, awakeSpeed = 0, caseLow = Integer.MAX_VALUE;
			for(int i = 0; i < n; i++) {
				RigidBody b = group[i];
				sum = sum * 1000003 + b.getCenterX() * 31 + b.getCenterY() * 7
						+ b.getOrientation(0) + (b.isSleeping() ? 1 : 0);
				caseLow = Math.min(caseLow, b.getCenterY());
				if(!b.isSleeping()) {
					awake++;
					awakeSpeed = Math.max(awakeSpeed,
							Math.max(Math.abs(b.getVelocityX()),
									Math.max(Math.abs(b.getVelocityY()),
											Math.abs(b.getVelocityZ()))));
				}
			}
			if(awake > 0) {
				fAwakeCases++;
				if(awakeSpeed > fAwakeSpeed) { fAwakeSpeed = awakeSpeed; fAwakeCase = c; }
			}
			if(FUZZ_STATS) {
				System.out.println("   case " + c + ": n=" + n + " awake=" + awake
						+ " awakeSpeed=" + awakeSpeed + " lowestY=" + caseLow
						+ " maxX=" + caseMaxX + " maxZ=" + caseMaxZ);
			}
		}
		return sum;
	}

	private static void randomPiles() {
		test("randomized piles never tunnel, explode, distort or hang");
		final int W = 1800, WEST = -8000;
		fWorstPen = 0; fPenCase = -1;
		fLowestY = Integer.MAX_VALUE; fLowCase = -1;
		fFastest = 0; fFastCase = -1;
		fMaxX = Integer.MIN_VALUE; fMaxZ = Integer.MIN_VALUE; fWallCase = -1;
		fMinX = Integer.MAX_VALUE; fMinZ = Integer.MAX_VALUE;
		fBadMatrix = 0; fMatrixCase = -1;
		fAwakeCases = 0; fAwakeCase = -1; fAwakeSpeed = 0;
		long first = fuzzPass();
		if(FUZZ_STATS) {
			System.out.println("   worst: pen=" + fWorstPen + " (case " + fPenCase
					+ ") fastest=" + fFastest + " (case " + fFastCase
					+ ") lowestY=" + fLowestY + " (case " + fLowCase
					+ ") maxX=" + fMaxX + " maxZ=" + fMaxZ + " (case " + fWallCase
					+ ") badMatrix=" + fBadMatrix + " awakeCases=" + fAwakeCases
					+ " worstAwakeSpeed=" + fAwakeSpeed);
		}
		long second = fuzzPass();
		check(first == second, "two passes over the same " + FUZZ_CASES
				+ " cases are identical (" + first + " vs " + second + ")");
		eq(fBadMatrix, 0, "every orientation stays a rotation (worst case "
				+ fMatrixCase + ")");
		atLeast(fLowestY, HALF - 72, "no cube is driven through the floor (case "
				+ fLowCase + ")");
		atMost(fMaxX, W - HALF + 40, "no cube is driven through the east wall (case "
				+ fWallCase + ")");
		atMost(fMaxZ, W - HALF + 40, "no cube is driven through the north wall (case "
				+ fWallCase + ")");
		atLeast(fMinX, WEST + HALF - 40,
				"no cube is driven through the west wall (case " + fWallCase + ")");
		atLeast(fMinZ, WEST + HALF - 40,
				"no cube is driven through the south wall (case " + fWallCase + ")");
		// the solver clamps linear velocity to MAX_LINEAR, 2048 units per frame
		atMost(fFastest, 2100, "the solver's own velocity clamp holds (case "
				+ fFastCase + ")");
		// A hard impact is deepest for one frame and gone the next. Solving one
		// pair to completion peaked at 32, which is where the 40 the wall bounds
		// above allow came from; the batched two phase pass converges less per
		// pair and peaks at 99, with 9 of the 60 cases over 40. Put this back to
		// 40 if the pair pass ever goes back to nested rounds.
		atMost(fWorstPen, 100, "no pair is left interpenetrated (case " + fPenCase + ")");
		// A cube balanced exactly on the seam between two others keeps rocking
		// and never sleeps - a documented limitation, so a few cases are allowed
		// to end with one cube awake, but it has to be all but motionless.
		atMost(fAwakeCases, FUZZ_CASES / 10, "piles that never settle (worst case "
				+ fAwakeCase + ")");
		atMost(fAwakeSpeed, 24, "a cube left awake is nearly still (case "
				+ fAwakeCase + ")");
	}

	// ------------------------------------------------------------ framework

	private static int checks;
	private static int failures;
	private static String current = "";

	private static void test(String name) {
		current = name;
		System.out.println("-- " + name);
	}

	private static void check(boolean ok, String what) {
		checks++;
		if(!ok) {
			failures++;
			System.out.println("   FAIL " + what);
		}
	}

	private static void eq(int got, int want, String what) {
		check(got == want, what + " (want " + want + ", got " + got + ")");
	}

	private static void near(int got, int want, int tol, String what) {
		check(Math.abs(got - want) <= tol,
				what + " (want " + want + " +/-" + tol + ", got " + got + ")");
	}

	private static void atLeast(int got, int min, String what) {
		check(got >= min, what + " (want >= " + min + ", got " + got + ")");
	}

	private static void atMost(int got, int max, String what) {
		check(got <= max, what + " (want <= " + max + ", got " + got + ")");
	}

	public static void main(String[] args) {
		restOnFloor();
		frictionStopsSlide();
		wallStops();
		deepPenetrationRollback();
		spinTumblesAndStaysOrthonormal();
		slopeDoesNotSink();
		portalWarp();
		sleepingCubeFallsThroughAPortalOpening();
		oneFrameOfForceIsAnImpulse();
		offCentreForceSpinsTheBody();
		carriedCubeIgnoresAForce();
		cornerThrowSettles();
		deterministic();

		twoCubesStack();
		threeCubesStack();
		gravityIsTheSameForEveryMass();
		bigBoxesStillTumble();
		nonCubicBoxes();
		bigBoxesRest();
		slidingCubeKnocksRestingOne();
		carriedCubeShovesAndLeaves();
		carriedCubeIsNeverMoved();
		pusherCannotBuryACubeInAWall();
		carriedCubeCannotBuryACubeInAWall();
		aSleeperAnswersACubeInsideIt();
		supportLossWakesTheCubeAbove();
		releasedCubeSettlesInsteadOfExploding();
		cubeRidesOnACarriedCube();
		slowCarriedCubePushesASleepingOne();
		gameThrowSpeedMovesTheTarget();
		randomPiles();

		System.out.println();
		System.out.println(checks + " checks, " + failures + " failure(s)");
		if(failures > 0) {
			System.out.println("FAILED");
			System.exit(1);
		}
		System.out.println("OK");
	}
}
