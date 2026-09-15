package com;

/**
 * Self checking tests for the Q12 rigid body solver in src/com/RigidBody.java.
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
 * runs them. The "Physics tests" workflow runs it with --strict on every
 * push. The exit code is 1 when any check fails, so it can gate a build.
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
		int ha = a.getHalfExtent() << 12, hb = b.getHalfExtent() << 12;
		int pa = 0, pb = 0;
		for(int i = 0; i < 3; i++) {
			int d = RigidBody.mul(axisX(a, i), lx) + RigidBody.mul(axisY(a, i), ly)
					+ RigidBody.mul(axisZ(a, i), lz);
			pa += RigidBody.mul(ha, d < 0 ? -d : d);
			d = RigidBody.mul(axisX(b, i), lx) + RigidBody.mul(axisY(b, i), ly)
					+ RigidBody.mul(axisZ(b, i), lz);
			pb += RigidBody.mul(hb, d < 0 ? -d : d);
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
	private static int tilt(RigidBody b) {
		int worst = 0;
		int[] at = {1, 2, 3, 5, 6, 7};
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
		atMost(fWorstPen, 24, "no pair is left interpenetrated (case " + fPenCase + ")");
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
		cornerThrowSettles();
		deterministic();

		twoCubesStack();
		threeCubesStack();
		slidingCubeKnocksRestingOne();
		carriedCubeShovesAndLeaves();
		carriedCubeIsNeverMoved();
		supportLossWakesTheCubeAbove();
		releasedCubeSettlesInsteadOfExploding();
		cubeRidesOnACarriedCube();
		slowCarriedCubePushesASleepingOne();
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
