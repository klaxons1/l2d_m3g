package com;

/**
 * Standalone test harness for the Q12 rigid body solver.
 *
 * It builds small hand made triangle/quad worlds, runs RigidBody.step for a
 * number of frames and prints the resulting state trace as CSV. It is the
 * companion to RigidBodyTests.java: when one of those checks fails and the
 * numbers need eyeballing frame by frame, this dumps them. Like the tests it
 * stays free of M3G dependencies (it only uses RigidBody), so both build
 * without the rest of the game.
 *
 * Single body scenarios print one row per frame. The multi body ones
 * (stack2, stack3, sweep, carry, supportloss) drive several cubes the way
 * GameScreen does - pose the held ones, step the free ones against the
 * world, then RigidBody.collideBodies for the cube vs cube pass - and print
 * one row per body per frame, prefixed with the body index.
 */
public final class RigidBodyHarness {

	private static Collider quad(int[] a, int[] b, int[] c, int[] d) {
		Collider col = new Collider();
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

	private static Collider floor() {
		final int S = 20000;
		// winding gives a downward normal (into the solid), as in engine rooms
		return quad(
				new int[]{-S, 0, -S}, new int[]{S, 0, -S},
				new int[]{S, 0, S}, new int[]{-S, 0, S});
	}

	private static Collider wall() {
		final int S = 20000, H = 10000, W = 1800;
		// solid x > W, normal +x
		return quad(
				new int[]{W, 0, -S}, new int[]{W, H, -S},
				new int[]{W, H, S}, new int[]{W, 0, S});
	}

	private static Collider wallZ() {
		final int S = 20000, H = 10000, W = 1800;
		// solid z > W, normal +z
		return quad(
				new int[]{-S, 0, W}, new int[]{S, 0, W},
				new int[]{S, H, W}, new int[]{-S, H, W});
	}

	private static Collider ramp() {
		// ~20 degree slope rising in -x (plane y = -0.364x), solid below
		return quad(
				new int[]{-4000, 1456, 20000}, new int[]{-4000, 1456, -20000},
				new int[]{0, 0, -20000}, new int[]{0, 0, 20000});
	}

	/** Held cube pose of the "carry" scenario: ploughs along +x at 60 units
	 *  per frame, then lifts straight up. Deterministic, and it exercises a
	 *  kinematic body sweeping through a resting one. */
	private static void carryPose(int frame, RigidBody held) {
		if(frame < 60) {
			held.moveKinematic(-2400 + frame * 60, 700, 0);
		} else {
			held.moveKinematic(-2400 + 59 * 60, 700 + (frame - 59) * 80, 0);
		}
	}

	/** Kinematic shelf of the "supportloss" scenario: it holds a cube up for
	 *  40 frames and is then teleported away, so the cube must wake and fall
	 *  (collideBodies compares supportBody against prevSupport). */
	private static void shelfPose(int frame, RigidBody shelf) {
		shelf.moveKinematic(0, 1500, frame < 40 ? 0 : 20000);
	}

	private static void printState(int frame, RigidBody body) {
		printState(frame, -1, body);
	}

	/** index >= 0 adds a body column, used by the multi body scenarios. */
	private static void printState(int frame, int index, RigidBody body) {
		StringBuffer sb = new StringBuffer();
		sb.append(frame);
		if(index >= 0) sb.append(',').append(index);
		sb.append(',').append(body.getCenterX());
		sb.append(',').append(body.getCenterY());
		sb.append(',').append(body.getCenterZ());
		sb.append(',').append(body.getVelocityX());
		sb.append(',').append(body.getVelocityY());
		sb.append(',').append(body.getVelocityZ());
		for(int i = 0; i < 9; i++) sb.append(',').append(body.getOrientation(i));
		sb.append(',').append(body.isSleeping() ? 1 : 0);
		sb.append(',').append(body.lastSubsteps);
		sb.append(',').append(body.getContactCount());
		System.out.println(sb.toString());
	}

	private static void run(String scenario, int frames) {
		RigidBody body = new RigidBody(500);
		RigidBody[] group = null;
		Collider[] cols = new Collider[4];
		int count = 0;

		if(scenario.equals("drop")) {
			cols[count++] = floor();
			body.reset(0, 3000, 0);
		} else if(scenario.equals("slide")) {
			cols[count++] = floor();
			body.reset(0, 501, 0);
			body.setVelocity(180, 0, 0);
		} else if(scenario.equals("wall")) {
			cols[count++] = floor();
			cols[count++] = wall();
			body.reset(0, 501, 0);
			body.setVelocity(250, 0, 0);
		} else if(scenario.equals("spin")) {
			cols[count++] = floor();
			body.reset(0, 1500, 0);
			body.setAngularVelocity(0, 0, 200); // ~0.05 rad/frame
		} else if(scenario.equals("fastdrop")) {
			cols[count++] = floor();
			body.reset(0, 501, 0);
			body.setVelocity(0, -1500, 0);
		} else if(scenario.equals("ramp")) {
			cols[count++] = ramp();
			cols[count++] = floor();
			body.reset(-1281, 1152, 0);
		} else if(scenario.equals("corner")) {
			cols[count++] = floor();
			cols[count++] = wall();
			cols[count++] = wallZ();
			body.reset(0, 502, 0);
			body.setVelocity(1000, 0, 1000);
			body.setAngularVelocity(120, 60, 90);
		} else if(scenario.equals("warp")) {
			body.reset(0, 500, 0);
			body.setVelocity(100, 0, 0);
		} else if(scenario.equals("stack2")) {
			// one cube dropped onto another: they must settle into a stack
			cols[count++] = floor();
			group = new RigidBody[]{body, new RigidBody(500)};
			group[0].reset(0, 500, 0);
			group[1].reset(0, 1520, 0);
		} else if(scenario.equals("stack3")) {
			// three cubes falling onto each other, the hardest convergence case
			cols[count++] = floor();
			group = new RigidBody[]{body, new RigidBody(500), new RigidBody(500)};
			group[0].reset(0, 500, 0);
			group[1].reset(0, 1600, 0);
			group[2].reset(0, 2700, 0);
		} else if(scenario.equals("sweep")) {
			// a sliding cube knocks a resting one along the floor
			cols[count++] = floor();
			group = new RigidBody[]{body, new RigidBody(500)};
			group[0].reset(0, 500, 0);
			group[1].reset(-3000, 500, 0);
			group[1].setVelocity(1200, 0, 0);
		} else if(scenario.equals("carry")) {
			// a carried (kinematic) cube ploughs through a resting one, then
			// lifts away; the resting cube must be shoved and settle again
			cols[count++] = floor();
			group = new RigidBody[]{body, new RigidBody(500)};
			group[0].reset(-2400, 700, 0);
			group[0].setKinematic(true);
			group[1].reset(0, 500, 0);
		} else if(scenario.equals("supportloss")) {
			// a cube rests on a kinematic shelf that is teleported away: the
			// cube must wake up and fall to the floor instead of floating
			cols[count++] = floor();
			group = new RigidBody[]{body, new RigidBody(500)};
			group[0].reset(0, 1500, 0);
			group[0].setKinematic(true);
			group[1].reset(0, 2500, 0);
		} else if(scenario.equals("blast")) {
			// one frame of a force at the top face: the explosion case, where a
			// single frame of force lands as an impulse of the same number and
			// the half extent of lever arm tumbles the cube as it launches
			cols[count++] = floor();
			body.reset(0, 500, 0);
		} else if(scenario.equals("wind")) {
			// the same kind of push held across every frame, in free space so
			// nothing answers it but the drag: it converges on LINEAR_DRAG x
			// magnitude and stays exactly straight
			body.reset(0, 3000, 0);
		} else if(scenario.equals("drive")) {
			// the wheel case: a force held every frame below the centre, on the
			// floor and into a wall, so it drives instead of flying off it
			cols[count++] = floor();
			cols[count++] = wall();
			body.reset(-1500, 500, 0);
		} else {
			System.out.println("UNKNOWN SCENARIO " + scenario);
			return;
		}

		float[] warpMatrix = {
			-1, 0, 0, 5000,
			0, 1, 0, 0,
			0, 0, -1, 0,
			0, 0, 0, 1
		};

		boolean carry = scenario.equals("carry");
		boolean shelf = scenario.equals("supportloss");
		boolean blast = scenario.equals("blast");
		boolean wind = scenario.equals("wind");
		boolean drive = scenario.equals("drive");

		for(int f = 0; f < frames; f++) {
			if(group != null) {
				// exactly the order GameScreen uses: held cubes are posed,
				// every free cube steps against the world geometry, then all
				// cubes are collided against each other
				if(carry) carryPose(f, group[0]);
				if(shelf) shelfPose(f, group[0]);
				for(int i = 0; i < group.length; i++) {
					if(!group[i].isKinematic()) {
						group[i].step(cols, count, true);
					}
				}
				RigidBody.collideBodies(group, group.length);
				for(int i = 0; i < group.length; i++) printState(f, i, group[i]);
				continue;
			}
			boolean collide = !scenario.equals("warp");
			if(blast && f == 0) {
				// 150 units/frame of translation and 0.45 rad/frame of spin
				body.applyForceAt(0, 1000, 0, 1, 0, 0, 150 << 12);
			}
			if(wind) {
				// re-aimed at the body's own centre, so the lever arm stays zero
				body.applyForceAt(body.getCenterX(), body.getCenterY(),
						body.getCenterZ(), 1, 0, 0, 40 << 12);
			}
			if(drive) {
				body.applyForceAt(body.getCenterX(), body.getCenterY() - 400,
						body.getCenterZ(), 1, 0, 0, 40 << 12);
			}
			body.step(count == 0 ? null : cols, count, collide);
			if(scenario.equals("warp") && f == 10) body.warp(warpMatrix);
			printState(f, body);
		}
	}

	public static void main(String[] args) {
		String scenario = args.length > 0 ? args[0] : "drop";
		int frames = args.length > 1 ? Integer.parseInt(args[1]) : 120;
		run(scenario, frames);
	}
}
