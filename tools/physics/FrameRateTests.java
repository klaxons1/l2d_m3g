package com;

// Frame rate invariance. Each scenario is driven through FPS.tick with a
// synthetic wall clock, the way GameScreen.paint does, at several rates and
// compared against the nominal 20 fps one. Fixed point integration of the same
// physical second is not bit-identical across rates, so the checks carry a
// tolerance; the frame counted code they replaced was off by the frame ratio.
//
// Run with tools/physics/run_tests.sh fps.
public final class FrameRateTests {

	private static int checks, failures;

	private static final int[] RATES = {20, 10, 40, 60, 120};

	private static void check(String what, int got, int want, int tol) {
		checks++;
		int d = got - want;
		if(d < 0) d = -d;
		if(d > tol) {
			failures++;
			System.out.println("FAIL " + what + ": got " + got + ", want " + want + " +/-" + tol);
		}
	}

	private static int tol(int value, int percent) {
		if(value < 0) value = -value;
		int t = value * percent / 100;
		return t < 1 ? 1 : t;
	}

	// FPS keeps a private last tick, so the wall clock is never reset and stays
	// monotonic across scenarios. Microseconds, because an integer millisecond
	// clock cannot express 60 fps.
	private static long wall;

	private static void frame(int fps) {
		wall += 1000000L / fps;
		FPS.tick(wall / 1000L);
	}

	private static int frames(int fps, int seconds) {
		return fps * seconds;
	}

	// GameObject.setCharacterSize(2005): the Character constructor zeroes the
	// size and the game sets it afterwards.
	private static Character player() {
		Character ch = new Character(0, 0);
		ch.set(802, 1503);
		return ch;
	}

	// Scene.update: gravity into the speed, the step lag into the position.
	private static int gravityQ, lagQ;

	private static void gravity(Character ch) {
		int dt = FPS.dt;
		gravityQ += 20 * dt;
		int g = gravityQ >> 12;
		gravityQ &= SolverMath.F - 1;
		lagQ += (int) (20L * dt * (dt - SolverMath.F) / (2 * SolverMath.F));
		int lag = lagQ >> 12;
		lagQ &= SolverMath.F - 1;
		ch.getSpeed().y -= g;
		ch.getPosition().y += lag;
	}

	private static int fall(int fps, int seconds) {
		gravityQ = lagQ = 0;
		Character ch = player();
		ch.getPosition().set(0, 20000, 0);
		for(int i = frames(fps, seconds); i > 0; i--) {
			frame(fps);
			gravity(ch);
			ch.update();
		}
		return ch.getPosition().y - 20000;
	}

	// GameObject.updateMovement on the floor: the moveZ input of
	// Player.moveForward, gravity, the integration and the floor damping.
	private static int dampX, dampY, dampZ;

	private static int walk(int fps, int seconds) {
		gravityQ = lagQ = dampX = dampY = dampZ = 0;
		Character ch = player();
		for(int i = frames(fps, seconds); i > 0; i--) {
			frame(fps);
			ch.standOn(ch.getPosition().y + 1, 0, 0);
			ch.moveZ(-150);
			gravity(ch);
			ch.update();
			Vector3D speed = ch.getSpeed();
			int bleed = Character.floorBleed();
			dampX += speed.x * bleed;
			dampY += speed.y * bleed;
			dampZ += speed.z * bleed;
			speed.x -= dampX >> 12;
			speed.y -= dampY >> 12;
			speed.z -= dampZ >> 12;
			dampX &= SolverMath.F - 1;
			dampY &= SolverMath.F - 1;
			dampZ &= SolverMath.F - 1;
		}
		return ch.getPosition().z;
	}

	private static int turn(int fps, int seconds) {
		Character ch = player();
		for(int i = frames(fps, seconds); i > 0; i--) {
			frame(fps);
			ch.rotY(5);                            // Player.rotLeft
		}
		return ch.getRotation().y * 360 / (1 << 14);
	}

	private static int jumpApex(int fps) {
		gravityQ = lagQ = 0;
		Character ch = player();
		ch.standOn(ch.getPosition().y + 1, 0, 0);
		ch.jump(150, 1.2F);                        // Player.jump
		int apex = ch.getPosition().y;
		for(int i = frames(fps, 2); i > 0; i--) {
			frame(fps);
			gravity(ch);
			ch.update();
			if(ch.getPosition().y > apex) apex = ch.getPosition().y;
		}
		return apex;
	}

	private static int jumpTime(int fps) {
		gravityQ = lagQ = 0;
		Character ch = player();
		int ground = ch.getPosition().y;
		ch.standOn(ground + 1, 0, 0);
		ch.jump(150, 1.2F);
		long start = FPS.ms;
		for(int i = frames(fps, 4); i > 0; i--) {
			frame(fps);
			gravity(ch);
			ch.update();
			if(ch.getPosition().y <= ground) return (int) (FPS.ms - start);
		}
		return -1;
	}

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
		return quad(new int[]{-S, 0, -S}, new int[]{S, 0, -S},
				new int[]{S, 0, S}, new int[]{-S, 0, S});
	}

	private static final RigidBody.Collider[] COLS = new RigidBody.Collider[]{floor()};
	private static final float[] IDENTITY = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

	private static int cubeRest(int fps, int seconds) {
		RigidBody body = new RigidBody(500);
		body.reset(0, 6000, 0);
		for(int i = frames(fps, seconds); i > 0; i--) {
			frame(fps);
			body.step(COLS, 1, true, FPS.dt);
		}
		return body.getCenterY();
	}

	private static int cubeSleep(int fps) {
		RigidBody body = new RigidBody(500);
		body.reset(0, 6000, 0);
		long start = FPS.ms;
		for(int i = frames(fps, 6); i > 0; i--) {
			frame(fps);
			body.step(COLS, 1, true, FPS.dt);
			if(body.isSleeping()) return (int) (FPS.ms - start);
		}
		return -1;
	}

	private static int cubeThrow(int fps, int seconds) {
		RigidBody body = new RigidBody(500);
		body.reset(0, 500, 0);
		body.setVelocity(500, 0, 0);               // Cube.THROW_SPEED
		for(int i = frames(fps, seconds); i > 0; i--) {
			frame(fps);
			body.step(COLS, 1, true, FPS.dt);
		}
		return body.getCenterX();
	}

	// Cube.updateHeld: the hand walks 100 units a nominal frame and the pose
	// delta becomes the hand velocity that shoves the resting cube.
	private static int cubeShove(int fps, int seconds) {
		RigidBody held = new RigidBody(500);
		RigidBody target = new RigidBody(500);
		RigidBody[] pair = new RigidBody[]{held, target};
		held.setKinematic(true);
		held.setKinematicPose(-4000, 500, 0, IDENTITY, FPS.dt);
		target.reset(0, 500, 0);

		long handQ = -4000L * FPS.FRAME_MS;
		for(int i = frames(fps, seconds); i > 0; i--) {
			frame(fps);
			handQ += 100L * FPS.dtMs;
			held.setKinematicPose((int) (handQ / FPS.FRAME_MS), 500, 0, IDENTITY, FPS.dt);
			target.step(COLS, 1, true, FPS.dt);
			RigidBody.collideBodies(pair, 2);
		}
		return target.getCenterX();
	}

	private static int reloadMs(int fps) {
		Magazine m = new Magazine(12, 10);         // ten nominal frames
		m.setAmmo(50);
		m.recount();
		m.takeRounds(12);
		m.reload();
		long start = FPS.ms;
		for(int i = frames(fps, 4); i > 0; i--) {
			frame(fps);
			m.update();
			if(!m.isReloading()) return (int) (FPS.ms - start);
		}
		return -1;
	}

	private static int thinks(int fps, int seconds) {
		long thinkAt = FPS.ms;
		int count = 0;
		for(int i = frames(fps, seconds); i > 0; i--) {
			frame(fps);
			if(FPS.ms >= thinkAt) {                // Zombie.action
				thinkAt = FPS.ms + 400;
				count++;
			}
		}
		return count;
	}

	private static int hitchMs() {
		long start = FPS.ms;
		wall += 1000000L;
		FPS.tick(wall / 1000L);
		return (int) (FPS.ms - start);
	}

	// Percent of the wall time that became game time at a rate whose frames are
	// shorter than the clock can measure individually.
	private static int fineClockPercent(int fps) {
		long startMs = FPS.ms, startWall = wall;
		for(int i = frames(fps, 2); i > 0; i--) frame(fps);
		return (int) ((FPS.ms - startMs) * 1000000L / (wall - startWall));
	}

	public static void main(String[] args) {
		wall = 1000000L;
		FPS.tick(wall / 1000L);

		int base = RATES[0];
		int want;

		want = fall(base, 2);
		System.out.println("-- a falling character covers the same distance in two seconds");
		for(int i = 1; i < RATES.length; i++) {
			check("fall@" + RATES[i], fall(RATES[i], 2), want, tol(want, 1));
		}

		want = walk(base, 2);
		System.out.println("-- a walking character covers the same distance in two seconds");
		for(int i = 1; i < RATES.length; i++) {
			check("walk@" + RATES[i], walk(RATES[i], 2), want, tol(want, 2));
		}

		want = turn(base, 1);
		System.out.println("-- turning five degrees a step is a hundred degrees a second");
		for(int i = 1; i < RATES.length; i++) {
			check("turn@" + RATES[i], turn(RATES[i], 1), want, tol(want, 1));
		}

		want = jumpApex(base);
		int wantTime = jumpTime(base);
		System.out.println("-- a jump peaks at the same height and lands at the same time");
		for(int i = 1; i < RATES.length; i++) {
			check("jumpApex@" + RATES[i], jumpApex(RATES[i]), want, tol(want, 2));
			// The arc is the same but the ground is noticed within a step, and
			// the nominal step covers 180 units of the fall.
			check("jumpTime@" + RATES[i], jumpTime(RATES[i]), wantTime,
					FPS.FRAME_MS + 1000 / RATES[i]);
		}

		want = cubeRest(base, 3);
		System.out.println("-- a dropped cube comes to rest at the same height");
		for(int i = 1; i < RATES.length; i++) {
			check("cubeRest@" + RATES[i], cubeRest(RATES[i], 3), want, 20);
		}

		want = cubeSleep(base);
		System.out.println("-- a dropped cube falls asleep after the same time");
		for(int i = 1; i < RATES.length; i++) {
			check("cubeSleep@" + RATES[i], cubeSleep(RATES[i]), want, tol(want, 10));
		}

		want = cubeThrow(base, 1);
		System.out.println("-- a thrown cube travels the same distance in a second");
		for(int i = 1; i < RATES.length; i++) {
			check("cubeThrow@" + RATES[i], cubeThrow(RATES[i], 1), want, tol(want, 7));
		}

		want = cubeShove(base, 2);
		System.out.println("-- a carried cube shoves a resting one the same distance");
		for(int i = 1; i < RATES.length; i++) {
			check("cubeShove@" + RATES[i], cubeShove(RATES[i], 2), want, tol(want, 3));
		}

		// Against the authored duration: the counter completes on a step
		// boundary, so a coarse rate lands up to one frame late.
		want = 10 * FPS.FRAME_MS;
		System.out.println("-- a reload written as ten frames lasts half a second");
		for(int i = 0; i < RATES.length; i++) {
			check("reload@" + RATES[i], reloadMs(RATES[i]), want,
					FPS.FRAME_MS + 1000 / RATES[i] + 20);
		}

		want = thinks(base, 5);
		System.out.println("-- the AI thinks the same number of times in five seconds");
		for(int i = 1; i < RATES.length; i++) {
			check("thinks@" + RATES[i], thinks(RATES[i], 5), want, 1);
		}

		System.out.println("-- a one second stall plays a quarter of one");
		check("hitch", hitchMs(), 250, 0);

		System.out.println("-- game time keeps up with a clock finer than it can measure");
		check("fineClock@400", fineClockPercent(400), 1000, 10);   // per mille

		System.out.println();
		System.out.println(checks + " checks, " + failures + " failure(s)");
		if(failures > 0) System.exit(1);
		System.out.println("OK");
	}
}
