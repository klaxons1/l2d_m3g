package com;

// Frame rate invariance. Every scenario is driven through Clock.tick with a
// synthetic wall clock, the way GameScreen.paint does, at several rates and
// compared against the nominal 20 fps one. The game used to count frames, so at
// 60 fps everything moved three times as fast as at 20; now every rate is
// integrated over Clock.dt and every duration counted in Clock.ms.
// The results are not bit-identical across rates: fixed point integration of
// the same physical second differs in the last units, and semi-implicit Euler
// carries half a step of error that shrinks with the frame. The tolerances are
// a few percent, where the frame counted code was off by the frame ratio.
// Run with tools/physics/run_tests.sh fps.
public final class FrameRateTests {

	private static int checks, failures;

	// Rates the scenarios run at. 20 fps is the nominal frame (Clock.FRAME_MS);
	// 10 fps is played in two nominal steps a frame, 120 fps in steps of a
	// third of one.
	private static final int[] RATES = {20, 10, 40, 60, 120};

	private static void check(String what, int got, int want, int tol) {
		checks++;
		int d = got - want;
		if(d < 0) d = -d;
		if(d > tol) {
			failures++;
			System.out.println("FAIL " + what + ": got " + got + ", want " + want
					+ " +/-" + tol);
		}
	}

	private static int tol(int value, int percent) {
		if(value < 0) value = -value;
		int t = value * percent / 100;
		return t < 1 ? 1 : t;
	}

	// ---- the synthetic clock ----

	// Never reset: Clock keeps a private "last tick", so the wall clock has to
	// stay monotonic across scenarios or the first frame of the next one would
	// see the whole gap. In microseconds, so 60 fps really is 16.67 ms a frame
	// (16, 17, 16, ...) and every rate covers the same wall time.
	private static long wall;

	// One rendered frame at this rate; it plays the returned number of steps.
	private static int frame(int fps) {
		wall += 1000000L / fps;
		Clock.tick(wall / 1000L);
		return Clock.parts;
	}

	private static int frames(int fps, int seconds) {
		return fps * seconds;
	}

	// ---- scenarios ----

	// The player's capsule, GameObject.setCharacterSize(2005). The Character
	// constructor zeroes the size and the game sets it afterwards, so a probe
	// that skips set() gets a speed limit of zero and never moves at all.
	private static Character player() {
		Character ch = new Character(0, 0);
		ch.set(802, 1503);
		return ch;
	}

	// Scene.update gravity plus Character.update integration, no floor.
	private static int fall(int fps, int seconds) {
		Character ch = player();
		ch.getPosition().set(0, 20000, 0);
		for(int i = frames(fps, seconds); i > 0; i--) {
			for(int p = frame(fps); p > 0; p--) {
				ch.getSpeed().y -= gravity();
				ch.update();
			}
		}
		return 20000 - ch.getPosition().y;
	}

	// Scene.update's gravity with its remainder: 20 units per nominal frame,
	// which is a fraction of a unit on a short step.
	private static int gravityQ;

	private static int gravity() {
		gravityQ += 20 * Clock.dt;
		int g = gravityQ >> 12;
		gravityQ &= SolverMath.F - 1;
		return g;
	}

	// GameObject's damping remainders: the floor bleed is a fraction of a unit
	// on a short step and truncating it would damp a fast device harder.
	private static int dampX, dampY, dampZ;

	// A grounded walk: the moveZ input, gravity, integration and the floor
	// damping of GameObject.updateMovement. standOn keeps it on the floor the
	// way the floor snap does (it also sets onFloor, which moveZ needs).
	private static int walk(int fps, int seconds) {
		dampX = dampY = dampZ = 0;
		Character ch = player();
		for(int i = frames(fps, seconds); i > 0; i--) {
			for(int p = frame(fps); p > 0; p--) {
				ch.standOn(ch.getPosition().y + 1, 0, 0);
				ch.moveZ(-150);                        // Player.moveForward
				ch.getSpeed().y -= gravity();
				ch.update();
				Vector3D speed = ch.getSpeed();
				int bleed = SolverMath.mul(3072, Clock.dt);
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
		}
		return ch.getPosition().z;
	}

	// Player.rotLeft: five degrees per nominal frame, so a hundred a second.
	private static int turn(int fps, int seconds) {
		Character ch = player();
		for(int i = frames(fps, seconds); i > 0; i--) {
			for(int p = frame(fps); p > 0; p--) ch.rotY(5);
		}
		return ch.getRotation().y * 360 / (1 << 14);   // Q14 to degrees
	}

	// Player.jump: an impulse, so it must not be scaled by the step length.
	private static int jumpApex(int fps) {
		Character ch = player();
		ch.standOn(ch.getPosition().y + 1, 0, 0);
		ch.jump(150, 1.2F);
		int apex = ch.getPosition().y;
		for(int i = frames(fps, 2); i > 0; i--) {
			for(int p = frame(fps); p > 0; p--) {
				ch.getSpeed().y -= gravity();
				ch.update();
				if(ch.getPosition().y > apex) apex = ch.getPosition().y;
			}
		}
		return apex;
	}

	// Air time of the same jump, in ms.
	private static int jumpTime(int fps) {
		Character ch = player();
		int ground = ch.getPosition().y;
		ch.standOn(ground + 1, 0, 0);
		ch.jump(150, 1.2F);
		long start = Clock.ms;
		for(int i = frames(fps, 4); i > 0; i--) {
			for(int p = frame(fps); p > 0; p--) {
				ch.getSpeed().y -= gravity();
				ch.update();
				if(ch.getPosition().y <= ground) return (int) (Clock.ms - start);
			}
		}
		return -1;
	}

	// ---- the solver ----

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

	// A cube dropped onto the floor: where it comes to rest.
	private static int cubeRest(int fps, int seconds) {
		RigidBody body = new RigidBody(500);
		body.reset(0, 6000, 0);
		for(int i = frames(fps, seconds); i > 0; i--) {
			for(int p = frame(fps); p > 0; p--) body.step(COLS, 1, true, Clock.dt);
		}
		return body.getCenterY();
	}

	// How long that drop takes to fall asleep, in ms.
	private static int cubeSleep(int fps) {
		RigidBody body = new RigidBody(500);
		body.reset(0, 6000, 0);
		long start = Clock.ms;
		for(int i = frames(fps, 6); i > 0; i--) {
			for(int p = frame(fps); p > 0; p--) {
				body.step(COLS, 1, true, Clock.dt);
				if(body.isSleeping()) return (int) (Clock.ms - start);
			}
		}
		return -1;
	}

	// A thrown cube: how far the release speed carries it in a second.
	private static int cubeThrow(int fps, int seconds) {
		RigidBody body = new RigidBody(500);
		body.reset(0, 500, 0);
		body.setVelocity(500, 0, 0);            // Cube.THROW_SPEED
		for(int i = frames(fps, seconds); i > 0; i--) {
			for(int p = frame(fps); p > 0; p--) body.step(COLS, 1, true, Clock.dt);
		}
		return body.getCenterX();
	}

	// A carried cube ploughing into a resting one, posed the way Cube.updateHeld
	// does. The hand walks 100 units per nominal frame; the pose delta becomes
	// the hand velocity, so it has to be divided by the step length or the shove
	// would be three times weaker at 60 fps.
	private static int cubeShove(int fps, int seconds) {
		RigidBody held = new RigidBody(500);
		RigidBody target = new RigidBody(500);
		RigidBody[] pair = new RigidBody[]{held, target};
		held.setKinematic(true);
		held.setKinematicPose(-4000, 500, 0, IDENTITY, Clock.dt);
		target.reset(0, 500, 0);

		long handQ = -4000L * Clock.FRAME_MS;   // hand position, Q12 ms
		for(int i = frames(fps, seconds); i > 0; i--) {
			for(int p = frame(fps); p > 0; p--) {
				handQ += 100L * Clock.dtMs;
				held.setKinematicPose((int) (handQ / Clock.FRAME_MS), 500, 0, IDENTITY, Clock.dt);
				target.step(COLS, 1, true, Clock.dt);
				RigidBody.collideBodies(pair, 2);
			}
		}
		return target.getCenterX();
	}

	// ---- the timers ----

	// Magazine: a reload written as a frame count must last that many ms.
	private static int reloadMs(int fps) {
		Magazine m = new Magazine(12, 10);      // 10 nominal frames = 500 ms
		m.setAmmo(50);
		m.recount();
		m.takeRounds(12);
		m.reload();
		long start = Clock.ms;
		for(int i = frames(fps, 4); i > 0; i--) {
			for(int p = frame(fps); p > 0; p--) {
				m.update();
				if(!m.isReloading()) return (int) (Clock.ms - start);
			}
		}
		return -1;
	}

	// The Bot AI cadence idiom: a stamp in Clock.ms, not a frame count.
	private static int thinks(int fps, int seconds) {
		long thinkAt = Clock.ms;
		int count = 0;
		for(int i = frames(fps, seconds); i > 0; i--) {
			for(int p = frame(fps); p > 0; p--) {
				if(Clock.ms >= thinkAt) {       // Zombie.action, every 400 ms
					thinkAt = Clock.ms + 400;
					count++;
				}
			}
		}
		return count;
	}

	// A platform clock that only moves every 16 ms: half the frames report no
	// time at all, and the game must not lose it.
	private static int coarseClockMs(int steps) {
		long before = Clock.ms;
		long start = wall;
		for(int i = 0; i < steps; i++) {
			wall += (i & 1) * 16000L;           // 0, 16, 0, 16, ...
			Clock.tick(wall / 1000L);
		}
		// percent of the wall time that became game time
		return (int) ((Clock.ms - before) * 1000000L / (wall - start));
	}

	public static void main(String[] args) {
		// The first tick has no previous one to measure against and plays
		// nothing; take it here so no scenario loses its first frame. wall is
		// in microseconds, tick takes milliseconds.
		wall = 1000000L;
		Clock.tick(wall / 1000L);

		int base = RATES[0];

		int want = fall(base, 2);
		System.out.println("-- a falling character covers the same distance in two seconds");
		for(int i = 1; i < RATES.length; i++) {
			check("fall@" + RATES[i], fall(RATES[i], 2), want, tol(want, 3));
		}

		want = walk(base, 2);
		System.out.println("-- a walking character covers the same distance in two seconds");
		// The remainders make the input, the damping and the integration exact;
		// what is left is the speed itself being a whole unit, so the fixed
		// point of a short step lands up to a unit lower (~2% at 120 fps).
		for(int i = 1; i < RATES.length; i++) {
			check("walk@" + RATES[i], walk(RATES[i], 2), want, tol(want, 3));
		}

		want = turn(base, 1);
		System.out.println("-- turning five degrees a step is a hundred degrees a second");
		for(int i = 1; i < RATES.length; i++) {
			check("turn@" + RATES[i], turn(RATES[i], 1), want, tol(want, 3));
		}

		want = jumpApex(base);
		int wantTime = jumpTime(base);
		System.out.println("-- a jump peaks at about the same height and lands at about the same time");
		// The widest tolerance here, and it is not a bug but the integrator:
		// gravity is applied before the position, so a step loses half a step of
		// climb. That loss is part of the tuned feel at the nominal frame (the
		// apex is 720 where the true arc is 810) and it shrinks as the frame
		// gets shorter, so a finer rate jumps up to 12% higher. Making the two
		// agree would mean either changing the nominal jump or stepping the
		// character at 20 Hz and throwing the smoothness away.
		for(int i = 1; i < RATES.length; i++) {
			check("jumpApex@" + RATES[i], jumpApex(RATES[i]), want, tol(want, 13));
			// one frame of the rate is the resolution of the landing test
			check("jumpTime@" + RATES[i], jumpTime(RATES[i]), wantTime,
					tol(wantTime, 8) + 1000 / RATES[i]);
		}

		want = cubeRest(base, 3);
		System.out.println("-- a dropped cube comes to rest at the same height");
		for(int i = 1; i < RATES.length; i++) {
			check("cubeRest@" + RATES[i], cubeRest(RATES[i], 3), want, 20);
		}

		want = cubeSleep(base);
		System.out.println("-- a dropped cube falls asleep after the same time");
		for(int i = 1; i < RATES.length; i++) {
			// A finer step leaves a resting contact less jitter, so it settles
			// into the sleep threshold sooner: the same second, a little earlier.
			check("cubeSleep@" + RATES[i], cubeSleep(RATES[i]), want, tol(want, 20));
		}

		want = cubeThrow(base, 1);
		System.out.println("-- a thrown cube travels the same distance in a second");
		for(int i = 1; i < RATES.length; i++) {
			check("cubeThrow@" + RATES[i], cubeThrow(RATES[i], 1), want, tol(want, 8));
		}

		want = cubeShove(base, 2);
		System.out.println("-- a carried cube shoves a resting one the same distance");
		for(int i = 1; i < RATES.length; i++) {
			check("cubeShove@" + RATES[i], cubeShove(RATES[i], 2), want, tol(want, 8));
		}

		// Against the authored duration, not against the nominal rate: the
		// counter can only complete on a step boundary, so a coarse rate lands
		// up to one nominal frame late and a fine one is the more accurate.
		want = 10 * Clock.FRAME_MS;
		System.out.println("-- a reload written as ten frames lasts half a second");
		for(int i = 0; i < RATES.length; i++) {
			check("reload@" + RATES[i], reloadMs(RATES[i]), want,
					Clock.FRAME_MS + 1000 / RATES[i] + 20);
		}

		want = thinks(base, 5);
		System.out.println("-- the AI thinks the same number of times in five seconds");
		for(int i = 1; i < RATES.length; i++) {
			check("thinks@" + RATES[i], thinks(RATES[i], 5), want, 1);
		}

		System.out.println("-- a clock that only ticks every 16 ms keeps game time (percent of wall time)");
		check("coarseClock", coarseClockMs(200), 1000, 20);   // 1000 = percent

		System.out.println();
		System.out.println(checks + " checks, " + failures + " failure(s)");
		if(failures > 0) System.exit(1);
		System.out.println("OK");
	}
}
