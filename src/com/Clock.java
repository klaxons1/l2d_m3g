package com;

// One clock for the game, ticked once per frame before anything moves.
//
// dt is the step length in Q12 nominal frames (F == one FRAME_MS), so every
// per-frame constant in the game keeps its value and only the integration
// scales with it: speeds stay units per nominal frame, accelerations units per
// nominal frame squared. dtMs is the same step in whole milliseconds and ms the
// game time so far, for the counters that are durations rather than rates (AI
// cadence, weapon reload, message timers).
//
// A frame is never simulated as one big step. A long one is split into nominal
// sized parts (GameScreen plays Clock.parts of them) because the whole game is
// tuned per nominal frame and a double step covers more ground than two single
// ones, and a short one is carried over to the next frame, which is what keeps
// the game at full speed both on a slow device and on a clock that only ticks
// every 16 ms.
final class Clock extends SolverMath {

	// The nominal frame. Everything tuned "per frame" in this game is per this.
	static final int FRAME_MS = 50;              // 20 fps

	// Shorter steps than this are not simulated but saved up: below it the
	// fixed point rates start to lose their fraction to truncation.
	private static final int MIN_STEP = FRAME_MS / 8;   // 6 ms, ~160 fps

	// A frame longer than this many nominal frames plays slow instead of doing
	// more and more work per frame, so a hitch cannot spiral.
	private static final int MAX_PARTS = 4;

	static int dt = F;          // this step, Q12 nominal frames, at most F
	static int dtMs = FRAME_MS; // this step, whole ms
	static int parts = 1;       // steps to play this frame, 0 when it was too short
	// The whole frame's game time, for what is drawn once per frame rather than
	// stepped (the blood and splinter sprites), which dtMs would under-advance
	// on a frame split into parts.
	static int frameMs = FRAME_MS;
	static long ms;             // game time in ms

	private static long last = -1;
	private static int pending; // wall ms saved up but not simulated yet

	// Everything is derived from the whole milliseconds the platform clock
	// reported and whatever does not fit in this frame's steps stays pending, so
	// game time never drifts away from wall time: at 120 fps the steps are 8, 8,
	// 9 ms and not 8.33 rounded down to 8 every time.
	static void tick(long now) {
		int elapsed = last < 0 ? 0 : (int) (now - last);
		last = now;
		if(elapsed < 0) elapsed = 0;             // the clock went backwards
		pending += elapsed;

		if(pending < MIN_STEP) {                 // too short to simulate: carry it
			parts = 0;
			dt = dtMs = frameMs = 0;
			return;
		}

		parts = (pending + FRAME_MS - 1) / FRAME_MS;
		if(parts > MAX_PARTS) parts = MAX_PARTS;

		dtMs = pending / parts;
		if(dtMs > FRAME_MS) dtMs = FRAME_MS;     // a hitch plays slow
		dt = dtMs * F / FRAME_MS;

		pending -= dtMs * parts;
		if(pending > FRAME_MS) pending = FRAME_MS;
		frameMs = dtMs * parts;
		ms += frameMs;
	}
}
