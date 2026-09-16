package com;

// The frame clock, ticked once per frame before anything moves.
//
// The game was written around a 20 fps limit, so FRAME_MS is the nominal frame
// and dt the length of this one in Q12 nominal frames: every speed stays units
// per nominal frame and only the integration scales. dtMs is the same length in
// whole milliseconds and ms the game time so far, for what is a duration rather
// than a rate. The length is clamped, not saved up, and never zero: the hand
// velocity of a carried cube divides by it.
final class FPS extends SolverMath {

	static final int FRAME_MS = 50;
	private static final int MIN_MS = 1;
	private static final int MAX_MS = 250;

	static int dt = F;
	static int dtMs = FRAME_MS;
	static long ms;
	static int fps;

	private static long last = -1;
	private static long second;
	private static int frames;

	static void tick(long now) {
		int elapsed;
		if(last < 0) {
			elapsed = FRAME_MS;
			second = now;
		} else {
			elapsed = (int) (now - last);
		}
		last = now;

		if(elapsed < MIN_MS) elapsed = MIN_MS;
		if(elapsed > MAX_MS) elapsed = MAX_MS;
		dtMs = elapsed;
		dt = elapsed * F / FRAME_MS;
		ms += elapsed;

		frames++;
		if(now - second >= 1000) {
			second = now;
			fps = frames;
			frames = 0;
		}
	}
}
