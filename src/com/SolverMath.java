package com;

// Fixed point math and the contact micro-ops shared by RigidBody (body against
// world) and BodyPair (body against body): both extend this so their innermost
// loops call mul() unqualified, which 1.3 has no static import for. All static.
class SolverMath {
	// Q12 scale.
	public static final int F = 4096;

	// Q12 arithmetic and the Q24 matrix evaluation both inertia tensors need. mul()
	// is the innermost operation in the engine.

	// Q12 multiply (arithmetic shift, matches the C fixed point code).
	public static int mul(int a, int b) {
		return (int) ((long) a * b >> 12);
	}
	// Q12 division: a / b with both values Q12, result Q12.
	public static int divQ(int a, int b) {
		return (int) (((long) a << 12) / b);
	}
	// Integer square root (plain integer domain, floor).
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
	// Q12 square root.
	static int sqrtQ(int x) {
		return isqrt((long) x << 12);
	}
	// keep^exp, keep Q12 and exp in Q12 nominal frames, integer only because
	// CLDC has no Math.pow: the whole frames go by square and multiply, the
	// fraction by the same over a chain of square roots. Friction is a power of
	// the frame length, not a multiple of it. Memoised on the last call, which
	// is the one every other object in the frame makes.
	private static int powKeep = -1, powExp = -1, powVal;
	static int powQ(int keep, int exp) {
		if(keep == powKeep && exp == powExp) return powVal;
		int r = F;
		int b = keep;
		for(int n = exp >> 12; n > 0; n >>= 1) {
			if((n & 1) != 0) r = mul(r, b);
			if(n > 1) b = mul(b, b);
		}
		b = keep;
		for(int f = exp & (F - 1), bit = F >> 1; f != 0 && bit != 0; bit >>= 1) {
			b = sqrtQ(b);
			if((f & bit) != 0) {
				r = mul(r, b);
				f &= ~bit;
			}
		}
		powKeep = keep;
		powExp = exp;
		powVal = r;
		return r;
	}
	// Q12 magnitude of a Q12 vector.
	static int norm3(int x, int y, int z) {
		return isqrt((long) x * x + (long) y * y + (long) z * z);
	}
	static int abs(int v) { return v < 0 ? -v : v; }

	// Evaluates a Q24 3x3 matrix against a Q12 vector, result Q12.
	static int eval24X(int[] m, int x, int y, int z) {
		return (int) (((long) m[0] * x + (long) m[1] * y + (long) m[2] * z) >> 24);
	}
	static int eval24Y(int[] m, int x, int y, int z) {
		return (int) (((long) m[3] * x + (long) m[4] * y + (long) m[5] * z) >> 24);
	}
	static int eval24Z(int[] m, int x, int y, int z) {
		return (int) (((long) m[6] * x + (long) m[7] * y + (long) m[8] * z) >> 24);
	}

	// ---- solver micro-ops ----
	// Both passes run the same sequential impulse scheme against different mass
	// sources; the pieces that are easy to get subtly wrong live here, one copy.

	static int angularEffectiveMass(int rx, int ry, int rz, int[] invI,
			int dx, int dy, int dz) {
		int rdx = mul(ry, dz) - mul(rz, dy);
		int rdy = mul(rz, dx) - mul(rx, dz);
		int rdz = mul(rx, dy) - mul(ry, dx);
		int wx = eval24X(invI, rdx, rdy, rdz);
		int wy = eval24Y(invI, rdx, rdy, rdz);
		int wz = eval24Z(invI, rdx, rdy, rdz);
		return mul(mul(wy, rz) - mul(wz, ry), dx)
				+ mul(mul(wz, rx) - mul(wx, rz), dy)
				+ mul(mul(wx, ry) - mul(wy, rx), dz);
	}

	// The sliding direction of a contact: rv with its component along n
	// removed and normalised, into tanX/Y/Z. False when rv is (nearly)
	// parallel to n, so there is no sliding for friction to resist.
	static int tanX, tanY, tanZ;
	static boolean contactTangent(int rvx, int rvy, int rvz,
			int nx, int ny, int nz) {
		int vn = mul(rvx, nx) + mul(rvy, ny) + mul(rvz, nz);
		int tx = rvx - mul(nx, vn);
		int ty = rvy - mul(ny, vn);
		int tz = rvz - mul(nz, vn);
		int tl = norm3(tx, ty, tz);
		if(tl < 1) return false;
		tanX = divQ(tx, tl); tanY = divQ(ty, tl); tanZ = divQ(tz, tl);
		return true;
	}

	// Clamped accumulated impulse: adds delta to acc[i], clamps the total to [lo, hi]
	// and returns the part applied this sweep, so a contact at its limit stops
	// pushing. [0, MAX_VALUE] for a normal impulse, [-mu jn, mu jn] for friction.
	static int accumulate(int[] acc, int i, int delta, int lo, int hi) {
		int next = acc[i] + delta;
		if(next > hi) next = hi;
		else if(next < lo) next = lo;
		int applied = next - acc[i];
		acc[i] = next;
		return applied;
	}
}
