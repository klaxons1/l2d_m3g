package com;

// Cube against cube.
//
// The world pass in RigidBody only ever sees one box against the static
// triangle soup, so two dynamic boxes are resolved here, in a pass over every
// pair that runs once per frame after all bodies have stepped:
//
//  - a separating axis test over the 15 box-box axes (6 face normals plus 9
//    edge cross products) rejects non-touching pairs and picks the axis of
//    least penetration,
//  - a face axis resolves by clipping the incident face against the reference
//    face (Sutherland-Hodgman, deepest PAIR_MAX_CONTACTS kept),
//  - an edge axis resolves to the single closest point pair of the two extreme
//    edges,
//  - contacts are solved with sequential impulses applied to BOTH bodies (equal
//    and opposite, so momentum is conserved) plus a position projection that
//    re-derives the penetration from local contact anchors every iteration, so
//    a stack converges instead of settling at a fixed residual overlap.
//
// A carried (kinematic) body and a sleeping body take part with zero inverse
// mass: immovable obstacles that still lend their own velocity to the
// contact. A sleeper is woken by a hard impact or a moving neighbour, and a
// body resting on another body may sleep as well (bodySupport), otherwise a
// stack could never come to rest.

final class BodyPair extends SolverMath {
	// ===================== cube against cube =====================

	// Deepest contacts kept for one pair: a clipped face manifold has at
	// most four meaningful support points, and every pair is solved several
	// times per frame.
	private static final int PAIR_MAX_CONTACTS = 4;
	// Clipping keeps a point this far outside the reference face footprint
	// (Q12 units) so a contact exactly on a face edge is not lost to
	// truncation.
	private static final int CLIP_SLOP = 4 << 12;
	// An edge-edge axis wins over the best face axis only when it is this
	// much shallower (Q12 units): a face manifold is far more stable, so
	// near ties go to the face case.
	private static final int EDGE_AXIS_BIAS = 8 << 12;
	private static final int PAIR_IMPULSE_ITERATIONS = 8;
	private static final int PAIR_POSITION_ITERATIONS = 6;
	// Walks over the whole pair list per frame, see collideBodies.
	private static final int PAIR_ROUNDS = 3;
	// Cube against cube: less bouncy than the world material (a stack must
	// not ping-pong) but just as grippy, so cubes can rest on each other.
	private static final int BODY_RESTITUTION = 614;   // 0.15
	private static final int BODY_FRICTION = 4096;     // 1.0
	// A sleeping body is woken by a contact closing faster than this.
	private static final int WAKE_SPEED = 60 << 12;
	// A sleeping body is also woken when the body it leans on moves faster
	// than this, otherwise it would hang in the air while its support
	// slides away.
	private static final int WAKE_NEIGHBOUR_SPEED = 48 << 12;
	// Cross product length (Q24) below which two box axes count as parallel
	// and their edge-edge axis is skipped (~0.9 degrees): the face axes
	// already separate boxes aligned that closely.
	private static final int PARALLEL_EPS = 1 << 18;
	// Same-normal pair contacts closer than this (squared, Q12) are one
	// contact; clipping can emit a corner twice when an incident edge lies
	// exactly on a clip plane.
	private static final long PAIR_MERGE_DIST2 = (long) (8 << 12) * (8 << 12);
	private static final int PAIR_MERGE_DOT = F * 3 / 4;
	// Upward component a pair contact needs to count as support. Wider than
	// the world pass' ground cone (F * 7 / 10): a cube balanced on the seam
	// between two cubes is held up by contacts whose normals are tilted well
	// past 45 degrees, and a body that is not recognised as supported may
	// never sleep - it would keep re-resolving its own weight forever.
	private static final int SUPPORT_UP = F / 2;
	// Position projection strength by manifold size. The penetration is
	// re-derived from the local anchors on every iteration, so a single
	// contact needs a much bigger step than a four point face manifold to
	// converge in the same number of sweeps; without this an edge-edge
	// impact keeps a third of its depth.
	private static final int[] PAIR_BETA = {F / 2, F * 3 / 8, F * 5 / 16, F * 3 / 16};
	// Zero inverse inertia, for bodies that must not rotate (carried,
	// asleep).
	private static final int[] ZERO_I = new int[9];
	// Clipping polygon buffers: 4 incident corners plus at most one point
	// per clip plane.
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
	// Contact anchors in each body's local frame, see addPairContact.
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

	// Resolves every box-box pair for one frame. Called after all bodies stepped
	// against the world, so a cube lands on, slides off, pushes and stacks on
	// another cube instead of passing through it.
	//
	// The pair list is walked PAIR_ROUNDS times and contacts are regenerated each
	// round, so a round sees what the previous one moved and support propagates
	// from the ground up. Null entries in bodies are allowed.
	static void collide(RigidBody[] bodies, int count) {
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

	// World space box axis i (column i of the row-major orientation).
	private static int axisX(RigidBody b, int i) { return b.r[i]; }
	private static int axisY(RigidBody b, int i) { return b.r[3 + i]; }
	private static int axisZ(RigidBody b, int i) { return b.r[6 + i]; }
	private static int halfExt(RigidBody b, int i) {
		return i == 0 ? b.hx : (i == 1 ? b.hy : b.hz);
	}

	// Velocity a body lends to a contact: a carried cube moves with the hand
	// even though its own simulated velocity is kept at zero.
	private static int velX(RigidBody x) { return x.kinematic ? x.kvx : x.vx; }
	private static int velY(RigidBody x) { return x.kinematic ? x.kvy : x.vy; }
	private static int velZ(RigidBody x) { return x.kinematic ? x.kvz : x.vz; }

	// Fastest point speed of a body (translation plus spin at the corner).
	private static int bodySpeed(RigidBody x) {
		return norm3(velX(x), velY(x), velZ(x))
				+ mul(norm3(x.wx, x.wy, x.wz), x.cornerRadius);
	}

	private static int clamp14(int t) {
		if(t < 0) return 0;
		if(t > 16384) return 16384;
		return t;
	}

	// Overlap of the two boxes along a unit axis (Q12); <= 0 separates.
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

	// Separating axis test plus manifold generation for one pair of boxes.
	// Fills the pair scratch and returns the number of contacts.
	private static int generateContacts(RigidBody a, RigidBody b) {
		pairContacts = 0;

		int m = RigidBody.CONTACT_MARGIN;
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

	// Face manifold: clip the incident face of inc against the face of ref
	// whose outward normal is s * ref.axis[k] (that normal points from ref
	// toward inc). The stored contact normals always point from a to b.
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
			if(pen < -(RigidBody.SURFACE_TOUCH << 12)) continue;
			// contact halfway between the incident point and the reference
			// plane, so neither body owns the whole lever arm
			addPairContact(a, b,
					clipX[i] + mul(nx, pen >> 1),
					clipY[i] + mul(ny, pen >> 1),
					clipZ[i] + mul(nz, pen >> 1),
					cnx, cny, cnz, pen);
		}
	}

	// Sutherland-Hodgman clip of the scratch polygon against one side plane of the
	// reference face: keeps the points with (p - ref.center) . axis <= limit
	// (+ slop).
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

	// Edge manifold: the extreme edge of a along the separation axis against
	// the extreme edge of b along its opposite. The closest point pair of the
	// two segments is the contact.
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

	// Closest points of two segments (all Q12), Ericson's segment/segment test
	// evaluated in plain units with Q14 parameters: the Q12 products of a
	// world-scale distance would overflow 64 bits otherwise. Results land in
	// cp1* and cp2*.
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

	// Stores one contact whose normal points from a to b, keeping the
	// PAIR_MAX_CONTACTS deepest points.
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

	// Notes which body is held up by the other, so a stacked cube may sleep
	// exactly like one resting on the floor.
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

	// True when the world geometry holds body x against a move along
	// (dx, dy, dz): one of the contacts its last step produced pushes back
	// the other way.
	private static boolean worldBlocked(RigidBody x, int dx, int dy, int dz) {
		for(int i = 0; i < x.numContacts; i++) {
			if(mul(x.cnx[i], dx) + mul(x.cny[i], dy) + mul(x.cnz[i], dz) < 0) {
				return true;
			}
		}
		return false;
	}

	// True when another body holds x up and the move would push x into that
	// support. Together with worldBlocked this keeps a stack from paying for
	// the pair above it by sinking into the pair below: two corrections that
	// each push a shared body the other way never converge.
	private static boolean bodyBlocked(RigidBody x, int dx, int dy, int dz) {
		return x.bodySupport
				&& mul(x.supportNX, dx) + mul(x.supportNY, dy)
						+ mul(x.supportNZ, dz) < 0;
	}

	private static boolean blocked(RigidBody x, int dx, int dy, int dz) {
		return worldBlocked(x, dx, dy, dz) || bodyBlocked(x, dx, dy, dz);
	}

	// Which of the two bodies must not take contact i: whoever the world or its
	// own support holds in place gives up its share, so the whole response goes
	// to the body that can actually move. A cube resting on the floor then
	// carries a stack instead of being squashed into the floor, and the stack
	// comes to rest instead of keeping residual downward velocity that the floor
	// only answers on the next step. When neither body could move at all - a
	// carried cube pressing a cube onto the floor - the hold is released again,
	// because a pair of immovable bodies has no solution. Results land in
	// heldA/heldB.
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

	// True when the sleeping body a must join the pair solve.
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
		// A moving carried cube is player controlled and about to displace
		// whatever it touches, so a sleeper in its way always joins the solve: at
		// a slow walk the hand velocity stays under the neighbour threshold below.
		// A parked hand is a shelf instead, and a cube resting on it may sleep.
		if(b.kinematic && bodySpeed(b) > 0) return true;
		// the body it rests on is moving: hanging around would leave the
		// sleeper floating once its support slid away
		return bodySpeed(b) > WAKE_NEIGHBOUR_SPEED;
	}

	// World space anchor scratch, filled by worldAnchor.
	private static int anchorX, anchorY, anchorZ;

	// p + R * local[i * 3 ..]: the anchor of contact i on body x, in world
	// space. The position projection re-derives the current penetration from
	// these, so a contact stays attached to the same point of each box as the
	// boxes move instead of pushing out a stale depth once per iteration.
	private static void worldAnchor(RigidBody x, int[] local, int i) {
		int lx = local[i * 3], ly = local[i * 3 + 1], lz = local[i * 3 + 2];
		anchorX = x.px + mul(lx, x.r[0]) + mul(ly, x.r[1]) + mul(lz, x.r[2]);
		anchorY = x.py + mul(lx, x.r[3]) + mul(ly, x.r[4]) + mul(lz, x.r[5]);
		anchorZ = x.pz + mul(lx, x.r[6]) + mul(ly, x.r[7]) + mul(lz, x.r[8]);
	}

	private static void anchorOnA(RigidBody a, int i) { worldAnchor(a, pral, i); }
	private static void anchorOnB(RigidBody b, int i) { worldAnchor(b, prbl, i); }

	// Penetration of contact i now, re-derived from the local anchors.
	private static int currentPen(RigidBody a, RigidBody b, int i) {
		anchorOnA(a, i);
		int awx = anchorX, awy = anchorY, awz = anchorZ;
		anchorOnB(b, i);
		int sep = mul(anchorX - awx, pnx[i]) + mul(anchorY - awy, pny[i])
				+ mul(anchorZ - awz, pnz[i]);
		return ppen[i] - sep;
	}

	// Sequential impulses for one pair: equal and opposite on both bodies,
	// with restitution, Coulomb friction and a converging position projection.
	// A static (carried or sleeping) body has zero inverse mass and inertia, so
	// it absorbs nothing and only lends its velocity.
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
			pbias[i] = -vn > RigidBody.RESTITUTION_SPEED ? -mul(BODY_RESTITUTION, vn) : 0;
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
				// A held body keeps the r x (n j) term further down while its
				// linear share is zeroed. Deliberate: it is worth at most 1/4096
				// rad/frame even on an off-center landing (measured), and
				// RigidBody.integrate() re-derives w from l every frame, so pinning w to
				// ZERO_I for the rest of this solve costs nothing either.
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
						+ angularEffectiveMass(rax, ray, raz, iiAc, nx, ny, nz)
						+ angularEffectiveMass(rbx, rby, rbz, iiBc, nx, ny, nz);
				if(kn > 0) {
					int dN = accumulate(paccN, i, divQ(pbias[i] - vn, kn),
							0, Integer.MAX_VALUE);
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
				if(!contactTangent(rvx, rvy, rvz, nx, ny, nz)) continue;

				int kt = imAc + imBc
						+ angularEffectiveMass(rax, ray, raz, iiAc, tanX, tanY, tanZ)
						+ angularEffectiveMass(rbx, rby, rbz, iiBc, tanX, tanY, tanZ);
				if(kt <= 0) continue;
				int vt = mul(rvx, tanX) + mul(rvy, tanY) + mul(rvz, tanZ);
				int maxFric = abs(mul(BODY_FRICTION, paccN[i]));
				int dT = accumulate(paccT, i, -divQ(vt, kt), -maxFric, maxFric);
				if(dT == 0) continue;

				int imp = mul(dT, imBc);
				bvx += mul(tanX, imp); bvy += mul(tanY, imp); bvz += mul(tanZ, imp);
				imp = mul(dT, imAc);
				avx -= mul(tanX, imp); avy -= mul(tanY, imp); avz -= mul(tanZ, imp);
				blx += mul(rby, mul(tanZ, dT)) - mul(rbz, mul(tanY, dT));
				bly += mul(rbz, mul(tanX, dT)) - mul(rbx, mul(tanZ, dT));
				blz += mul(rbx, mul(tanY, dT)) - mul(rby, mul(tanX, dT));
				alx -= mul(ray, mul(tanZ, dT)) - mul(raz, mul(tanY, dT));
				aly -= mul(raz, mul(tanX, dT)) - mul(rax, mul(tanZ, dT));
				alz -= mul(rax, mul(tanY, dT)) - mul(ray, mul(tanX, dT));
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

	// Position only de-penetration for one pair. Unlike the world pass this
	// re-derives the penetration from the local anchors every iteration, so
	// the sweep converges on the slop instead of leaving a fixed fraction of
	// the overlap behind (a stack would otherwise sink visibly into itself).
	private static void correctPairPositions(RigidBody a, RigidBody b,
			boolean aStatic, boolean bStatic, int imA, int imB,
			int[] iiA, int[] iiB, int count) {
		int beta = count <= PAIR_BETA.length ? PAIR_BETA[count - 1] : PAIR_BETA[PAIR_BETA.length - 1];
		for(int iter = 0; iter < PAIR_POSITION_ITERATIONS; iter++) {
			for(int i = 0; i < count; i++) {
				int pen = currentPen(a, b, i);
				if(pen <= RigidBody.POSITION_SLOP << 12) continue;
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
						+ angularEffectiveMass(rax, ray, raz, iiAc, nx, ny, nz)
						+ angularEffectiveMass(rbx, rby, rbz, iiBc, nx, ny, nz);
				if(k <= 0) continue;
				int dp = divQ(mul(pen - (RigidBody.POSITION_SLOP << 12), beta), k);

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

}
