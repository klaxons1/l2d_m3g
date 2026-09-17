package com;

// Box against box, once per frame after every body has stepped against the
// world. A separating axis test over the 15 box-box axes picks the axis of
// least penetration: a face axis clips the incident face against the reference
// one, an edge axis takes the closest points of the two extreme edges.
//
// Every manifold of the frame is batched and solved in two phases: sequential
// impulse sweeps on both bodies, then position projections re-derived from
// local anchors. Sweeping the whole batch rather than finishing one pair at a
// time lets a correction reach the pairs that share a body, for about half the
// work. A carried or sleeping body joins with zero inverse mass: immovable, but
// still lending its velocity to the contact.

final class BodyPair extends SolverMath {

	// Deepest contacts kept for one pair: a clipped face manifold has at most
	// four meaningful support points.
	private static final int PAIR_MAX_CONTACTS = 4;
	// Clipping keeps a point this far outside the reference face footprint
	// (Q12 units) so a contact exactly on a face edge is not lost to
	// truncation.
	private static final int CLIP_SLOP = 4 << 12;
	// An edge-edge axis wins over the best face axis only when it is this
	// much shallower (Q12 units): a face manifold is far more stable, so
	// near ties go to the face case.
	private static final int EDGE_AXIS_BIAS = 8 << 12;
	// Sweeps per phase over the whole batch. Below these the transient overlap
	// of a hard impact grows, above them the projection rocks settled piles.
	private static final int PAIR_VELOCITY_SWEEPS = 12;
	private static final int PAIR_POSITION_SWEEPS = 9;
	// Box against box: less bouncy than the world material (a stack must
	// not ping-pong) but just as grippy, so cubes can rest on each other.
	private static final int BODY_RESTITUTION = 614;   // 0.15
	private static final int BODY_FRICTION = 4096;     // 1.0
	// A sleeping body is woken by a contact closing faster than this.
	private static final int WAKE_SPEED = 60 << 12;
	// A sleeping body is also woken when the body it leans on moves faster
	// than this, otherwise it would hang in the air while its support
	// slides away.
	private static final int WAKE_NEIGHBOUR_SPEED = 48 << 12;
	// A sleeper also wakes when a body is this deep inside it. Settled contact
	// sits a few units deep, so this only fires on being actually inside
	// something, which no amount of resting explains.
	private static final int WAKE_PENETRATION = 100 << 12;
	// Cross product length (Q24) below which two box axes count as parallel
	// and their edge-edge axis is skipped (~0.9 degrees): the face axes
	// already separate boxes aligned that closely.
	private static final int PARALLEL_EPS = 1 << 18;
	// Same-normal pair contacts closer than this (squared, Q12) are one
	// contact; clipping can emit a corner twice when an incident edge lies
	// exactly on a clip plane.
	private static final long PAIR_MERGE_DIST2 = (long) (8 << 12) * (8 << 12);
	private static final int PAIR_MERGE_DOT = F * 3 / 4;
	// Upward component a pair contact needs to count as support. Wider than the
	// world pass' ground cone (F * 7 / 10): a cube balanced on the seam between two
	// cubes is held by normals tilted well past 45 degrees, and a body not
	// recognised as supported may never sleep.
	private static final int SUPPORT_UP = F / 2;
	// Position projection strength by manifold size: the penetration is re-derived
	// from the local anchors every iteration, so one contact needs a much bigger
	// step than a four point face manifold. Without it an edge-edge impact keeps a
	// third of its depth.
	private static final int[] PAIR_BETA = {F / 2, F * 3 / 8, F * 5 / 16, F * 3 / 16};
	// Zero inverse inertia, for bodies that must not rotate (carried,
	// asleep).
	private static final int[] ZERO_I = new int[9];
	// Clipping polygon buffers: 4 incident corners plus at most one point
	// per clip plane.
	private static final int CLIP_MAX = 10;

	// ---- generation scratch, one pair at a time ----
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
	private static final int[] pbias = new int[PAIR_MAX_CONTACTS];
	private static final int[] clipX = new int[CLIP_MAX];
	private static final int[] clipY = new int[CLIP_MAX];
	private static final int[] clipZ = new int[CLIP_MAX];
	private static final int[] clipOX = new int[CLIP_MAX];
	private static final int[] clipOY = new int[CLIP_MAX];
	private static final int[] clipOZ = new int[CLIP_MAX];
	// closest point pair scratch, see closestPairPoints
	private static int cp1x, cp1y, cp1z, cp2x, cp2y, cp2z;

	// ---- box accessors, so the axis code below stays readable ----

	// World space box axis i (column i of the row-major orientation).
	private static int axisX(RigidBody b, int i) { return b.r[i]; }
	private static int axisY(RigidBody b, int i) { return b.r[3 + i]; }
	private static int axisZ(RigidBody b, int i) { return b.r[6 + i]; }

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
			pa += mul(a.halfExtent(i), d < 0 ? -d : d);
			d = mul(axisX(b, i), lx) + mul(axisY(b, i), ly) + mul(axisZ(b, i), lz);
			pb += mul(b.halfExtent(i), d < 0 ? -d : d);
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
		int hn = ref.halfExtent(k), hu = ref.halfExtent(u), hv = ref.halfExtent(v);

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
		int hj = inc.halfExtent(bestJ), hu2 = inc.halfExtent(iu), hv2 = inc.halfExtent(iv);
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
		int ax = a.px + mul(su * a.halfExtent(u), axisX(a, u)) + mul(sv * a.halfExtent(v), axisX(a, v));
		int ay = a.py + mul(su * a.halfExtent(u), axisY(a, u)) + mul(sv * a.halfExtent(v), axisY(a, v));
		int az = a.pz + mul(su * a.halfExtent(u), axisZ(a, u)) + mul(sv * a.halfExtent(v), axisZ(a, v));
		int ex = mul(a.halfExtent(i), axisX(a, i));
		int ey = mul(a.halfExtent(i), axisY(a, i));
		int ez = mul(a.halfExtent(i), axisZ(a, i));

		u = (j + 1) % 3; v = (j + 2) % 3;
		du = mul(axisX(b, u), nx) + mul(axisY(b, u), ny) + mul(axisZ(b, u), nz);
		dv = mul(axisX(b, v), nx) + mul(axisY(b, v), ny) + mul(axisZ(b, v), nz);
		su = du <= 0 ? 1 : -1; sv = dv <= 0 ? 1 : -1;
		int bx = b.px + mul(su * b.halfExtent(u), axisX(b, u)) + mul(sv * b.halfExtent(v), axisX(b, v));
		int by = b.py + mul(su * b.halfExtent(u), axisY(b, u)) + mul(sv * b.halfExtent(v), axisY(b, v));
		int bz = b.pz + mul(su * b.halfExtent(u), axisZ(b, u)) + mul(sv * b.halfExtent(v), axisZ(b, v));
		ex = mul(b.halfExtent(j), axisX(b, j));
		ey = mul(b.halfExtent(j), axisY(b, j));
		ez = mul(b.halfExtent(j), axisZ(b, j));

		closestPairPoints(ax - ex, ay - ey, az - ez, ax + ex, ay + ey, az + ez,
				bx - ex, by - ey, bz - ez, bx + ex, by + ey, bz + ez);

		addPairContact(a, b, (cp1x + cp2x) >> 1, (cp1y + cp2y) >> 1, (cp1z + cp2z) >> 1,
				nx, ny, nz, pen);
	}

	// Closest points of two segments (Ericson), in plain units with Q14 parameters:
	// the Q12 products of a world-scale distance would overflow.
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

		// Anchors in each body's local frame (local = R^T * world offset): the position
		// projection re-derives the penetration from them, so it converges.
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

	// How deep a kinematic body is pressed into this pair, and along which
	// normal, so the hand carrying it can tell yielding from not yielding and
	// pressing in from backing out (RigidBody.pressPen).
	private static void recordPress(RigidBody a, RigidBody b, int i) {
		if(a.kinematic && ppen[i] > a.pressPen) {
			a.pressPen = ppen[i];
			a.pressNX = pnx[i]; a.pressNY = pny[i]; a.pressNZ = pnz[i];
		}
		if(b.kinematic && ppen[i] > b.pressPen) {
			b.pressPen = ppen[i];
			b.pressNX = -pnx[i]; b.pressNY = -pny[i]; b.pressNZ = -pnz[i];
		}
	}

	// Whoever the world or its own support holds gives up its share, so the whole
	// response goes to the body that can move and a cube on the floor carries a
	// stack instead of being squashed into it. When neither can move the contact
	// is left unsolved, so a shove cannot drive a cube into geometry this pass
	// cannot see.
	private static boolean heldA, heldB;

	private static void pairHeld(RigidBody a, RigidBody b, boolean aStatic,
			boolean bStatic, int nx, int ny, int nz) {
		heldA = !aStatic && a.heldAgainst(-nx, -ny, -nz);
		heldB = !bStatic && b.heldAgainst(nx, ny, nz);
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
		// A moving carried cube is about to displace whatever it touches, so a sleeper in
		// its way always joins: at a slow walk the hand velocity stays under the
		// neighbour threshold. A parked hand is a shelf, and a cube may rest on it.
		if(b.kinematic && bodySpeed(b) > 0) return true;
		// A parked hand is no shelf for a cube it is inside, though. Left asleep
		// the pair is skipped whole, nothing reports how deep the hand is
		// pressed (RigidBody.pressPen), and the hand takes another step.
		for(int i = 0; i < count; i++) {
			if(ppen[i] > WAKE_PENETRATION) return true;
		}
		// the body it rests on is moving: hanging around would leave the
		// sleeper floating once its support slid away
		return bodySpeed(b) > WAKE_NEIGHBOUR_SPEED;
	}

	// World space anchor scratch, filled by worldAnchor.
	private static int anchorX, anchorY, anchorZ;

	// p + R * local[i * 3 ..]: contact i's anchor on body x in world space, so a
	// contact stays attached to the same point of each box as they move.
	private static void worldAnchor(RigidBody x, int[] local, int i) {
		int lx = local[i * 3], ly = local[i * 3 + 1], lz = local[i * 3 + 2];
		anchorX = x.px + mul(lx, x.r[0]) + mul(ly, x.r[1]) + mul(lz, x.r[2]);
		anchorY = x.py + mul(lx, x.r[3]) + mul(ly, x.r[4]) + mul(lz, x.r[5]);
		anchorZ = x.pz + mul(lx, x.r[6]) + mul(ly, x.r[7]) + mul(lz, x.r[8]);
	}



	// The frame's pair batch. Slots double when a level needs more, since
	// dropping a pair would drop its constraint.
	static final int INITIAL_PAIR_SLOTS = 8;
	static int pairSlots = INITIAL_PAIR_SLOTS;
	static int pairCount, contactCount;
	static RigidBody[] pairA = new RigidBody[INITIAL_PAIR_SLOTS];
	static RigidBody[] pairB = new RigidBody[INITIAL_PAIR_SLOTS];
	static boolean[] pairAStatic = new boolean[INITIAL_PAIR_SLOTS];
	static boolean[] pairBStatic = new boolean[INITIAL_PAIR_SLOTS];
	static int[] pairAHeld = new int[INITIAL_PAIR_SLOTS];
	static int[] pairBHeld = new int[INITIAL_PAIR_SLOTS];
	static int[] pairManifold = new int[INITIAL_PAIR_SLOTS];
	static int[] cPair = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS];
	static int[] cIndex = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS];
	static int[] cNX = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS];
	static int[] cNY = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS];
	static int[] cNZ = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS];
	static int[] cX = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS];
	static int[] cY = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS];
	static int[] cZ = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS];
	static int[] cPen = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS];
	static int[] cAnchorA = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS * 3];
	static int[] cAnchorB = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS * 3];
	static int[] cAccN = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS];
	static int[] cAccT = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS];
	static int[] cBias = new int[INITIAL_PAIR_SLOTS * PAIR_MAX_CONTACTS];

	private static int[] grown(int[] src, int n) {
		int[] dst = new int[n];
		System.arraycopy(src, 0, dst, 0, src.length);
		return dst;
	}

	private static void growBatch() {
		int slots = pairSlots << 1;
		int contacts = slots * PAIR_MAX_CONTACTS;
		RigidBody[] a = new RigidBody[slots], b = new RigidBody[slots];
		System.arraycopy(pairA, 0, a, 0, pairSlots);
		System.arraycopy(pairB, 0, b, 0, pairSlots);
		pairA = a; pairB = b;
		boolean[] sa = new boolean[slots], sb = new boolean[slots];
		System.arraycopy(pairAStatic, 0, sa, 0, pairSlots);
		System.arraycopy(pairBStatic, 0, sb, 0, pairSlots);
		pairAStatic = sa; pairBStatic = sb;
		pairAHeld = grown(pairAHeld, slots);
		pairBHeld = grown(pairBHeld, slots);
		pairManifold = grown(pairManifold, slots);
		cPair = grown(cPair, contacts);
		cIndex = grown(cIndex, contacts);
		cNX = grown(cNX, contacts);
		cNY = grown(cNY, contacts);
		cNZ = grown(cNZ, contacts);
		cX = grown(cX, contacts);
		cY = grown(cY, contacts);
		cZ = grown(cZ, contacts);
		cPen = grown(cPen, contacts);
		cAnchorA = grown(cAnchorA, contacts * 3);
		cAnchorB = grown(cAnchorB, contacts * 3);
		cAccN = grown(cAccN, contacts);
		cAccT = grown(cAccT, contacts);
		cBias = grown(cBias, contacts);
		pairSlots = slots;
	}

	// Every box-box pair for one frame, after all bodies stepped against the
	// world. Null entries are allowed.
	static void collide(RigidBody[] bodies, int count) {
		for(int i = 0; i < count; i++) {
			RigidBody x = bodies[i];
			if(x == null) continue;
			x.prevSupport = x.supportBody;
			x.supportBody = null;
			x.bodySupport = false;
			x.supportNX = x.supportNY = x.supportNZ = 0;
			x.pairTouched = false;
			x.pressPen = x.pressNX = x.pressNY = x.pressNZ = 0;
		}

		pairCount = 0;
		contactCount = 0;
		for(int i = 0; i < count; i++) {
			RigidBody a = bodies[i];
			if(a == null) continue;
			for(int j = i + 1; j < count; j++) {
				RigidBody b = bodies[j];
				if(b == null) continue;
				addPair(a, b);
			}
		}

		for(int s = 0; s < PAIR_VELOCITY_SWEEPS; s++) {
			for(int c = 0; c < contactCount; c++) velocitySweep(c);
		}
		for(int s = 0; s < PAIR_POSITION_SWEEPS; s++) {
			for(int c = 0; c < contactCount; c++) positionSweep(c);
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
			// A cube held up by another cube is only at rest once this pass has
			// cancelled the gravity its own step could not see.
			if(x.bodySupport && !x.kinematic) x.updateSleep();
		}
	}

	// Generate one pair's manifold into the batch instead of solving it here.
	private static void addPair(RigidBody a, RigidBody b) {
		if(pairCount >= pairSlots) growBatch();
		int n = generateContacts(a, b);
		if(n == 0) return;

		recordSupport(a, b, n);
		if(a.sleeping && !b.sleeping && shouldWake(a, b, n)) a.wake();
		if(b.sleeping && !a.sleeping && shouldWake(b, a, n)) b.wake();
		boolean aStatic = a.kinematic || a.sleeping;
		boolean bStatic = b.kinematic || b.sleeping;
		if(aStatic && bStatic) return;

		int p = pairCount++;
		pairA[p] = a; pairB[p] = b;
		pairAStatic[p] = aStatic; pairBStatic[p] = bStatic;
		pairManifold[p] = n;
		int ab = 0, bb = 0;
		int base = contactCount;
		for(int i = 0; i < n; i++) {
			pairHeld(a, b, aStatic, bStatic, pnx[i], pny[i], pnz[i]);
			recordPress(a, b, i);
			if(heldA) ab |= 1 << i;
			if(heldB) bb |= 1 << i;
			int c = contactCount++;
			cPair[c] = p; cIndex[c] = i;
			cNX[c] = pnx[i]; cNY[c] = pny[i]; cNZ[c] = pnz[i];
			cX[c] = ppx[i]; cY[c] = ppy[i]; cZ[c] = ppz[i];
			cPen[c] = ppen[i];
			for(int k = 0; k < 3; k++) {
				cAnchorA[c * 3 + k] = pral[i * 3 + k];
				cAnchorB[c * 3 + k] = prbl[i * 3 + k];
			}
			cAccN[c] = 0; cAccT[c] = 0;
		}
		pairAHeld[p] = ab; pairBHeld[p] = bb;
		a.pairTouched = !aStatic;
		b.pairTouched = !bStatic;

		// Restitution targets once per pair, from the approach velocities.
		int avx = velX(a), avy = velY(a), avz = velZ(a);
		int bvx = velX(b), bvy = velY(b), bvz = velZ(b);
		for(int i = 0; i < n; i++) {
			int c = base + i;
			int rax = cX[c] - a.px, ray = cY[c] - a.py, raz = cZ[c] - a.pz;
			int rbx = cX[c] - b.px, rby = cY[c] - b.py, rbz = cZ[c] - b.pz;
			int vax = avx + mul(a.wy, raz) - mul(a.wz, ray);
			int vay = avy + mul(a.wz, rax) - mul(a.wx, raz);
			int vaz = avz + mul(a.wx, ray) - mul(a.wy, rax);
			int vbx = bvx + mul(b.wy, rbz) - mul(b.wz, rby);
			int vby = bvy + mul(b.wz, rbx) - mul(b.wx, rbz);
			int vbz = bvz + mul(b.wx, rby) - mul(b.wy, rbx);
			int vn = mul(vbx - vax, cNX[c]) + mul(vby - vay, cNY[c])
					+ mul(vbz - vaz, cNZ[c]);
			cBias[c] = -vn > RigidBody.RESTITUTION_SPEED
					? -mul(BODY_RESTITUTION, vn) : 0;
		}
	}

	// One Gauss-Seidel sweep of one contact. Writing straight to the bodies is
	// what lets the next contact in the sweep see the result.
	private static void velocitySweep(int c) {
		int p = cPair[c], i = cIndex[c];
		RigidBody a = pairA[p], b = pairB[p];
		boolean aStatic = pairAStatic[p], bStatic = pairBStatic[p];
		boolean aHeld = ((pairAHeld[p] >> i) & 1) != 0;
		boolean bHeld = ((pairBHeld[p] >> i) & 1) != 0;
		int imA = aStatic ? 0 : a.invMass, imB = bStatic ? 0 : b.invMass;
		int[] iiA = aStatic ? ZERO_I : a.invIWorld;
		int[] iiB = bStatic ? ZERO_I : b.invIWorld;
		int imAc = aHeld ? 0 : imA, imBc = bHeld ? 0 : imB;
		int[] iiAc = aHeld ? ZERO_I : iiA, iiBc = bHeld ? ZERO_I : iiB;
		int nx = cNX[c], ny = cNY[c], nz = cNZ[c];
		int rax = cX[c] - a.px, ray = cY[c] - a.py, raz = cZ[c] - a.pz;
		int rbx = cX[c] - b.px, rby = cY[c] - b.py, rbz = cZ[c] - b.pz;
		int avx = velX(a), avy = velY(a), avz = velZ(a);
		int bvx = velX(b), bvy = velY(b), bvz = velZ(b);
		int vax = avx + mul(a.wy, raz) - mul(a.wz, ray);
		int vay = avy + mul(a.wz, rax) - mul(a.wx, raz);
		int vaz = avz + mul(a.wx, ray) - mul(a.wy, rax);
		int vbx = bvx + mul(b.wy, rbz) - mul(b.wz, rby);
		int vby = bvy + mul(b.wz, rbx) - mul(b.wx, rbz);
		int vbz = bvz + mul(b.wx, rby) - mul(b.wy, rbx);
		int rvx = vbx - vax, rvy = vby - vay, rvz = vbz - vaz;
		int vn = mul(rvx, nx) + mul(rvy, ny) + mul(rvz, nz);

		int kn = imAc + imBc
				+ angularEffectiveMass(rax, ray, raz, iiAc, nx, ny, nz)
				+ angularEffectiveMass(rbx, rby, rbz, iiBc, nx, ny, nz);
		if(kn > 0) {
			int dN = accumulate(cAccN, c, divQ(cBias[c] - vn, kn), 0, Integer.MAX_VALUE);
			if(dN != 0) {
				if(!bStatic) {
					int imp = mul(dN, imBc);
					b.vx += mul(nx, imp); b.vy += mul(ny, imp); b.vz += mul(nz, imp);
					b.lx += mulL(rby, mulL(nz, dN)) - mulL(rbz, mulL(ny, dN));
					b.ly += mulL(rbz, mulL(nx, dN)) - mulL(rbx, mulL(nz, dN));
					b.lz += mulL(rbx, mulL(ny, dN)) - mulL(rby, mulL(nx, dN));
					b.wx = eval24X(iiBc, b.lx, b.ly, b.lz);
					b.wy = eval24Y(iiBc, b.lx, b.ly, b.lz);
					b.wz = eval24Z(iiBc, b.lx, b.ly, b.lz);
				}
				if(!aStatic) {
					int imp = mul(dN, imAc);
					a.vx -= mul(nx, imp); a.vy -= mul(ny, imp); a.vz -= mul(nz, imp);
					a.lx -= mulL(ray, mulL(nz, dN)) - mulL(raz, mulL(ny, dN));
					a.ly -= mulL(raz, mulL(nx, dN)) - mulL(rax, mulL(nz, dN));
					a.lz -= mulL(rax, mulL(ny, dN)) - mulL(ray, mulL(nx, dN));
					a.wx = eval24X(iiAc, a.lx, a.ly, a.lz);
					a.wy = eval24Y(iiAc, a.lx, a.ly, a.lz);
					a.wz = eval24Z(iiAc, a.lx, a.ly, a.lz);
				}
			}
		}

		if(cAccN[c] <= 0) return;
		rvx = (bvx + mul(b.wy, rbz) - mul(b.wz, rby)) - (avx + mul(a.wy, raz) - mul(a.wz, ray));
		rvy = (bvy + mul(b.wz, rbx) - mul(b.wx, rbz)) - (avy + mul(a.wz, rax) - mul(a.wx, raz));
		rvz = (bvz + mul(b.wx, rby) - mul(b.wy, rbx)) - (avz + mul(a.wx, ray) - mul(a.wy, rax));
		if(!contactTangent(rvx, rvy, rvz, nx, ny, nz)) return;
		int kt = imAc + imBc
				+ angularEffectiveMass(rax, ray, raz, iiAc, tanX, tanY, tanZ)
				+ angularEffectiveMass(rbx, rby, rbz, iiBc, tanX, tanY, tanZ);
		if(kt <= 0) return;
		int vt = mul(rvx, tanX) + mul(rvy, tanY) + mul(rvz, tanZ);
		int maxFric = abs(mul(BODY_FRICTION, cAccN[c]));
		int dT = accumulate(cAccT, c, -divQ(vt, kt), -maxFric, maxFric);
		if(dT == 0) return;
		if(!bStatic) {
			int imp = mul(dT, imBc);
			b.vx += mul(tanX, imp); b.vy += mul(tanY, imp); b.vz += mul(tanZ, imp);
			b.lx += mulL(rby, mulL(tanZ, dT)) - mulL(rbz, mulL(tanY, dT));
			b.ly += mulL(rbz, mulL(tanX, dT)) - mulL(rbx, mulL(tanZ, dT));
			b.lz += mulL(rbx, mulL(tanY, dT)) - mulL(rby, mulL(tanX, dT));
			b.wx = eval24X(iiBc, b.lx, b.ly, b.lz);
			b.wy = eval24Y(iiBc, b.lx, b.ly, b.lz);
			b.wz = eval24Z(iiBc, b.lx, b.ly, b.lz);
		}
		if(!aStatic) {
			int imp = mul(dT, imAc);
			a.vx -= mul(tanX, imp); a.vy -= mul(tanY, imp); a.vz -= mul(tanZ, imp);
			a.lx -= mulL(ray, mulL(tanZ, dT)) - mulL(raz, mulL(tanY, dT));
			a.ly -= mulL(raz, mulL(tanX, dT)) - mulL(rax, mulL(tanZ, dT));
			a.lz -= mulL(rax, mulL(tanY, dT)) - mulL(ray, mulL(tanX, dT));
			a.wx = eval24X(iiAc, a.lx, a.ly, a.lz);
			a.wy = eval24Y(iiAc, a.lx, a.ly, a.lz);
			a.wz = eval24Z(iiAc, a.lx, a.ly, a.lz);
		}
	}

	// Penetration from the bodies' current poses, so a projection sweep sees how
	// far apart the pair already is.
	private static int contactPenetration(RigidBody a, RigidBody b, int c) {
		worldAnchor(a, cAnchorA, c);
		int awx = anchorX, awy = anchorY, awz = anchorZ;
		worldAnchor(b, cAnchorB, c);
		int sep = mul(anchorX - awx, cNX[c]) + mul(anchorY - awy, cNY[c])
				+ mul(anchorZ - awz, cNZ[c]);
		return cPen[c] - sep;
	}

	private static void positionSweep(int c) {
		int p = cPair[c];
		RigidBody a = pairA[p], b = pairB[p];
		boolean aStatic = pairAStatic[p], bStatic = pairBStatic[p];
		int pen = contactPenetration(a, b, c);
		if(pen <= RigidBody.POSITION_SLOP << 12) return;
		int nx = cNX[c], ny = cNY[c], nz = cNZ[c];
		pairHeld(a, b, aStatic, bStatic, nx, ny, nz);
		boolean aMove = !aStatic && !heldA;
		boolean bMove = !bStatic && !heldB;
		int imAc = aMove ? (aStatic ? 0 : a.invMass) : 0;
		int imBc = bMove ? (bStatic ? 0 : b.invMass) : 0;
		int[] iiAc = aMove ? (aStatic ? ZERO_I : a.invIWorld) : ZERO_I;
		int[] iiBc = bMove ? (bStatic ? ZERO_I : b.invIWorld) : ZERO_I;

		worldAnchor(a, cAnchorA, c);
		int awx = anchorX, awy = anchorY, awz = anchorZ;
		worldAnchor(b, cAnchorB, c);
		int cx = (awx + anchorX) >> 1, cy = (awy + anchorY) >> 1, cz = (awz + anchorZ) >> 1;
		int rax = cx - a.px, ray = cy - a.py, raz = cz - a.pz;
		int rbx = cx - b.px, rby = cy - b.py, rbz = cz - b.pz;

		int k = imAc + imBc
				+ angularEffectiveMass(rax, ray, raz, iiAc, nx, ny, nz)
				+ angularEffectiveMass(rbx, rby, rbz, iiBc, nx, ny, nz);
		if(k <= 0) return;
		int beta = pairManifold[p] <= PAIR_BETA.length
				? PAIR_BETA[pairManifold[p] - 1] : PAIR_BETA[PAIR_BETA.length - 1];
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
