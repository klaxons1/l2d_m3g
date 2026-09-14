package com;

import javax.microedition.m3g.Appearance;
import javax.microedition.m3g.IndexBuffer;
import javax.microedition.m3g.Mesh;
import javax.microedition.m3g.PolygonMode;
import javax.microedition.m3g.Transform;
import javax.microedition.m3g.TriangleStripArray;
import javax.microedition.m3g.VertexArray;
import javax.microedition.m3g.VertexBuffer;

/**
 * Portal style weighted storage cube with a real rigid body.
 *
 * The cube is an oriented box simulated by {@link RigidBody}: it falls,
 * rests, tumbles, slides, bounces off the room geometry and can be pushed
 * around. It can also be picked up and carried in front of the camera;
 * while held the rigid body is moved kinematically. Portals warp its
 * position, velocity and orientation just like the player.
 *
 * The visible mesh is generated in code: six faces with a small vertex
 * grid so the face centre can be painted in a different colour without a
 * texture. The mesh lives at the body centre, so the rigid body orientation
 * matrix is used as the model transform directly.
 */
public final class Cube extends GameObject {

	/** Half size of the cube (also the rigid body half extent). */
	public static final int HALF = 500;

	private static final int HOLD_DIST = 1900;
	private static final int GRAB_RANGE = 3400;
	private static final int THROW_SPEED = 320;
	private static final int FALL_LIMIT = 30000;
	private static final int MAX_NEAR_MESHES = 8;

	/**
	 * Push-out iterations at the final carried position: two extra passes
	 * resolve corners where the cube touches more than one surface.
	 */
	private static final int CARRY_PUSH_PASSES = 3;

	private static final int COLOR_BODY = 0xb4b4be;
	private static final int COLOR_EDGE = 0x64646e;
	private static final int COLOR_MARK = 0xff5fa0;

	private static Mesh mesh;
	private static boolean meshFailed;

	private final Player player;
	private final PortalManager pm;

	private final RigidBody body = new RigidBody(HALF);
	private final Vector3D spawn = new Vector3D();
	private final Vector3D dir = new Vector3D();
	private final Vector3D tmp = new Vector3D();
	private final Vector3D tmpSpeed = new Vector3D();
	private final Vector3D sweep = new Vector3D();
	private final Ray carryRay = new Ray();

	private boolean held;

	// scratch collision meshes, reused every frame (no per frame allocation)
	private final MeshData[] meshBuf = new MeshData[MAX_NEAR_MESHES];
	private final RigidBody.Collider[] colliders = new RigidBody.Collider[MAX_NEAR_MESHES];

	// scratch rendering transform
	private final Transform modelTransform = new Transform();
	private final float[] modelMatrix = new float[16];
	// scratch portal warp matrix
	private final Transform warpTransform = new Transform();
	private final float[] warpMatrix = new float[16];

	public Cube(Vector3D spawnPoint, Player player, PortalManager pm) {
		this.player = player;
		this.pm = pm;

		for(int i = 0; i < MAX_NEAR_MESHES; i++) colliders[i] = new RigidBody.Collider();

		this.spawn.set(spawnPoint);

		Character ch = this.character;
		ch.reset();
		ch.set(HALF, HALF * 2);
		ch.getPosition().set(spawnPoint.x, spawnPoint.y, spawnPoint.z);

		body.reset(spawnPoint.x, spawnPoint.y + HALF, spawnPoint.z);

		// alive but indestructible: dead objects skip collisions and get
		// removed by the scene cleanup
		this.setHp(1000);
	}

	public final boolean isHeld() {
		return held;
	}

	/** Picks the cube up if it is near and roughly under the crosshair. */
	public final boolean tryGrab() {
		if(held || player == null || player.isDead()) return false;

		Character pc = player.getCharacter();
		Vector3D pp = pc.getPosition();
		int cx = body.getCenterX(), cy = body.getCenterY(), cz = body.getCenterZ();

		int dx = cx - pp.x;
		int dy = cy - (pp.y + pc.getHeight());
		int dz = cz - pp.z;

		long d2 = (long) dx * dx + (long) dy * dy + (long) dz * dz;
		if(d2 > (long) GRAB_RANGE * GRAB_RANGE) return false;

		Vector3D pr = pc.getRotation();
		dir.setFromRotation(pr.x, pr.y);

		long dot = (long) dir.x * dx + (long) dir.y * dy + (long) dir.z * dz;
		if(dot <= 0) return false;

		double len = Math.sqrt((double) d2) * (1 << 14);
		if(len > 0 && dot < len * 0.5) return false;

		held = true;
		heldThroughPortal = -1;
		return true;
	}

	/** Drops the cube, giving it the player's look velocity. */
	public final void drop() {
		if(!held) return;
		held = false;

		if(player == null) {
			body.setVelocity(0, 0, 0);
			return;
		}

		Vector3D pr = player.getCharacter().getRotation();
		dir.setFromRotation(pr.x, pr.y);
		body.setVelocity(
				(dir.x * THROW_SPEED) >> 14,
				((dir.y * THROW_SPEED) >> 14) + 40,
				(dir.z * THROW_SPEED) >> 14);
	}

	public final void toggleGrab() {
		if(held) drop();
		else tryGrab();
	}

	public final void update(Scene scene) {
		House house = scene.getHouse();
		Character ch = this.character;
		Vector3D cpos = ch.getPosition();

		if(held) {
			updateHeld(house, ch, cpos);
			syncCharacter(house);
			return;
		}

		// Scene.update resolves pairwise character overlaps before update():
		// absorb the push it gave the kinematic capsule and feed it to the
		// rigid body as a nudge plus velocity.
		int feetX = body.getCenterX();
		int feetY = body.getCenterY() - HALF;
		int feetZ = body.getCenterZ();
		int pushX = cpos.x - feetX;
		int pushY = cpos.y - feetY;
		int pushZ = cpos.z - feetZ;
		if(pushX != 0 || pushY != 0 || pushZ != 0) {
			body.nudge(pushX, pushY, pushZ);
		}

		int oldCx = body.getCenterX();
		int oldCy = body.getCenterY();
		int oldCz = body.getCenterZ();

		// While crossing a portal opening the wall/floor is intangible.
		boolean ghost = false;
		if(pm != null && pm.isLinked()) {
			tmpSpeed.set(body.getVelocityX(), body.getVelocityY(), body.getVelocityZ());
			ghost = pm.isInOpening(oldCx, oldCy, oldCz, HALF, tmpSpeed);
		}

		int part = this.getPart();
		int count = house.fillNearMeshes(part, meshBuf, MAX_NEAR_MESHES);
		for(int i = 0; i < count; i++) {
			MeshData m = meshBuf[i];
			RigidBody.Collider c = colliders[i];
			c.verts = m.getVerts();
			c.pols = m.getPols();
			c.norms = m.getNorms();
			c.quads = m.getQuadsCount();
			c.tris = m.getTrisCount();
			c.scale8 = (int) (256 * m.getScale());
			c.offX = (int) (m.getOffsetX() * m.getScale());
			c.offY = (int) (m.getOffsetY() * m.getScale());
			c.offZ = (int) (m.getOffsetZ() * m.getScale());
		}

		body.step(colliders, count, !ghost);

		// fell out of the level: back to the spawn point
		if(body.getCenterY() < spawn.y - FALL_LIMIT) {
			respawn();
			syncCharacter(house);
			return;
		}

		updatePortalCrossing(oldCx, oldCy, oldCz, house);

		syncCharacter(house);
	}

	/** Warps the cube if the last move crossed a portal. Returns the crossed
	 *  portal index (-1 when none), tracked while the cube is carried. */

	/**
	 * If the cube cannot reach the hand target (blocked by geometry), the
	 * player can no longer hold it: drop it when the gap is this large.
	 */
	private static final int HOLD_DROP_DIST = 1200;

	private int heldOldX, heldOldY, heldOldZ;
	/** Portal the carried cube has crossed while the player stays behind (-1 = none). */
	private int heldThroughPortal = -1;

	private void updateHeld(House house, Character ch, Vector3D cpos) {
		if(player == null || player.isDead()) {
			held = false;
			return;
		}

		Character pc = player.getCharacter();
		Vector3D pp = pc.getPosition();
		Vector3D pr = pc.getRotation();
		dir.setFromRotation(pr.x, pr.y);

		// Camera frame in Q14: forward is the look vector, right is the
		// forward projected against world up, up completes the basis.
		int fx = dir.x, fy = dir.y, fz = dir.z;
		int rxx = -fz, rxy = 0, rxz = fx;
		long rl2 = (long) rxx * rxx + (long) rxz * rxz;
		// Looking nearly straight up/down makes the projected forward
		// degenerate; derive the right vector from yaw alone.
		if(rl2 < (long) (1 << 11) * (1 << 11)) {
			float yr = pr.y * MathUtils.FPI * 2 / (1 << 14);
			rxx = (int) (Math.cos(yr) * (1 << 14));
			rxz = (int) (-Math.sin(yr) * (1 << 14));
		} else {
			int rl = (int) Math.sqrt(rl2);
			rxx = (int) ((long) rxx * (1 << 14) / rl);
			rxz = (int) ((long) rxz * (1 << 14) / rl);
		}
		// up = right x forward
		int ux = (int) (((long) rxy * fz - (long) rxz * fy) >> 14);
		int uy = (int) (((long) rxz * fx - (long) rxx * fz) >> 14);
		int uz = (int) (((long) rxx * fy - (long) rxy * fx) >> 14);
		// The cube local +Z faces the holder (opposite the look direction).
		int bx = -fx, by = -fy, bz = -fz;

		tmp.set(
				pp.x + ((fx * HOLD_DIST) >> 14),
				pp.y + pc.getHeight() - 150 + ((fy * HOLD_DIST) >> 14),
				pp.z + ((fz * HOLD_DIST) >> 14));

		heldOldX = body.getCenterX();
		heldOldY = body.getCenterY();
		heldOldZ = body.getCenterZ();

		// Camera relative pose on the holder's side: columns right, up,
		// toward-holder, with the hand point as translation.
		float[] pose = modelMatrix;
		pose[0] = rxx / (float) (1 << 14);
		pose[1] = ux / (float) (1 << 14);
		pose[2] = bx / (float) (1 << 14);
		pose[3] = tmp.x;
		pose[4] = rxy / (float) (1 << 14);
		pose[5] = uy / (float) (1 << 14);
		pose[6] = by / (float) (1 << 14);
		pose[7] = tmp.y;
		pose[8] = rxz / (float) (1 << 14);
		pose[9] = uz / (float) (1 << 14);
		pose[10] = bz / (float) (1 << 14);
		pose[11] = tmp.z;
		pose[12] = 0;
		pose[13] = 0;
		pose[14] = 0;
		pose[15] = 1;

		int handX = tmp.x, handY = tmp.y, handZ = tmp.z;
		boolean linked = pm != null && pm.isLinked();

		// Choose which side of the portal pair the cube must live on this
		// frame. The side only switches when the hand path actually
		// crosses a portal plane, so the cube cannot flip back and forth
		// (which used to make it jitter while being dragged through).
		// The holder followed the cube through: both are in the same room
		// now, so stop mapping the hand through the portal.
		if(heldThroughPortal >= 0 && player.getPart() == this.getPart()) {
			heldThroughPortal = -1;
		}

		boolean through = false;
		int srcPortal = -1;
		if(linked && heldThroughPortal < 0) {
			// Segment crossing only: the hand can move past the plane in a
			// single frame, so any end-depth gate would miss the crossing.
			srcPortal = pm.findCrossedPortal(
					heldOldX, heldOldY, heldOldZ, handX, handY, handZ);
			if(srcPortal >= 0) through = true;
		} else if(linked && heldThroughPortal >= 0) {
			srcPortal = heldThroughPortal;
			int dst = pm.getLinkedPortal(srcPortal);
			// Far-side hand target: the holder-side pose mapped through.
			pm.getPortalTransform(srcPortal, warpTransform);
			modelTransform.set(pose);
			warpTransform.postMultiply(modelTransform);
			warpTransform.get(warpMatrix);

			int farX = (int) warpMatrix[3];
			int farY = (int) warpMatrix[7];
			int farZ = (int) warpMatrix[11];
			// The segment back to the hand crosses the destination portal
			// plane exactly when the holder pulled the cube out again.
			through = pm.findCrossedPortal(
					heldOldX, heldOldY, heldOldZ, farX, farY, farZ) != dst;
		}

		float[] finalPose = pose;
		if(through) {
			// Map the whole hand pose (position and camera basis) through.
			heldThroughPortal = srcPortal;
			pm.getPortalTransform(srcPortal, warpTransform);
			modelTransform.set(pose);
			warpTransform.postMultiply(modelTransform);
			warpTransform.get(warpMatrix);
			finalPose = warpMatrix;

			int room = pm.getRoomId(pm.getLinkedPortal(srcPortal));
			if(room >= 0) this.setPart(room);
		} else if(heldThroughPortal >= 0) {
			// Returned to the holder's side.
			heldThroughPortal = -1;
			if(player.getPart() >= 0) this.setPart(player.getPart());
		}

		int cx = (int) finalPose[3], cy = (int) finalPose[7], cz = (int) finalPose[11];

		// Walls are intangible while the carried cube crosses an opening,
		// checked on both sides of the pair.
		boolean ghost = false;
		if(linked) {
			tmpSpeed.set(cx - heldOldX, cy - heldOldY, cz - heldOldZ);
			ghost = pm.isInOpening(heldOldX, heldOldY, heldOldZ, HALF, tmpSpeed)
					|| pm.isInOpening(cx, cy, cz, HALF, tmpSpeed);
		}

		int resolvedX = cx, resolvedY = cy, resolvedZ = cz;
		if(!ghost) {
			int dx = cx - heldOldX;
			int dy = cy - heldOldY;
			int dz = cz - heldOldZ;
			long d2 = (long) dx * dx + (long) dy * dy + (long) dz * dz;
			int part = this.getPart();

			if(d2 > 0) {
				// Sweep a ray from the current center along the whole path
				// and stop one collision radius before the first surface.
				// A sphere cast at a target already deep inside a wall
				// cannot resolve (a center beyond the wall plane produces
				// no push-out), which let the carried cube clip through.
				int dist = (int) Math.sqrt(d2);
				carryRay.reset();
				carryRay.getStart().set(heldOldX, heldOldY, heldOldZ);
				carryRay.getDir().set(dx, dy, dz);
				house.rayCast(part, carryRay);

				if(carryRay.isCollision()) {
					int allowed = carryRay.getDistance() - HALF;
					if(allowed < 0) allowed = 0;
					if(allowed < dist) {
						resolvedX = heldOldX + (int) ((long) dx * allowed / dist);
						resolvedY = heldOldY + (int) ((long) dy * allowed / dist);
						resolvedZ = heldOldZ + (int) ((long) dz * allowed / dist);
					}
				}
			}

			// Final push-out resolves the cube extent around the ray hit
			// (corners, edge contacts) and converges against two surfaces.
			for(int pass = 0; pass < CARRY_PUSH_PASSES; pass++) {
				sweep.set(resolvedX, resolvedY, resolvedZ);
				if(!house.sphereCast(part, sweep, HALF)) break;
				resolvedX = sweep.x;
				resolvedY = sweep.y;
				resolvedZ = sweep.z;
			}
		}

		// Geometry kept the cube too far from the hand point: the holder
		// cannot reach it anymore, so let go (drop() throws it along the
		// look direction so it does not hang in the air).
		int gapX = resolvedX - cx, gapY = resolvedY - cy, gapZ = resolvedZ - cz;
		long gap2 = (long) gapX * gapX + (long) gapY * gapY + (long) gapZ * gapZ;
		if(gap2 > (long) HOLD_DROP_DIST * HOLD_DROP_DIST) {
			held = false;
			heldThroughPortal = -1;
			body.setKinematicPose(resolvedX, resolvedY, resolvedZ, finalPose);
			drop();
			return;
		}

		finalPose[3] = resolvedX;
		finalPose[7] = resolvedY;
		finalPose[11] = resolvedZ;
		body.setKinematicPose(resolvedX, resolvedY, resolvedZ, finalPose);
	}

	private int updatePortalCrossing(int oldCx, int oldCy, int oldCz, House house) {
		if(pm == null || !pm.isLinked()) return -1;

		int nx = body.getCenterX(), ny = body.getCenterY(), nz = body.getCenterZ();
		int crossed = pm.findCrossedPortal(oldCx, oldCy, oldCz, nx, ny, nz);
		if(crossed < 0) return -1;

		pm.getPortalTransform(crossed, warpTransform);
		warpTransform.get(warpMatrix);
		// The warp mirrors the crossing point exactly; do not add a fixed
		// push-out, it would make the cube lurch on every transition. Wall
		// collisions stay ghosted while it coasts out of the opening.
		body.warp(warpMatrix);

		int dst = pm.getLinkedPortal(crossed);
		int newRoom = pm.getRoomId(dst);
		if(newRoom >= 0) this.setPart(newRoom);
		return crossed;
	}

	/** Copies the rigid body state into the Character/room bookkeeping. */
	private void syncCharacter(House house) {
		Vector3D cpos = this.character.getPosition();
		cpos.set(body.getCenterX(), body.getCenterY() - HALF, body.getCenterZ());
		// Character speed is not simulated for the cube; keep it neutral so
		// it cannot interfere with pairwise pushes.
		this.character.getSpeed().set(0, 0, 0);
		house.recomputePart(this);
	}

	public final void respawn() {
		held = false;
		heldThroughPortal = -1;
		body.reset(spawn.x, spawn.y + HALF, spawn.z);
	}

	public final void render(Renderer g3d) {
		initMesh();
		if(mesh == null) return;

		for(int i = 0; i < 9; i++) {
			modelMatrix[(i / 3) * 4 + (i % 3)] = body.getOrientation(i) / (float) RigidBody.F;
		}
		modelMatrix[3] = body.getCenterX();
		modelMatrix[7] = body.getCenterY();
		modelMatrix[11] = body.getCenterZ();
		modelMatrix[12] = 0;
		modelMatrix[13] = 0;
		modelMatrix[14] = 0;
		modelMatrix[15] = 1;
		modelTransform.set(modelMatrix);

		g3d.addMesh(mesh, modelTransform);
	}

	public final boolean damage(GameObject obj, int dmg) {
		return false;
	}

	public final boolean isTimeToRenew() {
		return false;
	}

	private static void initMesh() {
		if(mesh != null || meshFailed) return;

		try {
			final int H = HALF;
			final int T = HALF / 3;
			final int[] grid = {-H, -T, T, H};

			final int faces = 6;
			final int vpf = 16;
			final int count = faces * vpf;

			short[] pos = new short[count * 3];
			byte[] nrm = new byte[count * 3];
			byte[] col = new byte[count * 3];
			int[] idx = new int[faces * 9 * 4];
			int[] lens = new int[faces * 9];

			int vi = 0, ii = 0, si = 0;

			for(int f = 0; f < faces; f++) {
				int axis = f >> 1;
				int sign = (f & 1) == 0 ? 1 : -1;
				int uAxis = (axis + 1) % 3;
				int vAxis = (axis + 2) % 3;

				int base = vi;

				for(int j = 0; j < 4; j++) {
					for(int i = 0; i < 4; i++) {
						int px = 0, py = 0, pz = 0;

						if(axis == 0) px = sign * H;
						else if(axis == 1) py = sign * H;
						else pz = sign * H;

						int u = grid[i];
						if(uAxis == 0) px = u;
						else if(uAxis == 1) py = u;
						else pz = u;

						int v = grid[j];
						if(vAxis == 0) px = v;
						else if(vAxis == 1) py = v;
						else pz = v;

						pos[vi * 3] = (short) px;
						pos[vi * 3 + 1] = (short) py;
						pos[vi * 3 + 2] = (short) pz;

						nrm[vi * 3 + axis] = (byte) (sign * 127);

						boolean rim = (i == 0 || i == 3 || j == 0 || j == 3);
						boolean centre = (i == 1 || i == 2) && (j == 1 || j == 2);
						int c = rim ? COLOR_EDGE : (centre ? COLOR_MARK : COLOR_BODY);

						col[vi * 3] = (byte) (c >> 16);
						col[vi * 3 + 1] = (byte) (c >> 8);
						col[vi * 3 + 2] = (byte) c;

						vi++;
					}
				}

			for(int j = 0; j < 3; j++) {
				for(int i = 0; i < 3; i++) {
					// Positive faces are wound CCW outward; mirror the
					// winding on negative faces so culling keeps them.
					if(sign > 0) {
						idx[ii++] = base + j * 4 + i;
						idx[ii++] = base + j * 4 + i + 1;
						idx[ii++] = base + (j + 1) * 4 + i;
						idx[ii++] = base + (j + 1) * 4 + i + 1;
					} else {
						idx[ii++] = base + j * 4 + i;
						idx[ii++] = base + (j + 1) * 4 + i;
						idx[ii++] = base + j * 4 + i + 1;
						idx[ii++] = base + (j + 1) * 4 + i + 1;
					}
					lens[si++] = 4;
				}
			}
			}

			VertexArray vaPos = new VertexArray(count, 3, 2);
			vaPos.set(0, count, pos);

			VertexArray vaNrm = new VertexArray(count, 3, 1);
			vaNrm.set(0, count, nrm);

			VertexArray vaCol = new VertexArray(count, 3, 1);
			vaCol.set(0, count, col);

			VertexBuffer vb = new VertexBuffer();
			vb.setPositions(vaPos, 1.0f, null);
			vb.setNormals(vaNrm);
			vb.setColors(vaCol);
			vb.setDefaultColor(0xffffffff);

			IndexBuffer ib = new TriangleStripArray(idx, lens);

			PolygonMode pmode = new PolygonMode();
			// Outward wound faces with back-face culling: the cube interior
			// is never drawn when the camera enters the box.
			pmode.setCulling(PolygonMode.CULL_BACK);
			pmode.setWinding(PolygonMode.WINDING_CCW);
			pmode.setShading(PolygonMode.SHADE_SMOOTH);
			pmode.setPerspectiveCorrectionEnable(false);

			Appearance ap = new Appearance();
			ap.setPolygonMode(pmode);
			// no Material: the scene has no lights and vertex colors are
			// drawn unlit

			mesh = new Mesh(vb, new IndexBuffer[]{ib}, new Appearance[]{ap});
		} catch(Throwable t) {
			// M3G mesh creation unavailable on this handset: the cube is
			// still simulated and can be grabbed/thrown, just not rendered
			meshFailed = true;
			mesh = null;
		}
	}
}
