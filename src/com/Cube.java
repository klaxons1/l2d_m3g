package com;

import java.util.Vector;

import javax.microedition.m3g.Appearance;
import javax.microedition.m3g.IndexBuffer;
import javax.microedition.m3g.Mesh;
import javax.microedition.m3g.PolygonMode;
import javax.microedition.m3g.Transform;
import javax.microedition.m3g.TriangleStripArray;
import javax.microedition.m3g.VertexArray;
import javax.microedition.m3g.VertexBuffer;

// Portal style weighted storage cube: an oriented box simulated by RigidBody.
// It falls, tumbles and slides off the room geometry, collides with the other
// cubes (collideCubes), is pushed by walking characters, can be carried
// (kinematic while held) and goes through portals. The mesh is generated in
// code at the body centre, so the body's matrix is the model transform.
public final class Cube extends GameObject {

	// Half size of the cube (also the rigid body half extent).
	public static final int HALF = 500;

	private static final int HOLD_DIST = 1900;
	private static final int GRAB_RANGE = 3400;
	// Release speed, units/frame. The floor bleeds ~20 off anything sliding: 320
	// died inside two widths, 500 sends the target about two. Keep it under ~1200
	// - no swept test, so a faster pair steps clean past (measured from ~1400).
	private static final int THROW_SPEED = 500;
	private static final int FALL_LIMIT = 30000;
	private static final int MAX_NEAR_MESHES = 8;

	// Push-out iterations at the final carried position: the extra passes
	// resolve corners touching more than one surface.
	private static final int CARRY_PUSH_PASSES = 3;

	// A cube catches a character this far under its top and out from its edge. A
	// jump lifts the feet ~650 and a cube is 1000 tall: it takes a real jump.
	private static final int MANTLE = 450;

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
	private MeshData[] meshBuf = new MeshData[MAX_NEAR_MESHES];
	private RigidBody.Collider[] colliders = new RigidBody.Collider[MAX_NEAR_MESHES];

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

	// Picks the cube up if it is near and roughly under the crosshair.
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
		// carried cubes are immovable obstacles for the other cubes: they push
		// them out of the way with the hand velocity instead of reacting
		body.setKinematic(true);
		return true;
	}

	// Drops the cube, giving it the player's look velocity.
	public final void drop() {
		if(!held) return;
		held = false;
		body.setKinematic(false);

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

		int oldCx = body.getCenterX();
		int oldCy = body.getCenterY();
		int oldCz = body.getCenterZ();

		// Inside a portal opening the wall/floor is intangible. The feet are
		// tested as well as the centre, the way Player does it: a cube at rest on
		// a floor portal is exactly tangent to its plane, so the centre alone
		// misses it. 125 covers the solver's resting wobble (72 under a deep
		// penetration) without dropping a cube off a low ledge over a portal.
		boolean ghost = false;
		if(pm != null && pm.isLinked()) {
			tmpSpeed.set(body.getVelocityX(), body.getVelocityY(), body.getVelocityZ());
			ghost = pm.isInOpening(oldCx, oldCy, oldCz, HALF, tmpSpeed)
					|| pm.isInOpening(oldCx, oldCy - HALF, oldCz, HALF / 4, tmpSpeed);
		}

		int part = this.getPart();
		growNearMeshes(house, part);
		int count = house.fillNearMeshes(part, meshBuf, meshBuf.length);
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

		body.step(colliders, count, !ghost, FPS.dt);

		// fell out of the level: back to the spawn point
		if(body.getCenterY() < spawn.y - FALL_LIMIT) {
			respawn();
			syncCharacter(house);
			return;
		}

		updatePortalCrossing(oldCx, oldCy, oldCz, house);

		pushedByCharacters(scene);

		syncCharacter(house);
	}

	// A room with more neighbours than the buffer holds used to lose the rest
	// silently, and the cube then had no collider on that side at all.
	private void growNearMeshes(House house, int part) {
		if(part < 0) return;
		int need = house.getNeighbourRooms(part).length + 1;
		if(need <= meshBuf.length) return;

		MeshData[] meshes = new MeshData[need];
		RigidBody.Collider[] cols = new RigidBody.Collider[need];
		System.arraycopy(meshBuf, 0, meshes, 0, meshBuf.length);
		System.arraycopy(colliders, 0, cols, 0, colliders.length);
		for(int i = meshBuf.length; i < need; i++) cols[i] = new RigidBody.Collider();
		meshBuf = meshes;
		colliders = cols;
	}

	// Blocked by geometry, the cube cannot reach the hand target: drop it when
	// the gap is this large.
	private static final int HOLD_DROP_DIST = 1200;


	private int heldOldX, heldOldY, heldOldZ;
	// Portal the carried cube crossed while the player stayed behind (-1 = none).
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

		// Pressed against another body: give up the part of the hand step that
		// presses deeper and let the rest slide along. Any press counts, not a
		// deep one - the overlap relaxes the moment the hand stops, so a depth
		// threshold shook between held and full speed and crept into the jam.
		if(body.pressPen > 0) {
			int into = pressDepth(body, resolvedX - heldOldX, resolvedY - heldOldY,
					resolvedZ - heldOldZ);
			int n2 = SolverMath.mul(body.pressNX, body.pressNX)
					+ SolverMath.mul(body.pressNY, body.pressNY)
					+ SolverMath.mul(body.pressNZ, body.pressNZ);
			if(into > 0 && n2 > 0) {
				resolvedX -= (int) (((long) body.pressNX * into) / n2);
				resolvedY -= (int) (((long) body.pressNY * into) / n2);
				resolvedZ -= (int) (((long) body.pressNZ * into) / n2);
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
			body.setKinematicPose(resolvedX, resolvedY, resolvedZ, finalPose, FPS.dt);
			drop();
			return;
		}

		finalPose[3] = resolvedX;
		finalPose[7] = resolvedY;
		finalPose[11] = resolvedZ;
		body.setKinematicPose(resolvedX, resolvedY, resolvedZ, finalPose, FPS.dt);
	}

	// How much of a hand motion goes into whatever the pair pass reported the
	// carried cube pressed against. The press normal is not unit length, so the
	// caller divides its square out to cancel that part of the motion.
	private static int pressDepth(RigidBody body, int mx, int my, int mz) {
		return SolverMath.mul(body.pressNX, mx) + SolverMath.mul(body.pressNY, my)
				+ SolverMath.mul(body.pressNZ, mz);
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

	// Copies the body state into the Character/room bookkeeping.
	private void syncCharacter(House house) {
		Vector3D cpos = this.character.getPosition();
		cpos.set(body.getCenterX(), body.getCenterY() - HALF, body.getCenterZ());
		// Not simulated for the cube; keep it neutral for the next frame's test.
		this.character.getSpeed().set(0, 0, 0);
		house.recomputePart(this);
	}

	// Above this a character rides the cube: Scene.update skips its capsule test
	// and the cube carries them. The AABB top, so a tumbling cube still catches.
	final int rideY() {
		return body.boxMaxY - MANTLE;
	}

	// Raise a character onto this cube and carry them this frame's motion.
	// False when held: a carried cube is not stood on.
	final boolean supportCharacter(Character ch) {
		if(held) return false;

		Vector3D p = ch.getPosition();
		int top = body.boxMaxY;
		if(top <= p.y) return false;
		// not from underneath: a cube in the air would lift them onto it
		if(p.y < body.boxMinY) return false;

		// In the mantle band the capsule catches the edge one radius out, where a
		// rider stands; deeper in only the centre counts, so a walk pushes.
		int reach = (top - p.y <= MANTLE) ? ch.getRadius() : 0;
		if(p.x < body.boxMinX - reach || p.x > body.boxMaxX + reach) return false;
		if(p.z < body.boxMinZ - reach || p.z > body.boxMaxZ + reach) return false;

		ch.standOn(top, SolverMath.mul(body.getVelocityX(), FPS.dt),
				SolverMath.mul(body.getVelocityZ(), FPS.dt));
		return true;
	}

	// Scratch for the push pass, grown once: this cube plus whoever pushes it.
	private static RigidBody[] pushBodies = new RigidBody[0];

	// Walking characters shove this cube with the box that follows their capsule
	// (GameObject.pushBody), through the same pair pass a carried cube uses: a
	// force compounds until the cube is launched, contacts slide it upright.
	private void pushedByCharacters(Scene scene) {
		Vector objects = scene.getHouse().getObjects();
		if(pushBodies.length < objects.size() + 1) pushBodies = new RigidBody[objects.size() + 1];

		int count = 0, sx = 0, sy = 0, sz = 0, biggest = 0;
		pushBodies[count++] = body;

		for(int i = 0; i < objects.size(); i++) {
			GameObject obj = (GameObject) objects.elementAt(i);
			RigidBody pusher = obj.pushBody;
			if(pusher == null) continue;             // a cube: box vs box is collideCubes'
			Character ch = obj.getCharacter();
			// Floor level only: airborne the box keeps its grounded height and
			// would catch the cube's top edge. Reach is the box's height against
			// the cube's, or a cube up on a step is held by the capsule test and
			// pushed by nothing, and the two deadlock.
			if(!ch.isOnFloor()) continue;
			int feet = ch.getPosition().y;
			if(body.boxMaxY <= feet || body.boxMinY >= feet + 2 * pusher.getHalfY()) continue;

			pushBodies[count++] = pusher;
			int kx = pusher.kvx >> 12, ky = pusher.kvy >> 12, kz = pusher.kvz >> 12;
			sx += kx; sy += ky; sz += kz;
			int one = kx * kx + ky * ky + kz * kz;
			if(one > biggest) biggest = one;
		}

		for(int i = count; i < pushBodies.length; i++) pushBodies[i] = null;

		// One pass, and only a net one: the solver cannot balance two kinematic
		// lenders, so opposite faces would hand it the cube to the last solved.
		long net = (long) sx * sx + (long) sy * sy + (long) sz * sz;
		if(count > 1 && net * 4 >= biggest) RigidBody.collideBodies(pushBodies, count);
	}

	public final void respawn() {
		held = false;
		heldThroughPortal = -1;
		body.reset(spawn.x, spawn.y + HALF, spawn.z);
		body.setKinematic(false);
	}

	// Scratch body list for the pair pass (grown once, never per frame).
	private static RigidBody[] pairBodies = new RigidBody[0];

	// Collides every cube with every other one: they stack, bounce apart and a
	// carried cube shoves the others aside. Once per frame after Scene.update
	// has stepped each body against the world; this only collects and re-syncs.
	public static void collideCubes(Cube[] cubes, House house) {
		if(cubes == null) return;

		if(pairBodies.length < cubes.length) pairBodies = new RigidBody[cubes.length];
		int count = 0;
		for(int i = 0; i < cubes.length; i++) {
			Cube c = cubes[i];
			if(c == null) continue;
			pairBodies[count++] = c.body;
		}
		// do not hold on to cubes of a level that had more of them
		for(int i = count; i < pairBodies.length; i++) pairBodies[i] = null;
		if(count < 2) return;

		RigidBody.collideBodies(pairBodies, count);

		// The pass moved the bodies after they synced their characters: bring
		// the capsules and the room back in line for the next frame.
		for(int i = 0; i < cubes.length; i++) {
			Cube c = cubes[i];
			if(c != null) c.syncCharacter(house);
		}
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
