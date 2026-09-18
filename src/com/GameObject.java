package com;

public abstract class GameObject extends RoomObject {

	private long frameQ;
	private int dampX, dampY, dampZ;
	protected final Character character = new Character(0, 0);
	private int hp;

	// Kinematic box the size of the capsule: what a cube pushes against when this
	// character walks into it (Cube.pushedByCharacters).
	RigidBody pushBody;
	private int pushBodyY;
	private static final float[] PUSH_POSE = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

	protected final void setCharacterSize(int modelHeight) {
		this.character.set((int) ((float) modelHeight / 2.5F), (int) ((float) modelHeight * 0.75F));
		this.pushBody = new RigidBody(this.character.getRadius());
		this.pushBody.setKinematic(true);
		this.pushBody.drags = true;
	}

	protected final void rotY(int angleQ8) {
		this.character.rotY(angleQ8);
	}

	protected final void moveZ(int d) {
		this.character.moveZ(d);
	}

	protected final void jump(int jump, float force) {
		this.character.jump(150, 1.2F);
	}

	public void update(Scene scene) {
		updateMovement(scene, true);
	}

	/**
	 * Integrates character movement.
	 *
	 * @param walls test wall collisions (disabled inside a portal opening)
	 */
	protected final void updateMovement(Scene scene, boolean walls) {
		updateMovement(scene, walls, true);
	}

	/**
	 * @param walls     test wall collisions
	 * @param floorSnap snap to the floor (and dampen speed on floor contact)
	 */
	protected final void updateMovement(Scene scene, boolean walls, boolean floorSnap) {
		slideOutOfPress();
		this.character.update();
		this.character.collisionTest(this.getPart(), scene.getHouse(), walls, floorSnap);
		// Cubes are bodies, not geometry, so the floor snap cannot see them.
		// Before onFloor: a cube is floor for walking, jumping and the damping.
		scene.standOnCubes(this.character);
		if(this.character.isOnFloor()) {
			Vector3D speed = this.character.getSpeed();
			int bleed = Character.floorBleed();
			this.dampX += speed.x * bleed;
			this.dampY += speed.y * bleed;
			this.dampZ += speed.z * bleed;
			speed.x -= this.dampX >> 12;
			speed.y -= this.dampY >> 12;
			speed.z -= this.dampZ >> 12;
			this.dampX &= SolverMath.F - 1;
			this.dampY &= SolverMath.F - 1;
			this.dampZ &= SolverMath.F - 1;
		}
		posePushBody();

		this.frameQ += FPS.dt;
	}

	// The push box is a box and the character's own test is a sphere, so walking
	// diagonally into a cube buries the box while the sphere still reports a
	// glancing touch. Give up the part of the step that presses deeper and slide
	// along, the way a carried cube slides along its jam: left alone, the
	// penetration grows until the shallowest axis flips and the pair solve
	// throws the cube sideways.
	private final void slideOutOfPress() {
		RigidBody box = this.pushBody;
		if(box == null || box.pressPen <= 0) return;

		Vector3D sp = this.character.getSpeed();
		int into = SolverMath.mul(box.pressNX, sp.x) + SolverMath.mul(box.pressNY, sp.y)
				+ SolverMath.mul(box.pressNZ, sp.z);
		int n2 = SolverMath.mul(box.pressNX, box.pressNX)
				+ SolverMath.mul(box.pressNY, box.pressNY)
				+ SolverMath.mul(box.pressNZ, box.pressNZ);
		if(into <= 0 || n2 <= 0) return;

		sp.x -= (int) (((long) box.pressNX * into) / n2);
		sp.y -= (int) (((long) box.pressNY * into) / n2);
		sp.z -= (int) (((long) box.pressNZ * into) / n2);
	}

	// Posed last, so the box lends the pass this frame's walk. Airborne it keeps
	// its grounded height, and a bigger step than a radius (first frame, warp,
	// respawn, ledge) is posed twice: a new place, not a shove.
	private final void posePushBody() {
		Vector3D pos = this.character.getPosition();
		int radius = this.character.getRadius();

		if(this.character.isOnFloor() || this.pushBodyY == 0) this.pushBodyY = pos.y + radius;
		long dx = pos.x - this.pushBody.getCenterX();
		long dy = this.pushBodyY - this.pushBody.getCenterY();
		long dz = pos.z - this.pushBody.getCenterZ();
		if(dx * dx + dy * dy + dz * dz > (long) radius * radius) {
			this.pushBody.setKinematicPose(pos.x, this.pushBodyY, pos.z, PUSH_POSE, FPS.dt);
		}
		this.pushBody.setKinematicPose(pos.x, this.pushBodyY, pos.z, PUSH_POSE, FPS.dt);
	}

	// true - если персонаж убит
	public boolean damage(GameObject obj, int dmg) {
		boolean var3 = this.isDead();
		this.hp -= dmg;
		if(this.hp < 0) {
			this.hp = 0;
		}

		if(var3 != this.isDead()) {
			this.frameQ = 0;
			return true;
		} else {
			return false;
		}
	}

	public final Character getCharacter() {
		return this.character;
	}

	public final int getHp() {
		return this.hp;
	}

	public final boolean isDead() {
		return this.hp <= 0;
	}

	public boolean isTimeToRenew() {
		return this.isDead() && this.frameQ > (25L << 12);
	}

	public final void setHp(int hp) {
		this.hp = hp;
	}

	public final int getFrame() {
		return (int) (this.frameQ >> 12);
	}

	public final int getPosX() {
		return this.character.getPosition().x;
	}

	public final int getPosY() {
		return this.character.getPosition().y;
	}

	public final int getPosZ() {
		return this.character.getPosition().z;
	}

	public final int getHeight() {
		return this.character.getHeight();
	}
}
