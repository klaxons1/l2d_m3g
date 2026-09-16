package com;

public abstract class GameObject extends RoomObject {

	private int frame;
	protected final Character character = new Character(0, 0);
	private int hp;

	protected final void setCharacterSize(int modelHeight) {
		this.character.set((int) ((float) modelHeight / 2.5F), (int) ((float) modelHeight * 0.75F));
	}

	protected final void rotY(int angle) {
		this.character.rotY(angle);
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
		updateMovement(scene, walls, floorSnap, false);
	}

	/**
	 * @param supportCubes also stand on the physics cubes in the scene: they are
	 *                     bodies, not house geometry, so the floor snap above
	 *                     cannot see them
	 */
	protected final void updateMovement(Scene scene, boolean walls, boolean floorSnap, boolean supportCubes) {
		this.character.update();
		this.character.collisionTest(this.getPart(), scene.getHouse(), walls, floorSnap);
		// Before the onFloor test: standing on a cube is standing on floor as
		// far as walking, jumping and the damping below are concerned.
		if(supportCubes) scene.standOnCubes(this.character);
		if(this.character.isOnFloor()) {
			Vector3D speed = this.character.getSpeed();
			speed.x /= 4;
			speed.y /= 4;
			speed.z /= 4;
		}

		++this.frame;
	}

	// true - если персонаж убит
	public boolean damage(GameObject obj, int dmg) {
		boolean var3 = this.isDead();
		this.hp -= dmg;
		if(this.hp < 0) {
			this.hp = 0;
		}

		if(var3 != this.isDead()) {
			this.frame = 0;
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
		return this.isDead() && this.frame > 25;
	}

	public final void setHp(int hp) {
		this.hp = hp;
	}

	public final int getFrame() {
		return this.frame;
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
