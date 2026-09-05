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
	 * Интегрирует движение персонажа.
	 *
	 * @param collide false - столкновения со стенами отключены
	 *                (используется, когда объект находится в проёме портала)
	 */
	protected final void updateMovement(Scene scene, boolean collide) {
		updateMovement(scene, collide, true);
	}

	/**
	 * @param walls     проверять столкновения со стенами
	 * @param floorSnap прижимать к полу (и гасить скорость при контакте с ним)
	 */
	protected final void updateMovement(Scene scene, boolean walls, boolean floorSnap) {
		this.character.update();
		
		this.character.collisionTest(this.getPart(), scene.getHouse(), walls, floorSnap);
		
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
