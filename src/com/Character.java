package com;

public final class Character {

	private static Vector3D tmpVec = new Vector3D();
	
	private Vector3D pos = new Vector3D();
	private Vector3D rot = new Vector3D();
	private Vector3D speed = new Vector3D();
	private int radius, height;
	
	private boolean onFloor = false;
	private boolean colDetected = false;

	public Character(int radius, int height) {
		reset();
		set(0, 0);
	}

	public final void set(int radius, int height) {
		this.radius = radius;
		this.height = height;
	}

	public final void reset() {
		onFloor = false;
		colDetected = false;
		
		pos.set(0, 0, 0);
		rot.set(0, 0, 0);
		speed.set(0, 0, 0);
	}

	public final void collisionTest(int part, House house) {
		collisionTest(part, house, true, true);
	}

	/**
	 * @param walls     test wall collisions (disabled inside a portal opening)
	 * @param floorSnap snap to the floor (a portal in the floor disables it too)
	 */
	public final void collisionTest(int part, House house, boolean walls, boolean floorSnap) {
		if(walls) {
			tmpVec.set(pos);
			tmpVec.y += height;

			colDetected = house.sphereCast(part, tmpVec, radius);
			if(colDetected) {
				pos.set(tmpVec);
				pos.y -= height;
			}
		} else {
			colDetected = false;
		}

		onFloor = false;
		if(floorSnap) {
			int floorY = house.getFloorY(part, pos.x, pos.y + height, pos.z);
			if(floorY != Integer.MAX_VALUE && floorY > pos.y) {
				pos.y = floorY;
				onFloor = true;
			}
		}
	}

	// Raised onto a support that is not level geometry (a physics cube): the
	// same snap collisionTest does for a floor. carry is one frame of the
	// support's own motion, so riding a sliding cube works.
	public final void standOn(int y, int carryX, int carryZ) {
		if(y > pos.y) {
			pos.y = y;
			pos.x += carryX;
			pos.z += carryZ;
			onFloor = true;
		}
	}

	// ? расстояние до другого персонажа
	public final long distanceSquared(Character ch) {
		Vector3D pos1 = pos;
		Vector3D pos2 = ch.pos;
		
		int xDist = pos1.x - pos2.x;
		int yDist = pos1.y - pos2.y;
		int zDist = pos1.z - pos2.z;
		
		return xDist * xDist + yDist * yDist + zDist * zDist;
	}

	public static void collisionTest(Character c1, Character c2) {
		Vector3D pos1 = c1.pos;
		Vector3D pos2 = c2.pos;
		
		int rSum = c1.radius + c2.radius;
		
		int dx = pos1.x - pos2.x;
		int dy = pos1.y - pos2.y;
		int dz = pos1.z - pos2.z;
		
		if(Math.abs(dx) <= rSum && Math.abs(dy) <= rSum && Math.abs(dz) <= rSum) {
			long distSqr = (long)dx*dx + (long)dy*dy + (long)dz*dz;
			
			if(distSqr < rSum * rSum) {
				if(distSqr != 0L) {
					distSqr = (long) (1.0F / MathUtils.invSqrt(distSqr));
				} else {
					dx = 1;
				}

				int dist = (int) (rSum - distSqr);

				tmpVec.set(dx, dy, dz);
				tmpVec.setLength(dist / 2);

				// Apply the separation as a velocity impulse, not as a
				// direct position snap: a position snap bypasses the wall
				// collision test and could teleport an entity (or the
				// player) through geometry and out of the level. The
				// impulse is integrated together with the rest of the
				// movement this frame, so walls resolve it normally.
				c1.speed.add(tmpVec);
				c2.speed.sub(tmpVec);
			}

		}
	}

	public final void moveZ(int d) {
		if(onFloor) {
			speed.x += (int) ((float) Math.sin(rot.y / (float)(1 << 14) * MathUtils.FPI * 2) * d);
			speed.z += (int) ((float) Math.cos(rot.y / (float)(1 << 14) * MathUtils.FPI * 2) * d);
		}

	}

	public final void moveX(int d) {
		if(onFloor) {
			speed.x += (int) ((float) Math.cos(rot.y / (float)(1 << 14) * MathUtils.FPI * 2) * d);
			speed.z -= (int) ((float) Math.sin(rot.y / (float)(1 << 14) * MathUtils.FPI * 2) * d);
		}

	}

	public final void rotY(int angle) {
		rot.y = (rot.y + angle * (1 << 14) / 360) & ((1 << 14) - 1);
	}

	public final void rotX(int angle) {
		rot.x += angle * (1 << 14) / 360;
	}

	public final void jump(int jump, float accel) {
		if(onFloor) {
			speed.y += jump;
			speed.x = (int) (speed.x * accel);
			speed.y = (int) (speed.y * accel);
			speed.z = (int) (speed.z * accel);
		}

	}

	public final void update() {
		tmpVec.set(speed);
		
		//Limit speed
		int radLimit = (int) (radius * 0.8F);
		if(tmpVec.lengthSquared() > radLimit * radLimit) {
			tmpVec.setLength(radLimit);
		}

		pos.x += tmpVec.x;
		pos.y += tmpVec.y;
		pos.z += tmpVec.z;
	}

	public final int getRadius() {
		return radius;
	}

	public final int getHeight() {
		return height;
	}

	public final Vector3D getSpeed() {
		return speed;
	}

	public final Vector3D getPosition() {
		return pos;
	}

	public final Vector3D getRotation() {
		return rot;
	}

	public final boolean isOnFloor() {
		return onFloor;
	}

	public final boolean isColDetected() {
		return colDetected;
	}
}
