package com;

public final class Player extends GameObject {

   private int money = 0;
   private int frags = 0;
   private Arsenal arsenal;
   private boolean damage = false;

   // Owns the two portals fired by the Portal Gun (null = no portals).
   private PortalManager portalManager;

   // The player's proxy in the cube pair pass: a kinematic box the size of their
   // capsule, so that a cube they walk into takes their walk instead of only
   // stopping them (Cube.pushedByPlayer). Created on the first update, once the
   // capsule radius is known, and posed from the feet up.
   private RigidBody pushBody;
   private int pushBodyY;
   private static final float[] PUSH_POSE = {
      1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

   final RigidBody pushBody() {
      return pushBody;
   }

   public Player(int width_g3d, int height_g3d, Vector3D pos, Object hudInfo) {
      this(width_g3d, height_g3d, pos, hudInfo, null);
   }

   public Player(int width_g3d, int height_g3d, Vector3D pos, Object hudInfo, PortalManager portalManager) {
      this.portalManager = portalManager;
      this.set(width_g3d, height_g3d, pos, hudInfo);
   }

   public final void set(int width_g3d, int height_g3d, Vector3D pos, Object hudInfo) {
      this.getCharacter().reset();
      this.getCharacter().getPosition().set(pos.x, pos.y, pos.z);
      // Dropped, and re-created at the new position on the next update: a
      // respawn is a teleport, and posing the old box there would lend the cube
      // pair pass the whole jump as a hand velocity.
      this.pushBody = null;
      this.setHp(100);
      this.money = 0;
      this.frags = 0;
      this.setCharacterSize(2005);
      if(this.arsenal != null) {
         this.arsenal.destroy();
         this.arsenal = null;
      }

      this.arsenal = new Arsenal(width_g3d, height_g3d, portalManager);
      if(hudInfo != null) {
         this.money = ((HUDInfo)hudInfo).money;
         int[] var8 = ((HUDInfo)hudInfo).ammo;
         Object[] var5 = this.arsenal.getWeapons();

         for(int var6 = 0; var6 < var5.length && var6 < var8.length; ++var6) {
            if(var5[var6] instanceof Weapon) {
               ((Weapon) var5[var6]).reset();
            } else if(var5[var6] instanceof PortalGun) {
               ((PortalGun) var5[var6]).reset();
            }

            if(var8[var6] == -1) {
               // The Portal Gun is never removed between levels.
               if(!(var5[var6] instanceof PortalGun)) var5[var6] = null;
            } else if(var6 < 5) {
               var5[var6] = Arsenal.createWeapon(var6);
               ((Weapon) var5[var6]).setAmmo(var8[var6]);
            }
         }

         Object cw = this.arsenal.currentWeapon();
         if(cw instanceof Weapon) {
            ((Weapon) cw).createSprite(width_g3d, height_g3d);
         } else if(cw instanceof PortalGun) {
            ((PortalGun) cw).createSprite(width_g3d, height_g3d);
         }
      }

   }

   public final void destroy() {
      this.arsenal.destroy();
      this.arsenal = null;
   }

   public final void render(Renderer g3d) {
      Weapon.renderSplinter(g3d);
      PortalGun.renderSplinter(g3d);
   }

   public final void update(Scene scene) {
      House house = scene.getHouse();
      Character ch = this.getCharacter();
      Vector3D pos = ch.getPosition();
      Vector3D speed = ch.getSpeed();
      int eyeOffset = ch.getHeight();
      int radius = ch.getRadius();

      int oldX = pos.x;
      int oldZ = pos.z;
      int oldEyeY = pos.y + eyeOffset;
      int oldFeetY = pos.y;

      // Inside a portal opening there is no wall (or floor): disable the
      // corresponding collisions so the player can walk straight through
      // instead of stumbling on the portal wall. Floor snapping and speed
      // damping stay active to avoid jitter.
      boolean ghost = false;
      boolean noFloor = false;
      boolean warped = false;
      if(this.portalManager != null) {
         ghost = this.portalManager.isInOpening(oldX, oldEyeY, oldZ, radius, speed)
               || this.portalManager.isInOpening(oldX, oldFeetY, oldZ, radius / 2, speed);
         noFloor = this.portalManager.isInFloorOpening(oldX, oldFeetY, oldZ, radius, speed)
               || this.portalManager.isInFloorOpening(oldX, oldEyeY, oldZ, radius, speed);
      }

      // The weighted storage cubes are rigid bodies, not house geometry, so the
      // floor snap cannot see them: standing on one goes through
      // Scene.standOnCubes, pushing one through the box posed at the end below.
      this.updateMovement(scene, !ghost, !noFloor, true);

      if(this.portalManager != null) {
         // Test the crossing at the eye point first, then at the feet
         // (the portal may lie on the floor or hang low on a wall).
         int refOffset = eyeOffset;
         int crossed = this.portalManager.findCrossedPortal(
               oldX, oldEyeY, oldZ,
               pos.x, pos.y + eyeOffset, pos.z);

         if(crossed < 0) {
            refOffset = 0;
            crossed = this.portalManager.findCrossedPortal(
                  oldX, oldFeetY, oldZ,
                  pos.x, pos.y, pos.z);
         }

         if(crossed >= 0) {
            this.portalManager.teleport(crossed, ch, refOffset);
            int newRoom = this.portalManager.getRoomId(this.portalManager.getLinkedPortal(crossed));
            if(newRoom >= 0) this.setPart(newRoom);
            house.recomputePart(this);
            warped = true;
         }
      }

      // The push box follows the capsule, at the end of the move so that it
      // lends the pair pass this frame's walk. In the air it keeps the height it
      // had on the ground: Cube.pushedByPlayer only solves it while the player
      // is on the floor, and a box that follows a jump would come back down with
      // the fall as its hand velocity and slam whatever it lands on.
      if(pushBody == null) {
         pushBody = new RigidBody(radius);
         pushBody.setKinematic(true);
         pushBodyY = pos.y + radius;
         warped = true;
      }
      if(ch.isOnFloor()) pushBodyY = pos.y + radius;
      // A jump in position - the first frame, a portal warp - is a new place, not
      // a hand velocity, so the box is posed there first and then again: the pair
      // pass sees no motion, and a cube at the destination does not take the
      // whole warp as a shove.
      if(warped) pushBody.setKinematicPose(pos.x, pushBodyY, pos.z, PUSH_POSE);
      pushBody.setKinematicPose(pos.x, pushBodyY, pos.z, PUSH_POSE);

      Object currentWeapon = this.arsenal.currentWeapon();
      GameObject var2 = null;

      if(currentWeapon instanceof Weapon) {
         var2 = ((Weapon) currentWeapon).update(house, this);
      } else if(currentWeapon instanceof PortalGun) {
         ((PortalGun) currentWeapon).update(house, this);
      }

      if(var2 instanceof Zombie) {
         this.money += 10;
      }

      if(var2 instanceof BigZombie) {
         this.money += 30;
      }

      if(var2 != null) {
         ++this.frags;
      }
   }

   public final boolean damage(GameObject obj, int dmg) {
      this.damage = true;
      return super.damage(obj, dmg);
	   //return false;
   }

   public final boolean isDamaged() {
      boolean var1 = this.damage;
      this.damage = false;
      return var1;
   }

   public final int getMoney() {
      return this.money;
   }

   public final int getFrags() {
      return this.frags;
   }

   public final void pay(int price) {
      this.money -= price;
   }

   public final Arsenal getArsenal() {
      return this.arsenal;
   }

   public final PortalManager getPortalManager() {
      return this.portalManager;
   }

   public final boolean isTimeToRenew() {
      return this.isDead() && this.getFrame() > 45;
   }

   public final void fire() {
      Object w = this.arsenal.currentWeapon();
      if(w instanceof Weapon) {
         ((Weapon) w).fire();
      } else if(w instanceof PortalGun) {
         ((PortalGun) w).fire();
      }
   }

   public final void jump() {
      this.jump(150, 1.2F);
   }

   public final void rotLeft() {
      this.rotY(5);
   }

   public final void rotRight() {
      this.rotY(-5);
   }

   public final void rotX(int angle) {
      this.getCharacter().rotX(angle);
   }

   public final void moveForward() {
      this.moveZ(-150);
      shakeCurrentWeapon();
   }

   public final void moveBackward() {
      this.moveZ(150);
      shakeCurrentWeapon();
   }

   public final void moveLeft() {
      this.getCharacter().moveX(-150);
      shakeCurrentWeapon();
   }

   public final void moveRight() {
      this.getCharacter().moveX(150);
      shakeCurrentWeapon();
   }

   private void shakeCurrentWeapon() {
      Object w = this.arsenal.currentWeapon();
      if(w instanceof Weapon) {
         ((Weapon) w).enableShake();
      } else if(w instanceof PortalGun) {
         ((PortalGun) w).enableShake();
      }
   }

   public final Object getHUDInfo() {
      Object[] var1 = this.arsenal.getWeapons();
      int[] var2 = new int[var1.length];

      for(int var3 = 0; var3 < var1.length; ++var3) {
         if(var1[var3] instanceof Weapon) {
            Weapon w = (Weapon) var1[var3];
            var2[var3] = w.getAmmo() + w.getRounds();
         } else if(var1[var3] instanceof PortalGun) {
            // The Portal Gun has unlimited shots.
            var2[var3] = 9999;
         } else {
            var2[var3] = -1;
         }
      }

      return new HUDInfo(this.money, var2);
   }
}
