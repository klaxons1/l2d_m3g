package com;

public final class Player extends GameObject {

   private int money = 0;
   private int frags = 0;
   private Arsenal arsenal;
   private boolean damage = false;


   private PortalManager portalManager;

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
               if(!(var5[var6] instanceof PortalGun)) {
                  var5[var6] = null;
               }
            } else if(var6 < 5) {
               var5[var6] = Arsenal.createWeapon(var6);
               ((Weapon) var5[var6]).setAmmo(var8[var6]);
            }
         }

         Object cw = this.arsenal.currentWeapon();
         if(cw instanceof Weapon) ((Weapon) cw).createSprite(width_g3d, height_g3d);
         else if(cw instanceof PortalGun) ((PortalGun) cw).createSprite(width_g3d, height_g3d);
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

      // В проёме портала стены (или пола) нет: отключаем столкновения, чтобы
      // можно было пройти сквозь портал не спотыкаясь о стену, в которой он висит.
      // Прижатие к полу и торможение при этом сохраняются - иначе игрока дёргает.
      boolean ghost = false;
      boolean noFloor = false;
      if(this.portalManager != null) {
         ghost = this.portalManager.isInOpening(oldX, oldEyeY, oldZ, radius, speed)
               || this.portalManager.isInOpening(oldX, oldFeetY, oldZ, radius / 2, speed);
         noFloor = this.portalManager.isInFloorOpening(oldX, oldFeetY, oldZ, radius, speed)
               || this.portalManager.isInFloorOpening(oldX, oldEyeY, oldZ, radius, speed);
      }

      this.updateMovement(scene, !ghost, !noFloor);

      if(this.portalManager != null) {
         // сначала проверяем пересечение точкой глаз, затем - ногами
         // (портал может лежать на полу или висеть низко на стене)
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
         }
      }

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
      if(w instanceof Weapon) ((Weapon) w).fire();
      else if(w instanceof PortalGun) ((PortalGun) w).fire();
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
      Object w = this.arsenal.currentWeapon();
      if(w instanceof Weapon) ((Weapon) w).enableShake();
      else if(w instanceof PortalGun) ((PortalGun) w).enableShake();
   }

   public final void moveBackward() {
      this.moveZ(150);
      Object w = this.arsenal.currentWeapon();
      if(w instanceof Weapon) ((Weapon) w).enableShake();
      else if(w instanceof PortalGun) ((PortalGun) w).enableShake();
   }

   public final void moveLeft() {
      this.getCharacter().moveX(-150);
      Object w = this.arsenal.currentWeapon();
      if(w instanceof Weapon) ((Weapon) w).enableShake();
      else if(w instanceof PortalGun) ((PortalGun) w).enableShake();
   }

   public final void moveRight() {
      this.getCharacter().moveX(150);
      Object w = this.arsenal.currentWeapon();
      if(w instanceof Weapon) ((Weapon) w).enableShake();
      else if(w instanceof PortalGun) ((PortalGun) w).enableShake();
   }

   public final Object getHUDInfo() {
      Object[] var1 = this.arsenal.getWeapons();
      int[] var2 = new int[var1.length];

      for(int var3 = 0; var3 < var1.length; ++var3) {
         if(var1[var3] instanceof Weapon) {
            Weapon w = (Weapon) var1[var3];
            var2[var3] = w.getAmmo() + w.getRounds();
         } else if(var1[var3] instanceof PortalGun) {
            var2[var3] = 9999; // PortalGun неограничен
         } else {
            var2[var3] = -1;
         }
      }

      return new HUDInfo(this.money, var2);
   }
}
