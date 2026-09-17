package com;

// Магазин (для патронов)
final class Magazine {

   private final short capacity; // Максимальное кол-во патронов в магазине (вместимость)
   private short ammo; // Боезапас
   private short rounds; // Текущее кол-во патронов
   private final short reloadTime; // Продолжительность перезарядки, мс
   private short reloadMs = -1;


   public Magazine(int capacity, int reloadTime) {
      this.capacity = (short)capacity;
      this.reloadTime = (short)(reloadTime * FPS.FRAME_MS);
   }

   public final void setAmmo(int ammo) {
      this.ammo = (short)ammo;
   }

   public final void addAmmo(int number) {
      this.ammo = (short)(this.ammo + number);
   }

   // ? Если есть патроны, начать перезарядку
   final void reload() {
      if(this.ammo != 0) {
         if(this.reloadMs == -1) {
            this.reloadMs = 0;
         }

      }
   }

   // ? Пересчет кол-ва пройденных циклов перезарядки
   final void update() {
      if(this.reloadMs >= 0) {
         this.reloadMs = (short)(this.reloadMs + FPS.dtMs);
         if(this.reloadMs > this.reloadTime) {
            this.reloadMs = -1;
            this.recount();
         }
      }

   }

   // ? Пересчет кол-ва патронов
   final void recount() {
      this.rounds = (short)Math.min(this.capacity, this.ammo);
      this.ammo -= this.rounds;
   }

   // ? true, если нужно перезаряжаться (продолжать перезарядку)
   final boolean isReloading() {
      return this.reloadMs != -1;
   }

   // ? Процент перезарядки
   final int percentage() {
      return this.reloadMs < 0 ? 0 : 100 * this.reloadMs / this.reloadTime;
   }

   final int getRounds() {
      return this.rounds;
   }

   final int getAmmo() {
      return this.ammo;
   }

   // Пересчет кол-ва патронов в магазине
   final void takeRounds(int number) {
      this.rounds = (short)(this.rounds - number);
   }
}
