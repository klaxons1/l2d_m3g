package com;

import home.Main;
import javax.microedition.lcdui.Graphics;

public final class Setting extends Selectable {

   private Main main;
   private Menu menu;


   public Setting(Main main, Menu menu) {
      this.main = main;
      this.menu = menu;
      IniFile var4 = main.getGameText$6783a6a7();
      String[] var3 = new String[3];
      this.set(main.getFont(), var3, (String)null, var4.getString("BACK"));
      this.setItems();
   }

   private void setItems() {
      IniFile var1 = this.main.getGameText$6783a6a7();
      String[] var2;
      (var2 = this.getItems())[0] = this.main.isSound()?var1.getString("SOUND_ON"):var1.getString("SOUND_OFF");
      var2[1] = var1.getString("DISPLAY_SIZE") + " " + this.main.getDisplaySize();

      String recursion = var1.getString("PORTAL_RECURSION");
      if(recursion == null) recursion = "PORTAL IN PORTAL";
      var2[2] = recursion + (this.main.isPortalRecursion() ? " ON" : " OFF");
   }

   protected final void paint(Graphics g) {
      this.menu.drawBackground(g);
      super.paint(g);
   }

   protected final void onRightSoftKey() {
      this.main.saveSettingToStore();
      this.main.setCurrent(this.menu);
   }

   protected final void onLeftSoftKey() {
      this.onKey6();
   }

   private void changeItem(int var1, int direction) {
      if(var1 == 0) {
         this.main.setSound(!this.main.isSound());
      }

      if(var1 == 1) {
         this.main.setDisplaySize(this.main.getDisplaySize() + direction * 5);
      }

      if(var1 == 2) {
         this.main.setPortalRecursion(!this.main.isPortalRecursion());
      }

      this.setItems();
      this.repaint();
   }

   protected final void onKey4() {
      this.changeItem(this.itemIndex(), -1);
   }

   protected final void onKey6() {
      this.changeItem(this.itemIndex(), 1);
   }
}
