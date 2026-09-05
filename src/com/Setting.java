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
      String[] var3 = new String[5];
      this.set(main.getFont(), var3, (String)null, var4.getString("BACK"));
      this.setItems();
   }

   private void setItems() {
      IniFile var1 = this.main.getGameText$6783a6a7();
      String[] var2;
      (var2 = this.getItems())[0] = this.main.isSound()?var1.getString("SOUND_ON"):var1.getString("SOUND_OFF");
      var2[1] = var1.getString("DISPLAY_SIZE") + " " + this.main.getDisplaySize();

      String var3 = var1.getString("PORTAL_RECURSION");
      if(var3 == null) {
         var3 = "PORTAL IN PORTAL";
      }
      var2[2] = var3 + (this.main.isPortalRecursion() ? " ON" : " OFF");

      String var4 = var1.getString("BLOOM");
      if(var4 == null) {
         var4 = "GLOW";
      }
      var2[3] = var4 + (this.main.isBloom() ? " ON" : " OFF");

      String var5 = var1.getString("PORTAL_RES");
      if(var5 == null) {
         var5 = "PORTAL RES";
      }
      var2[4] = var5 + " " + this.main.getPortalTexSizeName();
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

   protected final void onKey4() {
      int var1;
      if((var1 = this.itemIndex()) == 0) {
         this.main.setSound(!this.main.isSound());
      }

      if(var1 == 1) {
         this.main.setDisplaySize(this.main.getDisplaySize() - 5);
      }

      if(var1 == 2) {
         this.main.setPortalRecursion(!this.main.isPortalRecursion());
      }

      if(var1 == 3) {
         this.main.setBloom(!this.main.isBloom());
      }

      if(var1 == 4) {
         this.main.cyclePortalTexSize(-1);
      }

      this.setItems();
      this.repaint();
   }

   protected final void onKey6() {
      int var1;
      if((var1 = this.itemIndex()) == 0) {
         this.main.setSound(!this.main.isSound());
      }

      if(var1 == 1) {
         this.main.setDisplaySize(this.main.getDisplaySize() + 5);
      }

      if(var1 == 2) {
         this.main.setPortalRecursion(!this.main.isPortalRecursion());
      }

      if(var1 == 3) {
         this.main.setBloom(!this.main.isBloom());
      }

      if(var1 == 4) {
         this.main.cyclePortalTexSize(1);
      }

      this.setItems();
      this.repaint();
   }
}
