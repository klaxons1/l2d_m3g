package com;

import home.Main;
import javax.microedition.lcdui.Canvas;
import javax.microedition.lcdui.Graphics;
import javax.microedition.lcdui.Image;
import javax.microedition.m3g.Transform;

public final class GameScreen extends Canvas {

	private final Main main;
	private final Font font;
	private final int levelNumber; // Номер уровня
	private final Object hudInfo; // Исп-ся для перехода на след. уровень. Хранит кол-во денег и патронов в каждом оружии
	private final int width; // Ширина экрана
	private final int height; // Высота экрана
	private boolean сhanged = true; // true, если произошло изменение
	private Keyboard keys;
	private int key; // Код нажатой клавиши
	private int x; // x точки нажатия на экран
	private int y; // y точки нажатия на экран
	private int dirX; // x вектора, в направлении которого провели пальцем по экрану (dirX=x2-x1) 
	private int dirY; // y вектора, в направлении которого провели пальцем по экрану (dirY=y2-y1)
	private boolean run;
	private boolean paused = false; // true, если нажали на паузу
	private int framesToEnd = 0; // счетчик кадров в течении которых выводятся сообщения "УРОВЕНЬ ЗАВЕРШЕН" и "КОНЕЦ ИГРЫ"
	private int framesToExit = 0; // счетчик кадров в течении которых выводится сообщение "НАЙДИТЕ ВЫХОД"
	private int hp; // здоровье игрока
	private int rounds; // кол-во патронов в магазине
	private int money;
	private int frags = 0; // счетчик фрагов
	private Player player;
	private Scene scene;
	private Image imgSight;
	private Image imgLife;
	private Image imgPatron;
	private Image imgMoney;
	private Image imgSkull;
	private MusicPlayer musicPlayer;
	private long lastFPSCheck;
	private int frames;
	
	private int fps, usedHeap;
	
	// Portal system
	private PortalManager portalManager;

	public GameScreen(Main main, String levelFile, int levelNumber, Object hudInfo) {
		this.main = main;
		this.levelNumber = levelNumber;
		this.font = main.getFont();
		this.hudInfo = hudInfo;
		this.setFullScreenMode(true);
		this.width = this.getWidth();
		this.height = this.getHeight();

		try {
			this.keys = new Keyboard(this);
			this.imgSight = this.createImage("/sight.png");
			this.imgLife = this.createImage("/life.png");
			this.imgPatron = this.createImage("/patron.png");
			this.imgMoney = this.createImage("/money.png");
			this.imgSkull = this.createImage("/skull.png");
			this.portalManager = new PortalManager();
			this.scene = Respawn.createScene(this.width, (int) ((float) this.height / 1.25F * ((float) main.getDisplaySize() / 100.0F)), levelFile);
			if(this.scene.getHouse().getSkybox() != null) {
				//this.scene.getHouse().getSkybox().setAnimation(true);
			}

			this.player = new Player(this.scene.getG3D().getWidth(), this.scene.getG3D().getHeight(), this.scene.getStartPoint(), this.hudInfo, this.portalManager);
			this.scene.getHouse().addObject((RoomObject) this.player);
			if(main.isSound()) {
				this.musicPlayer = new MusicPlayer("/music.mid");
				this.musicPlayer.setLoopCount(-1);
			}

		} catch(Exception var5) {
			var5.printStackTrace();
		}
	}

	private Image createImage(String file) {
		try {
			return Arsenal.resize(Image.createImage(file), (float) this.width / 240.0F, (float) this.height / 320.0F);
		} catch(Exception var2) {
			System.out.println("ERROR create image " + file);
			return null;
		}
	}

	private void destroy() {
		try {
			this.scene.destroy();
			this.scene = null;
			this.player.destroy();
			this.player = null;
			this.imgSight = this.imgLife = this.imgPatron = this.imgMoney = this.imgSkull = null;
			if(this.musicPlayer != null) {
				this.musicPlayer.stop();
				this.musicPlayer.destroy();
				this.musicPlayer = null;
			}

		} catch(Exception var2) {
			var2.printStackTrace();
		}
	}

	private final void drawMessage(Graphics g, String str) {
		Renderer var3 = this.scene.getG3D();
		int var4 = this.height / 2 - var3.getHeight() / 2;
		this.font.drawString(g, str, var3.getWidth() / 2, var3.getHeight() / 2 + this.imgSight.getHeight() + var4, 3);
	}

	public final void draw(Graphics g) {
		Renderer renderer = this.scene.getG3D();
		boolean var3 = this.player.isDead();
		int playerHeight = this.player.getCharacter().getHeight();
		if(var3 && (playerHeight = (int) ((float) playerHeight / Math.max(0.4F * (float) this.player.getFrame(), 1.0F))) < this.player.getCharacter().getRadius()) {
			playerHeight = this.player.getCharacter().getRadius();
		}

		Vector3D var5;
		Vector3D var10000 = var5 = this.player.getCharacter().getPosition();
		scene.getHouse().recomputePart(player);
		int part = this.player.getPart();
		var10000.y += playerHeight;
		
		int var4 = this.height / 2 - renderer.getHeight() / 2;
		
		// === bindTarget + clear (ОДИН раз) ===
		renderer.prepareRender(g, 0, var4);
		renderer.setCamera(var5, player.getCharacter().getRotation());
		
		// === Portal pass (composite camera, depth в main camera space) ===
		renderPortalViews(g, renderer, var4, part);
		
		// === Main scene поверх portal'ов ===
		renderer.setCamera(var5, player.getCharacter().getRotation());
		this.scene.renderOnly(part, var10000);
		
		var5.y -= playerHeight;
		int var6;
		int var7;
		int var8;
		int var9;
		int[] var12;
		/*if(var3) {
		 var12 = var2.getDisplay();

		 for(var9 = 0; var9 < var12.length; ++var9) {
		 var6 = (var8 = var12[var9]) >> 16 & 255;
		 var7 = var8 >> 8 & 255;
		 var8 &= 255;
		 var8 = (var6 + var7 + var8) / 3;
		 var12[var9] = var8 << 16 | var8 << 8 | var8;
		 }
		 }

		 if(this.player.isDamaged()) {
		 var12 = var2.getDisplay();

		 for(var9 = 0; var9 < var12.length; ++var9) {
		 var6 = (var8 = var12[var9]) >> 16 & 255;
		 var7 = var8 >> 8 & 255;
		 var8 &= 255;
		 var12[var9] = (var6 + var7 + var8) / 3 << 16;
		 }
		 }*/

		this.scene.flush(g, 0, var4);
		if(!var3) {
			int oldClipX = g.getClipX();
			int oldClipY = g.getClipY();
			int oldClipW = g.getClipWidth();
			int oldClipH = g.getClipHeight();
			g.setClip(0, var4, renderer.getWidth(), renderer.getHeight());
			this.player.getArsenal().drawWeapon(g, var4, renderer.getWidth(), renderer.getHeight());
			g.setClip(oldClipX, oldClipY, oldClipW, oldClipH);
		}

		g.drawImage(this.imgSight, renderer.getWidth() / 2, var4 + renderer.getHeight() / 2, 3);
		if(var3) {
			this.drawMessage(g, this.main.getGameText$6783a6a7().getString("GAME_OVER"));
		} else if(this.framesToEnd > 0) {
			if(!this.main.isLastLevel(this.levelNumber)) {
				this.drawMessage(g, this.main.getGameText$6783a6a7().getString("LEVEL_COMPLETE"));
			} else {
				this.drawMessage(g, this.main.getGameText$6783a6a7().getString("GAME_COMPLETE"));
			}
		} else if(this.framesToExit > 0 && this.framesToExit < 45) {
			this.drawMessage(g, this.main.getGameText$6783a6a7().getString("FIND_EXIT"));
		} else if(this.player.getHp() <= 15) {
			if(this.scene.getFrame() / 8 % 2 == 0) {
				this.drawMessage(g, this.main.getGameText$6783a6a7().getString("BUY_MEDICINE_CHEST"));
			}
		} else if(!this.player.getArsenal().isPortalGunSelected() && this.player.getArsenal().currentWeapon() instanceof Weapon && ((Weapon) this.player.getArsenal().currentWeapon()).getAmmo() <= 20 && this.scene.getFrame() / 8 % 2 == 0) {
			this.drawMessage(g, this.main.getGameText$6783a6a7().getString("BUY_PATRONS"));
		}

		int var10;
		if(this.сhanged) {
			g.setColor(0);
			g.fillRect(0, 0, this.width, var4);
			g.fillRect(0, var4 + renderer.getHeight(), this.width, this.height - (var4 + renderer.getHeight()));
			var10 = var4 / 2;
			g.drawImage(this.imgMoney, 4, var10, 6);
			this.font.drawString(g, " " + this.player.getMoney() + " " + fps + " " + usedHeap, 4 + this.imgMoney.getWidth(), var10, 6);
			g.drawImage(this.imgSkull, this.width - 4, var10, 10);
			this.font.drawString(g, this.player.getFrags() + "/" + this.scene.getEnemyCount(), this.width - 4 - this.imgSkull.getWidth(), var10, 10);
			var10 = this.height - var4 / 2;
			g.drawImage(this.imgLife, 4, var10, 6);
			this.font.drawString(g, " " + this.player.getHp(), this.imgLife.getWidth(), var10, 6);
			g.drawImage(this.imgPatron, this.width - 4, var10, 10);
			Object curWeapon = this.player.getArsenal().currentWeapon();
			if(curWeapon instanceof Weapon) {
				Weapon var13 = (Weapon) curWeapon;
				this.font.drawString(g, var13.getRounds() + "/" + var13.getAmmo() + " ", this.width - this.imgPatron.getWidth(), var10, 10);
			} else if(curWeapon instanceof PortalGun) {
				PortalGun pg = (PortalGun) curWeapon;
				String portalLabel = pg.getNextPortalIdx() == 0 ? "BLUE" : "ORANGE";
				int portalColor = pg.getNextPortalIdx() == 0 ? 0x3366FF : 0xFF6600;
				this.font.drawString(g, portalLabel + " ", this.width - this.imgPatron.getWidth(), var10, 10);
			}
			this.сhanged = false;
		}

		if(this.paused) {
			g.setColor(0);
			g.fillRect(0, 0, this.width, var4);
			g.fillRect(0, var4 + renderer.getHeight(), this.width, this.height - (var4 + renderer.getHeight()));

			for(var10 = 0; var10 < this.height; var10 += 2) {
				g.drawLine(0, var10, this.width, var10);
			}

			IniFile var11 = this.main.getGameText$6783a6a7();
			this.font.drawString(g, var11.getString("PAUSE"), this.width / 2, this.height / 2, 3);
			this.font.drawString(g, var11.getString("MENU"), this.width - 4, this.height - 4, 40);
			this.font.drawString(g, var11.getString("CONTINUE"), 4, this.height - 4, 36);
		}

	}

	/**
	 * Рендерит вид через порталы (ПЕРЕД основной сценой).
	 *
	 * Подход:
	 * Composite camera = V_main * T_portal.
	 * Depth values совпадают с main camera space → main scene корректно
	 * перезатирает portal content своей геометрией.
	 */
	private void renderPortalViews(Graphics g, Renderer renderer, int viewportY, int playerPart) {
		PortalManager pm = this.portalManager;
		if (pm == null) return;
		
		if (!pm.isActive(0) || !pm.isActive(1)) {
			if (pm.isActive(0)) drawPortalFrame(g, renderer, viewportY, 0);
			if (pm.isActive(1)) drawPortalFrame(g, renderer, viewportY, 1);
			return;
		}
		
		float[] projBackup = new float[16];
		renderer.saveProjection(projBackup);
		Vector3D savedCamPos = new Vector3D(renderer.camPos);
		Vector3D savedCamRot = new Vector3D(renderer.camRot);
		
		for (int portalIdx = 0; portalIdx < 2; portalIdx++) {
			int linkedIdx = pm.getLinkedPortal(portalIdx);
			
			int[] bounds = new int[4];
			if (!pm.getPortalScreenBounds(portalIdx, renderer, bounds)) continue;
			
			int clipX1 = bounds[0];
			int clipY1 = bounds[1];
			int clipX2 = bounds[2];
			int clipY2 = bounds[3];
			
			int margin = 8;
			clipX1 = Math.max(0, clipX1 - margin);
			clipY1 = Math.max(0, clipY1 - margin);
			clipX2 = Math.min(renderer.width, clipX2 + margin);
			clipY2 = Math.min(renderer.height, clipY2 + margin);
			
			if (clipX2 <= clipX1 || clipY2 <= clipY1) continue;
			
			// Composite camera matrix (V_main * T_portal) — depth в main camera space
			Transform portalCam = new Transform();
			pm.getPortalCameraTransform(portalIdx, linkedIdx, portalCam);
			
			// Ставим камеру через матрицу напрямую (вызывает g3d.setCamera!)
			renderer.setPortalCamera(portalCam);
			renderer.setClip(clipX1, clipY1, clipX2, clipY2);
			
			// Рендерим сцену из destination portal'а
			int dstRoomId = pm.getRoomId(linkedIdx);
			if (dstRoomId >= 0) {
				this.scene.getHouse().renderPortalView(
					renderer, dstRoomId,
					clipX1, clipY1, clipX2, clipY2
				);
			}
			
			drawPortalFrame(g, renderer, viewportY, portalIdx);
		}
		
		renderer.setCamera(savedCamPos, savedCamRot);
		renderer.restoreProjection(projBackup);
		renderer.setClip(0, 0, renderer.width, renderer.height);
	}
	
	/**
	 * Извлекает позицию и углы Эйлера из матрицы камеры.
	 * Матрица задаёт world-to-camera трансформацию.
	 * 
	 * Извлекаем:
	 *   camPos = -R^T * T (позиция камеры в world space)
	 *   camRot = Эйлеровы углы из R (YXZ порядок, как в Renderer.setCamera)
	 */
	/** Полный atan для любого x, через MathUtils.atan (работает для [-1,1]) */
	private static float fullAtan(float x) {
		float ax = (x < 0.0f) ? -x : x;
		if (ax <= 1.0f) {
			return MathUtils.atan(x);
		}
		float sign = (x > 0.0f) ? 1.0f : -1.0f;
		return sign * (float)(Math.PI / 2.0) - MathUtils.atan(1.0f / x);
	}
	
	/**
	 * Рисует цветную рамку портала через 2D Graphics.
	 */
	private void drawPortalFrame(Graphics g, Renderer renderer, int viewportY, int portalIdx) {
		PortalManager pm = this.portalManager;
		int[] bounds = new int[4];
		if (!pm.getPortalScreenBounds(portalIdx, renderer, bounds)) return;
		
		int x1 = bounds[0];
		int y1 = bounds[1] + viewportY;
		int x2 = bounds[2];
		int y2 = bounds[3] + viewportY;
		
		int color = (portalIdx == 0) ? 0x3366FF : 0xFF6600; // Синий / Оранжевый
		
		int oldColor = g.getColor();
		g.setColor(color);
		
		// Рисуем толстую рамку (3 пикселя)
		for (int i = 0; i < 3; i++) {
			g.drawRect(x1 + i, y1 + i, (x2 - x1) - 2 * i - 1, (y2 - y1) - 2 * i - 1);
		}
		
		g.setColor(oldColor);
	}
	
	protected final void pointerPressed(int x, int y) {
		this.x = x;
		this.y = y;
		if(GUIScreen.isLeftSoft(x, y, this.getWidth(), this.getHeight())) {
			this.keyPressed(this.keys.KEY7);
		}

		if(GUIScreen.isRightSoft(x, y, this.getWidth(), this.getHeight())) {
			this.keyPressed(this.keys.KEY9);
		}

	}

	protected final void pointerDragged(int x, int y) {
		this.dirX = x - this.x;
		this.dirY = y - this.y;
		this.x = x;
		this.y = y;
	}

	protected final void pointerReleased(int x, int y) {
		this.x = x;
		this.y = y;
		this.dirX = this.dirY = 0;
	}

	protected final void keyPressed(int key) {
		this.key = key;
		this.keys.keyPressed(key);
		if(this.paused) {
			if(this.key == this.keys.KEY7) {
				this.key = 0;
				this.paused = false;
				this.start();
				repaint();
			} else if(this.key == this.keys.KEY9) {
				this.destroy();
				Menu var3 = new Menu(this.main);
				this.main.setCurrent(var3);
			}
		} else if(this.key == this.keys.KEY9) {
			this.paused = true;
			this.stop();
			this.repaint();
		} else if((this.key == 49 || this.key == this.keys.KEY7) && !this.player.isDead()) {
			this.stop();
			this.main.setCurrent(new Shop(this.main, this, this.player));
		}

		if(key == -26) {
			Vector3D var4 = this.player.getCharacter().getPosition();
			System.out.println(var4.x + ", " + var4.y + ", " + var4.z + ";");
		}

	}

	protected final void keyReleased(int key) {
		this.key = 0;
		this.keys.keyReleased(key);
	}

	public final void paint(Graphics g) {
		long frameStart = System.currentTimeMillis();
		if(!paused) {
			if(!this.player.isDead()) {
				if(this.keys.keyUp()) this.player.moveForward();
				if(this.keys.keyDown()) this.player.moveBackward();

				if(this.keys.keyLeft()) this.player.rotLeft();
				if(this.keys.keyRight()) this.player.rotRight();

				if(this.keys.key7()) this.player.moveLeft();
				if(this.keys.key9()) this.player.moveRight();

				if(this.keys.keyCentre()) this.player.fire();

				if(this.key == 42) this.player.rotX(-3);

				if(this.key == 35) this.player.rotX(3);

				if(this.key == 48) this.player.jump();

				if(this.key == 51) {
					this.key = 0;
					this.player.getArsenal().nextWeapon(this.scene.getG3D().getWidth(), this.scene.getG3D().getHeight());
				}

				if(this.dirX * this.dirX > this.dirY * this.dirY) {
					if(this.dirX < 0) this.player.rotLeft();
					if(this.dirX > 0) this.player.rotRight();
				} else {
					if(this.dirY > 0) this.player.rotX(-3);
					if(this.dirY < 0) this.player.rotX(3);
				}
			}

			if(this.player.isTimeToRenew()) {
				this.framesToEnd = this.framesToExit = 0;
				this.scene.reset();
				this.player.set(this.scene.getG3D().getWidth(), this.scene.getG3D().getHeight(), this.scene.getStartPoint(), this.hudInfo);
			}

			this.scene.update(this.player);
			if(this.scene.getFrame() % 2 == 0) {
				if(this.framesToEnd == 0 && this.scene.isLevelCompleted(this.player)) {
					this.framesToEnd = 1;
				}

				if(this.framesToExit == 0 && this.scene.isWinner(this.player)) {
					this.framesToExit = 1;
				}
			}

			if(this.framesToEnd > 0)
				this.framesToEnd++;
			if(this.framesToExit > 0)
				this.framesToExit++;

			if(this.framesToEnd > 45) {
				this.main.addAvailableLevel(this.levelNumber);
				Object var11 = this.player.getHUDInfo();
				this.stop();
				this.destroy();
				Menu var12 = new Menu(this.main);
				LevelSelection var13 = new LevelSelection(this.main, var12, var11);
				this.main.setCurrent(var13);
				return;
			}

			if(!this.сhanged) {
				Object curW = this.player.getArsenal().currentWeapon();
				int curRounds = (curW instanceof Weapon) ? ((Weapon) curW).getRounds() : 0;
				this.сhanged = this.player.getHp() != this.hp || curRounds != this.rounds || this.player.getMoney() != this.money || this.player.getFrags() != this.frags;
				if(this.сhanged) {
					this.hp = this.player.getHp();
					this.rounds = curRounds;
					this.money = this.player.getMoney();
					this.frags = this.player.getFrags();
				}
			}
		}

		сhanged = true; //todo togglable fps
		if(!this.сhanged) {
			Renderer var9 = this.scene.getG3D();
			int var4 = this.height / 2 - var9.getHeight() / 2;
			g.setClip(0, var4, var9.getWidth(), var9.getHeight());
			draw(g);
		} else {
			draw(g);
		}

		long frameEnd = System.currentTimeMillis();
		long sleepTime = 50L - (frameEnd - frameStart);
		if(sleepTime < 3L) sleepTime = 3L;
		//sleepTime = 1;

		frames++;
		if(frameEnd - lastFPSCheck > 1000) {
			lastFPSCheck = frameEnd;
			fps = frames;
			usedHeap = (int) ((Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1024);
			frames = 0;
		}

		/*try {
		 Thread.sleep(sleepTime);
		 } catch(InterruptedException ex) {
		 ex.printStackTrace();
		 }*/
		Thread.yield();
		if(run) {
			repaint();
		}
	}

	public final void start() {
		this.сhanged = true;

		GameScreen var1 = this;
		if(this.musicPlayer != null) {
			try {
				var1.musicPlayer.start();
				//return;
			} catch(Exception var2) {
				var2.printStackTrace();
			}
		}

		if(!this.run) {
			this.run = true;
			//repaint();
		}
	}

	private void stop() {
		if(this.run) {
			this.run = false;
		}

		GameScreen var1 = this;
		if(this.musicPlayer != null) {
			try {
				var1.musicPlayer.stop();
				return;
			} catch(Exception var2) {
				var2.printStackTrace();
			}
		}

	}
}
