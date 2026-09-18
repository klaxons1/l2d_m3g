package com;

import java.util.Random;
import java.util.Vector;
import javax.microedition.lcdui.Graphics;

public class Scene {

	private final Random random;
	private Vector respawns;
	private Bot[] bots;
	private long frameQ;
	private int gravityQ;
	private int lagQ;
	private long cleanAt;
	private long placeAt;
	private static final int CLEAN_MS = 5 * FPS.FRAME_MS;
	private static final int GRAVITY = 20;
	private Renderer g3d;
	private House house;
	private int miny;
	private Respawn start;
	private Respawn finish;
	private Respawn[] enemies; // респауны ботов
	private final int frequency;
	private final int max_enemy_count; // Максимально возможное кол-во ботов на сцене
	private int enemy_count; // Кол-во добавленных ботов
	private int part;

	Scene(int width, int height, House house, Respawn start, Respawn finish, Respawn[] enemies, int max_enemy_count, int frequency) {
		this.random = new Random();
		this.respawns = new Vector();
		this.frameQ = 0;
		this.cleanAt = FPS.ms;
		this.placeAt = FPS.ms;
		this.g3d = new Renderer(width, height);
		this.house = house;
		this.start = start;
		this.finish = finish;
		this.enemies = enemies;
		this.max_enemy_count = max_enemy_count;
		this.frequency = frequency;
		this.countWorldSize(house);
		
		this.bots = new Bot[Math.min(max_enemy_count, 5)];
		
		if(bots.length > 0) this.bots[0] = new BigZombie(new Vector3D());

		for(width = 1; width < this.bots.length; ++width) {
			this.bots[width] = new Zombie(new Vector3D());
		}

	}

	public final void reset() {
		Vector var1 = this.house.getObjects();
		int var2 = 0;

		while(var2 < var1.size()) {
			if(var1.elementAt(var2) instanceof Bot) {
				var1.removeElementAt(var2);
			} else {
				++var2;
			}
		}

		this.frameQ = 0;
		this.enemy_count = 0;
		this.part = -1;
	}

	public final void destroy() {
		this.g3d.destroy();
		this.g3d = null;
		this.house.destroy();
		this.house = null;

		for(int var1 = 0; var1 < this.bots.length; ++var1) {
			this.bots[var1].destroy();
			this.bots[var1] = null;
		}

		this.bots = null;
	}

	private void countWorldSize(House house) {
		int minX = Integer.MAX_VALUE;
		int maxX = Integer.MIN_VALUE;
		int minY = Integer.MAX_VALUE;
		int maxY = Integer.MIN_VALUE;
		int minZ = Integer.MAX_VALUE;
		int maxZ = Integer.MIN_VALUE;
		
		Room[] rooms = house.getRooms();

		for(int i = 0; i < rooms.length; ++i) {
			Room room = rooms[i];
			
			minX = Math.min(minX, room.getMinX());
			minY = Math.min(minY, room.getMinY());
			minZ = Math.min(minZ, room.getMinZ());
			maxX = Math.max(maxX, room.getMaxX());
			maxY = Math.max(maxY, room.getMaxY());
			maxZ = Math.max(maxZ, room.getMaxZ());
		}

		System.out.println("World size:");
		System.out.println("Size X " + (maxX - minX));
		System.out.println("Size Y " + (maxY - minY));
		System.out.println("Size Z " + (maxZ - minZ));
		this.miny = minY;
	}

	private int random(int x) {
		return Math.abs(this.random.nextInt()) % x;
	}

	public final Vector3D getStartPoint() {
		return this.start.point;
	}

	// добавляет в Vector respawns респауны ботов из комнаты (?) под номером part 
	private void addRespawn(int part, Vector respawns) {
		for(int var3 = 0; var3 < this.enemies.length; ++var3) {
			Respawn var4;
			if((var4 = this.enemies[var3]).part == part) {
				respawns.addElement(var4);
			}
		}

	}

	public final int render(Graphics g, int x, int y, int part, Vector3D camPos) {
		prepare(g, x, y);
		return renderHouse(part, camPos);
	}

	/** Binds the frame and clears the buffers (no world rendering). */
	public final void prepare(Graphics g, int x, int y) {
		g3d.prepareRender(g, x, y);
	}

	/** Renders the world around the player into the already bound frame. */
	public final int renderHouse(int part, Vector3D camPos) {
		return this.house.render(this.g3d, part, camPos);
	}

	public final void flush(Graphics g, int x, int y) {
		this.g3d.flush(g);
	}

	public final void update(Player player) {
		int var2 = player.getPart();
		Vector var3 = this.house.getObjects();
		int var5 = var2;
		Scene var4 = this;
		Vector var6 = this.house.getObjects();
		int var7;
		GameObject var8;
		int var10;

		// Уборка ботов из дальних комнат и провалившихся ботов 
		if(FPS.ms >= this.cleanAt) {
			this.cleanAt = FPS.ms + CLEAN_MS;
			for(var7 = 0; var7 < var6.size(); ++var7) {
				var8 = (GameObject) var6.elementAt(var7);
				if(!(var8 instanceof Bot)) continue;
				
				House var10000 = var4.house;
				int var10001 = var8.getPart();
				int var11 = var5;
				var10 = var10001;
				House var9 = var10000;

				// Проверка, в той же комнате или в соседней находится игрок
				boolean var21;
				if(var10 != -1 && var5 != -1) {
					if(var10 == var5) {
						var21 = true;
					} else {
						Room[] var12 = var9.getNeighbourRooms(var10);
						int var13 = 0;

						while(true) {
							if(var13 >= var12.length) {
								var21 = false;
								break;
							}

							if(var12[var13].getId() == var11) {
								var21 = true;
								break;
							}

							++var13;
						}
					}
				} else {
					var21 = false;
				}

				// Если игрок и бот в разных комнатах, или бот провалился, то убрать его
				if(!var21 || var8.getPart() == -1 || var8.getCharacter().getPosition().y < var4.miny << 1) {
					if(!var8.isDead()) {
						--var4.enemy_count;
					}

					var4.house.removeObject(var8);
				}
			}
		}

		var7 = 0;

		// Уборка трупов, если подошло убирать
		while(var7 < var6.size()) {
			if((var8 = (GameObject) var6.elementAt(var7)).isTimeToRenew()) {
				var4.house.removeObject(var8);
			} else {
				++var7;
			}
		}

		// Расстановка ботов
		int var24;
		int var26;
		if(this.part != var2 || FPS.ms >= this.placeAt) {
			this.placeAt = FPS.ms + this.frequency * FPS.FRAME_MS;
			int var10002 = player.getPosX();
			int var10003 = player.getPosZ();
			var26 = player.getCharacter().getRadius() * 7;
			var7 = var10003;
			var24 = var10002;
			var5 = var2;
			var4 = this;

			// Если кол-во добавленных ботов < макс. кол-ва ботов, то 
			if(this.enemy_count < this.max_enemy_count) {
				Vector var27 = this.house.getObjects();

				for(var10 = 0; var10 < var4.bots.length && var4.enemy_count < var4.max_enemy_count; ++var10) {
					Bot var29;
					if((!((var29 = var4.bots[var10]) instanceof BigZombie) || var4.random(8) == 0) && !var27.contains(var29)) {
						Scene var18 = var4;
						Respawn var20;
						if(var5 == -1) {
							var20 = null;
						} else {
							var4.respawns.removeAllElements();
							// Расстановка респаунов ботов в той комнате, где нажодится игрок
							var4.addRespawn(var5, var4.respawns);
							Room[] var14 = var4.house.getNeighbourRooms(var5);

							// Расстановка респаунов ботов в соседних от игрока комнатах
							for(int var15 = 0; var15 < var14.length; ++var15) {
								var18.addRespawn(var14[var15].getId(), var18.respawns);
							}

							// Выбор рандомного респауна
							var20 = var18.respawns.isEmpty() ? null : (Respawn) var18.respawns.elementAt(var18.random(var18.respawns.size()));
						}

						// Если респаун не нулевой, 
						// тогда, если крадрат растояния от игрока до респауна > (радиус_игрока*7)^2,
						// то добавить бота в комнату
						Respawn var30 = var20;
						if(var20 != null) {
							Vector3D var32 = var30.point;
							int var31 = var32.z;
							int var19 = var30.point.x;
							var19 = var24 - var19;
							var31 = var7 - var31;
							if(var19 * var19 + var31 * var31 > var26 * var26) {
								var29.set(var32);
								var4.house.addObject((RoomObject) var29);
								++var4.enemy_count;
							}
						}
					}
				}
			}

			this.part = var2;
		}

		Vector var23 = var3;
		var4 = this;

		// recomputePart
		GameObject var25;
		for(var24 = 0; var24 < var23.size(); ++var24) {
			var25 = (GameObject) var23.elementAt(var24);
			var4.house.recomputePart(var25);
		}

		Vector var22 = var3;

		// collisionTest
		for(var24 = 0; var24 < var22.size(); ++var24) {
			if(!(var25 = (GameObject) var22.elementAt(var24)).isDead()) {
				for(var26 = var24 + 1; var26 < var22.size(); ++var26) {
					GameObject var28;
					if(!(var28 = (GameObject) var22.elementAt(var26)).isDead()) {
						// Cube against cube is rigid body physics: Cube.collideCubes
						// resolves every pair once the bodies have stepped, so the
						// capsule push below would only fight it (a capsule cannot
						// stack, tumble or conserve momentum between two boxes).
						if(var25 instanceof Cube && var28 instanceof Cube) continue;
						// On top of a cube the two feet spheres are wrong: the
						// cube carries the character itself.
						if((var25 instanceof Cube) != (var28 instanceof Cube)) {
							Cube cube = (Cube) (var25 instanceof Cube ? var25 : var28);
							GameObject rider = var25 instanceof Cube ? var28 : var25;
							if(rider.getCharacter().getPosition().y >= cube.rideY()) continue;
							Character.collisionTest(rider.getCharacter(),
									cube.getCharacter(), 1);
							continue;
						}
						Character.collisionTest(var25.getCharacter(), var28.getCharacter());
					}
				}
			}
		}

		var23 = var3;
		var4 = this;

		int dt = FPS.dt;
		this.gravityQ += GRAVITY * dt;
		int gravity = this.gravityQ >> 12;
		this.gravityQ &= SolverMath.F - 1;
		// Gravity goes into the speed before the speed goes into the position, so
		// a step climbs half a step less than it falls, and that half step grows
		// with the frame. The tuned arc is the nominal one: give it back.
		this.lagQ += (int) (GRAVITY * (long) dt * (dt - SolverMath.F) / (2 * SolverMath.F));
		int lag = this.lagQ >> 12;
		this.lagQ &= SolverMath.F - 1;
		for(var24 = 0; var24 < var23.size(); ++var24) {
			var25 = (GameObject) var23.elementAt(var24);
			Vector3D var17 = var25.getCharacter().getSpeed();
			var17.y -= gravity;
			var25.getCharacter().getPosition().y += lag;
			var25.update(var4);
		}

		this.frameQ += FPS.dt;
	}

	// Cubes are not level geometry, so the floor snap cannot see them. standOn
	// only raises, so the highest cube wins.
	final void standOnCubes(Character ch) {
		Vector objects = this.house.getObjects();
		for(int i = 0; i < objects.size(); i++) {
			GameObject obj = (GameObject) objects.elementAt(i);
			if(obj instanceof Cube) ((Cube) obj).supportCharacter(ch);
		}
	}

	public final House getHouse() {
		return this.house;
	}

	public final Renderer getG3D() {
		return this.g3d;
	}

	public final int getFrame() {
		return (int) (this.frameQ >> 12);
	}

	public final int getEnemyCount() {
		return this.max_enemy_count;
	}

	public final boolean isLevelCompleted(Player player) {
		Character var3;
		Vector3D var4 = (var3 = player.getCharacter()).getPosition();
		Vector3D var2 = this.finish.point;
		int var10000 = var4.x;
		int var10001 = var4.y;
		int var10002 = var4.z;
		int var10003 = var2.x;
		int var10004 = var2.y;
		int var10005 = var2.z;
		int var8 = var3.getRadius();
		int var7 = var10005;
		int var6 = var10004;
		int var5 = var10003;
		int var11 = var10002;
		int var10 = var10001;
		int var9 = var10000;
		var9 = var5 - var9;
		var10 = var6 - var10;
		var11 = var7 - var11;
		return (Math.abs(var9) <= var8 && Math.abs(var10) <= var8 && Math.abs(var11) <= var8 ? var9 * var9 + var10 * var10 + var11 * var11 <= var8 * var8 : false) && this.isWinner(player);
	}

	public final boolean isWinner(Player player) {
		return true;//player.getFrags() >= this.max_enemy_count;
	}
}
