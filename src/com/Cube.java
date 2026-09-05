package com;

import javax.microedition.m3g.Appearance;
import javax.microedition.m3g.IndexBuffer;
import javax.microedition.m3g.Material;
import javax.microedition.m3g.Mesh;
import javax.microedition.m3g.PolygonMode;
import javax.microedition.m3g.TriangleStripArray;
import javax.microedition.m3g.VertexArray;
import javax.microedition.m3g.VertexBuffer;

/**
 * Физический кубик - "утяжелённый грузовой куб" в духе Portal.
 *
 * Обычный объект комнаты: падает под гравитацией, лежит на полу, его можно
 * толкать (столкновения объектов между собой уже есть в Scene.update) и
 * протаскивать сквозь порталы. Дополнительно его можно взять в руки -
 * тогда физика отключается и кубик едет перед камерой.
 *
 * Меш строится в коде: шесть граней по сетке 3x3, чтобы можно было
 * раскрасить середину грани отдельным цветом без текстуры. Нормали
 * настоящие, поэтому при включённом динамическом свете кубик подсвечивается
 * порталами вместе с комнатой.
 */
public final class Cube extends GameObject {

	/** Полуразмер кубика. */
	public static final int HALF = 500;

	/** На каком расстоянии перед игроком висит взятый кубик. */
	private static final int HOLD_DIST = 1900;
	/** Дальность, с которой кубик можно взять. */
	private static final int GRAB_RANGE = 3400;
	/** Начальная скорость броска. */
	private static final int THROW_SPEED = 320;

	private static final int COLOR_BODY = 0xb4b4be;
	private static final int COLOR_EDGE = 0x64646e;
	private static final int COLOR_MARK = 0xff5fa0;

	private static Mesh mesh;
	private static boolean meshFailed;

	private final Player player;
	private final PortalManager pm;

	private final Vector3D dir = new Vector3D();
	private final Vector3D tmp = new Vector3D();

	private boolean held;

	public Cube(Vector3D spawn, Player player, PortalManager pm) {
		this.player = player;
		this.pm = pm;

		Character ch = this.character;
		ch.reset();
		ch.set(HALF, HALF);
		ch.getPosition().set(spawn.x, spawn.y, spawn.z);

		// кубик неразрушим, но "живым" быть обязан: мёртвые объекты
		// выпадают из проверки столкновений и убираются сборщиком сцены
		this.setHp(1000);
	}

	public final boolean isHeld() {
		return held;
	}

	/** Взять, если кубик рядом и игрок смотрит на него. */
	public final boolean tryGrab() {
		if(held || player == null || player.isDead()) return false;

		Character pc = player.getCharacter();
		Vector3D pp = pc.getPosition();
		Vector3D p = this.character.getPosition();

		int dx = p.x - pp.x;
		int dy = (p.y + HALF) - (pp.y + pc.getHeight());
		int dz = p.z - pp.z;

		long d2 = (long) dx * dx + (long) dy * dy + (long) dz * dz;
		if(d2 > (long) GRAB_RANGE * GRAB_RANGE) return false;

		Vector3D pr = pc.getRotation();
		dir.setFromRotation(pr.x, pr.y);

		long dot = (long) dir.x * dx + (long) dir.y * dy + (long) dir.z * dz;
		if(dot <= 0) return false;

		// косинус угла между взглядом и направлением на кубик
		double len = Math.sqrt((double) d2) * (1 << 14);
		if(len > 0 && dot < len * 0.5) return false;

		held = true;
		return true;
	}

	/** Отпустить кубик, придав ему скорость взгляда. */
	public final void drop() {
		if(!held) return;

		held = false;

		Vector3D speed = this.character.getSpeed();
		if(player == null) {
			speed.set(0, 0, 0);
			return;
		}

		Vector3D pr = player.getCharacter().getRotation();
		dir.setFromRotation(pr.x, pr.y);

		speed.set(
			(dir.x * THROW_SPEED) >> 14,
			((dir.y * THROW_SPEED) >> 14) + 40,
			(dir.z * THROW_SPEED) >> 14
		);
	}

	public final void toggleGrab() {
		if(held) drop();
		else tryGrab();
	}

	public final void update(Scene scene) {
		House house = scene.getHouse();
		Character ch = this.character;
		Vector3D pos = ch.getPosition();
		Vector3D speed = ch.getSpeed();

		if(held) {
			if(player == null || player.isDead()) {
				held = false;
			} else {
				Character pc = player.getCharacter();
				Vector3D pp = pc.getPosition();
				Vector3D pr = pc.getRotation();
				dir.setFromRotation(pr.x, pr.y);

				// точка перед камерой; чуть ниже уровня глаз
				tmp.set(
					pp.x + ((dir.x * HOLD_DIST) >> 14),
					pp.y + pc.getHeight() - 150 + ((dir.y * HOLD_DIST) >> 14),
					pp.z + ((dir.z * HOLD_DIST) >> 14)
				);

				// если упёрли кубик в стену - выталкиваем его наружу
				house.sphereCast(this.getPart(), tmp, HALF);

				pos.set(tmp.x, tmp.y - HALF, tmp.z);
				speed.set(0, 0, 0);
				house.recomputePart(this);
				return;
			}
		}

		int oldX = pos.x;
		int oldY = pos.y;
		int oldZ = pos.z;
		int oldCenterY = oldY + HALF;

		// в проёме портала стены нет
		boolean ghost = false;
		boolean noFloor = false;
		if(pm != null) {
			ghost = pm.isInOpening(oldX, oldCenterY, oldZ, HALF, speed);
			noFloor = pm.isInFloorOpening(oldX, oldCenterY, oldZ, HALF, speed);
		}

		this.updateMovement(scene, !ghost, !noFloor);

		if(pm != null) {
			int crossed = pm.findCrossedPortal(
					oldX, oldCenterY, oldZ,
					pos.x, pos.y + HALF, pos.z);

			if(crossed >= 0) {
				pm.teleport(crossed, ch, HALF);

				int newRoom = pm.getRoomId(pm.getLinkedPortal(crossed));
				if(newRoom >= 0) this.setPart(newRoom);
				house.recomputePart(this);
			}
		}
	}

	public final void render(Renderer g3d) {
		initMesh();
		if(mesh == null) return;

		Vector3D p = this.character.getPosition();
		tmp.set(p.x, p.y + HALF, p.z);

		g3d.addMesh(mesh, tmp, this.character.getRotation());
	}

	/** Кубик неразрушим. */
	public final boolean damage(GameObject obj, int dmg) {
		return false;
	}

	/** И никогда не убирается из комнаты. */
	public final boolean isTimeToRenew() {
		return false;
	}

	private static void initMesh() {
		if(mesh != null || meshFailed) return;

		try {
			final int H = HALF;
			final int T = HALF / 3;
			final int[] grid = {-H, -T, T, H};

			final int faces = 6;
			final int vpf = 16;
			final int count = faces * vpf;

			short[] pos = new short[count * 3];
			byte[] nrm = new byte[count * 3];
			byte[] col = new byte[count * 3];
			int[] idx = new int[faces * 9 * 4];
			int[] lens = new int[faces * 9];

			int vi = 0, ii = 0, si = 0;

			for(int f = 0; f < faces; f++) {
				int axis = f >> 1;
				int sign = (f & 1) == 0 ? 1 : -1;
				int uAxis = (axis + 1) % 3;
				int vAxis = (axis + 2) % 3;

				int base = vi;

				for(int j = 0; j < 4; j++) {
					for(int i = 0; i < 4; i++) {
						int px = 0, py = 0, pz = 0;

						// вдоль нормали - на грань, две другие оси - по сетке
						if(axis == 0) px = sign * H;
						else if(axis == 1) py = sign * H;
						else pz = sign * H;

						int u = grid[i];
						if(uAxis == 0) px = u;
						else if(uAxis == 1) py = u;
						else pz = u;

						int v = grid[j];
						if(vAxis == 0) px = v;
						else if(vAxis == 1) py = v;
						else pz = v;

						pos[vi * 3] = (short) px;
						pos[vi * 3 + 1] = (short) py;
						pos[vi * 3 + 2] = (short) pz;

						nrm[vi * 3 + axis] = (byte) (sign * 127);

						boolean rim = (i == 0 || i == 3 || j == 0 || j == 3);
						boolean centre = (i == 1 || i == 2) && (j == 1 || j == 2);
						int c = rim ? COLOR_EDGE : (centre ? COLOR_MARK : COLOR_BODY);

						col[vi * 3] = (byte) (c >> 16);
						col[vi * 3 + 1] = (byte) (c >> 8);
						col[vi * 3 + 2] = (byte) c;

						vi++;
					}
				}

				for(int j = 0; j < 3; j++) {
					for(int i = 0; i < 3; i++) {
						idx[ii++] = base + j * 4 + i;
						idx[ii++] = base + j * 4 + i + 1;
						idx[ii++] = base + (j + 1) * 4 + i;
						idx[ii++] = base + (j + 1) * 4 + i + 1;
						lens[si++] = 4;
					}
				}
			}

			VertexArray vaPos = new VertexArray(count, 3, 2);
			vaPos.set(0, count, pos);

			VertexArray vaNrm = new VertexArray(count, 3, 1);
			vaNrm.set(0, count, nrm);

			VertexArray vaCol = new VertexArray(count, 3, 1);
			vaCol.set(0, count, col);

			VertexBuffer vb = new VertexBuffer();
			vb.setPositions(vaPos, 1.0f, null);
			vb.setNormals(vaNrm);
			vb.setColors(vaCol);
			vb.setDefaultColor(0xffffffff);

			IndexBuffer ib = new TriangleStripArray(idx, lens);

			PolygonMode pmode = new PolygonMode();
			pmode.setCulling(PolygonMode.CULL_NONE);
			pmode.setShading(PolygonMode.SHADE_SMOOTH);
			pmode.setPerspectiveCorrectionEnable(false);

			Appearance ap = new Appearance();
			ap.setPolygonMode(pmode);

			// материал ставим только когда в сцене есть свет, иначе кубик
			// без источников стал бы чёрным
			if(MeshData.lighting) {
				Material material = new Material();
				material.setColor(Material.AMBIENT, 0xffffffff);
				material.setColor(Material.DIFFUSE, 0xffffffff);
				material.setColor(Material.SPECULAR, 0xff000000);
				material.setShininess(0);
				material.setVertexColorTrackingEnable(true);
				ap.setMaterial(material);
			}

			mesh = new Mesh(vb, new IndexBuffer[]{ib}, new Appearance[]{ap});
		} catch (Throwable t) {
			System.out.println("CUBE: " + t);
			meshFailed = true;
			mesh = null;
		}
	}
}
