package com;

import java.io.IOException;
import javax.microedition.lcdui.Graphics;
import javax.microedition.lcdui.Image;

/**
 * Portal Gun: fires the two portals (blue and orange).
 * Shots alternate: the first shot places the blue portal, the next one
 * the orange portal, and so on.
 */
public final class PortalGun {

	private static final Ray ray = new Ray();

	private static final Splinter splinter = new Splinter();

	private static final Particles particles = new Particles();

	private static final Vector3D dirVector = new Vector3D();
	private static final Vector3D hitPoint = new Vector3D();
	private static final Vector3D hitNormal = new Vector3D();

	// Portal to fire next (0 = blue, 1 = orange).
	private int nextPortalIdx = PortalManager.BLUE;

	private short shotMs = -1;
	private static final short SHOT_TIME = 100;
	private static final short DELAY = 200;
	private static final int BOB_KEEP = 3584;

	// Weapon sprites.
	private Image imgWeapon;
	private Image imgFire;

	private float kW, kH;
	private short dx_fire = 1;
	private short dy_fire = 1;
	private short dx_max = 0;
	private short dy_max = 0;
	private short dx = 0;
	private short dy = 0;
	private int bobX, bobY;   // Q12 dx/dy: a step is a fraction of a pixel
	private short widthShift = 2;
	private short heightShift = 5;
	private boolean shake = false;

	// Sprite texture files.
	private String fileWeapon;
	private String fileFire;

	// Portals this gun places portals into.
	private PortalManager portalManager;

	// Reuses the existing pistol/fire sprites for the in-hand view.
	private static final String PORTAL_WEAPON_FILE = "/pistol.png";
	private static final String PORTAL_FIRE_FILE = "/fire.png";

	public PortalGun(PortalManager portalManager) {
		this.portalManager = portalManager;
		this.fileWeapon = PORTAL_WEAPON_FILE;
		this.fileFire = PORTAL_FIRE_FILE;
		this.kW = 1.0f;
		this.kH = 1.0f;
	}

	public final void reset() {
		this.imgWeapon = this.imgFire = null;
	}

	public final void createSprite(int width_g3d, int height_g3d) {
		Image[] imgs = createImages(this.fileWeapon, this.fileFire, width_g3d, height_g3d);
		if(imgs == null) return;
		this.imgWeapon = imgs[0];
		this.imgFire = imgs[1];
		this.dx_fire = (short) ((int) ((float) this.imgWeapon.getWidth() * this.kW));
		this.dy_fire = (short) ((int) ((float) this.imgWeapon.getHeight() * this.kH));
		this.dx_max = (short) (this.imgWeapon.getWidth() / 5);
		this.dy_max = (short) (this.imgWeapon.getHeight() / 5);
	}

	private Image[] createImages(String file1, String file2, int width, int height) {
		float sw = (float) width / 240.0F;
		float sh = (float) height / 320.0F;
		try {
			return new Image[]{createImage(file1, sw, sh), createImage(file2, sw, sh)};
		} catch(IOException e) {
			e.printStackTrace();
			return null;
		}
	}

	private static Image createImage(String file, float scaleW, float scaleH) throws IOException {
		Image img = Image.createImage(file);
		int nw = (int) ((float) img.getWidth() * scaleW);
		int nh = (int) ((float) img.getHeight() * scaleH);
		return (img.getWidth() == nw && img.getHeight() == nh) ? img : Arsenal.resize(img, nw, nh);
	}

	private boolean isFire() {
		return this.shotMs >= 0;
	}

	public final void draw(Graphics g, int x, int y, int width, int height) {
		if(this.imgWeapon == null) return;

		if(this.isFire()) {
			g.drawImage(this.imgFire, width - this.dx_fire + this.dx, height - this.dy_fire + this.dy + y, 3);
		}
		g.drawImage(this.imgWeapon, width + this.dx, height + this.dy + y, 40);
	}

	public final void enableShake() {
		this.shake = true;
	}

	public final boolean isReloading() {
		return false;
	}

	public final int reloadingPercentage() {
		return 0;
	}

	public final int getRounds() {
		return 999;
	}

	public final int getAmmo() {
		return 999;
	}

	public final boolean isTwoHands() {
		return false;
	}

	// Animation and shot logic; always null, the gun never damages characters.
	public final GameObject update(House house, GameObject player) {
		boolean fired = (this.shotMs == 0);

		particles.update();

		if(this.isFire()) {
			this.shotMs = (short) (this.shotMs + FPS.dtMs);
			if(this.shotMs > SHOT_TIME) {
				this.shotMs = (short) (-DELAY);
			}
		}

		if(this.shotMs < -1) {
			this.shotMs = (short) (this.shotMs + FPS.dtMs);
			if(this.shotMs > -1) this.shotMs = -1;
		}

		int dt = FPS.dt;
		if(this.isFire()) {
			this.bobX += (Math.abs(this.widthShift) << 1) * dt;
			this.bobY += (Math.abs(this.heightShift) << 1) * dt;
		}

		if(this.shake) {
			this.bobX += this.widthShift * dt;
			this.bobY += this.heightShift * dt;
			this.shake = false;
		} else {
			int keep = SolverMath.powQ(BOB_KEEP, dt);
			this.bobX = SolverMath.mul(this.bobX, keep);
			this.bobY = SolverMath.mul(this.bobY, keep);
		}

		if(this.bobY <= 0) {
			this.bobY = 0;
			this.heightShift = (short) (-this.heightShift);
		}
		if(this.bobY > this.dy_max << 12) {
			this.bobY = this.dy_max << 12;
			this.heightShift = (short) (-this.heightShift);
		}
		if(this.bobX <= 0) {
			this.bobX = 0;
			this.widthShift = (short) (-this.widthShift);
		}
		if(this.bobX >= this.dx_max << 12) {
			this.bobX = this.dx_max << 12;
			this.widthShift = (short) (-this.widthShift);
		}

		this.dx = (short) (this.bobX >> 12);
		this.dy = (short) (this.bobY >> 12);

		// On the firing frame place the portal.
		if(fired && house != null && player != null) {
			Vector3D playerPos = player.getCharacter().getPosition();
			Vector3D playerRot = player.getCharacter().getRotation();

			dirVector.setFromRotation(playerRot.x, playerRot.y);

			ray.getStart().set(
				playerPos.x,
				playerPos.y + player.getCharacter().getHeight(),
				playerPos.z
			);
			ray.getDir().set(dirVector.x << 1, dirVector.y << 1, dirVector.z << 1);

			ray.reset();
			house.rayCast(player.getPart(), ray);

			if(ray.isCollision()) {
				// The ray is reused inside PortalManager, so copy its data.
				hitPoint.set(ray.getCollisionPoint());
				// RayCast only accepts hits with dir * normal > 0, i.e. it
				// stores the normal pointing INTO the wall, away from the
				// shooter. The portal needs the normal into the room, flip it.
				hitNormal.set(-ray.normal.x, -ray.normal.y, -ray.normal.z);

				// Room owning the hit point: probe a little in front of the
				// wall, otherwise computePart may miss the room.
				int part = house.computePart(player.getPart(),
					hitPoint.x + ((hitNormal.x * 200) >> 12),
					hitPoint.y + ((hitNormal.y * 200) >> 12),
					hitPoint.z + ((hitNormal.z * 200) >> 12));
				if(part == -1) part = player.getPart();

				int shotIdx = nextPortalIdx;
				if(portalManager.tryPlacePortal(nextPortalIdx, hitPoint, hitNormal, part, house, ray)) {
					nextPortalIdx = portalManager.getLinkedPortal(nextPortalIdx);
				}

				// Hit effect: dust splinter plus sparks in the shot color.
				splinter.set(hitPoint.x, hitPoint.y, hitPoint.z);
				particles.burst(hitPoint.x, hitPoint.y, hitPoint.z,
					hitNormal.x, hitNormal.y, hitNormal.z,
					portalManager.getColor(shotIdx));
			}
		}

		return null;
	}

	public final void fire() {
		if(this.shotMs == -1) {
			this.shotMs = 0;
		}
	}

	public final void addAmmo(int number) {
		// The Portal Gun has unlimited shots.
	}

	public final void setAmmo(int number) {
		// The Portal Gun has unlimited shots.
	}

	public final int getNextPortalIdx() {
		return nextPortalIdx;
	}

	public static void renderSplinter(Renderer g3d) {
		if(splinter.isShatters()) {
			splinter.render(g3d, 1500);
		}
		particles.render(g3d);
	}

	/** Whether the fire animation is playing (used by the crosshair). */
	public final boolean isShooting() {
		return this.shotMs >= 0;
	}
}
