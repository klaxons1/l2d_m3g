package com;

import java.io.IOException;
import javax.microedition.lcdui.Graphics;
import javax.microedition.lcdui.Image;

/**
 * Portal Gun — стреляет двумя порталами (синий и оранжевый).
 * Попеременное переключение: 1-й выстрел → синий, 2-й → оранжевый.
 */
public final class PortalGun {

	private static final Ray ray = new Ray();
	
	private static final Splinter splinter = new Splinter();
	
	private static final Vector3D dirVector = new Vector3D();
	private static final Vector3D hitPoint = new Vector3D();
	private static final Vector3D hitNormal = new Vector3D();
	
	// Текущий портал для выстрела (0 = синий, 1 = оранжевый)
	private int nextPortalIdx = PortalManager.BLUE;
	
	// Анимация выстрела
	private short frame = -1;
	private static final short SHOT_TIME = 2;
	private static final short DELAY = 4;
	
	// Спрайты оружия
	private Image imgWeapon;
	private Image imgFire;
	
	private float kW, kH;
	private short dx_fire = 1;
	private short dy_fire = 1;
	private short dx_max = 0;
	private short dy_max = 0;
	private short dx = 0;
	private short dy = 0;
	private short widthShift = 2;
	private short heightShift = 5;
	private boolean shake = false;
	
	// Текстуры для спрайтов
	private String fileWeapon;
	private String fileFire;
	
	// Ссылка на PortalManager
	private PortalManager portalManager;
	
	// Для кастомного поведения: строка с именами файлов оружия
	private static final String PORTAL_WEAPON_FILE = "/pistol.png"; // переиспользуем
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
		if (imgs == null) return;
		this.imgWeapon = imgs[0];
		this.imgFire = imgs[1];
		this.dx_fire = (short)((int)((float)this.imgWeapon.getWidth() * this.kW));
		this.dy_fire = (short)((int)((float)this.imgWeapon.getHeight() * this.kH));
		this.dx_max = (short)(this.imgWeapon.getWidth() / 5);
		this.dy_max = (short)(this.imgWeapon.getHeight() / 5);
	}
	
	private Image[] createImages(String file1, String file2, int width, int height) {
		float sw = (float)width / 240.0F;
		float sh = (float)height / 320.0F;
		try {
			return new Image[]{createImage(file1, sw, sh), createImage(file2, sw, sh)};
		} catch (IOException e) {
			e.printStackTrace();
			return null;
		}
	}
	
	private static Image createImage(String file, float scaleW, float scaleH) throws IOException {
		Image img = Image.createImage(file);
		int nw = (int)((float)img.getWidth() * scaleW);
		int nh = (int)((float)img.getHeight() * scaleH);
		return (img.getWidth() == nw && img.getHeight() == nh) ? img : Arsenal.resize(img, nw, nh);
	}
	
	private boolean isFire() {
		return this.frame >= 0;
	}
	
	public final void draw(Graphics g, int x, int y, int width, int height) {
		if (this.imgWeapon == null) return;
		
		if (this.isFire()) {
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
	
	/**
	 * Обновление анимации и логика выстрела.
	 * Вызывается каждый кадр.
	 * 
	 * @param house   дом (для rayCast)
	 * @param player  игрок
	 * @return null (портал-пушка не убивает)
	 */
	public final GameObject update(House house, GameObject player) {
		boolean fired = (this.frame == 0);
		
		if (this.isFire()) {
			++this.frame;
			if (this.frame > SHOT_TIME) {
				this.frame = (short)(-DELAY);
			}
		}
		
		if (this.frame < -1) {
			++this.frame;
		}
		
		// Анимация отдачи
		if (this.isFire()) {
			this.dx = (short)(this.dx + (Math.abs(this.widthShift) << 1));
			this.dy = (short)(this.dy + (Math.abs(this.heightShift) << 1));
		}
		
		if (this.shake) {
			this.dx += this.widthShift;
			this.dy += this.heightShift;
			this.shake = false;
		} else {
			this.dx = (short)(this.dx + -this.dx / 8);
			this.dy = (short)(this.dy + -this.dy / 8);
		}
		
		if (this.dy <= 0) {
			this.dy = 0;
			this.heightShift = (short)(-this.heightShift);
		}
		if (this.dy > this.dy_max) {
			this.dy = this.dy_max;
			this.heightShift = (short)(-this.heightShift);
		}
		if (this.dx <= 0) {
			this.dx = 0;
			this.widthShift = (short)(-this.widthShift);
		}
		if (this.dx >= this.dx_max) {
			this.dx = this.dx_max;
			this.widthShift = (short)(-this.widthShift);
		}
		
		// В момент выстрела (frame == 0) — размещаем портал
		if (fired && house != null && player != null) {
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
			
			if (ray.isCollision()) {
				// ray переиспользуется внутри PortalManager, поэтому копируем данные
				hitPoint.set(ray.getCollisionPoint());
				// RayCast принимает попадание только при dir * normal > 0, то есть
				// хранит нормаль, смотрящую ОТ стрелка (внутрь стены).
				// Порталу нужна нормаль в комнату - разворачиваем.
				hitNormal.set(-ray.normal.x, -ray.normal.y, -ray.normal.z);
				
				// комната, в которой находится точка попадания: берём точку чуть
				// впереди стены, иначе computePart может промахнуться мимо комнаты
				int part = house.computePart(player.getPart(),
						hitPoint.x + ((hitNormal.x * 200) >> 12),
						hitPoint.y + ((hitNormal.y * 200) >> 12),
						hitPoint.z + ((hitNormal.z * 200) >> 12));
				if (part == -1) part = player.getPart();
				
				if (portalManager.tryPlacePortal(nextPortalIdx, hitPoint, hitNormal, part, house, ray)) {
					nextPortalIdx = portalManager.getLinkedPortal(nextPortalIdx);
				}
				
				// Эффект попадания
				splinter.set(hitPoint.x, hitPoint.y, hitPoint.z);
			}
		}
		
		return null;
	}
	
	public final void fire() {
		if (this.frame == -1) {
			this.frame = 0;
		}
	}
	
	public final void addAmmo(int number) {
		// Портал-пушка неограничена
	}
	
	public final void setAmmo(int number) {
		// Портал-пушка неограничена
	}
	
	public final int getNextPortalIdx() {
		return nextPortalIdx;
	}
	
	public static void renderSplinter(Renderer g3d) {
		if (splinter.isShatters()) {
			splinter.render(g3d, 1500);
		}
	}
}
