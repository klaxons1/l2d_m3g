package com;

final class Blood {

	private static final Texture bloodTex = Texture.createTexture("/blood.png");
	
	private GameObject parent;
	// Milliseconds since the hit, MAX_VALUE when there is nothing to draw.
	private int time = Integer.MAX_VALUE;
	private Sprite sprite;

	public Blood(GameObject obj) {
		parent = obj;

		sprite = new Sprite(bloodTex);
		sprite.setScale(5);
	}

	public final void reset() {
		sprite.setScale(5);
		time = Integer.MAX_VALUE;
	}

	public final void destroy() {
		parent = null;
		sprite.destroy();
		sprite = null;
	}

	public final void bleed() {
		time = 0;
		sprite.mirX = !sprite.mirX;
		sprite.mirY = !sprite.mirY;
	}

	public final void render(Renderer g3d, int sz) {
		Character parentCh = parent.getCharacter();
		Vector3D parentPos = parentCh.getPosition();

		sprite.getPosition().set(parentPos.x, parentPos.y + parentCh.getHeight(), parentPos.z);

		time += Clock.frameMs;   // drawn once per frame, not per step
		// One step per nominal frame, ceil so the first drawn frame is full.
		int frame = (time + Clock.FRAME_MS - 1) / Clock.FRAME_MS;
		sprite.setScale(5 * frame);
		sprite.setOffset(0, -sprite.getHeight() / 2 - frame * 40);
		g3d.addSprite(sprite);
	}

	public final boolean isBleeding() {
		return time < 7 * Clock.FRAME_MS;
	}
}
