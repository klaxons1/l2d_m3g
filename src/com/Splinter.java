 package com;

final class Splinter {

	private static Texture texture = Texture.createTexture("/splinter.png");
	private int time = Integer.MAX_VALUE;
	private Sprite sprite;

	public Splinter() {
		this.sprite = new Sprite(texture);
		this.sprite.setScale(5);
	}

	public final void set(int x, int y, int z) {
		this.sprite.getPosition().set(x, y, z);
		this.time = 0;
	}

	public final void render(Renderer g3d, int sz) {
		this.time += FPS.dtMs;
		int frame = (this.time + FPS.FRAME_MS - 1) / FPS.FRAME_MS;
		this.sprite.setScale(5 * frame);
		this.sprite.setOffset(0, -this.sprite.getHeight() / 2 - frame * 40);
		g3d.addSprite(this.sprite);
	}

	// true, если проигрывается анимация осколка
	public final boolean isShatters() {
		return this.time < 3 * FPS.FRAME_MS;
	}
}
