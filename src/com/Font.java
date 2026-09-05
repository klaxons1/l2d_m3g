package com;

import javax.microedition.lcdui.Graphics;

/**
 * Шрифт игры. Вся начинка (загрузка полосы глифов с маркерной строкой,
 * UTF-8 конфиг, O(1) поиск глифа для латиницы и кириллицы, стили
 * normal/selected/active) взята из FontRenderer проекта CovertOps3D-RE;
 * здесь остаётся только привычный игре API с якорями (anchor).
 */
public final class Font {

	private final FontRenderer renderer = new FontRenderer();
	private int style = FontRenderer.STYLE_NORMAL;

	public Font(String file) {
		renderer.loadFont(file == null ? FontRenderer.DEFAULT_CONFIG : file);
	}

	/** 0 - обычный, 1 - выделенный, 2 - активный. */
	public final void setStyle(int line) {
		style = line;
	}

	public final void drawString(Graphics g, String str, int x, int y, int anchor) {
		if(str == null) return;

		int renderX = x;
		int renderY = y;

		if((anchor & Graphics.RIGHT) != 0) renderX = x - stringWidth(str);
		if((anchor & Graphics.BOTTOM) != 0) renderY = y - getHeight();
		if((anchor & Graphics.HCENTER) != 0) renderX -= stringWidth(str) >> 1;
		if((anchor & Graphics.VCENTER) != 0) renderY -= getHeight() >> 1;
		if((anchor & Graphics.BASELINE) != 0) renderY -= getHeight() + 1;

		renderer.drawLargeString(str, g, renderX, renderY, style);
	}

	public final int charWidth(char ch) {
		return renderer.getLargeCharWidth(ch);
	}

	public final int getHeight() {
		return renderer.getLargeCharHeight();
	}

	public final int stringWidth(String str) {
		return renderer.getLargeTextWidth(str);
	}

	/** Доступ к движку шрифта, если понадобятся его прочие возможности. */
	public final FontRenderer getRenderer() {
		return renderer;
	}
}
