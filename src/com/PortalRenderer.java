package com;

import javax.microedition.m3g.Transform;

/**
 * Renders the views through portals with depth buffer bands.
 *
 * The depth buffer window (0 at the camera, 1 at the far plane) is split:
 * the near half belongs to the world around the player, while each visible
 * portal view renders into its own far band. Since the bands never overlap,
 * the depth test resolves occlusion between the world and the portal views
 * without any render-to-texture:
 *   1) prepareFrame  - before the frame is bound, classify each portal and
 *                      project its screen rectangle;
 *   2) renderBanded  - after binding/clearing the frame but before the world,
 *                      draw the rooms seen through the portals with the
 *                      virtual camera into their bands; the elliptical window
 *                      is then written into the world band as an invisible
 *                      depth mask, so the portal wall cannot overwrite the
 *                      view;
 *   3) the world around the player renders into the near band;
 *   4) renderQuads   - the neon outline (and flat fill for portals without
 *                      a pair) is drawn with the shared depth buffer.
 *
 * Recursion: a portal visible inside a portal view gets its own, farther
 * band, one per recursion level. The number of levels comes from
 * PortalManager.
 *
 * The oblique near plane of the virtual camera is replaced by the plane of
 * the destination portal, so geometry in front of that portal never shows.
 *
 * When the player steps into an opening (the window covers the screen and
 * vertices cross the near plane), the view falls back to drawing the
 * destination room straight into the frame (MODE_DIRECT).
 */
public final class PortalRenderer {

	private static final int MODE_NONE = 0;
	private static final int MODE_FLAT = 1;
	private static final int MODE_DIRECT = 2;
	/** The view through the portal is already in the frame; the window owns a depth band. */
	private static final int MODE_BANDED = 3;

	/**
	 * Depth buffer split (window coordinates, 0 at the camera). The near
	 * half is the world around the player, the far half is shared by portal
	 * views. The nearer portal gets the nearer band, so when windows overlap
	 * it wins the depth test regardless of draw order.
	 */
	private static final float MAIN_FAR = 0.5f;

	/** Number of bands handed out this frame and the width of each band. */
	private int bandCount = 1;
	private float bandStep = 0.5f;

	/**
	 * Window mask pull toward the camera. The M3G depth test is LEQUAL, so a
	 * wall lying exactly in the portal plane would pass on equality and
	 * overwrite the view without this bias.
	 */
	private static final float MASK_BIAS = 0.0004f;

	/** NDC depth pull for the portal window itself, see Renderer.setDepthBias. */
	private static final float DEPTH_BIAS = 0.00015f;

	private static final int MAX_LEVELS = 3;

	private final PortalManager pm;

	private final Transform mainCam = new Transform();
	private final Transform[] camStack = new Transform[MAX_LEVELS + 1];
	private final int[][] boxStack = new int[MAX_LEVELS + 1][4];
	private final int[] tmpBox = new int[4];

	private final float[] plane = new float[4];
	private final float[] camMat = new float[16];
	private final float[] forward = new float[3];

	private final int[][] bbox = new int[PortalManager.COUNT][4];
	private final int[] mode = new int[PortalManager.COUNT];

	private int levels = 1;

	public PortalRenderer(PortalManager pm) {
		this.pm = pm;
		for(int i = 0; i < camStack.length; i++) camStack[i] = new Transform();
	}

	/**
	 * First pass: classify the portals, project their screen rectangles and
	 * capture the main camera. Must run BEFORE the frame Graphics is bound
	 * (i.e. before Scene.prepare).
	 */
	public final void prepareFrame(Renderer g3d) {
		if(levels != pm.getLevels()) {
			levels = pm.getLevels();
			if(levels > MAX_LEVELS) levels = MAX_LEVELS;
		}

		mode[0] = MODE_NONE;
		mode[1] = MODE_NONE;

		if(!pm.isActive(0) && !pm.isActive(1)) return;

		boolean linked = pm.isLinked();

		g3d.getCameraTransform(mainCam);
		mainCam.get(camMat);
		// Look direction is minus the third column of the camera matrix.
		forward[0] = -camMat[2];
		forward[1] = -camMat[6];
		forward[2] = -camMat[10];

		for(int i = 0; i < PortalManager.COUNT; i++) {
			// Back-facing or off-screen portals need neither projection nor
			// a view pass.
			if(!pm.isVisible(i, g3d)) continue;

			int state = pm.projectQuad(i, g3d, bbox[i]);
			if(state == PortalManager.NOT_VISIBLE) continue;

			// A portal without a pair is just a colored window.
			if(!linked) {
				mode[i] = MODE_FLAT;
				continue;
			}

			// The whole outline is behind the near plane: no screen
			// rectangle exists. Only while the eye actually crosses the
			// opening may the linked room paint the whole frame; a wide
			// tolerance here used to flip the view full-screen at random.
			if(state == PortalManager.NEAR_CLIPPED) {
				if(pm.isEyeEntering(i, g3d.camPos, forward)) {
					bbox[i][0] = 0;
					bbox[i][1] = 0;
					bbox[i][2] = g3d.width;
					bbox[i][3] = g3d.height;
					mode[i] = MODE_DIRECT;
				} else {
					mode[i] = MODE_FLAT;
				}
				continue;
			}

			mode[i] = MODE_BANDED;
		}
	}

	/**
	 * Second pass: depth-band portal views. Call AFTER binding and clearing
	 * the frame but BEFORE rendering the world around the player.
	 *
	 * For every banded portal:
	 *  1) viewport = its screen rectangle, depth range = its far band;
	 *  2) render the room seen through the portal with the virtual camera;
	 *  3) switch to the world band and write an invisible depth mask shaped
	 *     like the window, so the wall cannot overwrite the drawn view.
	 */
	public final void renderBanded(Renderer g3d, House house) {
		if(mode[0] != MODE_BANDED && mode[1] != MODE_BANDED) return;

		g3d.getCameraTransform(mainCam);

		// The nearer portal gets the nearer band: overlapping windows are
		// resolved by the depth test no matter the draw order.
		int first = 0, second = 1;
		if(distanceSq(1, g3d) < distanceSq(0, g3d)) {
			first = 1;
			second = 0;
		}

		int portals = 0;
		if(mode[first] == MODE_BANDED) portals++;
		if(mode[second] == MODE_BANDED) portals++;

		// bands = portals * levels: one band per recursion level.
		bandCount = portals * levels;
		if(bandCount < 1) bandCount = 1;
		bandStep = (1f - MAIN_FAR) / bandCount;

		int order = 0;
		if(mode[first] == MODE_BANDED) {
			renderBandedPortal(g3d, house, first, order++, portals);
		}
		if(mode[second] == MODE_BANDED) {
			renderBandedPortal(g3d, house, second, order, portals);
		}

		// The world around the player owns the near band.
		g3d.clearClipPlane();
		g3d.setCameraTransform(mainCam);
		g3d.setDepthRange(0f, MAIN_FAR);
		g3d.setClip(0, 0, g3d.width, g3d.height);
	}

	private long distanceSq(int idx, Renderer g3d) {
		if(!pm.isActive(idx)) return Long.MAX_VALUE;

		Vector3D p = pm.getPosition(idx);
		long dx = p.x - g3d.camPos.x;
		long dy = p.y - g3d.camPos.y;
		long dz = p.z - g3d.camPos.z;
		return dx * dx + dy * dy + dz * dz;
	}

	/** Top-level portal: its view (with all recursion) plus the window mask. */
	private void renderBandedPortal(Renderer g3d, House house, int idx, int order, int portals) {
		boolean drawn = renderBandedLevel(g3d, house, idx, 0, mainCam, bbox[idx], order, portals);

		if(!drawn) {
			mode[idx] = MODE_FLAT;
			return;
		}

		// Window mask in the world band: stops the wall from overwriting.
		writeMask(g3d, idx, bbox[idx], mainCam, 0f, MAIN_FAR);
	}

	/**
	 * Draws the view through portal idx into its depth band, recursing into a
	 * portal visible inside that view first.
	 *
	 * @param cam   camera looking at this portal
	 * @param box   screen rectangle of the window (main screen coordinates)
	 * @param order top-level portal order (0 = nearest)
	 * @return true if the view was drawn
	 */
	private boolean renderBandedLevel(Renderer g3d, House house, int idx, int level,
			Transform cam, int[] box, int order, int portals) {
		int dst = pm.getLinkedPortal(idx);
		int room = pm.getRoomId(dst);
		if(room < 0) return false;

		// Virtual camera for this recursion level.
		Transform virtual = camStack[level];
		pm.getVirtualCamera(idx, cam, virtual);
		g3d.setCameraTransform(virtual);

		// --- find the portal visible inside this view ---
		int inner = -1;
		int[] innerBox = boxStack[level + 1];

		if(level + 1 < levels && pm.isLinked()) {
			long innerArea = 0;

			for(int j = 0; j < PortalManager.COUNT; j++) {
				if(!pm.isFrontFacing(j, g3d.camPos)) continue;
				if(pm.projectQuad(j, g3d, tmpBox) != PortalManager.VISIBLE) continue;

				int x1 = tmpBox[0] > box[0] ? tmpBox[0] : box[0];
				int y1 = tmpBox[1] > box[1] ? tmpBox[1] : box[1];
				int x2 = tmpBox[2] < box[2] ? tmpBox[2] : box[2];
				int y2 = tmpBox[3] < box[3] ? tmpBox[3] : box[3];
				if(x2 - x1 < 2 || y2 - y1 < 2) continue;

				long area = (long) (x2 - x1) * (y2 - y1);
				if(area <= innerArea) continue;

				inner = j;
				innerArea = area;
				innerBox[0] = x1;
				innerBox[1] = y1;
				innerBox[2] = x2;
				innerBox[3] = y2;
			}
		}

		// --- the deeper level first, into its farther band ---
		boolean innerDrawn = false;
		if(inner >= 0) {
			innerDrawn = renderBandedLevel(g3d, house, inner, level + 1,
					virtual, innerBox, order, portals);
		}

		float near = MAIN_FAR + bandSlot(order, level, portals) * bandStep;
		float far = near + bandStep;

		// The inner window mask belongs to THIS level's band and must be
		// written before this level's geometry.
		if(innerDrawn) {
			writeMask(g3d, inner, innerBox, virtual, near, far);
		}

		// --- the linked room into its own band ---
		g3d.setDepthRange(near, far);
		g3d.setCameraTransform(virtual);

		pm.getPlane(dst, plane);
		g3d.setClipPlane(plane[0], plane[1], plane[2], plane[3]);

		house.renderPortalView(g3d, room, box[0], box[1], box[2], box[3]);

		g3d.clearClipPlane();
		g3d.setCameraTransform(cam);
		return true;
	}

	/**
	 * Band index: deeper levels get farther bands; within a level the nearer
	 * top-level portal goes first.
	 */
	private int bandSlot(int order, int level, int portals) {
		int slot = order + level * portals;
		if(slot >= bandCount) slot = bandCount - 1;
		return slot;
	}

	/**
	 * Invisible depth write shaped like the window: anything farther than it
	 * can no longer overwrite the view already in the frame.
	 */
	private void writeMask(Renderer g3d, int idx, int[] box, Transform cam, float near, float far) {
		if(pm.getMaskQuad(idx) == null) return;

		g3d.clearClipPlane();
		g3d.setCameraTransform(cam);
		g3d.setDepthRange(near, far);
		g3d.setDepthBias(MASK_BIAS);
		g3d.setClip(box[0], box[1], box[2], box[3]);

		g3d.addMesh(pm.getMaskQuad(idx), pm.getQuadTransform(idx));

		g3d.setDepthBias(0);
	}

	/**
	 * Third pass: portal windows and outlines, drawn together with the rest
	 * of the frame (after the world, before flush).
	 */
	public final void renderQuads(Renderer g3d, House house) {
		if(mode[0] == MODE_NONE && mode[1] == MODE_NONE) return;

		boolean cameraDirty = false;

		for(int i = 0; i < PortalManager.COUNT; i++) {
			if(mode[i] == MODE_DIRECT) {
				renderDirect(g3d, house, i);
				cameraDirty = true;
			}
		}

		if(cameraDirty) {
			g3d.clearClipPlane();
			g3d.setCameraTransform(mainCam);
		}

		boolean any = false;
		for(int i = 0; i < PortalManager.COUNT; i++) {
			if(mode[i] != MODE_FLAT && mode[i] != MODE_BANDED) continue;

			if(!any) {
				// Window and outline lie almost in the wall plane and start
				// fighting it over depth at long range; a pull in NDC works
				// equally well at any distance.
				g3d.setDepthBias(DEPTH_BIAS);
				g3d.setClip(0, 0, g3d.width, g3d.height);
				any = true;
			}

			if(mode[i] == MODE_BANDED) {
				// The view is already in the frame and protected by the
				// mask: draw only the bright outline around the window.
				pm.setWindow(i, PortalManager.HIDDEN_WINDOW);
			} else {
				pm.setWindow(i, -1);
			}
			g3d.addMesh(pm.getQuad(i), pm.getQuadTransform(i));
		}

		if(any) {
			g3d.setDepthBias(0);
			g3d.setClip(0, 0, g3d.width, g3d.height);
		}
	}

	/** Fallback: view through the portal straight into the frame rectangle. */
	private void renderDirect(Renderer g3d, House house, int idx) {
		int dst = pm.getLinkedPortal(idx);
		int room = pm.getRoomId(dst);
		if(room < 0) return;

		int[] b = bbox[idx];

		pm.getVirtualCamera(idx, mainCam, camStack[0]);
		g3d.setCameraTransform(camStack[0]);

		pm.getPlane(dst, plane);
		g3d.setClipPlane(plane[0], plane[1], plane[2], plane[3]);

		g3d.setClip(b[0], b[1], b[2], b[3]);
		g3d.clearDepth();

		house.renderPortalView(g3d, room, b[0], b[1], b[2], b[3]);
	}
}
