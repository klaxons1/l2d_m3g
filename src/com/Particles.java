package com;

import java.util.Random;
import javax.microedition.m3g.Appearance;
import javax.microedition.m3g.CompositingMode;
import javax.microedition.m3g.IndexBuffer;
import javax.microedition.m3g.Mesh;
import javax.microedition.m3g.PolygonMode;
import javax.microedition.m3g.Transform;
import javax.microedition.m3g.TriangleStripArray;
import javax.microedition.m3g.VertexArray;
import javax.microedition.m3g.VertexBuffer;

/**
 * Simple particle burst used by the Portal Gun at the hit point.
 *
 * A burst is COUNT particles of a single color. They are drawn as small
 * camera-facing quads (billboards rotated to the camera axes) with additive
 * blending, so no texture is needed and the effect is very cheap.
 */
public final class Particles {

	private static final int COUNT = 12;
	private static final int LIFE = 12 * FPS.FRAME_MS;
	private static final int SIZE = 70;            // half size, world units

	private static final Random rnd = new Random();

	private static Mesh mesh;
	private static VertexBuffer vb;
	private static boolean meshFailed;

	private final int[] px = new int[COUNT];
	private final int[] py = new int[COUNT];
	private final int[] pz = new int[COUNT];
	private final int[] vx = new int[COUNT];
	private final int[] vy = new int[COUNT];
	private final int[] vz = new int[COUNT];
	private final int[] life = new int[COUNT];

	private int color = 0xffffff;
	private int alive;

	private final Transform tmp = new Transform();

	private static void initMesh() {
		if(mesh != null || meshFailed) return;

		try {
			short[] pos = new short[]{
				(short) -SIZE, (short) -SIZE, 0,
				(short) SIZE, (short) -SIZE, 0,
				(short) -SIZE, (short) SIZE, 0,
				(short) SIZE, (short) SIZE, 0
			};

			VertexArray va = new VertexArray(4, 3, 2);
			va.set(0, 4, pos);

			vb = new VertexBuffer();
			vb.setPositions(va, 1.0f, null);
			vb.setDefaultColor(0xffffffff);

			IndexBuffer ib = new TriangleStripArray(new int[]{0, 1, 2, 3}, new int[]{4});

			PolygonMode pm = new PolygonMode();
			pm.setCulling(PolygonMode.CULL_NONE);
			pm.setShading(PolygonMode.SHADE_FLAT);
			pm.setPerspectiveCorrectionEnable(false);

			CompositingMode cm = new CompositingMode();
			cm.setDepthTestEnable(true);
			// Particles do not write depth and are added onto the frame,
			// producing a spark look.
			cm.setDepthWriteEnable(false);
			cm.setBlending(CompositingMode.ALPHA_ADD);

			Appearance ap = new Appearance();
			ap.setPolygonMode(pm);
			ap.setCompositingMode(cm);

			mesh = new Mesh(vb, new IndexBuffer[]{ib}, new Appearance[]{ap});
		} catch(Throwable t) {
			System.out.println("PARTICLES: " + t);
			meshFailed = true;
			mesh = null;
			vb = null;
		}
	}

	/**
	 * Starts a burst at the hit point.
	 *
	 * @param nx,ny,nz surface normal in Q12 (particles fly away from the wall)
	 * @param color    particle color (0xRRGGBB)
	 */
	public final void burst(int x, int y, int z, int nx, int ny, int nz, int color) {
		this.color = color;
		alive = COUNT;

		for(int i = 0; i < COUNT; i++) {
			px[i] = x;
			py[i] = y;
			pz[i] = z;

			// CLDC's Random has no nextInt(bound), use raw bits instead.
			vx[i] = (nx * 40 >> 12) + (rnd.nextInt() & 127) - 63;
			vy[i] = (ny * 40 >> 12) + (rnd.nextInt() & 127) - 63;
			vz[i] = (nz * 40 >> 12) + (rnd.nextInt() & 127) - 63;

			life[i] = LIFE - (rnd.nextInt() & 3) * FPS.FRAME_MS;
		}
	}

	public final boolean isAlive() {
		return alive > 0;
	}

	public final void update() {
		if(alive <= 0) return;

		alive = 0;
		int dt = FPS.dt;
		int keep = SolverMath.powQ(3584, dt);      // the drag keeps 7/8 a frame
		int gravity = SolverMath.mul(6, dt);
		for(int i = 0; i < COUNT; i++) {
			if(life[i] <= 0) continue;

			px[i] += SolverMath.mul(vx[i], dt);
			py[i] += SolverMath.mul(vy[i], dt);
			pz[i] += SolverMath.mul(vz[i], dt);

			vx[i] = SolverMath.mul(vx[i], keep);
			vz[i] = SolverMath.mul(vz[i], keep);
			vy[i] = SolverMath.mul(vy[i], keep) - gravity;

			life[i] -= FPS.dtMs;
			++alive;
		}
	}

	public final void render(Renderer g3d) {
		if(alive <= 0) return;

		initMesh();
		if(mesh == null) return;

		vb.setDefaultColor(0xff000000 | color);

		float yaw = g3d.camRot.y * 360f / (1 << 14);
		float pitch = g3d.camRot.x * 360f / (1 << 14);

		for(int i = 0; i < COUNT; i++) {
			if(life[i] <= 0) continue;

			float s = (float) life[i] / LIFE;

			tmp.setIdentity();
			tmp.postTranslate(px[i], py[i], pz[i]);
			tmp.postRotate(yaw, 0, 1, 0);
			tmp.postRotate(pitch, 1, 0, 0);
			tmp.postScale(s, s, s);

			g3d.addMesh(mesh, tmp);
		}
	}
}
