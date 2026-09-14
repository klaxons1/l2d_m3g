package com;

import javax.microedition.lcdui.Graphics;
import javax.microedition.m3g.Background;
import javax.microedition.m3g.Camera;
import javax.microedition.m3g.Graphics3D;
import javax.microedition.m3g.Node;
import javax.microedition.m3g.Transform;

public final class Renderer {
	
	private final Graphics3D g3d = Graphics3D.getInstance();
	private final Background bck = new Background();
	/** Depth-only clear, used when rendering a portal view over the frame. */
	private final Background depthClearBck = new Background();
	
	private final int g3dClearFlags;
	
	public final Vector3D camPos = new Vector3D();
	public final Vector3D camRot = new Vector3D();
	private final Camera cam = new Camera();
	private final Transform camPers = new Transform();
	private final float[] camPersTmp = new float[16], camPersTmp2 = new float[16];
	private final Transform camTrans = new Transform();
	private final Transform invCam = new Transform();
	
	private final Transform tmpTrans = new Transform();
	private final Transform tmpTrans2 = new Transform();
	private final float[] tmpMat = new float[16];
	
	private int renderX, renderY;
	public int width, height;
	public float viewportPhysW, viewportPhysH;
	public float projXscale, projYscale;
	public float nearPlane;
	
	// --- oblique near plane clipping (for portal views) ---
	private final float[] clipPlane = new float[4];
	private boolean clipPlaneEnabled;
	
	/**
	 * NDC depth pull for subsequent setClip calls: z_ndc' = z_ndc - bias,
	 * i.e. geometry is moved toward the camera equally at any distance.
	 * Unlike CompositingMode.setDepthOffset this always works, even on M3G
	 * implementations that ignore that call.
	 */
	private float depthBias;
	
	//public float lightX = 475, lightY = 1500, lightZ = 7000;

	public Renderer(int width, int height) {
		this.width = width;
		this.height = height;
		
		float fovy = 73.5f;
		nearPlane = 10;
		
		setPerspective(camPersTmp, fovy, (float) width / height, nearPlane, 300000);
		System.arraycopy(camPersTmp, 0, camPersTmp2, 0, 16);
		camPers.set(camPersTmp);
		cam.setGeneric(camPers);
		
		bck.setColorClearEnable(false);
		
		depthClearBck.setColorClearEnable(false);
		depthClearBck.setDepthClearEnable(true);
		
		viewportPhysH = (float)(Math.tan(Math.toRadians(fovy / 2.0f)) * nearPlane) * 2f;
		viewportPhysW = viewportPhysH * width / height;
		
		projXscale = width / viewportPhysW;
		projYscale = height / viewportPhysH;
		
		g3dClearFlags = "1.0".equals(System.getProperty("microedition.m3g.version")) ? 0 : Graphics3D.OVERWRITE;
		
		//Hashtable params = g3d.getProperties();
		//System.out.println("maxLights: " + params.get("maxLights"));
	}

	public final void destroy() {
		//??? useless
	}

	public final int getWidth() {
		return this.width;
	}

	public final int getHeight() {
		return this.height;
	}
	
	private void setPerspective(float[] mat, float fovy, float aspect, float near, float far) {
		float tmp1 = (float) Math.tan(Math.toRadians(fovy / 2.0f));
		float tmp2 = far - near;

		mat[0] = 1.0f / (aspect * tmp1);
		mat[5] = 1.0f / tmp1;
		mat[10] = -(near + far) / tmp2;
		mat[11] = -2.0f * near * far / tmp2;
		mat[14] = -1.0f;
	}

	public final void setCamera(Vector3D pos, Vector3D rot) {
		camPos.set(pos);
		camRot.set(rot);
		
		camTrans.setIdentity();
		camTrans.postTranslate(pos.x, pos.y, pos.z);
		camTrans.postRotate(rot.y * 360f / (1 << 14), 0, 1, 0);
		camTrans.postRotate(rot.x * 360f / (1 << 14), 1, 0, 0);
		camTrans.postRotate(rot.z * 360f / (1 << 14), 0, 0, 1);
		
		//cam.getCompositeTransform(camTrans);
		//cam.getCompositeTransform(invCam);
		invCam.set(camTrans);
		invCam.invert();
	}
	
	/**
	 * Sets the camera from a ready camera-to-world matrix (used for the
	 * virtual camera looking through a portal). camPos is derived from the
	 * matrix as well, since room rendering relies on it.
	 */
	public final void setCameraTransform(Transform camToWorld) {
		camTrans.set(camToWorld);
		camTrans.get(tmpMat);
		camPos.set((int) tmpMat[3], (int) tmpMat[7], (int) tmpMat[11]);
		
		invCam.set(camTrans);
		invCam.invert();
	}
	
	/** Copies the current camera matrix (camera-to-world) into out. */
	public final void getCameraTransform(Transform out) {
		out.set(camTrans);
	}
	
	public final Transform getInvCam() {
		return invCam;
	}
	
	/**
	 * Sets a clipping plane in WORLD coordinates (a*x + b*y + c*z + d > 0 is
	 * the visible side). It replaces the near plane of the view frustum
	 * (oblique near plane clipping), so no geometry in front of the
	 * destination portal appears in the portal view. The plane is
	 * transformed into camera space, therefore call this AFTER
	 * setCameraTransform.
	 */
	public final void setClipPlane(float a, float b, float c, float d) {
		clipPlane[0] = a;
		clipPlane[1] = b;
		clipPlane[2] = c;
		clipPlane[3] = d;
		
		// plane_camera = transpose(cameraToWorld) * plane_world
		tmpTrans2.set(camTrans);
		tmpTrans2.transpose();
		tmpTrans2.transform(clipPlane);
		
		clipPlaneEnabled = true;
	}
	
	public final void clearClipPlane() {
		clipPlaneEnabled = false;
	}
	
	/**
	 * Depth pull for the following setClip calls, in NDC units.
	 * 0.0001f is about 3 low bits of a 16-bit depth buffer.
	 */
	public final void setDepthBias(float bias) {
		depthBias = bias;
	}
	
	/**
	 * Replaces the near plane of the projection with clipPlane
	 * (Eric Lengyel's oblique frustum method). The matrix is row-major, as
	 * required by Transform.set().
	 */
	private void applyClipPlane(float[] mat) {
		float a = clipPlane[0], b = clipPlane[1], c = clipPlane[2], d = clipPlane[3];
		
		float len = (float) Math.sqrt(a * a + b * b + c * c);
		if(len < 0.000001f) return;
		
		a /= len;
		b /= len;
		c /= len;
		d /= len;
		
		// d is the distance from the camera (coordinate origin) to the
		// plane. The camera must be behind the plane for clipping to apply.
		if(d > -nearPlane) return;
		
		float qx = ((a < 0 ? -1f : 1f) + mat[2]) / mat[0];
		float qy = ((b < 0 ? -1f : 1f) + mat[6]) / mat[5];
		float qz = -1f;
		float qw = (1f + mat[10]) / mat[11];
		
		float dot = a * qx + b * qy + c * qz + d * qw;
		if(dot > -0.000001f && dot < 0.000001f) return;
		
		float k = 2f / dot;
		
		mat[8] = a * k;
		mat[9] = b * k;
		mat[10] = c * k + 1f;
		mat[11] = d * k;
	}
	
	public final void setClip(int x1, int y1, int x2, int y2) {
		// Never hand Graphics3D a degenerate viewport: a zero/negative
		// width makes the projection divide by zero (NaN garbage across
		// half the frame) and setViewport throw IllegalArgumentException.
		if(x1 < 0) x1 = 0;
		if(y1 < 0) y1 = 0;
		if(x2 > width) x2 = width;
		if(y2 > height) y2 = height;
		if(x2 - x1 < 2 || y2 - y1 < 2) return;

		try {
			int w = width;
			int h = height;

			float[] mat = camPersTmp2;
			float[] matBck = camPersTmp;
			
			// Always start from the base projection: the caller may request
			// several different sub-frustums per frame (room and portals).
			System.arraycopy(matBck, 0, mat, 0, 16);

			mat[0] = matBck[0] * w / (x2 - x1);
			//mat[2] = (x1 - (w - (x2 - x1)) / 2) * 2 / (x2 - x1);
			mat[2] = (float)(x1 + x2 - w) / (x2 - x1);

			mat[5] = matBck[5] * h / (y2 - y1);
			mat[6] = (float)-(y1 + y2 - h) / (y2 - y1);
			
			if(clipPlaneEnabled) applyClipPlane(mat);
			if(depthBias != 0) mat[10] += depthBias;

			camPers.set(mat);
			cam.setGeneric(camPers);
			g3d.setCamera(cam, camTrans);
			
			g3d.setViewport(x1 + renderX, y1 + renderY, x2 - x1, y2 - y1);
		} catch (Exception e) {
			System.out.println(x1 + " " + y1 + " " + x2 + " " + y2);
			e.printStackTrace();
		}
	}

	public final void addSprite(Sprite obj) {
		Transform mat = tmpTrans;
		mat.setIdentity();
		
		Vector3D pos = obj.getPosition();
		mat.postTranslate(pos.x, pos.y, pos.z);
		mat.postScale(
				(obj.mirX ? -1 : 1) * obj.getWidth(), 
				(obj.mirY ? -1 : 1) * obj.getHeight(), 
				(obj.mirX ? -1 : 1) * obj.getWidth()
		);
		
		g3d.render(obj.s3d, mat);
	}

	public final void addMesh(Node node, Vector3D pos, Vector3D rot) {
		if(node == null) return; //todo WHY
		
		if(pos == null && rot == null) {
			g3d.render(node, null);
			return;
		}
		
		Transform mat = tmpTrans;
		
		mat.setIdentity();
		if(pos != null) mat.postTranslate(pos.x, pos.y, pos.z);
		if(rot != null) {
			mat.postRotate(rot.y * 360f / (1 << 14), 0, 1, 0);
			mat.postRotate(rot.x * 360f / (1 << 14), 1, 0, 0);
			mat.postRotate(rot.z * 360f / (1 << 14), 0, 0, 1);
		}
		
		g3d.render(node, mat);
	}
	
	/** Renders a node with a ready model transform. */
	public final void addMesh(Node node, Transform transform) {
		if(node == null) return;
		g3d.render(node, transform);
	}

	public final void prepareRender(Graphics g, int x, int y) {
		this.renderX = x;
		this.renderY = y;
		g3d.bindTarget(g, true, g3dClearFlags);
		g3d.setViewport(x, y, width, height);
		g3d.clear(bck);
		g3d.setDepthRange(0f, 1f);
		
		/*Light light = new Light();
		light.setMode(Light.OMNI);
		light.setColor(0xffffff);
		light.setAttenuation(0, 0.0001f, 0);
		
		Transform tmpMat = new Transform();
		tmpMat.postTranslate(lightX, lightY, lightZ);
		
		g3d.resetLights();
		g3d.addLight(light, tmpMat);*/
	}

	public final void flush(Graphics g) {
		g3d.releaseTarget();
	}
	
	// =========== Portal rendering support ===========
	
	/**
	 * Restricts the depth buffer window to [near, far] (0..1). NDC z in
	 * [-1,1] is mapped onto [near, far], letting different passes (the world
	 * around the player and views through portals) own disjoint depth bands.
	 */
	public final void setDepthRange(float near, float far) {
		g3d.setDepthRange(near, far);
	}
	
	/** Clears the depth buffer inside the current viewport. */
	public final void clearDepth() {
		g3d.clear(depthClearBck);
	}
}
