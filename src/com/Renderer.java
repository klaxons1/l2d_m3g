package com;

import javax.microedition.lcdui.Graphics;
import javax.microedition.m3g.Background;
import javax.microedition.m3g.Camera;
import javax.microedition.m3g.Graphics3D;
import javax.microedition.m3g.Image2D;
import javax.microedition.m3g.Node;
import javax.microedition.m3g.Transform;

public final class Renderer {
	
	private final Graphics3D g3d = Graphics3D.getInstance();
	private final Background bck = new Background();
	private final Background depthClearBck = new Background();
	private final Background texClearBck = new Background();
	
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
	
	private DynamicLights lights;
	
	private int renderX, renderY;
	public int width, height;
	public float viewportPhysW, viewportPhysH;
	public float projXscale, projYscale;
	public float nearPlane;
	
	// --- рендер в текстуру портала ---
	private boolean toTexture;
	private int texW, texH;
	private int frustumX1, frustumY1, frustumX2, frustumY2;
	
	// --- косая ближняя плоскость (oblique near plane clipping) ---
	private final float[] clipPlane = new float[4];
	private boolean clipPlaneEnabled;
	
	/**
	 * Сдвиг по глубине в NDC для следующих вызовов setClip.
	 * Прибавляется к mat[10]: z_ndc' = z_ndc - bias, то есть геометрия
	 * "подтягивается" к камере одинаково на любом расстоянии. В отличие от
	 * CompositingMode.setDepthOffset работает всегда, даже если реализация
	 * M3G этот вызов игнорирует (а KEmulator, похоже, игнорирует).
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
		
		texClearBck.setColorClearEnable(true);
		texClearBck.setDepthClearEnable(true);
		texClearBck.setColor(0);
		
		viewportPhysH = (float)(Math.tan(Math.toRadians(fovy / 2.0f)) * nearPlane) * 2f;
		viewportPhysW = viewportPhysH * width / height;
		
		projXscale = width / viewportPhysW;
		projYscale = height / viewportPhysH;
		
		g3dClearFlags = "1.0".equals(System.getProperty("microedition.m3g.version")) ? 0 : Graphics3D.OVERWRITE;
		
		// Graphics3D - синглтон, список источников света переживает смену уровня
		g3d.resetLights();
		
		//Hashtable params = g3d.getProperties();
		//System.out.println("maxLights: " + params.get("maxLights"));
	}

	public final void destroy() {
		//??? useless
	}

	/** Набор динамических источников света (null - освещение выключено). */
	public final void setLights(DynamicLights lights) {
		this.lights = lights;
	}

	private void applyLights() {
		if(lights != null) lights.apply(g3d);
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
	 * Ставит камеру готовой матрицей camera-to-world (для вида через портал).
	 * camPos обновляется из матрицы - он нужен рендеру комнат.
	 */
	public final void setCameraTransform(Transform camToWorld) {
		camTrans.set(camToWorld);
		camTrans.get(tmpMat);
		camPos.set((int) tmpMat[3], (int) tmpMat[7], (int) tmpMat[11]);
		
		invCam.set(camTrans);
		invCam.invert();
	}
	
	/** Копирует текущую матрицу камеры (camera-to-world). */
	public final void getCameraTransform(Transform out) {
		out.set(camTrans);
	}
	
	public final Transform getInvCam() {
		return invCam;
	}
	
	/**
	 * Задаёт плоскость отсечения в МИРОВЫХ координатах (a*x + b*y + c*z + d > 0 - видимая часть).
	 * Она заменяет ближнюю плоскость пирамиды видимости (oblique near plane clipping),
	 * благодаря чему в виде через портал не появляется геометрия перед выходным порталом.
	 * Плоскость пересчитывается в пространство камеры, поэтому вызывать ПОСЛЕ setCameraTransform.
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
	 * Сдвиг по глубине для последующих setClip (в единицах NDC).
	 * 0.0001f - примерно 3 младших разряда 16-битного буфера глубины.
	 */
	public final void setDepthBias(float bias) {
		depthBias = bias;
	}
	
	/**
	 * Заменяет ближнюю плоскость проекции на clipPlane (Eric Lengyel, oblique frustum).
	 * Матрица - row-major, как того требует Transform.set().
	 */
	private void applyClipPlane(float[] mat) {
		float a = clipPlane[0], b = clipPlane[1], c = clipPlane[2], d = clipPlane[3];
		
		float len = (float) Math.sqrt(a * a + b * b + c * c);
		if(len < 0.000001f) return;
		
		a /= len;
		b /= len;
		c /= len;
		d /= len;
		
		// d - расстояние от камеры (начала координат) до плоскости.
		// Камера должна быть ПОЗАДИ плоскости, иначе отсекать нечего.
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
		try {
			int w = width;
			int h = height;

			float[] mat = camPersTmp2;
			float[] matBck = camPersTmp;
			
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
			
			if(toTexture) {
				// прямоугольник экрана -> прямоугольник в текстуре портала
				int vx1 = mapTexX(x1);
				int vx2 = mapTexX(x2);
				int vy1 = mapTexY(y1);
				int vy2 = mapTexY(y2);
				
				if(vx2 <= vx1) vx2 = vx1 + 1;
				if(vy2 <= vy1) vy2 = vy1 + 1;
				
				g3d.setViewport(vx1, vy1, vx2 - vx1, vy2 - vy1);
			} else {
				g3d.setViewport(x1 + renderX, y1 + renderY, x2 - x1, y2 - y1);
			}
		} catch (Exception e) {
			System.out.println(x1 + " " + y1 + " " + x2 + " " + y2);
			e.printStackTrace();
		}
	}
	
	private int mapTexX(int x) {
		return (int) ((float) (x - frustumX1) * texW / (frustumX2 - frustumX1) + 0.5f);
	}
	
	private int mapTexY(int y) {
		return (int) ((float) (y - frustumY1) * texH / (frustumY2 - frustumY1) + 0.5f);
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
	
	/** Рендер узла с готовой матрицей модели. */
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
		applyLights();
		
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
	 * Начинает рендер в текстуру портала.
	 * (bx1, by1, bx2, by2) - прямоугольник ЭКРАНА основной камеры, который
	 * будет растянут на всю текстуру: сюда попадает bbox квада портала.
	 */
	public final void beginTextureTarget(Image2D target, int bx1, int by1, int bx2, int by2) {
		g3d.bindTarget(target, true, g3dClearFlags);
		
		toTexture = true;
		texW = target.getWidth();
		texH = target.getHeight();
		frustumX1 = bx1;
		frustumY1 = by1;
		frustumX2 = bx2;
		frustumY2 = by2;
		
		g3d.setViewport(0, 0, texW, texH);
		g3d.clear(texClearBck);
		applyLights();
	}
	
	public final void endTextureTarget() {
		g3d.releaseTarget();
		toTexture = false;
	}
	
	public final boolean isTextureTarget() {
		return toTexture;
	}
	
	/**
	 * Диапазон буфера глубины в window-координатах (0..1). NDC z [-1,1]
	 * отображается в [near, far], что позволяет отдать разным проходам
	 * непересекающиеся полосы глубины.
	 */
	public final void setDepthRange(float near, float far) {
		g3d.setDepthRange(near, far);
	}

	/** Очищает буфер глубины в пределах текущего viewport'а. */
	public final void clearDepth() {
		g3d.clear(depthClearBck);
	}
	
	/** Проверка поддержки рендера в Image2D конкретной реализацией M3G. */
	public final boolean checkTextureTargetSupport() {
		Image2D probe = null;
		try {
			probe = new Image2D(Image2D.RGB, 8, 8);
			g3d.bindTarget(probe, true, 0);
			g3d.releaseTarget();
			return true;
		} catch (Throwable t) {
			System.out.println("PORTAL: render-to-texture не поддерживается: " + t);
			try {
				g3d.releaseTarget();
			} catch (Throwable t2) {
			}
			return false;
		}
	}
	
	/**
	 * Возвращает Graphics3D для прямого доступа.
	 */
	public final Graphics3D getG3D() {
		return g3d;
	}
}
