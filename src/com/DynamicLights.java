package com;

import javax.microedition.m3g.Graphics3D;
import javax.microedition.m3g.Light;
import javax.microedition.m3g.Transform;

/**
 * Динамические источники света.
 *
 * В M3G освещение - фиксированная функция конвейера: свет считается по
 * вершинам, для этого мешу нужны нормали, а материалу - Material в
 * Appearance. Геометрия уровней тут нормали содержит, а Material игра
 * никогда не ставила, поэтому свет просто простаивал.
 *
 * Схема такая:
 *  - постоянный AMBIENT белого цвета с интенсивностью 1.0. Он даёт ровно
 *    ту же яркость, что была до включения света (текстура умножается на
 *    белый), поэтому уровень не темнеет и не требует переделки материалов;
 *  - поверх него - до нескольких OMNI-источников, которые ДОБАВЛЯЮТ цвет:
 *    порталы подсвечивают стену вокруг себя своим цветом, попадание
 *    портальной пушки даёт короткую вспышку.
 *
 * Свет по вершинам на комнате из сотни вершин - штука грубая, поэтому
 * радиусы намеренно большие: пятно света должно накрывать несколько
 * вершин, иначе оно просто не появится.
 */
public final class DynamicLights {

	/** Сколько динамических источников максимум (не считая ambient). */
	private static final int MAX = 4;

	private final Light ambient = new Light();
	private final Light[] pool = new Light[MAX];
	private final Transform[] trans = new Transform[MAX];

	private int count;
	private int limit = MAX;
	private boolean limitChecked;

	public DynamicLights() {
		ambient.setMode(Light.AMBIENT);
		ambient.setColor(0xffffff);
		ambient.setIntensity(1.0f);

		for(int i = 0; i < MAX; i++) {
			Light l = new Light();
			l.setMode(Light.OMNI);
			pool[i] = l;
			trans[i] = new Transform();
		}
	}

	/** Начало кадра: список источников очищается. */
	public final void begin() {
		count = 0;
	}

	/**
	 * @param radius расстояние, на котором вклад источника падает примерно вдвое
	 */
	public final void add(float x, float y, float z, int color, float intensity, float radius) {
		if(count >= limit) return;

		Light l = pool[count];
		l.setColor(color);
		l.setIntensity(intensity);
		// множитель = 1 / (1 + q * d^2)
		l.setAttenuation(1.0f, 0.0f, 1.0f / (radius * radius));

		Transform t = trans[count];
		t.setIdentity();
		t.postTranslate(x, y, z);

		count++;
	}

	/** Подсветка от установленных порталов. */
	public final void addPortals(PortalManager pm) {
		if(pm == null) return;

		float[] plane = new float[4];

		for(int i = 0; i < PortalManager.COUNT; i++) {
			if(!pm.isActive(i)) continue;

			pm.getPlane(i, plane);
			Vector3D p = pm.getPosition(i);

			// источник выносится в комнату: если оставить его в плоскости
			// стены, у самой стены получится N*L = 0 и подсветки не будет
			add(
				p.x + plane[0] * PORTAL_OFFSET,
				p.y + plane[1] * PORTAL_OFFSET,
				p.z + plane[2] * PORTAL_OFFSET,
				pm.getColor(i),
				PORTAL_INTENSITY,
				PORTAL_RADIUS
			);
		}
	}

	private static final float PORTAL_OFFSET = 400f;
	private static final float PORTAL_INTENSITY = 1.3f;
	private static final float PORTAL_RADIUS = 2600f;

	/** Применяет набор к Graphics3D. Вызывается на каждый проход рендера. */
	public final void apply(Graphics3D g3d) {
		if(!limitChecked) {
			limitChecked = true;
			try {
				Object max = g3d.getProperties().get("maxLights");
				if(max instanceof Integer) {
					// один слот всегда занят ambient
					int m = ((Integer) max).intValue() - 1;
					if(m < 0) m = 0;
					if(m < limit) limit = m;
				}
			} catch (Throwable t) {
			}
		}

		try {
			g3d.resetLights();
			g3d.addLight(ambient, null);

			for(int i = 0; i < count; i++) {
				g3d.addLight(pool[i], trans[i]);
			}
		} catch (Throwable t) {
			System.out.println("LIGHTS: " + t);
			count = 0;
		}
	}
}
