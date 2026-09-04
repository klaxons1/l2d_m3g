package com;

public final class MathUtils {
	public static float FPI = (float) Math.PI;
	public static float PI_TO_RAD = (float) (180.0 / Math.PI);

	public static float invSqrt(float val) {
		float half = 0.5F * val;
		int i = 1597463007 - (Float.floatToIntBits(val) >> 1);
		float x = Float.intBitsToFloat(i);
		
		return x * (1.5F - half * x * x);
	}

	//x range from -1 to 1
	public static float atan(float x) {
		float ax = x < 0.0f ? -x : x;
		return x * (FPI * 0.25f - 0.273f * (ax - 1.0f));
	}

	public static float atan2(float x1, float z1, float x2, float z2) {
		float y = x2 - x1;
		float x = z2 - z1;
		
		if (x == 0.0f) {
			return y > 0.0f ? 90.0f : -90.0f;
		}
		
		float ax = x < 0.0f ? -x : x;
		float ay = y < 0.0f ? -y : y;
		float res;
		
		if (ay < ax) {
			res = PI_TO_RAD * atan(y / x);
		} else {
			float tmp = x / y;
			res = -PI_TO_RAD * atan(tmp);
			res += (tmp > 0.0f) ? 90.0f : -90.0f;
		}
		
		if (x < 0.0f) {
			res += (y >= 0.0f) ? 180.0f : -180.0f;
		}
		
		return res;
	}

	public static Vector3D createNormal(int ax, int ay, int az, int bx, int by, int bz, int cx, int cy, int cz) {
		long abx = ax - bx, aby = ay - by, abz = az - bz;
		long acx = ax - cx, acy = ay - cy, acz = az - cz;
		
		double x = (double) (aby * acz - abz * acy);
		double y = (double) (abz * acx - abx * acz);
		double z = (double) (abx * acy - aby * acx);
		
		double len = Math.sqrt(x * x + y * y + z * z);
		if (len == 0.0) return new Vector3D(0, 0, 0);
		
		double scale = 4096.0 / len;
		return new Vector3D((int) (x * scale), (int) (y * scale), (int) (z * scale));
	}

	public static int distanceToLine(Vector3D point, Vector3D a, Vector3D b) {
		int dx = b.x - a.x;
		int dy = b.y - a.y;
		int dz = b.z - a.z;
		int px = point.x - a.x;
		int py = point.y - a.y;
		int pz = point.z - a.z;
		
		long lenSq = (long) dx * dx + (long) dy * dy + (long) dz * dz;
		long t = 0L;
		if (lenSq != 0L) {
			t = (((long) px * dx + (long) py * dy + (long) pz * dz) << 14) / lenSq;
		}

		if (t < 0L) t = 0L;
		else if (t > 16384L) t = 16384L;

		int cx = a.x + (int) ((long) dx * t >> 14) - point.x;
		int cy = a.y + (int) ((long) dy * t >> 14) - point.y;
		int cz = a.z + (int) ((long) dz * t >> 14) - point.z;
		
		return cx * cx + cy * cy + cz * cz;
	}

	public static int distanceToRay(Vector3D point, Vector3D a, Vector3D dir) {
		int dx = dir.x;
		int dy = dir.y;
		int dz = dir.z;
		int px = point.x - a.x;
		int py = point.y - a.y;
		int pz = point.z - a.z;
		
		long lenSq = (long) dx * dx + (long) dy * dy + (long) dz * dz;
		long t = 0L;
		if (lenSq != 0L) {
			t = (((long) px * dx + (long) py * dy + (long) pz * dz) << 14) / lenSq;
		}

		if (t < 0L) t = 0L;

		int cx = a.x + (int) ((long) dx * t >> 14) - point.x;
		int cy = a.y + (int) ((long) dy * t >> 14) - point.y;
		int cz = a.z + (int) ((long) dz * t >> 14) - point.z;
		
		return cx * cx + cy * cy + cz * cz;
	}

    public static boolean isPointOnPolygon(Vector3D point, Vector3D a, Vector3D b, Vector3D c, Vector3D d, Vector3D normal) {
        final int nx = normal.x > 0 ? normal.x : -normal.x;
        final int ny = normal.y > 0 ? normal.y : -normal.y;
        final int nz = normal.z > 0 ? normal.z : -normal.z;

        if (nx >= ny && nx >= nz) {
            return normal.x >= 0 ? 
                isPointOnPolygon(point.z, point.y, a.z, a.y, b.z, b.y, c.z, c.y, d.z, d.y) :
                isPointOnPolygon(point.z, point.y, d.z, d.y, c.z, c.y, b.z, b.y, a.z, a.y);
        }
        if (ny >= nx && ny >= nz) {
            return normal.y >= 0 ? 
                isPointOnPolygon(point.x, point.z, a.x, a.z, b.x, b.z, c.x, c.z, d.x, d.z) :
                isPointOnPolygon(point.x, point.z, d.x, d.z, c.x, c.z, b.x, b.z, a.x, a.z);
        }
        return normal.z <= 0 ? 
            isPointOnPolygon(point.x, point.y, a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y) :
            isPointOnPolygon(point.x, point.y, d.x, d.y, c.x, c.y, b.x, b.y, a.x, a.y);
    }
    
    public static boolean isPointOnPolygon(Vector3D point, Vector3D a, Vector3D b, Vector3D c, Vector3D normal) {
        final int nx = normal.x > 0 ? normal.x : -normal.x;
        final int ny = normal.y > 0 ? normal.y : -normal.y;
        final int nz = normal.z > 0 ? normal.z : -normal.z;

        if (nx >= ny && nx >= nz) {
            return normal.x >= 0 ? 
                isPointOnPolygon(point.z, point.y, a.z, a.y, b.z, b.y, c.z, c.y) :
                isPointOnPolygon(point.z, point.y, c.z, c.y, b.z, b.y, a.z, a.y);
        }
        if (ny >= nx && ny >= nz) {
            return normal.y >= 0 ? 
                isPointOnPolygon(point.x, point.z, a.x, a.z, b.x, b.z, c.x, c.z) :
                isPointOnPolygon(point.x, point.z, c.x, c.z, b.x, b.z, a.x, a.z);
        }
        return normal.z <= 0 ? 
            isPointOnPolygon(point.x, point.y, a.x, a.y, b.x, b.y, c.x, c.y) :
            isPointOnPolygon(point.x, point.y, c.x, c.y, b.x, b.y, a.x, a.y);
    }
    
    public static boolean isPointOnPolygon(int px, int pz, 
                                           int ax, int az,
                                           int bx, int bz,
                                           int cx, int cz, 
                                           int dx, int dz,
                                           int norY) {
        if (norY > 0) return isPointOnPolygon(px, pz, ax, az, bx, bz, cx, cz, dx, dz);
        if (norY < 0) return isPointOnPolygon(px, pz, dx, dz, cx, cz, bx, bz, ax, az);
        return false;
    }
    
    public static boolean isPointOnPolygon(int px, int pz, 
                                           int ax, int az,
                                           int bx, int bz,
                                           int cx, int cz, 
                                           int norY) {
        if (norY > 0) return isPointOnPolygon(px, pz, ax, az, bx, bz, cx, cz);
        if (norY < 0) return isPointOnPolygon(px, pz, cx, cz, bx, bz, ax, az);
        return false;
    }
    
    public static boolean isPointOnPolygon(int px, int py, int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
        return (x2 - x1) * (py - y1) <= (px - x1) * (y2 - y1) &&
               (x3 - x2) * (py - y2) <= (px - x2) * (y3 - y2) &&
               (x4 - x3) * (py - y3) <= (px - x3) * (y4 - y3) &&
               (x1 - x4) * (py - y4) <= (px - x4) * (y1 - y4);
    }
    
    public static boolean isPointOnPolygon(int px, int py, int x1, int y1, int x2, int y2, int x3, int y3) {
        return (x2 - x1) * (py - y1) <= (px - x1) * (y2 - y1) &&
               (x3 - x2) * (py - y2) <= (px - x2) * (y3 - y2) && 
               (x1 - x3) * (py - y3) <= (px - x3) * (y1 - y3);
    }
}
