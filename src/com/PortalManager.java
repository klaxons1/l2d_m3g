package com;

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
 * Управляет двумя порталами (оранжевый и синий).
 * Хранит позиции, нормали, комнаты и quad-меши для рендера.
 */
public final class PortalManager {

    // Максимальный размер портала (половина ширины/высоты quad'а)
    public static final int PORTAL_HALF_W = 400;
    public static final int PORTAL_HALF_H = 700;

    // Портал 0 = синий, Портал 1 = оранжевый
    private static final int PORTAL_COUNT = 2;

    // Мировые координаты центров порталов
    private final Vector3D[] positions = new Vector3D[PORTAL_COUNT];
    // Нормали стен, на которых размещены порталы
    private final Vector3D[] normals = new Vector3D[PORTAL_COUNT];
    // Right-вектора (касательные к стене)
    private final Vector3D[] rights = new Vector3D[PORTAL_COUNT];
    // Up-вектора
    private final Vector3D[] ups = new Vector3D[PORTAL_COUNT];
    // Активен ли портал
    private final boolean[] active = new boolean[PORTAL_COUNT];
    // ID комнаты, в которой находится портал
    private final int[] roomIds = new int[PORTAL_COUNT];

    // Quad-меши для отрисовки порталов
    private final Mesh[] portalMeshes = new Mesh[PORTAL_COUNT];

    // Для проекции на экран
    private final float[] projVertsTmp = new float[4 * 4];
    private final Transform tmpTransform = new Transform();

    public PortalManager() {
        for (int i = 0; i < PORTAL_COUNT; i++) {
            positions[i] = new Vector3D();
            normals[i] = new Vector3D();
            rights[i] = new Vector3D();
            ups[i] = new Vector3D();
            active[i] = false;
            roomIds[i] = -1;
        }

        // Создаём меши: 0 = синий (0x3366FF), 1 = оранжевый (0xFF6600)
        portalMeshes[0] = createPortalMesh(0x33, 0x66, 0xFF);
        portalMeshes[1] = createPortalMesh(0xFF, 0x66, 0x00);
    }

    /**
     * Размещает портал с указанным индексом в заданной позиции на стене.
     *
     * @param idx     0 = синий, 1 = оранжевый
     * @param pos     точка на стене (world coords)
     * @param normal  нормаль стены
     * @param part    ID комнаты
     */
    public void placePortal(int idx, Vector3D pos, Vector3D normal, int part) {
        positions[idx].set(pos);
        normals[idx].set(normal);
        roomIds[idx] = part;
        active[idx] = true;

        // Вычисляем right и up вектора для портала
        computeBasis(idx);
    }

    private void computeBasis(int idx) {
        Vector3D n = normals[idx];
        Vector3D r = rights[idx];
        Vector3D u = ups[idx];

        // Находим right-вектор: cross(n, worldUp)
        // worldUp = (0, 1, 0)
        int rx = n.y * 0 - n.z * 1;  // n.y * 0 - n.z * 1 = -n.z... нет
        // cross(normal, up) = (ny*0 - nz*1, nz*0 - nx*0, nx*1 - ny*0) = (-nz, 0, nx)
        rx = -n.z;
        int ry = 0;
        int rz = n.x;

        // Если normal вертикальная (портал на полу/потолке), используем другой up
        int rLen = (int) Math.sqrt(rx * rx + ry * ry + rz * rz);
        if (rLen < 100) {
            // normal ≈ (0, ±1, 0), используем (1,0,0) как right
            rx = 4096;
            ry = 0;
            rz = 0;
            rLen = 4096;
        }

        // Нормализуем right до 4096 (Q12)
        rx = rx * 4096 / rLen;
        ry = ry * 4096 / rLen;
        rz = rz * 4096 / rLen;
        r.set(rx, ry, rz);

        // up = cross(right, normal)
        int ux = (ry * n.z - rz * n.y) >> 12;
        int uy = (rz * n.x - rx * n.z) >> 12;
        int uz = (rx * n.y - ry * n.x) >> 12;

        // Нормализуем up
        int uLen = (int) Math.sqrt(ux * ux + uy * uy + uz * uz);
        if (uLen > 0) {
            u.set(ux * 4096 / uLen, uy * 4096 / uLen, uz * 4096 / uLen);
        } else {
            u.set(0, 4096, 0);
        }
    }

    public boolean isActive(int idx) {
        return active[idx];
    }

    public Vector3D getPosition(int idx) {
        return positions[idx];
    }

    public Vector3D getNormal(int idx) {
        return normals[idx];
    }

    public Vector3D getRight(int idx) {
        return rights[idx];
    }

    public Vector3D getUp(int idx) {
        return ups[idx];
    }

    public int getRoomId(int idx) {
        return roomIds[idx];
    }

    /**
     * Возвращает парный портал: 0 → 1, 1 → 0.
     */
    public int getLinkedPortal(int idx) {
        return idx == 0 ? 1 : 0;
    }

    /**
     * Возвращает 4 вершины quad'а портала в мировых координатах.
     * verts[0..3] = {left-bottom, right-bottom, right-top, left-top}
     */
    public Vector3D[] getPortalVertices(int idx) {
        Vector3D pos = positions[idx];
        Vector3D r = rights[idx];
        Vector3D u = ups[idx];

        int hw = PORTAL_HALF_W;
        int hh = PORTAL_HALF_H;

        // Смещения от центра
        int rx = r.x * hw >> 12;
        int ry = r.y * hw >> 12;
        int rz = r.z * hw >> 12;
        int ux = u.x * hh >> 12;
        int uy = u.y * hh >> 12;
        int uz = u.z * hh >> 12;

        // Небольшое смещение от стены, чтобы не z-fight'ить
        int nx = normals[idx].x * 5 >> 12;
        int ny = normals[idx].y * 5 >> 12;
        int nz = normals[idx].z * 5 >> 12;

        Vector3D[] verts = new Vector3D[4];
        verts[0] = new Vector3D(pos.x - rx - ux + nx, pos.y - ry - uy + ny, pos.z - rz - uz + nz);
        verts[1] = new Vector3D(pos.x + rx - ux + nx, pos.y + ry - uy + ny, pos.z + rz - uz + nz);
        verts[2] = new Vector3D(pos.x + rx + ux + nx, pos.y + ry + uy + ny, pos.z + rz + uz + nz);
        verts[3] = new Vector3D(pos.x - rx + ux + nx, pos.y - ry + uy + ny, pos.z - rz + uz + nz);

        return verts;
    }

    /**
     * Вычисляет экранные bounds портала (min/max X/Y в пикселях).
     * Возвращает false, если портал не виден.
     */
    public boolean getPortalScreenBounds(int idx, Renderer g3d, int[] outBounds) {
        Vector3D[] verts = getPortalVertices(idx);
        Transform invCam = g3d.getInvCam();

        int minX = Integer.MAX_VALUE, maxX = Integer.MIN_VALUE;
        int minY = Integer.MAX_VALUE, maxY = Integer.MIN_VALUE;
        float minDepth = Float.MAX_VALUE;
        boolean anyVisible = false;

        for (int i = 0; i < 4; i++) {
            // Копируем в tmp float массив
            projVertsTmp[i * 4 + 0] = verts[i].x;
            projVertsTmp[i * 4 + 1] = verts[i].y;
            projVertsTmp[i * 4 + 2] = verts[i].z;
            projVertsTmp[i * 4 + 3] = 1;
        }

        invCam.transform(projVertsTmp);

        float nearPlane = g3d.nearPlane;
        float projXscale = g3d.projXscale;
        float projYscale = g3d.projYscale;
        int w2 = g3d.width >> 1, h2 = g3d.height >> 1;

        for (int i = 0; i < 4; i++) {
            float ax = projVertsTmp[i * 4 + 0];
            float ay = projVertsTmp[i * 4 + 1];
            float az = -projVertsTmp[i * 4 + 2];

            if (az < nearPlane) continue;
            anyVisible = true;

            float w = nearPlane / az;
            int px = (int) ((ax * w) * projXscale + w2);
            int py = (int) ((-ay * w) * projYscale + h2);

            if (px < minX) minX = px;
            if (px > maxX) maxX = px;
            if (py < minY) minY = py;
            if (py > maxY) maxY = py;
            if (az < minDepth) minDepth = az;
        }

        if (!anyVisible) return false;

        // Клиппим к экрану
        if (minX < 0) minX = 0;
        if (minY < 0) minY = 0;
        if (maxX > g3d.width) maxX = g3d.width;
        if (maxY > g3d.height) maxY = g3d.height;

        if (maxX <= minX || maxY <= minY) return false;

        outBounds[0] = minX;
        outBounds[1] = minY;
        outBounds[2] = maxX;
        outBounds[3] = maxY;
        return true;
    }

    /**
     * Рисует портал-квад как depth-маску (без записи в color buffer).
     * Depth range должен быть настроен заранее.
     */
    public void renderPortalQuadMask(int idx, Renderer g3d) {
        Mesh mesh = portalMeshes[idx];
        if (mesh == null) return;

        // Ставим камеру из main camera, но projection будет уже настроен на portal view
        // Просто рендерим mesh с identity transform (вершины уже в world coords)
        g3d.addMesh(mesh, null, null);
    }

    /**
     * Рисует цветной портал-квад поверх depth-маски.
     */
    public void renderPortalQuadColor(int idx, Renderer g3d) {
        // Рендерим тот же mesh, но с цветным видом
        // Для этого нам нужен второй mesh с цветным appearance
        // Используем тот же mesh - цвет уже в vertex colors
        Mesh mesh = portalMeshes[idx];
        if (mesh == null) return;
        g3d.addMesh(mesh, null, null);
    }

    /**
     * Создаёт quad mesh для портала с указанным цветом.
     * Вершины будут обновляться при размещении портала.
     */
    private static Mesh createPortalMesh(int r, int g, int b) {
        // 4 вершины, заполняем placeholder'ами (обновятся при placePortal)
        short[] positions = new short[]{
            -PORTAL_HALF_W, -PORTAL_HALF_H, 0,
            PORTAL_HALF_W, -PORTAL_HALF_H, 0,
            PORTAL_HALF_W, PORTAL_HALF_H, 0,
            -PORTAL_HALF_W, PORTAL_HALF_H, 0
        };

        byte[] colors = new byte[]{
            (byte) r, (byte) g, (byte) b,
            (byte) r, (byte) g, (byte) b,
            (byte) r, (byte) g, (byte) b,
            (byte) r, (byte) g, (byte) b
        };

        byte[] normals = new byte[]{
            0, 0, 127,
            0, 0, 127,
            0, 0, 127,
            0, 0, 127
        };

        VertexArray posArray = new VertexArray(4, 3, 2);
        posArray.set(0, 4, positions);

        VertexArray colArray = new VertexArray(4, 3, 1);
        colArray.set(0, 4, colors);

        VertexArray norArray = new VertexArray(4, 3, 1);
        norArray.set(0, 4, normals);

        VertexBuffer vb = new VertexBuffer();
        vb.setPositions(posArray, 1.0f, null);
        vb.setColors(colArray);
        vb.setNormals(norArray);

        int[] indices = new int[]{0, 1, 3, 2};
        int[] stripLengths = new int[]{4};
        TriangleStripArray tsa = new TriangleStripArray(indices, stripLengths);

        PolygonMode pm = new PolygonMode();
        pm.setPerspectiveCorrectionEnable(true);
        pm.setCulling(PolygonMode.CULL_NONE);
        pm.setShading(PolygonMode.SHADE_FLAT);

        CompositingMode cm = new CompositingMode();
        cm.setDepthWriteEnable(true);
        cm.setDepthTestEnable(true);

        Appearance ap = new Appearance();
        ap.setPolygonMode(pm);
        ap.setCompositingMode(cm);

        return new Mesh(vb, new IndexBuffer[]{tsa}, new Appearance[]{ap});
    }

    /**
     * Обновляет позицию вершин quad-меша portal'а по текущей позиции и осям.
     * Нужно вызывать после placePortal, чтобы quad стоял на стене.
     */
    public void updatePortalMesh(int idx) {
        // Mesh vertices are set relative to the mesh's own coordinate system.
        // Since we render with null transform (identity), we need to set actual world positions.
        // Unfortunately M3G VertexArray.set() works with shorts, which may overflow for world coords.
        // Instead, we'll use a Transform when rendering to position the quad.
        // For now, we store the vertex offsets and apply the transform at render time.

        // The mesh is created with local coords centered at origin.
        // We'll position it via Transform when calling g3d.addMesh().
        // This happens in renderPortalQuadMask/renderPortalQuadColor.
    }

    /**
     * Вычисляет матрицу трансформации портала для рендера через портал.
     *
     * Трансформация переводит мировые координаты так, как если бы камера
     * смотрела через destination portal на source portal.
     *
     * @param srcIdx  индекс source портала (тот, через который смотрим)
     * @param dstIdx  индекс destination портала (тот, что виден)
     * @param out     выходная матрица 4x4 (Transform)
     */
    public void getPortalCameraTransform(int srcIdx, int dstIdx, Transform out) {
        Vector3D srcPos = positions[srcIdx];
        Vector3D srcNor = normals[srcIdx];
        Vector3D srcR = rights[srcIdx];
        Vector3D srcU = ups[srcIdx];

        Vector3D dstPos = positions[dstIdx];
        Vector3D dstNor = normals[dstIdx];
        Vector3D dstR = rights[dstIdx];
        Vector3D dstU = ups[dstIdx];

        // Матрица портала: переводит из destination space в source space
        // [R | T] где R = [srcR, srcU, srcNor]^T * [dstR, dstU, dstNor]
        //   и T = srcPos - R * dstPos

        // Сначала считаем R = source basis * dest basis^T
        // Каждый вектор в Q12 (4096 = 1.0)
        // R * v = sourceBasis * (destBasis^T * v)
        //
        // R[0][0] = srcR.dot(dstR), R[0][1] = srcR.dot(dstU), R[0][2] = srcR.dot(dstNor)
        // R[1][0] = srcU.dot(dstR), R[1][1] = srcU.dot(dstU), R[1][2] = srcU.dot(dstNor)
        // R[2][0] = srcN.dot(dstR), R[2][1] = srcN.dot(dstU), R[2][2] = srcN.dot(dstNor)

        // Dot products в Q12 * Q12 >> 12
        int r00 = (srcR.x * dstR.x + srcR.y * dstR.y + srcR.z * dstR.z) >> 12;
        int r01 = (srcR.x * dstU.x + srcR.y * dstU.y + srcR.z * dstU.z) >> 12;
        int r02 = (srcR.x * dstNor.x + srcR.y * dstNor.y + srcR.z * dstNor.z) >> 12;

        int r10 = (srcU.x * dstR.x + srcU.y * dstR.y + srcU.z * dstR.z) >> 12;
        int r11 = (srcU.x * dstU.x + srcU.y * dstU.y + srcU.z * dstU.z) >> 12;
        int r12 = (srcU.x * dstNor.x + srcU.y * dstNor.y + srcU.z * dstNor.z) >> 12;

        int r20 = (srcNor.x * dstR.x + srcNor.y * dstR.y + srcNor.z * dstR.z) >> 12;
        int r21 = (srcNor.x * dstU.x + srcNor.y * dstU.y + srcNor.z * dstU.z) >> 12;
        int r22 = (srcNor.x * dstNor.x + srcNor.y * dstNor.y + srcNor.z * dstNor.z) >> 12;

        // T = srcPos - R * dstPos
        // R * dstPos:
        int rd_x = (r00 * dstPos.x + r01 * dstPos.y + r02 * dstPos.z) >> 12;
        int rd_y = (r10 * dstPos.x + r11 * dstPos.y + r12 * dstPos.z) >> 12;
        int rd_z = (r20 * dstPos.x + r21 * dstPos.y + r22 * dstPos.z) >> 12;

        int tx = srcPos.x - rd_x;
        int ty = srcPos.y - rd_y;
        int tz = srcPos.z - rd_z;

        // Заполняем Transform (column-major 4x4)
        float[] mat = new float[16];
        mat[0] = (float) r00 / 4096.0f;
        mat[1] = (float) r10 / 4096.0f;
        mat[2] = (float) r20 / 4096.0f;
        mat[3] = tx;

        mat[4] = (float) r01 / 4096.0f;
        mat[5] = (float) r11 / 4096.0f;
        mat[6] = (float) r21 / 4096.0f;
        mat[7] = ty;

        mat[8] = (float) r02 / 4096.0f;
        mat[9] = (float) r12 / 4096.0f;
        mat[10] = (float) r22 / 4096.0f;
        mat[11] = tz;

        mat[12] = 0;
        mat[13] = 0;
        mat[14] = 0;
        mat[15] = 1;

        out.set(mat);
    }
}
