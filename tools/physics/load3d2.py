#!/usr/bin/env python3
"""Loader for the .3d2 level/model format used by the game.

This is a Python mirror of src/com/MeshData.java (only the parts needed
for rigid-body collision): meshes come out as Collider objects holding
raw storage-space vertices, reordered polygon indices (quads first,
tris second, clockwise winding), per-polygon normals and the world
transform the Java physics layer applies:

    world = (raw * scale8 >> 8) + offset

Usage:
    colliders = load3d2.load("res/city.3d2", 7500.0)
"""
import math
import struct


class Collider:
    __slots__ = ("verts", "pols", "norms", "quads", "tris",
                 "scale8", "off_x", "off_y", "off_z", "name")

    def __init__(self, name=""):
        self.verts = []
        self.pols = []
        self.norms = []
        self.quads = 0
        self.tris = 0
        self.scale8 = 256
        self.off_x = self.off_y = self.off_z = 0
        self.name = name


class _Reader:
    def __init__(self, data):
        self.d = data
        self.p = 0

    def u8(self):
        v = self.d[self.p]
        self.p += 1
        return v

    def s8(self):
        v = self.d[self.p] - 256 if self.d[self.p] >= 128 else self.d[self.p]
        self.p += 1
        return v

    def u16(self):
        v = struct.unpack_from(">H", self.d, self.p)[0]
        self.p += 2
        return v

    def s16(self):
        v = struct.unpack_from(">h", self.d, self.p)[0]
        self.p += 2
        return v

    def s32(self):
        v = struct.unpack_from(">i", self.d, self.p)[0]
        self.p += 4
        return v

    def f32(self):
        v = struct.unpack_from(">f", self.d, self.p)[0]
        self.p += 4
        return v

    def skip(self, n):
        self.p += n


def _attribute(r, count, dims, x_bytes, y_bytes, z_bytes):
    """Read a packed vertex attribute, mirroring loadVertexAttribute3D2."""
    out = [0] * (count * dims)
    for i in range(count):
        vals = []
        vals.append(r.s8() if x_bytes else r.s16())
        if dims >= 2:
            vals.append(r.s8() if y_bytes else r.s16())
        if dims >= 3:
            vals.append(r.s8() if z_bytes else r.s16())
        for k in range(dims):
            out[i * dims + k] = vals[k]
    return out


def _calculate_normals(verts, pols, quad_count):
    """Per-polygon normals in Q12, identical to MeshData.calculateNormals."""
    norms = [0] * ((quad_count + (len(pols) - quad_count * 4) // 3) * 3)
    n_idx = 0
    p_idx = 0
    for vpp in (4, 3):
        p_end = quad_count * 4 if vpp == 4 else len(pols)
        while p_idx < p_end:
            ia, ib, ic = pols[p_idx], pols[p_idx + 1], pols[p_idx + vpp - 1]
            ax, ay, az = verts[ia * 3:ia * 3 + 3]
            bx, by, bz = verts[ib * 3:ib * 3 + 3]
            cx, cy, cz = verts[ic * 3:ic * 3 + 3]
            abx, aby, abz = ax - bx, ay - by, az - bz
            acx, acy, acz = ax - cx, ay - cy, az - cz
            x = float(aby * acz - abz * acy)
            y = float(abz * acx - abx * acz)
            z = float(abx * acy - aby * acx)
            length = math.sqrt(x * x + y * y + z * z)
            if length != 0.0:
                s = 4096.0 / length
                # Java (int) cast truncates toward zero
                norms[n_idx * 3] = int(x * s)
                norms[n_idx * 3 + 1] = int(y * s)
                norms[n_idx * 3 + 2] = int(z * s)
            p_idx += vpp
            n_idx += 1
    return norms


def load(path, model_scale=7500.0):
    with open(path, "rb") as fh:
        r = _Reader(fh.read())

    r.skip(4)               # magic
    r.u16()                 # format version
    pos_scale = r.f32()
    r.f32()                 # uv scale
    for _ in range(6):      # model AABB
        r.s16()
    mesh_count = r.u16()

    colliders = []
    mesh_scale = model_scale / pos_scale if pos_scale else model_scale
    for mi in range(mesh_count):
        flags = r.s32()
        has_norms = flags & 1
        has_uvs = flags & 2
        has_cols = flags & 4
        uv_x_bytes = flags & 8
        uv_y_bytes = flags & 16
        has_bones = flags & 32

        amin = (r.s16(), r.s16(), r.s16())
        amax = (r.s16(), r.s16(), r.s16())
        x_bytes = amax[0] - amin[0] < 256
        y_bytes = amax[1] - amin[1] < 256
        z_bytes = amax[2] - amin[2] < 256
        off = tuple((amin[k] + 128) if (x_bytes, y_bytes, z_bytes)[k] else 0
                    for k in range(3))

        if uv_x_bytes:
            r.s16()
        if uv_y_bytes:
            r.s16()

        if has_bones:
            bone_count = r.u8()
            for _ in range(bone_count):
                r.u8()               # parent id
                r.skip(16 * 4)       # 4x4 float matrix

        vtx_count = r.u16()
        verts = _attribute(r, vtx_count, 3, x_bytes, y_bytes, z_bytes)
        if has_norms:
            _attribute(r, vtx_count, 3, True, True, True)
        if has_uvs:
            _attribute(r, vtx_count, 2, bool(uv_x_bytes),
                       bool(uv_y_bytes), True)
        if has_cols:
            _attribute(r, vtx_count, 3, True, True, True)

        total_quads = r.u16()
        total_tris = r.u16()
        submesh_count = r.u16()

        pols = [0] * (total_quads * 4 + total_tris * 3)
        qpos, tpos = 0, total_quads * 4
        for _ in range(submesh_count):
            quads = r.u16()
            tris = r.u16()
            idx_size = 1 if vtx_count <= 256 else 2
            for _ in range(quads):
                idx = [r.u8() if idx_size == 1 else r.u16() for _ in range(4)]
                # triangle-strip quad abcd -> cbda? Java stores c,d,b,a
                pols[qpos:qpos + 4] = [idx[2], idx[3], idx[1], idx[0]]
                qpos += 4
            for _ in range(tris):
                idx = [r.u8() if idx_size == 1 else r.u16() for _ in range(3)]
                pols[tpos:tpos + 3] = [idx[2], idx[1], idx[0]]
                tpos += 3

        if has_bones:
            max_bones = r.u8()
            for _ in range(vtx_count):
                for _ in range(max_bones):
                    bone_id = r.u8()
                    if bone_id == 255:
                        break
                    if max_bones > 1:
                        r.u8()

        c = Collider("mesh%d" % mi)
        c.verts = verts
        c.pols = pols
        c.quads = total_quads
        c.tris = total_tris
        c.norms = _calculate_normals(verts, pols, total_quads)
        c.scale8 = int(256 * mesh_scale)
        c.off_x = int(off[0] * mesh_scale)
        c.off_y = int(off[1] * mesh_scale)
        c.off_z = int(off[2] * mesh_scale)
        colliders.append(c)

    return colliders


if __name__ == "__main__":
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else "../../res/city.3d2"
    scale = float(sys.argv[2]) if len(sys.argv) > 2 else 7500.0
    ms = load(path, scale)
    print("%d meshes" % len(ms))
    for c in ms:
        wvx = [(c.verts[i * 3] * c.scale8 >> 8) + c.off_x for i in
               range(0, min(len(c.verts) // 3, 4000))]
        wvy = [(c.verts[i * 3 + 1] * c.scale8 >> 8) + c.off_y for i in
               range(0, min(len(c.verts) // 3, 4000))]
        wvz = [(c.verts[i * 3 + 2] * c.scale8 >> 8) + c.off_z for i in
               range(0, min(len(c.verts) // 3, 4000))]
        print("%s: v=%d q=%d t=%d s8=%d off=(%d,%d,%d) "
              "x[%d..%d] y[%d..%d] z[%d..%d]" % (
                  c.name, len(c.verts) // 3, c.quads, c.tris, c.scale8,
                  c.off_x, c.off_y, c.off_z,
                  min(wvx), max(wvx), min(wvy), max(wvy),
                  min(wvz), max(wvz)))
