#!/usr/bin/env python3
"""Deterministic throws of the physics cube across real level 2 geometry.

The level (res/city.3d2) is loaded with load3d2; spawn points are derived
from actual floor polygons (one anchor per coarse world grid cell) so
every throw starts in valid free space, and each anchor is exercised with
both a straight drop and a hard lateral throw aimed across the room.

Run: python3 level_throw.py [name-substring]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from rigid_body import RigidBody, F
from load3d2 import load

LEVEL = os.path.join(os.path.dirname(__file__), "..", "..", "res", "city.3d2")
HALF = 500
FRAMES = 420


def _world_vertex(mesh, vi):
    s = mesh.scale8
    return (
        (mesh.verts[vi * 3] * s >> 8) + mesh.off_x,
        (mesh.verts[vi * 3 + 1] * s >> 8) + mesh.off_y,
        (mesh.verts[vi * 3 + 2] * s >> 8) + mesh.off_z,
    )


def floor_anchors(meshes, bucket=3000):
    """One free-space spawn per coarse xz cell, above a top floor face."""
    cells = {}
    for mesh in meshes:
        p_idx = 0
        n_idx = 0
        for vpp in (4, 3):
            end = mesh.quads * 4 if vpp == 4 else len(mesh.pols)
            while p_idx < end:
                # quads come first: normals index matches quad order
                ny = mesh.norms[n_idx * 3 + 1]
                pts = [_world_vertex(mesh, mesh.pols[p_idx + j])
                       for j in range(vpp)]
                p_idx += vpp
                n_idx += 1
                # mesh normal points into solid; a top floor face points
                # downward (its contact normal pushes up)
                if ny > -int(0.7 * F):
                    continue
                cx = sum(p[0] for p in pts) // vpp
                cy = sum(p[1] for p in pts) // vpp
                cz = sum(p[2] for p in pts) // vpp
                key = (cx // bucket, cz // bucket)
                cur = cells.get(key)
                if cur is None or cy > cur[1]:
                    cells[key] = (cx, cy, cz)
    anchors = sorted(cells.values())
    return anchors


def ortho_error(r):
    """Max column length/orthogonality deviation from a rotation matrix."""
    m = 0
    for c in range(3):
        L2 = r[c] ** 2 + r[3 + c] ** 2 + r[6 + c] ** 2
        m = max(m, abs(L2 - F * F))
    cols = [(r[0], r[3], r[6]), (r[1], r[4], r[7]), (r[2], r[5], r[8])]
    for a in range(3):
        for b in range(a + 1, 3):
            m = max(m, abs(sum(cols[a][k] * cols[b][k] for k in range(3))))
    return m


def run(start, vel, ang, frames=FRAMES, colliders=None):
    if colliders is None:
        colliders = load(LEVEL, 7500.0)
    b = RigidBody(HALF)
    b.reset(*start)
    b.set_velocity(*vel)
    b.set_angular_velocity(*ang)

    report = {
        "tunnel": [], "edge": [], "deep": [], "matrix": [], "clamp": [],
        "nosettle": False, "end": None, "sleep_frame": None,
        "max_substeps": 0, "outside": [],
    }
    world_min = (-6000, -3000, -4000)
    world_max = (101000, 10000, 66000)
    sub_hist = []
    # substeps only count as suspicious when a surface feature was
    # within leap range AND the subdivided frame actually produced a
    # contact or residual depth: spin subdivisions in open air, or leap
    # subdivisions while falling tangentially past a side wall down a
    # void shaft, never touch geometry and are not tunnels
    near_hist = []
    last_ground = -999
    crossed = False
    INF_SEP = 2 ** 31 - 1
    NEAR_FEATURE = 600

    for f in range(frames):
        b.step(colliders, len(colliders), True)
        cx, cy, cz = b.px >> 12, b.py >> 12, b.pz >> 12
        report["max_substeps"] = max(report["max_substeps"], b.last_substeps)
        sub_hist.append(b.last_substeps)
        near_hist.append(
            b.last_substeps >= 8
            and b.min_separation != INF_SEP
            and b.min_separation <= NEAR_FEATURE
            and (b.num_contacts > 0 or b.max_penetration > 70 << 12))
        if b.ground_contact:
            last_ground = f
        # residual penetration that even minimum dt could not resolve
        if b.last_substeps >= 8 and b.max_penetration > (70 << 12):
            report["deep"].append((f, b.max_penetration >> 12, cx, cy, cz))
        if cy < -300 and not crossed:
            crossed = True
            recent_deep = any(f - 10 <= d[0] <= f for d in report["deep"])
            recent_sub = any(near_hist[g]
                             for g in range(max(0, f - 10), f + 1))
            if recent_deep or recent_sub or f - last_ground < 12:
                report["tunnel"].append((f, cx, cy, cz))
            else:
                # long airborne departure over an open edge into a void
                # shaft with no floor in the column: not a solver fault
                report["edge"].append((f, cx, cy, cz))
        if not (world_min[0] <= cx <= world_max[0] and
                world_min[1] <= cy <= world_max[1] and
                world_min[2] <= cz <= world_max[2]):
            report["outside"].append((f, cx, cy, cz))
        oe = ortho_error(b.r)
        if oe > F * F // 100:
            report["matrix"].append((f, oe))
        if (max(abs(b.vx), abs(b.vy), abs(b.vz)) > 2050 * F or
                max(abs(b.wx), abs(b.wy), abs(b.wz)) > 2 * F + 8):
            report["clamp"].append((f, b.vx, b.vy, b.vz, b.wx, b.wy, b.wz))
        if b.sleeping and report["sleep_frame"] is None:
            report["sleep_frame"] = f

    report["nosettle"] = report["sleep_frame"] is None
    report["end"] = (b.px >> 12, b.py >> 12, b.pz >> 12)
    return report


def build_throws(meshes):
    """Deterministic set: per anchor a drop plus a varied lateral throw."""
    anchors = floor_anchors(meshes)
    throws = []
    dirs = [
        (420, 0, 0), (-420, 40, 0), (0, 60, 420), (300, -180, 300),
        (-300, -180, -300), (0, 200, -420), (360, 120, -200),
        (-360, -120, 200),
    ]
    spins = [
        (0, 0, 0), (120, 60, -90), (-80, 140, 40), (60, -160, 100),
    ]
    # spread anchors across the whole level
    step = max(1, len(anchors) // 12)
    picked = anchors[::step][:12]
    ti = 0
    for a in picked:
        cx, cy, cz = a
        spawn = (cx, cy + HALF + 60, cz)
        throws.append(("drop %d" % ti, spawn, (0, -40, 0), (0, 0, 0)))
        d = dirs[ti % len(dirs)]
        w = spins[ti % len(spins)]
        throws.append(("throw %d" % ti, spawn, d, w))
        ti += 1
    return throws


def main():
    only = sys.argv[1] if len(sys.argv) > 1 else None
    meshes = load(LEVEL, 7500.0)
    throws = build_throws(meshes)
    global THROWS
    THROWS = throws
    problems = 0
    runnable = 0
    for name, start, vel, ang in throws:
        if only and only not in name:
            continue
        runnable += 1
        r = run(start, vel, ang, FRAMES, meshes)
        issues = []
        if r["tunnel"]:
            issues.append("TUNNEL %s" % r["tunnel"][:2])
        if r["deep"]:
            issues.append("DEEP %s" % r["deep"][:2])
        if r["matrix"]:
            issues.append("MATRIX %s" % r["matrix"][:2])
        if r["clamp"]:
            issues.append("CLAMP %s" % r["clamp"][:1])
        in_void = bool(r["edge"]) or bool(r["outside"])
        if r["nosettle"] and not in_void:
            issues.append("NO-SETTLE end=%s" % (r["end"],))
        status = "OK " if not issues else "BAD"
        if issues:
            problems += 1
        print("%s %-9s sleep=%-4s end=%-22s sub=%d %s" % (
            status, name, r["sleep_frame"], r["end"], r["max_substeps"],
            " | ".join(issues)))
    print()
    print("problem throws: %d / %d" % (problems, runnable))


THROWS = build_throws(load(LEVEL, 7500.0))
if __name__ == "__main__":
    main()
