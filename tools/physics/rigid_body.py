#!/usr/bin/env python3
"""Python reference for com.RigidBody.

A line-by-line transcription of the Q12 fixed point OBB solver in
src/com/RigidBody.java, used to verify the J2ME port. All integer
semantics match Java: multiplication keeps the arithmetic (floor) right
shift, while divisions truncate toward zero.
"""

F = 4096
MAX_CONTACTS = 24
VERTICES = 8
CONTACT_MARGIN = 64
SURFACE_TOUCH = 6
EDGE_MARGIN = 24
PENETRATION_THRESHOLD = 64
MIN_DT = 16
IMPULSE_ITERATIONS = 8
POSITION_ITERATIONS = 4
POSITION_SLOP = 3
MAX_LINEAR = 2048 << 12
MAX_ANGULAR = 2 * F
RESTITUTION_SPEED = 30 << 12
GRAVITY = 20 << 12
LINEAR_DRAG = 25
ANGULAR_DRAG = 20
RESTITUTION = 819
FRICTION = 4096
SLEEP_LOW = 120000
SLEEP_HIGH = 400000
SLEEP_TIME = 16


def tdiv(a, b):
    """Truncated integer division, like Java's a / b."""
    q = abs(a) // abs(b)
    return q if (a < 0) == (b < 0) else -q


def mul(a, b):
    return (a * b) >> 12


def divq(a, b):
    return tdiv(a << 12, b)


def isqrt(x):
    import math
    return math.isqrt(x) if x > 0 else 0


def abs_(v):
    return -v if v < 0 else v


class Collider:
    def __init__(self):
        self.verts = None
        self.pols = None
        self.norms = None
        self.quads = 0
        self.tris = 0
        self.scale8 = 256
        self.off_x = self.off_y = self.off_z = 0


def half_plane(pu, pv, au, av, bu, bv):
    return (bu - au) * (pv - av) <= (pu - au) * (bv - av)


def point_in_poly(pu, pv, au, av, bu, bv, cu, cv, du, dv, quad, flip):
    if not flip:
        if not half_plane(pu, pv, au, av, bu, bv): return False
        if not half_plane(pu, pv, bu, bv, cu, cv): return False
        if quad:
            if not half_plane(pu, pv, cu, cv, du, dv): return False
            if not half_plane(pu, pv, du, dv, au, av): return False
        else:
            if not half_plane(pu, pv, cu, cv, au, av): return False
    else:
        if not half_plane(pu, pv, cu, cv, bu, bv): return False
        if not half_plane(pu, pv, bu, bv, au, av): return False
        if quad:
            if not half_plane(pu, pv, au, av, du, dv): return False
            if not half_plane(pu, pv, du, dv, cu, cv): return False
        else:
            if not half_plane(pu, pv, au, av, cu, cv): return False
    return True


def edge_closest(px, py, pz, ax, ay, az, bx, by, bz):
    """Returns (distance_squared, closest_x, closest_y, closest_z)."""
    dx, dy, dz = bx - ax, by - ay, bz - az
    len_sq = dx * dx + dy * dy + dz * dz
    t = 0
    if len_sq != 0:
        t = (((px - ax) * dx + (py - ay) * dy + (pz - az) * dz) << 14) // len_sq
    if t < 0:
        t = 0
    elif t > 16384:
        t = 16384
    cx = ax + ((dx * t) >> 14)
    cy = ay + ((dy * t) >> 14)
    cz = az + ((dz * t) >> 14)
    ex, ey, ez = px - cx, py - cy, pz - cz
    return ex * ex + ey * ey + ez * ez, cx, cy, cz


class RigidBody:
    def __init__(self, half_extent_units):
        self.hx = self.hy = self.hz = half_extent_units << 12
        self.cpx = [0] * MAX_CONTACTS
        self.cpy = [0] * MAX_CONTACTS
        self.cpz = [0] * MAX_CONTACTS
        self.cnx = [0] * MAX_CONTACTS
        self.cny = [0] * MAX_CONTACTS
        self.cnz = [0] * MAX_CONTACTS
        self.cpen = [0] * MAX_CONTACTS
        self.reset(0, 0, 0)

    def reset(self, cx, cy, cz):
        self.mass = F
        self.inv_mass = F
        self.px, self.py, self.pz = cx << 12, cy << 12, cz << 12
        self.vx = self.vy = self.vz = 0
        self.lx = self.ly = self.lz = 0
        self.wx = self.wy = self.wz = 0
        self.r = [F, 0, 0, 0, F, 0, 0, 0, F]
        self.inv_i_local = [0] * 9
        self.inv_i_world = [0] * 9
        self._compute_local_inertia()
        self._recompute_world_inertia()
        self.sleeping = False
        self.sleep_counter = 0
        self.energy = 0
        self.last_substeps = 1
        self.num_contacts = 0
        self.max_penetration = 0
        self.ground_contact = False
        self._compute_vertices()

    def wake(self):
        self.sleeping = False
        self.sleep_counter = 0

    # accessors
    def get_center(self):
        return self.px >> 12, self.py >> 12, self.pz >> 12

    def get_velocity(self):
        return self.vx >> 12, self.vy >> 12, self.vz >> 12

    def get_half_extent(self):
        return self.hx >> 12

    def get_orientation(self, i):
        return self.r[i]

    def set_velocity(self, ux, uy, uz):
        self.vx, self.vy, self.vz = ux << 12, uy << 12, uz << 12
        self.wake()

    def set_angular_velocity(self, ax, ay, az):
        self.wx, self.wy, self.wz = ax, ay, az
        self._recompute_momentum()
        self.wake()

    def nudge(self, dx, dy, dz):
        self.px += dx << 12
        self.py += dy << 12
        self.pz += dz << 12
        self.vx += dx << 12
        self.vy += dy << 12
        self.vz += dz << 12
        self.wake()

    def move_kinematic(self, cx, cy, cz):
        nx, ny, nz = cx << 12, cy << 12, cz << 12
        self.vx, self.vy, self.vz = nx - self.px, ny - self.py, nz - self.pz
        self.px, self.py, self.pz = nx, ny, nz
        self.lx = self.ly = self.lz = 0
        self.wx = self.wy = self.wz = 0
        self.r = [F, 0, 0, 0, F, 0, 0, 0, F]
        self._recompute_world_inertia()
        self.wake()
        self._compute_vertices()

    def warp(self, m):
        x, y, z = self.px / F, self.py / F, self.pz / F
        self.px = int((m[0] * x + m[1] * y + m[2] * z + m[3]) * F)
        self.py = int((m[4] * x + m[5] * y + m[6] * z + m[7]) * F)
        self.pz = int((m[8] * x + m[9] * y + m[10] * z + m[11]) * F)

        def wvec(x, y, z):
            fx, fy, fz = x / F, y / F, z / F
            return (int((m[0] * fx + m[1] * fy + m[2] * fz) * F),
                    int((m[4] * fx + m[5] * fy + m[6] * fz) * F),
                    int((m[8] * fx + m[9] * fy + m[10] * fz) * F))

        self.vx, self.vy, self.vz = wvec(self.vx, self.vy, self.vz)
        self.wx, self.wy, self.wz = wvec(self.wx, self.wy, self.wz)

        for col in range(3):
            ax, ay, az = self.r[col] / F, self.r[3 + col] / F, self.r[6 + col] / F
            self.r[col] = int((m[0] * ax + m[1] * ay + m[2] * az) * F)
            self.r[3 + col] = int((m[4] * ax + m[5] * ay + m[6] * az) * F)
            self.r[6 + col] = int((m[8] * ax + m[9] * ay + m[10] * az) * F)
        self._fix_matrix()
        self._recompute_world_inertia()
        # angular momentum consistent with the rotated angular velocity
        self._recompute_momentum()
        self.wake()
        self._compute_vertices()

    # ---- simulation ----
    def step(self, colliders, count, world):
        if self.sleeping:
            self.vx = self.vy = self.vz = 0
            self.wx = self.wy = self.wz = 0
            self.lx = self.ly = self.lz = 0
            self._compute_vertices()
            self.energy = 0
            return

        fx = -tdiv(self.vx, LINEAR_DRAG)
        fy = -GRAVITY - tdiv(self.vy, LINEAR_DRAG)
        fz = -tdiv(self.vz, LINEAR_DRAG)
        mx = -tdiv(self.wx, ANGULAR_DRAG)
        my = -tdiv(self.wy, ANGULAR_DRAG)
        mz = -tdiv(self.wz, ANGULAR_DRAG)

        dt = F
        substeps = 1
        while True:
            self._backup()
            self._integrate(dt, fx, fy, fz, mx, my, mz)
            self._collide_world(colliders, count, world)
            if self.max_penetration > (PENETRATION_THRESHOLD << 12) and dt > MIN_DT:
                self._restore()
                dt >>= 1
                substeps += 1
                continue
            if self.num_contacts > 0:
                self._apply_impulses()
            break
        self.last_substeps = substeps

        # Safety clamps run on every frame (not only contact frames): a
        # pathological impact must never leave a runaway spin behind.
        self._clamp_velocity()

        self.energy = mul(self.vx, self.vx) + mul(self.vy, self.vy) + mul(self.vz, self.vz) \
            + mul(self.wx, self.wx) + mul(self.wy, self.wy) + mul(self.wz, self.wz)
        if not self.ground_contact:
            self.sleep_counter = 0
        elif self.energy >= SLEEP_HIGH:
            self.sleeping = False
            self.sleep_counter = 0
        elif self.energy <= SLEEP_LOW:
            self.sleep_counter += 1
            if self.sleep_counter >= SLEEP_TIME:
                self.sleeping = True
                self.vx = self.vy = self.vz = 0
                self.wx = self.wy = self.wz = 0
                self.lx = self.ly = self.lz = 0
        else:
            self.sleep_counter = 0

        self._compute_vertices()

    def _integrate(self, dt, fx, fy, fz, mx, my, mz):
        self.px += mul(self.vx, dt)
        self.py += mul(self.vy, dt)
        self.pz += mul(self.vz, dt)

        wxd, wyd, wzd = mul(self.wx, dt), mul(self.wy, dt), mul(self.wz, dt)
        skew = [0, -wzd, wyd, wzd, 0, -wxd, -wyd, wxd, 0]
        # In-place update, exactly like the C original and the Java port:
        # later rows read entries already modified by earlier rows.
        for row in range(3):
            for col in range(3):
                s0, s1, s2 = skew[row * 3], skew[row * 3 + 1], skew[row * 3 + 2]
                v = mul(s0, self.r[col]) + mul(s1, self.r[3 + col]) + mul(s2, self.r[6 + col])
                self.r[row * 3 + col] += v

        self.vx += divq(mul(fx, dt), self.mass)
        self.vy += divq(mul(fy, dt), self.mass)
        self.vz += divq(mul(fz, dt), self.mass)
        self.lx += mul(mx, dt)
        self.ly += mul(my, dt)
        self.lz += mul(mz, dt)

        self._fix_matrix()
        self._recompute_world_inertia()
        self.wx = self._eval24_x(self.inv_i_world, self.lx, self.ly, self.lz)
        self.wy = self._eval24_y(self.inv_i_world, self.lx, self.ly, self.lz)
        self.wz = self._eval24_z(self.inv_i_world, self.lx, self.ly, self.lz)

    def _compute_local_inertia(self):
        self.inv_i_local = [0] * 9
        x2, y2, z2 = mul(self.hx, self.hx), mul(self.hy, self.hy), mul(self.hz, self.hz)
        # inverse tensor in Q24 (Q12 is too coarse for 500 unit cubes)
        self.inv_i_local[0] = ((3 * F) << 24) // mul(self.mass, y2 + z2)
        self.inv_i_local[4] = ((3 * F) << 24) // mul(self.mass, x2 + z2)
        self.inv_i_local[8] = ((3 * F) << 24) // mul(self.mass, x2 + y2)
        self.il0 = tdiv(mul(self.mass, y2 + z2), 3)
        self.il4 = tdiv(mul(self.mass, x2 + z2), 3)
        self.il8 = tdiv(mul(self.mass, x2 + y2), 3)

    @staticmethod
    def _eval24_x(m, x, y, z):
        return (m[0] * x + m[1] * y + m[2] * z) >> 24

    @staticmethod
    def _eval24_y(m, x, y, z):
        return (m[3] * x + m[4] * y + m[5] * z) >> 24

    @staticmethod
    def _eval24_z(m, x, y, z):
        return (m[6] * x + m[7] * y + m[8] * z) >> 24

    def _recompute_momentum(self):
        iw = [0] * 9
        for row in range(3):
            for col in range(3):
                t0 = mul(self.r[row * 3], self.il0)
                t1 = mul(self.r[row * 3 + 1], self.il4)
                t2 = mul(self.r[row * 3 + 2], self.il8)
                iw[row * 3 + col] = mul(t0, self.r[col]) + mul(t1, self.r[col * 3 + 1]) + mul(t2, self.r[col * 3 + 2])
        self.lx = self._eval_x(iw, self.wx, self.wy, self.wz)
        self.ly = self._eval_y(iw, self.wx, self.wy, self.wz)
        self.lz = self._eval_z(iw, self.wx, self.wy, self.wz)

    def _recompute_world_inertia(self):
        for row in range(3):
            for col in range(3):
                t0 = mul(self.r[row * 3], self.inv_i_local[0])
                t1 = mul(self.r[row * 3 + 1], self.inv_i_local[4])
                t2 = mul(self.r[row * 3 + 2], self.inv_i_local[8])
                self.inv_i_world[row * 3 + col] = \
                    mul(t0, self.r[col]) + mul(t1, self.r[col * 3 + 1]) + mul(t2, self.r[col * 3 + 2])

    def _fix_matrix(self):
        xx, xy, xz = self.r[0], self.r[3], self.r[6]
        yx, yy, yz = self.r[1], self.r[4], self.r[7]

        # Stable Gram-Schmidt: normalize the x column, project the y column
        # onto it, then rebuild z as x cross y. The cross product makes the
        # frame exact even when the first order rotation update has driven
        # two raw columns nearly parallel (large per-frame spins after a
        # corner impact), where the old three-projection form degenerated.
        mx = self._norm3(xx, xy, xz)
        if mx == 0:
            self.r = [F, 0, 0, 0, F, 0, 0, 0, F]
            return
        xx, xy, xz = divq(xx, mx), divq(xy, mx), divq(xz, mx)

        d = mul(xx, yx) + mul(xy, yy) + mul(xz, yz)
        yx -= mul(xx, d); yy -= mul(xy, d); yz -= mul(xz, d)
        my = self._norm3(yx, yy, yz)
        if my == 0:
            self.r = [F, 0, 0, 0, F, 0, 0, 0, F]
            return
        yx, yy, yz = divq(yx, my), divq(yy, my), divq(yz, my)

        # z = x cross y (columns are x=(r0,r3,r6), y=(r1,r4,r7))
        zx = mul(xy, yz) - mul(xz, yy)
        zy = mul(xz, yx) - mul(xx, yz)
        zz = mul(xx, yy) - mul(xy, yx)

        self.r = [
            xx, yx, zx,
            xy, yy, zy,
            xz, yz, zz,
        ]

    # ---- contacts ----
    def _collide_world(self, colliders, count, world):
        self.num_contacts = 0
        self.max_penetration = 0
        self.ground_contact = False
        self._compute_vertices()
        if not world:
            return

        best_gap = [2 ** 31 - 1] * VERTICES
        best_nx = [0] * VERTICES
        best_ny = [0] * VERTICES
        best_nz = [0] * VERTICES
        best_pen = [0] * VERTICES

        band = CONTACT_MARGIN
        bmin_x, bmin_y, bmin_z = self.box_min
        bmax_x, bmax_y, bmax_z = self.box_max
        # swept broadphase against the pre-integration position
        bcr = self.corner_radius + CONTACT_MARGIN
        bx, by, bz = self._b[0] >> 12, self._b[1] >> 12, self._b[2] >> 12
        bmin_x = min(bmin_x, bx - bcr)
        bmin_y = min(bmin_y, by - bcr)
        bmin_z = min(bmin_z, bz - bcr)
        bmax_x = max(bmax_x, bx + bcr)
        bmax_y = max(bmax_y, by + bcr)
        bmax_z = max(bmax_z, bz + bcr)

        for ci in range(count):
            mesh = colliders[ci]
            verts, pols, norms = mesh.verts, mesh.pols, mesh.norms
            s8 = mesh.scale8
            ox, oy, oz = mesh.off_x, mesh.off_y, mesh.off_z

            x1 = tdiv((bmin_x - band - ox) << 8, s8) - 1
            y1 = tdiv((bmin_y - band - oy) << 8, s8) - 1
            z1 = tdiv((bmin_z - band - oz) << 8, s8) - 1
            x2 = tdiv((bmax_x + band - ox) << 8, s8) + 1
            y2 = tdiv((bmax_y + band - oy) << 8, s8) + 1
            z2 = tdiv((bmax_z + band - oz) << 8, s8) + 1

            p_idx, n_idx = 0, 0
            for vpp in (4, 3):
                p_end = mesh.quads * 4 if vpp == 4 else len(pols)
                while p_idx < p_end:
                    i1, i2, i3 = pols[p_idx] * 3, pols[p_idx + 1] * 3, pols[p_idx + 2] * 3
                    sax, say, saz = verts[i1], verts[i1 + 1], verts[i1 + 2]
                    sbx, sby, sbz = verts[i2], verts[i2 + 1], verts[i2 + 2]
                    scx, scy, scz = verts[i3], verts[i3 + 1], verts[i3 + 2]

                    mnx = min(sax, sbx, scx); mxx = max(sax, sbx, scx)
                    mny = min(say, sby, scy); mxy = max(say, sby, scy)
                    mnz = min(saz, sbz, scz); mxz = max(saz, sbz, scz)
                    sdx = sdy = sdz = 0
                    if vpp == 4:
                        i4 = pols[p_idx + 3] * 3
                        sdx, sdy, sdz = verts[i4], verts[i4 + 1], verts[i4 + 2]
                        mnx = min(mnx, sdx); mxx = max(mxx, sdx)
                        mny = min(mny, sdy); mxy = max(mxy, sdy)
                        mnz = min(mnz, sdz); mxz = max(mxz, sdz)
                    if mxx < x1 or mnx > x2 or mxy < y1 or mny > y2 or mxz < z1 or mnz > z2:
                        p_idx += vpp; n_idx += 1
                        continue

                    ax = ((sax * s8) >> 8) + ox
                    ay = ((say * s8) >> 8) + oy
                    az = ((saz * s8) >> 8) + oz
                    bx = ((sbx * s8) >> 8) + ox
                    by = ((sby * s8) >> 8) + oy
                    bz = ((sbz * s8) >> 8) + oz
                    cx = ((scx * s8) >> 8) + ox
                    cy = ((scy * s8) >> 8) + oy
                    cz = ((scz * s8) >> 8) + oz
                    dx = dy = dz = 0
                    if vpp == 4:
                        dx = ((sdx * s8) >> 8) + ox
                        dy = ((sdy * s8) >> 8) + oy
                        dz = ((sdz * s8) >> 8) + oz

                    mnx = norms[n_idx * 3]
                    mny = norms[n_idx * 3 + 1]
                    mnz = norms[n_idx * 3 + 2]
                    if mnx == 0 and mny == 0 and mnz == 0:
                        p_idx += vpp; n_idx += 1
                        continue
                    nx, ny, nz = -mnx, -mny, -mnz

                    axn, ayn, azn = abs_(mnx), abs_(mny), abs_(mnz)
                    dom_x = axn >= ayn and axn >= azn
                    dom_y = (not dom_x) and ayn >= axn and ayn >= azn
                    if dom_x:
                        au, av = az, ay; bu, bv = bz, by; cu, cv = cz, cy; du, dv = dz, dy
                        flip = mnx < 0
                    elif dom_y:
                        au, av = ax, az; bu, bv = bx, bz; cu, cv = cx, cz; du, dv = dx, dz
                        flip = mny < 0
                    else:
                        au, av = ax, ay; bu, bv = bx, by; cu, cv = cx, cy; du, dv = dx, dy
                        flip = mnz > 0

                    for k in range(VERTICES):
                        qx, qy, qz = self.vu[k * 3], self.vu[k * 3 + 1], self.vu[k * 3 + 2]
                        d = ((qx - ax) * mnx + (qy - ay) * mny + (qz - az) * mnz) >> 12
                        fx2 = qx - ((mnx * d) >> 12)
                        fy2 = qy - ((mny * d) >> 12)
                        fz2 = qz - ((mnz * d) >> 12)

                        if dom_x:
                            inside = point_in_poly(fz2, fy2, au, av, bu, bv, cu, cv, du, dv, vpp == 4, flip)
                        elif dom_y:
                            inside = point_in_poly(fx2, fz2, au, av, bu, bv, cu, cv, du, dv, vpp == 4, flip)
                        else:
                            inside = point_in_poly(fx2, fy2, au, av, bu, bv, cu, cv, du, dv, vpp == 4, flip)

                        if inside:
                            if d < -SURFACE_TOUCH:
                                continue
                            gap = -d
                            if gap < best_gap[k]:
                                best_gap[k] = gap
                                best_nx[k], best_ny[k], best_nz[k] = nx, ny, nz
                                best_pen[k] = d << 12 if d > 0 else 0
                        else:
                            cands = [
                                edge_closest(qx, qy, qz, ax, ay, az, bx, by, bz),
                                edge_closest(qx, qy, qz, bx, by, bz, cx, cy, cz),
                            ]
                            if vpp == 4:
                                cands.append(edge_closest(qx, qy, qz, cx, cy, cz, dx, dy, dz))
                                cands.append(edge_closest(qx, qy, qz, dx, dy, dz, ax, ay, az))
                            else:
                                cands.append(edge_closest(qx, qy, qz, cx, cy, cz, ax, ay, az))
                            best = min(cands, key=lambda c: c[0])
                            s2, ex, ey, ez = best
                            s = isqrt(s2)
                            if s <= EDGE_MARGIN and s < best_gap[k]:
                                best_gap[k] = s
                                if s > 0:
                                    best_nx[k] = ((qx - ex) << 12) // s
                                    best_ny[k] = ((qy - ey) << 12) // s
                                    best_nz[k] = ((qz - ez) << 12) // s
                                else:
                                    best_nx[k], best_ny[k], best_nz[k] = nx, ny, nz
                                best_pen[k] = 0

                    p_idx += vpp
                    n_idx += 1

        for k in range(VERTICES):
            if best_gap[k] <= CONTACT_MARGIN:
                self._add_contact(self.vq[k * 3], self.vq[k * 3 + 1], self.vq[k * 3 + 2],
                                  best_nx[k], best_ny[k], best_nz[k], best_pen[k])

    def _add_contact(self, x, y, z, nx, ny, nz, pen):
        if self.num_contacts >= MAX_CONTACTS:
            return
        i = self.num_contacts
        self.cpx[i], self.cpy[i], self.cpz[i] = x, y, z
        self.cnx[i], self.cny[i], self.cnz[i] = nx, ny, nz
        self.cpen[i] = pen
        if pen > self.max_penetration:
            self.max_penetration = pen
        if ny > F * 7 // 10:
            self.ground_contact = True
        self.num_contacts += 1

    # ---- impulses ----
    def _apply_impulses(self):
        acc_n = [0] * self.num_contacts
        acc_t = [0] * self.num_contacts
        v_bias = [0] * self.num_contacts

        # Prepass: restitution targets are fixed once from the approach
        # velocities before any impulse is applied, otherwise targets
        # computed mid-sweep are mutually inconsistent across contacts.
        for i in range(self.num_contacts):
            rx, ry, rz = self.cpx[i] - self.px, self.cpy[i] - self.py, self.cpz[i] - self.pz
            crx = mul(self.wy, rz) - mul(self.wz, ry)
            cry = mul(self.wz, rx) - mul(self.wx, rz)
            crz = mul(self.wx, ry) - mul(self.wy, rx)
            vn = mul(self.vx + crx, self.cnx[i]) + mul(self.vy + cry, self.cny[i]) \
                + mul(self.vz + crz, self.cnz[i])
            v_bias[i] = -mul(RESTITUTION, vn) if -vn > RESTITUTION_SPEED else 0

        # Sequential impulses with accumulated magnitudes: several
        # Gauss-Seidel sweeps let a multi-point face contact converge;
        # without them a resting box gets four independent impulses and
        # tumbles and jitters.
        for _ in range(IMPULSE_ITERATIONS):
            for i in range(self.num_contacts):
                rx, ry, rz = self.cpx[i] - self.px, self.cpy[i] - self.py, self.cpz[i] - self.pz
                nx, ny, nz = self.cnx[i], self.cny[i], self.cnz[i]

                crx = mul(self.wy, rz) - mul(self.wz, ry)
                cry = mul(self.wz, rx) - mul(self.wx, rz)
                crz = mul(self.wx, ry) - mul(self.wy, rx)
                rvx, rvy, rvz = self.vx + crx, self.vy + cry, self.vz + crz
                vn = mul(rvx, nx) + mul(rvy, ny) + mul(rvz, nz)

                rnx = mul(ry, nz) - mul(rz, ny)
                rny = mul(rz, nx) - mul(rx, nz)
                rnz = mul(rx, ny) - mul(ry, nx)
                irx = self._eval24_x(self.inv_i_world, rnx, rny, rnz)
                iry = self._eval24_y(self.inv_i_world, rnx, rny, rnz)
                irz = self._eval24_z(self.inv_i_world, rnx, rny, rnz)
                krx = mul(iry, rz) - mul(irz, ry)
                kry = mul(irz, rx) - mul(irx, rz)
                krz = mul(irx, ry) - mul(iry, rx)
                kn = self.inv_mass + mul(krx, nx) + mul(kry, ny) + mul(krz, nz)

                # Normal impulse; the restitution target comes from the
                # prepass and is enforced by every sweep.
                if kn > 0:
                    d_n = divq(v_bias[i] - vn, kn)
                    new_acc = max(0, acc_n[i] + d_n)
                    d_n = new_acc - acc_n[i]
                    acc_n[i] = new_acc
                    if d_n != 0:
                        imp = mul(d_n, self.inv_mass)
                        self.vx += mul(nx, imp)
                        self.vy += mul(ny, imp)
                        self.vz += mul(nz, imp)
                        self.lx += mul(ry, mul(nz, d_n)) - mul(rz, mul(ny, d_n))
                        self.ly += mul(rz, mul(nx, d_n)) - mul(rx, mul(nz, d_n))
                        self.lz += mul(rx, mul(ny, d_n)) - mul(ry, mul(nx, d_n))
                        self.wx = self._eval24_x(self.inv_i_world, self.lx, self.ly, self.lz)
                        self.wy = self._eval24_y(self.inv_i_world, self.lx, self.ly, self.lz)
                        self.wz = self._eval24_z(self.inv_i_world, self.lx, self.ly, self.lz)

                if acc_n[i] <= 0:
                    continue

                # Friction: tangent relative velocity, Coulomb clamp mu * jn
                crx = mul(self.wy, rz) - mul(self.wz, ry)
                cry = mul(self.wz, rx) - mul(self.wx, rz)
                crz = mul(self.wx, ry) - mul(self.wy, rx)
                rvx, rvy, rvz = self.vx + crx, self.vy + cry, self.vz + crz
                vnn = mul(rvx, nx) + mul(rvy, ny) + mul(rvz, nz)
                tx, ty, tz = rvx - mul(nx, vnn), rvy - mul(ny, vnn), rvz - mul(nz, vnn)
                tl = self._norm3(tx, ty, tz)
                if tl >= 1:
                    tx, ty, tz = divq(tx, tl), divq(ty, tl), divq(tz, tl)
                    rtx = mul(ry, tz) - mul(rz, ty)
                    rty = mul(rz, tx) - mul(rx, tz)
                    rtz = mul(rx, ty) - mul(ry, tx)
                    itx = self._eval24_x(self.inv_i_world, rtx, rty, rtz)
                    ity = self._eval24_y(self.inv_i_world, rtx, rty, rtz)
                    itz = self._eval24_z(self.inv_i_world, rtx, rty, rtz)
                    ktx = mul(ity, rz) - mul(itz, ry)
                    kty = mul(itz, rx) - mul(itx, rz)
                    ktz = mul(itx, ry) - mul(ity, rx)
                    kt = self.inv_mass + mul(ktx, tx) + mul(kty, ty) + mul(ktz, tz)
                    vt = mul(rvx, tx) + mul(rvy, ty) + mul(rvz, tz)
                    if kt > 0:
                        d_t = -divq(vt, kt)
                        max_fric = abs_(mul(FRICTION, acc_n[i]))
                        new_acc = acc_t[i] + d_t
                        if new_acc > max_fric: new_acc = max_fric
                        elif new_acc < -max_fric: new_acc = -max_fric
                        d_t = new_acc - acc_t[i]
                        acc_t[i] = new_acc
                        if d_t != 0:
                            imp = mul(d_t, self.inv_mass)
                            self.vx += mul(tx, imp)
                            self.vy += mul(ty, imp)
                            self.vz += mul(tz, imp)
                            self.lx += mul(ry, mul(tz, d_t)) - mul(rz, mul(ty, d_t))
                            self.ly += mul(rz, mul(tx, d_t)) - mul(rx, mul(tz, d_t))
                            self.lz += mul(rx, mul(ty, d_t)) - mul(ry, mul(tx, d_t))
                            self.wx = self._eval24_x(self.inv_i_world, self.lx, self.ly, self.lz)
                            self.wy = self._eval24_y(self.inv_i_world, self.lx, self.ly, self.lz)
                            self.wz = self._eval24_z(self.inv_i_world, self.lx, self.ly, self.lz)

        # De-penetration runs as its own position-only pass.
        self._correct_positions()

    def _angular_cross(self, vec, nx, ny, nz, dp):
        """invIWorld (Q24) applied to (r x n * dp), returns a Q12 vector."""
        ax = mul(vec[1], mul(nz, dp)) - mul(vec[2], mul(ny, dp))
        ay = mul(vec[2], mul(nx, dp)) - mul(vec[0], mul(nz, dp))
        az = mul(vec[0], mul(ny, dp)) - mul(vec[1], mul(nx, dp))
        return (self._eval24_x(self.inv_i_world, ax, ay, az),
                self._eval24_y(self.inv_i_world, ax, ay, az),
                self._eval24_z(self.inv_i_world, ax, ay, az))

    def _rotate_matrix(self, qx, qy, qz):
        # R += skew(q) * R in place (same first order update as integration)
        for row in range(3):
            if row == 0:
                s0, s1, s2 = 0, -qz, qy
            elif row == 1:
                s0, s1, s2 = qz, 0, -qx
            else:
                s0, s1, s2 = -qy, qx, 0
            for col in range(3):
                v = mul(s0, self.r[col]) + mul(s1, self.r[3 + col]) + mul(s2, self.r[6 + col])
                self.r[row * 3 + col] += v

    def _correct_positions(self):
        """
        Sequential position projection over every contact. Each iteration
        moves the center and rotates the body just like the velocity
        impulses do, but touches positions only, so it never injects
        momentum. Splitting the correction across all contacts (instead of
        one big snap on the deepest point) prevents a corner-jam energy
        pump.
        """
        beta = F // POSITION_ITERATIONS
        for _ in range(POSITION_ITERATIONS):
            for i in range(self.num_contacts):
                pen = self.cpen[i]
                if pen <= POSITION_SLOP << 12:
                    continue
                rx, ry, rz = self.cpx[i] - self.px, self.cpy[i] - self.py, self.cpz[i] - self.pz
                nx, ny, nz = self.cnx[i], self.cny[i], self.cnz[i]
                rnx = mul(ry, nz) - mul(rz, ny)
                rny = mul(rz, nx) - mul(rx, nz)
                rnz = mul(rx, ny) - mul(ry, nx)
                irx = self._eval24_x(self.inv_i_world, rnx, rny, rnz)
                iry = self._eval24_y(self.inv_i_world, rnx, rny, rnz)
                irz = self._eval24_z(self.inv_i_world, rnx, rny, rnz)
                krx = mul(iry, rz) - mul(irz, ry)
                kry = mul(irz, rx) - mul(irx, rz)
                krz = mul(irx, ry) - mul(iry, rx)
                k = self.inv_mass + mul(krx, nx) + mul(kry, ny) + mul(krz, nz)
                if k <= 0:
                    continue
                target = mul(pen - (POSITION_SLOP << 12), beta)
                dp = divq(target, k)
                self.px += mul(nx, mul(dp, self.inv_mass))
                self.py += mul(ny, mul(dp, self.inv_mass))
                self.pz += mul(nz, mul(dp, self.inv_mass))
                qx, qy, qz = self._angular_cross((rx, ry, rz), nx, ny, nz, dp)
                self._rotate_matrix(qx, qy, qz)
                self._recompute_world_inertia()
        self._fix_matrix()
        self._recompute_world_inertia()

    def _clamp_velocity(self):
        # Safety clamps keep pathological corner/edge contacts from
        # launching or over-spinning the body; momentum follows velocity.
        self.vx = max(-MAX_LINEAR, min(MAX_LINEAR, self.vx))
        self.vy = max(-MAX_LINEAR, min(MAX_LINEAR, self.vy))
        self.vz = max(-MAX_LINEAR, min(MAX_LINEAR, self.vz))
        self.wx = max(-MAX_ANGULAR, min(MAX_ANGULAR, self.wx))
        self.wy = max(-MAX_ANGULAR, min(MAX_ANGULAR, self.wy))
        self.wz = max(-MAX_ANGULAR, min(MAX_ANGULAR, self.wz))
        self._recompute_momentum()

    # ---- vertices ----
    def _compute_vertices(self):
        c0x, c0y, c0z = self.r[0], self.r[3], self.r[6]
        c1x, c1y, c1z = self.r[1], self.r[4], self.r[7]
        c2x, c2y, c2z = self.r[2], self.r[5], self.r[8]
        ex, ey, ez = self.hx, self.hy, self.hz

        self.vq = [0] * (VERTICES * 3)
        self.vu = [0] * (VERTICES * 3)
        mnx = mny = mnz = 2 ** 31 - 1
        mxx = mxy = mxz = -2 ** 31
        for k in range(VERTICES):
            sx = 1 if k in (1, 2, 6, 7) else -1
            sy = 1 if k in (4, 5, 6, 7) else -1
            sz = 1 if k in (2, 3, 4, 7) else -1
            ox3 = sx * mul(c0x, ex) + sy * mul(c1x, ey) + sz * mul(c2x, ez)
            oy3 = sx * mul(c0y, ex) + sy * mul(c1y, ey) + sz * mul(c2y, ez)
            oz3 = sx * mul(c0z, ex) + sy * mul(c1z, ey) + sz * mul(c2z, ez)
            wx, wy, wz = self.px + ox3, self.py + oy3, self.pz + oz3
            self.vq[k * 3], self.vq[k * 3 + 1], self.vq[k * 3 + 2] = wx, wy, wz
            ux, uy, uz = wx >> 12, wy >> 12, wz >> 12
            self.vu[k * 3], self.vu[k * 3 + 1], self.vu[k * 3 + 2] = ux, uy, uz
            mnx, mxx = min(mnx, ux), max(mxx, ux)
            mny, mxy = min(mny, uy), max(mxy, uy)
            mnz, mxz = min(mnz, uz), max(mxz, uz)
        self.box_min = (mnx, mny, mnz)
        self.box_max = (mxx, mxy, mxz)

        rxr = abs_(c0x) * (ex >> 12) + abs_(c1x) * (ey >> 12) + abs_(c2x) * (ez >> 12)
        ryr = abs_(c0y) * (ex >> 12) + abs_(c1y) * (ey >> 12) + abs_(c2y) * (ez >> 12)
        rzr = abs_(c0z) * (ex >> 12) + abs_(c1z) * (ey >> 12) + abs_(c2z) * (ez >> 12)
        cr = rxr >> 12
        if (ryr >> 12) > cr: cr = ryr >> 12
        if (rzr >> 12) > cr: cr = rzr >> 12
        self.corner_radius = cr

    # ---- rollback ----
    def _backup(self):
        self._b = (self.px, self.py, self.pz, self.vx, self.vy, self.vz,
                   self.lx, self.ly, self.lz, self.wx, self.wy, self.wz,
                   list(self.r), list(self.inv_i_world))

    def _restore(self):
        b = self._b
        (self.px, self.py, self.pz, self.vx, self.vy, self.vz,
         self.lx, self.ly, self.lz, self.wx, self.wy, self.wz,
         self.r, self.inv_i_world) = b
        self.inv_i_world = list(self.inv_i_world)

    # ---- math ----
    def _eval_x(self, m, x, y, z):
        return mul(m[0], x) + mul(m[1], y) + mul(m[2], z)

    def _eval_y(self, m, x, y, z):
        return mul(m[3], x) + mul(m[4], y) + mul(m[5], z)

    def _eval_z(self, m, x, y, z):
        return mul(m[6], x) + mul(m[7], y) + mul(m[8], z)

    @staticmethod
    def _norm3(x, y, z):
        return isqrt(x * x + y * y + z * z)
