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
EDGE_EDGE_MARGIN = 64
EDGE_SLOP = 16
# Residual depth (units) at which the end-of-frame safety rewind kicks
# in: the body is returned to its last provably collision-free pose
# instead of being left straddling a wall.
WEDGE_PENETRATION = 200
# A frame ending no deeper than this refreshes the last safe pose.
SAFE_POSE_PEN = 40
# Number of recent clean poses the wedge rewind may search back through.
SAFE_HISTORY = 12
# Largest rotation (Q12 radians) a single microstep may apply. The first
# order matrix update R += skew(w dt) R shears the basis badly beyond
# about a tenth of a radian; after re-orthonormalization the recomputed
# world inertia then amplifies angular velocity, so a fast spin must be
# subdivided exactly like a fast translating body.
MAX_MICRO_ROT = 320
PENETRATION_THRESHOLD = 64
MIN_DT = 16
# Rest damping: a supported, barely moving body has its velocities
# damped each solve, killing multi-frame micro-hop cycles on seamed
# floors that otherwise reset the sleep counter.
REST_SPEED = 14 * F
REST_DAMP = F // 4
# Rest-support test (see _support_is_stable): contact points must
# bracket the center projection within this slop, and only contacts at
# or below this height above the center count as supporting points.
SUPPORT_MARGIN = 120
SUPPORT_HIGH = 150
# Calm wedge-rewind frames without ground support after which a body stuck
# in an inescapable inter-room seam is put to sleep.
WEDGE_STREAK = 6
# The hover cycle before each rewind only regains a few frames of gravity;
# speeds above this mean the body is still genuinely in flight.
WEDGE_SLEEP_SPEED = 64 * F
# Pose based sleep: when every vertex stays within this many units of its
# position that many supported frames ago, the body is settled even if a
# degenerate single-contact seam keeps tiny residual velocity above the
# energy sleep threshold.
REST_POSE_SLOP = 8
REST_POSE_FRAMES = 12
# The 12 cube edges as vertex index pairs (see sign table in
# _compute_vertices), grouped by the axis they run along:
#   x edges (0,1)(2,3)(4,7)(5,6)
#   y edges (0,5)(1,6)(2,7)(3,4)
#   z edges (0,3)(1,2)(4,5)(6,7)
EDGES = 12
EDGE_A = (0, 2, 4, 5,  0, 1, 2, 3,  0, 1, 4, 6)
EDGE_B = (1, 3, 7, 6,  5, 6, 7, 4,  3, 2, 5, 7)
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
        self.min_separation = 2 ** 31 - 1
        self.micro_dt = F
        self.ground_contact = False
        # Restitution fires only on the first frame of a new contact.
        # There is no cross-frame warm starting, so without this gate a
        # cube sliding/spinning along a wall would receive a fresh bounce
        # every frame, each one injecting angular momentum at the contact
        # corner and pumping spin until the solver explodes.
        self.contact_last_frame = False
        self.restitution_open = True
        # Consecutive calm frames finished by a wedge rewind without
        # ground support: an inescapable inter-room seam; after a streak
        # the parked body is put to sleep rather than hover-replaying.
        self.wedge_streak = 0
        # pose history for position based sleeping
        self.rest_frames = 0
        self.rest_pose = None
        # History of the last provably collision-free poses (position +
        # orientation), used by the wedge safety rewind. The spawn pose is
        # assumed free (callers verify).
        self.safe_hist = [(self.px, self.py, self.pz, list(self.r))]
        # ---- body vs body state (see collide_bodies) ----
        # A carried cube is placed kinematically: it joins pair contacts as
        # an immovable obstacle that still lends its hand velocity.
        self.kinematic = False
        self.kvx = self.kvy = self.kvz = 0
        # Held up by another body instead of by the world; counts as ground
        # for the sleep bookkeeping, so a stack of cubes can come to rest.
        self.body_support = False
        self.support_body = None
        self.support_nx = 0
        self.support_ny = 0
        self.support_nz = 0
        self.prev_support = None
        self.pair_touched = False
        self._compute_vertices()

    def set_kinematic(self, kinematic):
        """Marks the body as carried: other bodies collide with it, it never
        reacts to them."""
        self.kinematic = kinematic
        if not kinematic:
            self.kvx = self.kvy = self.kvz = 0
        self.wake()

    def wake(self):
        self.sleeping = False
        self.sleep_counter = 0
        self.wedge_streak = 0
        self.rest_frames = 0
        self.rest_pose = None

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
        # the hand velocity other cubes are pushed with (see _body_vel)
        self.kvx, self.kvy, self.kvz = self.vx, self.vy, self.vz
        self.px, self.py, self.pz = nx, ny, nz
        self.lx = self.ly = self.lz = 0
        self.wx = self.wy = self.wz = 0
        self.r = [F, 0, 0, 0, F, 0, 0, 0, F]
        self._recompute_world_inertia()
        self.wake()
        self._compute_vertices()
        # kinematic placement (held cube) is sphereCast free: own it
        self.safe_hist = [(self.px, self.py, self.pz, list(self.r))]

    def set_kinematic_pose(self, cx, cy, cz, m):
        """Kinematic placement with an explicit camera relative orientation
        (row-major 4x4, as produced by an M3G Transform)."""
        nx, ny, nz = cx << 12, cy << 12, cz << 12
        self.kvx, self.kvy, self.kvz = nx - self.px, ny - self.py, nz - self.pz
        self.px, self.py, self.pz = nx, ny, nz
        self.vx = self.vy = self.vz = 0
        self.wx = self.wy = self.wz = 0
        self.lx = self.ly = self.lz = 0
        for row in range(3):
            for col in range(3):
                self.r[row * 3 + col] = int(m[row * 4 + col] * F)
        self._fix_matrix()
        self._recompute_world_inertia()
        self._recompute_momentum()
        self.wake()
        self._compute_vertices()
        self.safe_hist = [(self.px, self.py, self.pz, list(self.r))]

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
        # a portal jump is intentional: the destination is the new safe
        # pose, so a later wedge cannot rewind the cube back through it
        self.safe_hist = [(self.px, self.py, self.pz, list(self.r))]

    # ---- simulation ----
    def step(self, colliders, count, world):
        if self.sleeping:
            self.vx = self.vy = self.vz = 0
            self.wx = self.wy = self.wz = 0
            self.lx = self.ly = self.lz = 0
            self._compute_vertices()
            self.energy = 0
            return

        # Fastest point speed at the frame start: a body that was already
        # resting when a wedge rewind fires was parked in a concave seam
        # (its contacts flip on a one-unit drift), so the rewind must not
        # keep it awake forever.
        self._pre_speed = (self._norm3(self.vx, self.vy, self.vz)
                           + mul(self._norm3(self.wx, self.wy, self.wz),
                                 self.corner_radius))

        fx = -tdiv(self.vx, LINEAR_DRAG)
        fy = -GRAVITY - tdiv(self.vy, LINEAR_DRAG)
        fz = -tdiv(self.vz, LINEAR_DRAG)
        mx = -tdiv(self.wx, ANGULAR_DRAG)
        my = -tdiv(self.wy, ANGULAR_DRAG)
        mz = -tdiv(self.wz, ANGULAR_DRAG)

        # The frame is advanced in microsteps. A microstep that ends too
        # deep or that would leap past a nearby feature in one move is
        # rolled back and retried at half the length. Every accepted
        # microstep actually advances time, so the whole frame always
        # covers one full dt = F.
        # Bounce only on the first frame of a new contact (see reset).
        self.restitution_open = not self.contact_last_frame
        contacted_this_frame = False
        micro = F
        remaining = F
        depth = 0
        prev_deep = 0
        while remaining > 0:
            if micro > remaining:
                micro = remaining
            while True:
                self._backup()
                self._integrate(micro, fx, fy, fz, mx, my, mz)
                self.micro_dt = micro
                self._collide_world(colliders, count, world)
                # Subdivide for a feature this microstep would leap past
                # without a single contact to block it (wall-end spears).
                leap = (self.num_contacts == 0
                        and self.min_separation != 2 ** 31 - 1
                        and self.min_separation < self._micro_travel() + 2)
                # Subdivide a large rotation even with contacts present:
                # the first-order matrix update cannot take a tenth of a
                # radian in one step without shearing the basis and then
                # amplifying spin through the inertia tensor.
                rot_step = mul(self._norm3(self.wx, self.wy, self.wz), self.micro_dt)
                spin = rot_step > MAX_MICRO_ROT
                # Subdivide for freshly gained depth. If halving the step
                # barely changes the depth, the body was already embedded
                # before this frame started; rolling back further cannot
                # help, position projection resolves it instead.
                deep_units = self.max_penetration >> 12
                deep = (deep_units > PENETRATION_THRESHOLD
                        and (prev_deep == 0 or deep_units * 4 < prev_deep * 3))
                if (deep or leap or spin) and micro > MIN_DT:
                    self._restore()
                    prev_deep = deep_units
                    micro >>= 1
                    depth += 1
                    continue
                if self.num_contacts > 0:
                    self._apply_impulses()
                    contacted_this_frame = True
                    # only the first contact solve of a new impact bounces;
                    # later microsteps and frames resolve without restitution
                    self.restitution_open = False
                # Bound velocity before the next microstep integrates it,
                # so a pathological manifold can never launch the body
                # across the world inside a single frame.
                self._clamp_velocity()
                break
            remaining -= micro
            prev_deep = 0
        self.last_substeps = 1 << depth
        self.last_depth = depth
        self.contact_last_frame = contacted_this_frame

        # Cross-frame safety net. A fast throw can still plant the box
        # across a thin double-shell wall at an open end even after all
        # microsteps were accepted: the within-frame rollback only knows
        # the current frame's start pose, which may itself be embedded.
        # Remember the last pose that was provably outside the geometry
        # and, when a frame ends deeply wedged, rewind to it and drop the
        # velocity that drove the box in. A clean frame refreshes it.
        rewound = False
        if self.max_penetration > WEDGE_PENETRATION << 12:
            self._safety_rewind(colliders, count, world)
            rewound = True
        elif self.max_penetration <= SAFE_POSE_PEN << 12:
            self._remember_safe()

        # A calm, groundless body repeatedly rewound into the same
        # collision-free pose is caught in an inescapable inter-room seam
        # (the level has no valid surface beneath it): park it instead of
        # replaying the same fall and wedge forever. Any later push wakes
        # it again.
        if rewound and self._pre_speed <= WEDGE_SLEEP_SPEED \
                and not self.ground_contact:
            self.wedge_streak += 1
        elif self.ground_contact or self._pre_speed > WEDGE_SLEEP_SPEED:
            # supported, or still in genuine free flight: clear the streak
            self.wedge_streak = 0
        if self.wedge_streak >= WEDGE_STREAK:
            self.sleeping = True
            self.vx = self.vy = self.vz = 0
            self.wx = self.wy = self.wz = 0
            self.lx = self.ly = self.lz = 0

        # Safety clamps run on every frame (after any rewind): a
        # pathological impact must never leave a runaway spin behind.
        self._clamp_velocity()

        # A body held up by another body is only at rest once the pair pass
        # has cancelled the gravity its own step could not see, so its sleep
        # bookkeeping is skipped here and redone by collide_bodies (which
        # reads body_support from the pass that just ran). Every other body
        # is settled by now: the world contacts were solved inside this step.
        if not self.body_support:
            self._update_sleep()

        self._compute_vertices()
        self._update_pose_sleep()

    def _update_sleep(self):
        """Rest detection for a supported body: kinetic energy low for
        SLEEP_TIME consecutive frames puts it to sleep."""
        self.energy = mul(self.vx, self.vx) + mul(self.vy, self.vy) + mul(self.vz, self.vz) \
            + mul(self.wx, self.wx) + mul(self.wy, self.wy) + mul(self.wz, self.wz)
        # The counter accumulates on calm frames and only drains (never
        # resets instantly) on a restless one: a cube parked on a seam
        # between two coplanar floor faces receives an occasional
        # one-frame positional nudge, and an instant reset would keep it
        # awake forever even though it is plainly at rest. A genuinely
        # moving body holds high energy frame after frame and drains the
        # counter long before it could sleep.
        # A body held up by another body rests exactly like one held up by
        # the floor (see _record_support), otherwise a stack could never
        # come to rest and would keep re-resolving its own weight forever.
        if not self.ground_contact and not self.body_support:
            self.sleep_counter = 0
        elif self.energy <= SLEEP_LOW:
            self.sleep_counter += 1
            if self.sleep_counter >= SLEEP_TIME:
                self.sleeping = True
                self.vx = self.vy = self.vz = 0
                self.wx = self.wy = self.wz = 0
                self.lx = self.ly = self.lz = 0
        elif self.energy >= SLEEP_HIGH:
            self.sleeping = False
            self.sleep_counter = max(0, self.sleep_counter - 2)
        else:
            self.sleep_counter = max(0, self.sleep_counter - 1)

    def _update_pose_sleep(self):
        # Position based sleep: a supported body whose every vertex is
        # effectively parked (sub-slopped motion frame after frame) is at
        # rest even when a degenerate single-contact seam keeps residual
        # energy just above the energy threshold.
        if not (self.ground_contact or self.body_support) or self.sleeping:
            self.rest_frames = 0
            self.rest_pose = None
            return
        vq = tuple(self.vq)
        if self.rest_pose is None:
            self.rest_frames = 0
        else:
            moved = False
            for a, b in zip(self.rest_pose[1], vq):
                if abs(a - b) > REST_POSE_SLOP << 12:
                    moved = True
                    break
            center_dx = abs(self.px - self.rest_pose[0][0]) \
                + abs(self.py - self.rest_pose[0][1]) \
                + abs(self.pz - self.rest_pose[0][2])
            if moved or center_dx > REST_POSE_SLOP << 12:
                self.rest_frames = 0
            else:
                self.rest_frames += 1
                if self.rest_frames >= REST_POSE_FRAMES:
                    self.sleeping = True
                    self.vx = self.vy = self.vz = 0
                    self.wx = self.wy = self.wz = 0
                    self.lx = self.ly = self.lz = 0
                    self.rest_frames = 0
        # anchor the comparison every time the body moved enough; keep the
        # oldest still-quiet pose otherwise
        self.rest_pose = ((self.px, self.py, self.pz), vq)

    def _remember_safe(self):
        if self.safe_hist and self.safe_hist[-1][:3] == (self.px, self.py, self.pz):
            return
        self.safe_hist.append((self.px, self.py, self.pz, list(self.r)))
        if len(self.safe_hist) > SAFE_HISTORY:
            del self.safe_hist[0]

    def _micro_travel(self):
        # Max distance any point of the box covers in the current
        # microstep: translation plus angular motion at the corner.
        lin = self._norm3(self.vx, self.vy, self.vz)
        ang = mul(self._norm3(self.wx, self.wy, self.wz), self.corner_radius)
        speed = lin + ang
        return mul(speed, self.micro_dt) >> 12

    def _safety_rewind(self, colliders, count, world):
        """Return the body to its last provably collision-free pose.

        Triggered when a frame ends deeply wedged (straddling a thin
        double-shell wall at an open end). The within-frame rollback can
        only return to the start of the current frame, which may already
        be embedded; restoring a recent verified-clean pose instead
        always escapes. Recent poses are tested newest first and only
        accepted when the restored pose really is shallow, so a clean
        frame stored right before a corner impact can never position-
        project the cube deeper into that corner. The velocity that drove
        the body in is dropped; the lost motion is a few frames and only
        happens for genuinely pathological impacts.
        """
        self.vx = self.vy = self.vz = 0
        self.wx = self.wy = self.wz = 0
        self.lx = self.ly = self.lz = 0
        accepted = False
        for spx, spy, spz, sr in reversed(self.safe_hist):
            self.px, self.py, self.pz, self.r = spx, spy, spz, list(sr)
            self._recompute_world_inertia()
            self._compute_vertices()
            # Broadphase and backside cull are swept against the backup
            # pose; refresh it so they test the restored pose.
            self._backup()
            self._collide_world(colliders, count, world)
            if self.max_penetration <= SAFE_POSE_PEN << 12:
                accepted = True
                break
        if not accepted:
            # Every remembered pose is embedded: stay at the oldest one
            # and let ordinary position projection climb out of it.
            spx, spy, spz, sr = self.safe_hist[0]
            self.px, self.py, self.pz, self.r = spx, spy, spz, list(sr)
            self._fix_matrix()
            self._recompute_world_inertia()
            self._compute_vertices()
            self._backup()
            self._collide_world(colliders, count, world)
            if self.num_contacts > 0:
                self._correct_positions()
        if self._pre_speed >= REST_SPEED:
            self.sleeping = False
            self.sleep_counter = 0
        # a calm body parked in a geometrically tight seam keeps its
        # sleep counter, so the end-of-frame sleep bookkeeping can put it
        # to sleep instead of oscillating forever

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
        self.wx = self._evalI_x(self.inv_i_world, self.lx, self.ly, self.lz)
        self.wy = self._evalI_y(self.inv_i_world, self.lx, self.ly, self.lz)
        self.wz = self._evalI_z(self.inv_i_world, self.lx, self.ly, self.lz)

    def _compute_local_inertia(self):
        self.inv_i_local = [0] * 9
        x2, y2, z2 = mul(self.hx, self.hx), mul(self.hy, self.hy), mul(self.hz, self.hz)
        # inverse tensor in Q24 (Q12 is too coarse for 500 unit cubes)
        self.inv_i_local[0] = ((3 * F) << 28) // mul(self.mass, y2 + z2)
        self.inv_i_local[4] = ((3 * F) << 28) // mul(self.mass, x2 + z2)
        self.inv_i_local[8] = ((3 * F) << 28) // mul(self.mass, x2 + y2)
        self.il0 = tdiv(mul(self.mass, y2 + z2), 3)
        self.il4 = tdiv(mul(self.mass, x2 + z2), 3)
        self.il8 = tdiv(mul(self.mass, x2 + y2), 3)

    @staticmethod
    def _evalI_x(m, x, y, z):
        return (m[0] * x + m[1] * y + m[2] * z) >> 28

    @staticmethod
    def _evalI_y(m, x, y, z):
        return (m[3] * x + m[4] * y + m[5] * z) >> 28

    @staticmethod
    def _evalI_z(m, x, y, z):
        return (m[6] * x + m[7] * y + m[8] * z) >> 28

    def _recompute_momentum(self):
        # l = Iw w; Iw is built with the same column-major R indexing as
        # _recompute_world_inertia (see the note there).
        iw = [0] * 9
        for row in range(3):
            for col in range(3):
                t0 = mul(self.r[row], self.il0)
                t1 = mul(self.r[row + 3], self.il4)
                t2 = mul(self.r[row + 6], self.il8)
                iw[row * 3 + col] = mul(t0, self.r[col]) + mul(t1, self.r[col + 3]) + mul(t2, self.r[col + 6])
        self.lx = self._eval_x(iw, self.wx, self.wy, self.wz)
        self.ly = self._eval_y(iw, self.wx, self.wy, self.wz)
        self.lz = self._eval_z(iw, self.wx, self.wy, self.wz)

    def _recompute_world_inertia(self):
        # Rotate the local principal tensor into world space:
        # Iw = R Il R^T. The matrix is column-major (R[row][col] is
        # r[col*3+row]); using row-major indexing here silently produced
        # an anisotropic garbage tensor for any tilted orientation and
        # fed angular-energy pumps.
        for row in range(3):
            for col in range(3):
                t0 = mul(self.r[row], self.inv_i_local[0])
                t1 = mul(self.r[row + 3], self.inv_i_local[4])
                t2 = mul(self.r[row + 6], self.inv_i_local[8])
                self.inv_i_world[row * 3 + col] = \
                    mul(t0, self.r[col]) + mul(t1, self.r[col + 3]) \
                    + mul(t2, self.r[col + 6])

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
        self.min_separation = 2 ** 31 - 1
        self._compute_vertices()
        if not world:
            return

        best_gap = [2 ** 31 - 1] * VERTICES
        best_nx = [0] * VERTICES
        best_ny = [0] * VERTICES
        best_nz = [0] * VERTICES
        best_pen = [0] * VERTICES
        best_edge_gap = [2 ** 31 - 1] * EDGES
        best_epx = [0] * EDGES
        best_epy = [0] * EDGES
        best_epz = [0] * EDGES
        best_enx = [0] * EDGES
        best_eny = [0] * EDGES
        best_enz = [0] * EDGES

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

                    # Backside cull: shared walls between rooms carry two
                    # coincident faces with opposite normals. A face whose
                    # plane puts the pre-step cube center deeper than the
                    # box's maximal reach along the normal is on the far
                    # side of that wall; its "contacts" are false positives
                    # that would crush the cube between opposing normals.
                    # Use the pre-integration position: a face only counts
                    # as backside when the body could not reach it before
                    # the step (the post-step pose would hide faces the
                    # body just tunnelled through and defeat the rollback).
                    ccx, ccy, ccz = self._b[0] >> 12, self._b[1] >> 12, self._b[2] >> 12
                    dc = ((ccx - ax) * mnx + (ccy - ay) * mny
                          + (ccz - az) * mnz) >> 12
                    if dom_x:
                        reach = (mul(abs_(self.r[0]), self.hx)
                                 + mul(abs_(self.r[3]), self.hy)
                                 + mul(abs_(self.r[6]), self.hz)) >> 12
                    elif dom_y:
                        reach = (mul(abs_(self.r[1]), self.hx)
                                 + mul(abs_(self.r[4]), self.hy)
                                 + mul(abs_(self.r[7]), self.hz)) >> 12
                    else:
                        reach = (mul(abs_(self.r[2]), self.hx)
                                 + mul(abs_(self.r[5]), self.hy)
                                 + mul(abs_(self.r[8]), self.hz)) >> 12
                    if dc > reach + CONTACT_MARGIN:
                        p_idx += vpp; n_idx += 1
                        continue

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
                            if d < 0 and gap < self.min_separation:
                                self.min_separation = gap
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
                            if s < self.min_separation:
                                self.min_separation = s
                            if s <= EDGE_MARGIN and s < best_gap[k]:
                                best_gap[k] = s
                                if s > 0:
                                    best_nx[k] = tdiv((qx - ex) << 12, s)
                                    best_ny[k] = tdiv((qy - ey) << 12, s)
                                    best_nz[k] = tdiv((qz - ez) << 12, s)
                                else:
                                    best_nx[k], best_ny[k], best_nz[k] = nx, ny, nz
                                best_pen[k] = 0

                    # Edge vs edge: the end edge of a thin wall can spear a
                    # cube face while every cube vertex sits outside every
                    # polygon; the vertex-vs-face tests alone miss that.
                    eg = (best_edge_gap, best_epx, best_epy, best_epz,
                          best_enx, best_eny, best_enz)
                    self._edge_vs_cube_edges(ax, ay, az, bx, by, bz,
                                             nx, ny, nz, *eg)
                    self._edge_vs_cube_edges(bx, by, bz, cx, cy, cz,
                                             nx, ny, nz, *eg)
                    if vpp == 4:
                        self._edge_vs_cube_edges(cx, cy, cz, dx, dy, dz,
                                                 nx, ny, nz, *eg)
                        self._edge_vs_cube_edges(dx, dy, dz, ax, ay, az,
                                                 nx, ny, nz, *eg)
                    else:
                        self._edge_vs_cube_edges(cx, cy, cz, ax, ay, az,
                                                 nx, ny, nz, *eg)

                    p_idx += vpp
                    n_idx += 1

        for k in range(VERTICES):
            if best_gap[k] <= CONTACT_MARGIN:
                self._add_contact(self.vq[k * 3], self.vq[k * 3 + 1], self.vq[k * 3 + 2],
                                  best_nx[k], best_ny[k], best_nz[k], best_pen[k])
        for e in range(EDGES):
            if best_edge_gap[e] <= EDGE_EDGE_MARGIN:
                pen = max(0, EDGE_SLOP - best_edge_gap[e]) << 12
                self._add_contact(best_epx[e], best_epy[e], best_epz[e],
                                  best_enx[e], best_eny[e], best_enz[e], pen)

        # Shared walls between rooms carry two coincident faces with
        # opposite normals. When the box straddles such a wall both faces
        # report a deep contact; the opposing corrections cancel (leaving
        # the box embedded frame after frame) and the two impulse channels
        # pump angular energy. Collapse each such coincident, opposed pair
        # onto its deeper member. Genuine floor/ceiling or corridor-wall
        # sandwiches have contact points a body width apart and are kept.
        DUP_DIST = (300 << 12)
        alive = [True] * self.num_contacts
        for i in range(self.num_contacts):
            for j in range(i):
                if not (alive[i] and alive[j]):
                    continue
                dot = (mul(self.cnx[i], self.cnx[j])
                       + mul(self.cny[i], self.cny[j])
                       + mul(self.cnz[i], self.cnz[j]))
                if dot > -3 * F // 4:
                    continue
                dx = self.cpx[i] - self.cpx[j]
                dy = self.cpy[i] - self.cpy[j]
                dz = self.cpz[i] - self.cpz[j]
                if dx * dx + dy * dy + dz * dz > DUP_DIST * DUP_DIST:
                    continue
                # same double-shell face: drop the shallower contact
                if self.cpen[i] >= self.cpen[j]:
                    alive[j] = False
                else:
                    alive[i] = False
                    break
        if not all(alive):
            keep = [i for i in range(self.num_contacts) if alive[i]]
            self._compact_contacts(keep)

    def _compact_contacts(self, keep):
        n = len(keep)
        for slot, i in enumerate(keep):
            self.cpx[slot] = self.cpx[i]
            self.cpy[slot] = self.cpy[i]
            self.cpz[slot] = self.cpz[i]
            self.cnx[slot] = self.cnx[i]
            self.cny[slot] = self.cny[i]
            self.cnz[slot] = self.cnz[i]
            self.cpen[slot] = self.cpen[i]
        for slot in range(n, self.num_contacts):
            self.cpen[slot] = 0
        self.num_contacts = n
        self.max_penetration = 0
        self.ground_contact = False
        for slot in range(n):
            if self.cpen[slot] > self.max_penetration:
                self.max_penetration = self.cpen[slot]
            if self.cny[slot] > F * 7 // 10:
                self.ground_contact = True

    @staticmethod
    def _clamp_param(v):
        if v < 0:
            return 0
        if v > F:
            return F
        return v

    def _edge_vs_cube_edges(self, q0x, q0y, q0z, q1x, q1y, q1z,
                            fnx, fny, fnz, best_gap,
                            epx, epy, epz, enx, eny, enz):
        d2x, d2y, d2z = q1x - q0x, q1y - q0y, q1z - q0z
        c = d2x * d2x + d2y * d2y + d2z * d2z
        if c == 0:
            return
        for e in range(EDGES):
            a0 = EDGE_A[e] * 3
            a1 = EDGE_B[e] * 3
            a0x, a0y, a0z = self.vu[a0], self.vu[a0 + 1], self.vu[a0 + 2]
            a1x, a1y, a1z = self.vu[a1], self.vu[a1 + 1], self.vu[a1 + 2]
            d1x, d1y, d1z = a1x - a0x, a1y - a0y, a1z - a0z
            rx, ry, rz = a0x - q0x, a0y - q0y, a0z - q0z

            a = d1x * d1x + d1y * d1y + d1z * d1z
            b = d1x * d2x + d1y * d2y + d1z * d2z
            dd = d1x * rx + d1y * ry + d1z * rz
            ee = d2x * rx + d2y * ry + d2z * rz
            denom = a * c - b * b  # |d1 x d2|^2 (Lagrange identity)
            if a == 0 or denom <= (a * c >> 4):
                continue  # near-parallel edges

            s = self._clamp_param((b * ee - c * dd) * F // denom)
            t = self._clamp_param((b * s - ee * F) // c)
            s = self._clamp_param((b * t - dd * F) // a)
            t = self._clamp_param((b * s - ee * F) // c)

            px = a0x + ((d1x * s) >> 12)
            py = a0y + ((d1y * s) >> 12)
            pz = a0z + ((d1z * s) >> 12)
            qx = q0x + ((d2x * t) >> 12)
            qy = q0y + ((d2y * t) >> 12)
            qz = q0z + ((d2z * t) >> 12)

            dx, dy, dz = px - qx, py - qy, pz - qz
            dist = isqrt(dx * dx + dy * dy + dz * dz)
            if dist < self.min_separation:
                self.min_separation = dist
            if dist > EDGE_EDGE_MARGIN or dist >= best_gap[e]:
                continue

            if dist > 0:
                # cube edge point minus polygon edge point points out of
                # the solid; division truncates toward zero like Java
                exn = tdiv(dx << 12, dist)
                eyn = tdiv(dy << 12, dist)
                ezn = tdiv(dz << 12, dist)
            else:
                # segments intersect: d1 x d2, flipped towards the face's
                # outward side
                cx2 = d1y * d2z - d1z * d2y
                cy2 = d1z * d2x - d1x * d2z
                cz2 = d1x * d2y - d1y * d2x
                nl = isqrt(cx2 * cx2 + cy2 * cy2 + cz2 * cz2)
                if nl == 0:
                    continue
                exn = tdiv(cx2 << 12, nl)
                eyn = tdiv(cy2 << 12, nl)
                ezn = tdiv(cz2 << 12, nl)
                if exn * fnx + eyn * fny + ezn * fnz < 0:
                    exn, eyn, ezn = -exn, -eyn, -ezn

            best_gap[e] = dist
            epx[e], epy[e], epz[e] = px << 12, py << 12, pz << 12
            enx[e], eny[e], enz[e] = exn, eyn, ezn

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
            v_bias[i] = (-mul(RESTITUTION, vn)
                         if (-vn > RESTITUTION_SPEED and self.restitution_open)
                         else 0)

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
                irx = self._evalI_x(self.inv_i_world, rnx, rny, rnz)
                iry = self._evalI_y(self.inv_i_world, rnx, rny, rnz)
                irz = self._evalI_z(self.inv_i_world, rnx, rny, rnz)
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
                        self.wx = self._evalI_x(self.inv_i_world, self.lx, self.ly, self.lz)
                        self.wy = self._evalI_y(self.inv_i_world, self.lx, self.ly, self.lz)
                        self.wz = self._evalI_z(self.inv_i_world, self.lx, self.ly, self.lz)

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
                    itx = self._evalI_x(self.inv_i_world, rtx, rty, rtz)
                    ity = self._evalI_y(self.inv_i_world, rtx, rty, rtz)
                    itz = self._evalI_z(self.inv_i_world, rtx, rty, rtz)
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
                            self.wx = self._evalI_x(self.inv_i_world, self.lx, self.ly, self.lz)
                            self.wy = self._evalI_y(self.inv_i_world, self.lx, self.ly, self.lz)
                            self.wz = self._evalI_z(self.inv_i_world, self.lx, self.ly, self.lz)

        # De-penetration runs as its own position-only pass.
        self._correct_positions()

        # Rest damping: a supported, barely moving body has its velocities
        # damped each solve. This kills multi-frame micro-hop cycles on a
        # triangulated/seamed floor (which otherwise reset the sleep
        # counter) while positional projection keeps settling the body.
        # Only damp when the center of mass sits inside the contact
        # support polygon: a cube balanced on a single corner or on a
        # tilted edge is an unstable state that still has to tip over to
        # a face, and damping must not freeze it there.
        if self.ground_contact and self._support_is_stable():
            speed = self._norm3(self.vx, self.vy, self.vz) \
                + mul(self._norm3(self.wx, self.wy, self.wz), self.corner_radius)
            if speed < REST_SPEED:
                self.vx = mul(self.vx, REST_DAMP)
                self.vy = mul(self.vy, REST_DAMP)
                self.vz = mul(self.vz, REST_DAMP)
                self.wx = mul(self.wx, REST_DAMP)
                self.wy = mul(self.wy, REST_DAMP)
                self.wz = mul(self.wz, REST_DAMP)
                self._recompute_momentum()

    def _support_is_stable(self):
        """True when contacts bracket the center against gravity.

        The horizontal projection of the center of mass must be enclosed
        by contact points on both sides along both horizontal axes
        (contacts at or below the center height count as support). This
        accepts a cube resting on a face (four corners), a cube nestled
        in a floor/wall corner (wall plus floor points surround the
        center) or resting on any face after tumbling, but rejects a
        single-corner or single-edge balance, whose support points lie to
        one side so the cube still has to tip over. Orientation alone is
        not enough: a tilted cube can be held stable by a wall, while an
        axis-aligned cube could in principle perch on one corner.
        """
        margin = SUPPORT_MARGIN << 12
        high = (SUPPORT_HIGH << 12)
        minx = minz = 2 ** 31 - 1
        maxx = maxz = -2 ** 31
        n = 0
        for i in range(self.num_contacts):
            oy = self.cpy[i] - self.py
            if oy > high:
                continue
            n += 1
            ox = self.cpx[i] - self.px
            oz = self.cpz[i] - self.pz
            if ox < minx:
                minx = ox
            if ox > maxx:
                maxx = ox
            if oz < minz:
                minz = oz
            if oz > maxz:
                maxz = oz
        return n >= 2 and (minx <= margin and maxx >= -margin
                           and minz <= margin and maxz >= -margin)

    def _angular_cross(self, vec, nx, ny, nz, dp):
        """invIWorld (Q24) applied to (r x n * dp), returns a Q12 vector."""
        ax = mul(vec[1], mul(nz, dp)) - mul(vec[2], mul(ny, dp))
        ay = mul(vec[2], mul(nx, dp)) - mul(vec[0], mul(nz, dp))
        az = mul(vec[0], mul(ny, dp)) - mul(vec[1], mul(nx, dp))
        return (self._evalI_x(self.inv_i_world, ax, ay, az),
                self._evalI_y(self.inv_i_world, ax, ay, az),
                self._evalI_z(self.inv_i_world, ax, ay, az))

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
                irx = self._evalI_x(self.inv_i_world, rnx, rny, rnz)
                iry = self._evalI_y(self.inv_i_world, rnx, rny, rnz)
                irz = self._evalI_z(self.inv_i_world, rnx, rny, rnz)
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


# ============================================================== body vs body
# Cube-vs-cube contacts.
#
# The world pass above only ever sees one box against the static triangle
# soup, so two dynamic boxes are resolved in a separate pass over every pair
# (collide_bodies), run once per frame after all bodies have stepped:
#
#   - a separating axis test over the 15 box-box axes (6 face normals plus 9
#     edge cross products) rejects non-touching pairs and picks the axis of
#     least penetration,
#   - a face axis resolves by clipping the incident face against the
#     reference face (Sutherland-Hodgman, deepest PAIR_MAX_CONTACTS kept),
#   - an edge axis resolves to the single closest point pair of the two
#     extreme edges,
#   - contacts are solved with sequential impulses applied to BOTH bodies
#     (equal and opposite, so linear and angular momentum are conserved)
#     plus a position projection that re-derives the penetration from local
#     contact anchors on every iteration, so a stack converges instead of
#     settling at a fixed residual overlap.
#
# A carried (kinematic) body and a sleeping body take part with zero inverse
# mass: they act as immovable obstacles that still lend their own velocity to
# the contact. A sleeper is woken by a hard impact or by a moving neighbour,
# and a body resting on another body may sleep as well (body_support),
# otherwise a stack could never come to rest.

PAIR_MAX_CONTACTS = 4
# Clipping keeps a point this far outside the reference face footprint (Q12
# units) so a contact exactly on a face edge is not lost to truncation.
CLIP_SLOP = 4 << 12
# An edge-edge axis wins over the best face axis only when it is this much
# shallower (Q12 units): a face manifold is far more stable, so near ties go
# to the face case.
EDGE_AXIS_BIAS = 8 << 12
PAIR_IMPULSE_ITERATIONS = 8
PAIR_POSITION_ITERATIONS = 6
# Walks over the whole pair list per frame, see collide_bodies.
PAIR_ROUNDS = 3
# Position projection strength by manifold size. The penetration is re-derived
# from the local anchors on every iteration, so a single contact needs a much
# bigger step than a four point face manifold to converge in the same number
# of sweeps; without this an edge-edge impact keeps a third of its depth.
PAIR_BETA = (F // 2, F * 3 // 8, F * 5 // 16, F * 3 // 16)
# Cube against cube: less bouncy than the world material (a stack must not
# ping-pong) but just as grippy, so cubes can rest on each other.
BODY_RESTITUTION = 614
BODY_FRICTION = 4096
# A sleeping body is woken by a contact closing faster than this.
WAKE_SPEED = 60 << 12
# A sleeping body is also woken when the body it leans on moves faster than
# this, otherwise it would hang in the air while its support slides away.
WAKE_NEIGHBOUR_SPEED = 48 << 12
# Cross product length (Q24) below which two box axes count as parallel and
# their edge-edge axis is skipped (~0.9 degrees): the face axes already
# separate boxes aligned that closely.
PARALLEL_EPS = 1 << 18
# Same-normal pair contacts closer than this (squared, Q12) are one contact;
# clipping can emit a corner twice when an incident edge lies exactly on a
# clip plane.
PAIR_MERGE_DIST2 = (8 << 12) * (8 << 12)
PAIR_MERGE_DOT = F * 3 // 4

# Upward component a pair contact needs to count as support. Wider than the
# world pass' ground cone (F * 7 / 10): a cube balanced on the seam between
# two cubes is held up by contacts whose normals are tilted well past 45
# degrees in the frame of either cube, and a body that is not recognised as
# supported may never sleep - it would keep re-resolving its own weight
# forever and jitter instead of coming to rest.
SUPPORT_UP = F // 2
# Zero inverse inertia, for bodies that must not rotate (carried, asleep).
ZERO_I = [0] * 9

# ---- pair scratch: exactly one pair is generated and solved at a time ----
_pcount = 0
_ppx = [0] * PAIR_MAX_CONTACTS
_ppy = [0] * PAIR_MAX_CONTACTS
_ppz = [0] * PAIR_MAX_CONTACTS
_pnx = [0] * PAIR_MAX_CONTACTS
_pny = [0] * PAIR_MAX_CONTACTS
_pnz = [0] * PAIR_MAX_CONTACTS
_ppen = [0] * PAIR_MAX_CONTACTS
# contact anchors in each body's local frame, see _add_pair_contact
_pral = [0] * (PAIR_MAX_CONTACTS * 3)
_prbl = [0] * (PAIR_MAX_CONTACTS * 3)
_paccn = [0] * PAIR_MAX_CONTACTS
_pacct = [0] * PAIR_MAX_CONTACTS
_pbias = [0] * PAIR_MAX_CONTACTS
# clipping polygon buffers: 4 incident corners, at most one added per plane
_CLIP_MAX = 10
_qx = [0] * _CLIP_MAX
_qy = [0] * _CLIP_MAX
_qz = [0] * _CLIP_MAX
_ox = [0] * _CLIP_MAX
_oy = [0] * _CLIP_MAX
_oz = [0] * _CLIP_MAX


def _axis_x(b, i):
    """World space box axis i (column i of the row-major orientation)."""
    return b.r[i]


def _axis_y(b, i):
    return b.r[3 + i]


def _axis_z(b, i):
    return b.r[6 + i]


def _half(b, i):
    return b.hx if i == 0 else (b.hy if i == 1 else b.hz)


def _body_vel(x):
    """Velocity a body lends to a contact: a carried cube moves with the hand
    even though its own simulated velocity is kept at zero."""
    if x.kinematic:
        return (x.kvx, x.kvy, x.kvz)
    return (x.vx, x.vy, x.vz)


def _body_speed(x):
    """Fastest point speed of a body (translation plus spin at the corner)."""
    vx, vy, vz = _body_vel(x)
    return (RigidBody._norm3(vx, vy, vz)
            + mul(RigidBody._norm3(x.wx, x.wy, x.wz), x.corner_radius))


def _clamp14(t):
    if t < 0:
        return 0
    if t > 16384:
        return 16384
    return t


def _overlap_on_axis(a, b, lx, ly, lz):
    """Overlap of the two boxes along a unit axis (Q12); <= 0 separates."""
    pa = pb = 0
    for i in range(3):
        pa += mul(_half(a, i), abs_(mul(_axis_x(a, i), lx)
                                    + mul(_axis_y(a, i), ly)
                                    + mul(_axis_z(a, i), lz)))
        pb += mul(_half(b, i), abs_(mul(_axis_x(b, i), lx)
                                    + mul(_axis_y(b, i), ly)
                                    + mul(_axis_z(b, i), lz)))
    d = mul(b.px - a.px, lx) + mul(b.py - a.py, ly) + mul(b.pz - a.pz, lz)
    return pa + pb - abs_(d)


def _add_pair_contact(a, b, x, y, z, nx, ny, nz, pen):
    """Stores one contact whose normal points from a to b, keeping the
    PAIR_MAX_CONTACTS deepest points."""
    global _pcount
    for i in range(_pcount):
        dot = mul(nx, _pnx[i]) + mul(ny, _pny[i]) + mul(nz, _pnz[i])
        if dot < PAIR_MERGE_DOT:
            continue
        dx = x - _ppx[i]
        dy = y - _ppy[i]
        dz = z - _ppz[i]
        if dx * dx + dy * dy + dz * dz <= PAIR_MERGE_DIST2:
            if pen > _ppen[i]:
                _ppen[i] = pen
            return
    if _pcount < PAIR_MAX_CONTACTS:
        slot = _pcount
        _pcount += 1
    else:
        slot = 0
        for i in range(1, PAIR_MAX_CONTACTS):
            if _ppen[i] < _ppen[slot]:
                slot = i
        if pen <= _ppen[slot]:
            return
    _ppx[slot], _ppy[slot], _ppz[slot] = x, y, z
    _pnx[slot], _pny[slot], _pnz[slot] = nx, ny, nz
    _ppen[slot] = pen
    # Anchors in each body's local frame (local = R^T * world offset). The
    # position projection re-derives the current penetration from them, so
    # it converges while the bodies move instead of pushing out a stale
    # depth once per iteration.
    wx, wy, wz = x - a.px, y - a.py, z - a.pz
    _pral[slot * 3] = mul(wx, a.r[0]) + mul(wy, a.r[3]) + mul(wz, a.r[6])
    _pral[slot * 3 + 1] = mul(wx, a.r[1]) + mul(wy, a.r[4]) + mul(wz, a.r[7])
    _pral[slot * 3 + 2] = mul(wx, a.r[2]) + mul(wy, a.r[5]) + mul(wz, a.r[8])
    wx, wy, wz = x - b.px, y - b.py, z - b.pz
    _prbl[slot * 3] = mul(wx, b.r[0]) + mul(wy, b.r[3]) + mul(wz, b.r[6])
    _prbl[slot * 3 + 1] = mul(wx, b.r[1]) + mul(wy, b.r[4]) + mul(wz, b.r[7])
    _prbl[slot * 3 + 2] = mul(wx, b.r[2]) + mul(wy, b.r[5]) + mul(wz, b.r[8])


def _clip_polygon(R, ax, ay, az, limit, cnt):
    """Sutherland-Hodgman clip of the scratch polygon against one side plane
    of the reference face: keeps (p - R.center) . axis <= limit (+ slop)."""
    out = 0
    for i in range(cnt):
        if out >= _CLIP_MAX:
            break
        j = i + 1 if i + 1 < cnt else 0
        ds = (mul(_qx[i] - R.px, ax) + mul(_qy[i] - R.py, ay)
              + mul(_qz[i] - R.pz, az) - limit)
        de = (mul(_qx[j] - R.px, ax) + mul(_qy[j] - R.py, ay)
              + mul(_qz[j] - R.pz, az) - limit)
        s_in = ds <= CLIP_SLOP
        e_in = de <= CLIP_SLOP
        if e_in:
            if not s_in and out < _CLIP_MAX:
                ds -= CLIP_SLOP
                de -= CLIP_SLOP
                t = divq(ds, ds - de)
                _ox[out] = _qx[i] + mul(_qx[j] - _qx[i], t)
                _oy[out] = _qy[i] + mul(_qy[j] - _qy[i], t)
                _oz[out] = _qz[i] + mul(_qz[j] - _qz[i], t)
                out += 1
            if out < _CLIP_MAX:
                _ox[out], _oy[out], _oz[out] = _qx[j], _qy[j], _qz[j]
                out += 1
        elif s_in and out < _CLIP_MAX:
            ds -= CLIP_SLOP
            de -= CLIP_SLOP
            t = divq(ds, ds - de)
            _ox[out] = _qx[i] + mul(_qx[j] - _qx[i], t)
            _oy[out] = _qy[i] + mul(_qy[j] - _qy[i], t)
            _oz[out] = _qz[i] + mul(_qz[j] - _qz[i], t)
            out += 1
    for i in range(out):
        _qx[i], _qy[i], _qz[i] = _ox[i], _oy[i], _oz[i]
    return out


def _clip_face_pair(a, b, R, I, k, s, ref_is_a):
    """Face manifold: clip the incident face of I against the face of R whose
    outward normal is s * R.axis[k] (that normal points from R toward I)."""
    nx, ny, nz = s * _axis_x(R, k), s * _axis_y(R, k), s * _axis_z(R, k)
    u = (k + 1) % 3
    v = (k + 2) % 3
    ux, uy, uz = _axis_x(R, u), _axis_y(R, u), _axis_z(R, u)
    vx, vy, vz = _axis_x(R, v), _axis_y(R, v), _axis_z(R, v)
    hn, hu, hv = _half(R, k), _half(R, u), _half(R, v)

    # incident face: the face of I most anti-parallel to the reference normal
    best_j, best_s, best_dot = 0, -1, 2 ** 31 - 1
    for j in range(3):
        d = (mul(_axis_x(I, j), nx) + mul(_axis_y(I, j), ny)
             + mul(_axis_z(I, j), nz))
        if -d < best_dot:
            best_dot, best_j, best_s = -d, j, -1
        if d < best_dot:
            best_dot, best_j, best_s = d, j, 1
    iu = (best_j + 1) % 3
    iv = (best_j + 2) % 3
    hj, hu2, hv2 = _half(I, best_j), _half(I, iu), _half(I, iv)
    icx = I.px + mul(best_s * hj, _axis_x(I, best_j))
    icy = I.py + mul(best_s * hj, _axis_y(I, best_j))
    icz = I.pz + mul(best_s * hj, _axis_z(I, best_j))

    # the four incident face corners, in cyclic order
    cnt = 0
    for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        _qx[cnt] = (icx + mul(su * hu2, _axis_x(I, iu))
                    + mul(sv * hv2, _axis_x(I, iv)))
        _qy[cnt] = (icy + mul(su * hu2, _axis_y(I, iu))
                    + mul(sv * hv2, _axis_y(I, iv)))
        _qz[cnt] = (icz + mul(su * hu2, _axis_z(I, iu))
                    + mul(sv * hv2, _axis_z(I, iv)))
        cnt += 1

    for ax, ay, az, limit in ((ux, uy, uz, hu), (-ux, -uy, -uz, hu),
                              (vx, vy, vz, hv), (-vx, -vy, -vz, hv)):
        cnt = _clip_polygon(R, ax, ay, az, limit, cnt)
        if cnt == 0:
            return

    # stored contact normals always point from a to b
    cnx, cny, cnz = (nx, ny, nz) if ref_is_a else (-nx, -ny, -nz)
    for i in range(cnt):
        # depth below the reference face plane
        d = (mul(_qx[i] - R.px, nx) + mul(_qy[i] - R.py, ny)
             + mul(_qz[i] - R.pz, nz))
        pen = hn - d
        if pen < -(SURFACE_TOUCH << 12):
            continue
        # contact halfway between the incident point and the reference plane
        _add_pair_contact(a, b,
                          _qx[i] + mul(nx, pen >> 1),
                          _qy[i] + mul(ny, pen >> 1),
                          _qz[i] + mul(nz, pen >> 1),
                          cnx, cny, cnz, pen)


def _closest_pair_points(p1, p2, q1, q2):
    """Closest points of two segments (all Q12), Ericson's segment/segment
    test evaluated in plain units with Q14 parameters: the Q12 products of a
    world-scale distance would overflow 64 bits otherwise."""
    d1x, d1y, d1z = (p2[0] - p1[0]) >> 12, (p2[1] - p1[1]) >> 12, (p2[2] - p1[2]) >> 12
    d2x, d2y, d2z = (q2[0] - q1[0]) >> 12, (q2[1] - q1[1]) >> 12, (q2[2] - q1[2]) >> 12
    rx, ry, rz = (p1[0] - q1[0]) >> 12, (p1[1] - q1[1]) >> 12, (p1[2] - q1[2]) >> 12
    a = d1x * d1x + d1y * d1y + d1z * d1z
    e = d2x * d2x + d2y * d2y + d2z * d2z
    f = d2x * rx + d2y * ry + d2z * rz
    if a <= 0 and e <= 0:
        s = t = 0
    elif a <= 0:
        s = 0
        t = _clamp14(tdiv(f << 14, e))
    else:
        c = d1x * rx + d1y * ry + d1z * rz
        if e <= 0:
            t = 0
            s = _clamp14(-tdiv(c << 14, a))
        else:
            b = d1x * d2x + d1y * d2y + d1z * d2z
            denom = a * e - b * b
            s = _clamp14(tdiv((b * f - c * e) << 14, denom)) if denom != 0 else 0
            t = _clamp14(tdiv(b * s + (f << 14), e))
            if t == 0:
                s = _clamp14(-tdiv(c << 14, a))
            elif t == 16384:
                s = _clamp14(tdiv((b - c) << 14, a))
    c1 = (p1[0] + ((p2[0] - p1[0]) * s >> 14),
          p1[1] + ((p2[1] - p1[1]) * s >> 14),
          p1[2] + ((p2[2] - p1[2]) * s >> 14))
    c2 = (q1[0] + ((q2[0] - q1[0]) * t >> 14),
          q1[1] + ((q2[1] - q1[1]) * t >> 14),
          q1[2] + ((q2[2] - q1[2]) * t >> 14))
    return c1, c2


def _add_edge_edge_contact(a, b, i, j, nx, ny, nz, pen):
    """Edge manifold: the extreme edge of a along the separation axis against
    the extreme edge of b along its opposite."""
    u = (i + 1) % 3
    v = (i + 2) % 3
    du = (mul(_axis_x(a, u), nx) + mul(_axis_y(a, u), ny)
          + mul(_axis_z(a, u), nz))
    dv = (mul(_axis_x(a, v), nx) + mul(_axis_y(a, v), ny)
          + mul(_axis_z(a, v), nz))
    su = 1 if du >= 0 else -1
    sv = 1 if dv >= 0 else -1
    ax = (a.px + mul(su * _half(a, u), _axis_x(a, u))
          + mul(sv * _half(a, v), _axis_x(a, v)))
    ay = (a.py + mul(su * _half(a, u), _axis_y(a, u))
          + mul(sv * _half(a, v), _axis_y(a, v)))
    az = (a.pz + mul(su * _half(a, u), _axis_z(a, u))
          + mul(sv * _half(a, v), _axis_z(a, v)))
    ex = mul(_half(a, i), _axis_x(a, i))
    ey = mul(_half(a, i), _axis_y(a, i))
    ez = mul(_half(a, i), _axis_z(a, i))
    p1 = (ax - ex, ay - ey, az - ez)
    p2 = (ax + ex, ay + ey, az + ez)

    u = (j + 1) % 3
    v = (j + 2) % 3
    du = (mul(_axis_x(b, u), nx) + mul(_axis_y(b, u), ny)
          + mul(_axis_z(b, u), nz))
    dv = (mul(_axis_x(b, v), nx) + mul(_axis_y(b, v), ny)
          + mul(_axis_z(b, v), nz))
    su = 1 if du <= 0 else -1
    sv = 1 if dv <= 0 else -1
    bx = (b.px + mul(su * _half(b, u), _axis_x(b, u))
          + mul(sv * _half(b, v), _axis_x(b, v)))
    by = (b.py + mul(su * _half(b, u), _axis_y(b, u))
          + mul(sv * _half(b, v), _axis_y(b, v)))
    bz = (b.pz + mul(su * _half(b, u), _axis_z(b, u))
          + mul(sv * _half(b, v), _axis_z(b, v)))
    ex = mul(_half(b, j), _axis_x(b, j))
    ey = mul(_half(b, j), _axis_y(b, j))
    ez = mul(_half(b, j), _axis_z(b, j))
    q1 = (bx - ex, by - ey, bz - ez)
    q2 = (bx + ex, by + ey, bz + ez)

    c1, c2 = _closest_pair_points(p1, p2, q1, q2)
    _add_pair_contact(a, b, (c1[0] + c2[0]) >> 1, (c1[1] + c2[1]) >> 1,
                      (c1[2] + c2[2]) >> 1, nx, ny, nz, pen)


def _generate_contacts(a, b):
    """Separating axis test plus manifold generation for one pair of boxes.
    Fills the pair scratch and returns the number of contacts."""
    global _pcount
    _pcount = 0

    m = CONTACT_MARGIN
    if (a.box_max[0] + m < b.box_min[0] or b.box_max[0] + m < a.box_min[0]
            or a.box_max[1] + m < b.box_min[1]
            or b.box_max[1] + m < a.box_min[1]
            or a.box_max[2] + m < b.box_min[2]
            or b.box_max[2] + m < a.box_min[2]):
        return 0

    dx, dy, dz = b.px - a.px, b.py - a.py, b.pz - a.pz

    best = 2 ** 31 - 1          # least penetration over the face axes
    best_edge = 2 ** 31 - 1     # least penetration over the edge axes
    face_is_a = True
    face_k = 0
    face_s = 1
    edge_i = -1
    edge_j = -1
    edge_nx = edge_ny = edge_nz = 0

    # six face axes; every axis normal is turned to point from a to b
    for k in range(3):
        for which in range(2):
            ref = a if which == 0 else b
            nx, ny, nz = _axis_x(ref, k), _axis_y(ref, k), _axis_z(ref, k)
            if mul(dx, nx) + mul(dy, ny) + mul(dz, nz) < 0:
                nx, ny, nz = -nx, -ny, -nz
            ov = _overlap_on_axis(a, b, nx, ny, nz)
            if ov <= 0:
                return 0
            if ov < best:
                best = ov
                face_is_a = which == 0
                face_k = k
                # the reference face normal must point from the reference box
                # toward the other box: a's own normal, or b's negated one
                tx, ty, tz = (nx, ny, nz) if face_is_a else (-nx, -ny, -nz)
                face_s = 1 if (mul(tx, _axis_x(ref, k))
                               + mul(ty, _axis_y(ref, k))
                               + mul(tz, _axis_z(ref, k))) >= 0 else -1

    # nine edge cross product axes
    for i in range(3):
        pax, pay, paz = _axis_x(a, i), _axis_y(a, i), _axis_z(a, i)
        for j in range(3):
            pbx, pby, pbz = _axis_x(b, j), _axis_y(b, j), _axis_z(b, j)
            cx = pay * pbz - paz * pby
            cy = paz * pbx - pax * pbz
            cz = pax * pby - pay * pbx
            ln = isqrt(cx * cx + cy * cy + cz * cz)
            if ln < PARALLEL_EPS:
                continue
            nx, ny, nz = tdiv(cx << 12, ln), tdiv(cy << 12, ln), tdiv(cz << 12, ln)
            if mul(dx, nx) + mul(dy, ny) + mul(dz, nz) < 0:
                nx, ny, nz = -nx, -ny, -nz
            ov = _overlap_on_axis(a, b, nx, ny, nz)
            if ov <= 0:
                return 0
            if ov < best_edge:
                best_edge = ov
                edge_i, edge_j = i, j
                edge_nx, edge_ny, edge_nz = nx, ny, nz

    if edge_i >= 0 and best_edge < best - EDGE_AXIS_BIAS:
        _add_edge_edge_contact(a, b, edge_i, edge_j,
                               edge_nx, edge_ny, edge_nz, best_edge)
    else:
        ref = a if face_is_a else b
        inc = b if face_is_a else a
        _clip_face_pair(a, b, ref, inc, face_k, face_s, face_is_a)
    return _pcount


def _body_blocked(x, dx, dy, dz):
    """True when another body holds x up and the move would push x into that
    support. Together with _world_blocked this keeps a stack from paying for
    the projection of the pair above it by sinking into the pair below: two
    corrections that each push a shared body the other way never converge."""
    return (x.body_support
            and mul(x.support_nx, dx) + mul(x.support_ny, dy)
            + mul(x.support_nz, dz) < 0)


def _blocked(x, dx, dy, dz):
    """True when x cannot be moved along (dx, dy, dz) at all: the world
    geometry or the body it rests on is in the way."""
    return _world_blocked(x, dx, dy, dz) or _body_blocked(x, dx, dy, dz)


def _pair_held(a, b, a_static, b_static, i):
    """Which of the two bodies must not take contact i: whoever the world or
    its own support holds in place gives up its share, so the whole response
    goes to the body that can actually move. A cube resting on the floor then
    supports a stack instead of being squashed into it, and a stack comes to
    rest instead of keeping the residual velocity the floor only answers next
    frame.

    When neither body could move at all - a carried cube pressing a cube onto
    the floor, for instance - the hold is released again, because a pair with
    two immovable bodies has no solution and would stay interpenetrated."""
    a_held = not a_static and _blocked(a, -_pnx[i], -_pny[i], -_pnz[i])
    b_held = not b_static and _blocked(b, _pnx[i], _pny[i], _pnz[i])
    if (a_static or a_held) and (b_static or b_held):
        a_held = False
        b_held = False
    return a_held, b_held


def _record_support(a, b, count):
    """Notes which body is held up by the other, so a stacked cube may sleep
    exactly like one resting on the floor."""
    for i in range(count):
        if _pny[i] > SUPPORT_UP:
            b.body_support = True
            b.support_body = a
            # the support normal points out of the support into the body
            b.support_nx, b.support_ny, b.support_nz = _pnx[i], _pny[i], _pnz[i]
        elif _pny[i] < -SUPPORT_UP:
            a.body_support = True
            a.support_body = b
            a.support_nx = -_pnx[i]
            a.support_ny = -_pny[i]
            a.support_nz = -_pnz[i]


def _should_wake(a, b, count):
    """True when the sleeping body a must join the pair solve."""
    for i in range(count):
        nx, ny, nz = _pnx[i], _pny[i], _pnz[i]
        avx, avy, avz = _body_vel(a)
        bvx, bvy, bvz = _body_vel(b)
        rax, ray, raz = _ppx[i] - a.px, _ppy[i] - a.py, _ppz[i] - a.pz
        rbx, rby, rbz = _ppx[i] - b.px, _ppy[i] - b.py, _ppz[i] - b.pz
        vax = avx + mul(a.wy, raz) - mul(a.wz, ray)
        vay = avy + mul(a.wz, rax) - mul(a.wx, raz)
        vaz = avz + mul(a.wx, ray) - mul(a.wy, rax)
        vbx = bvx + mul(b.wy, rbz) - mul(b.wz, rby)
        vby = bvy + mul(b.wz, rbx) - mul(b.wx, rbz)
        vbz = bvz + mul(b.wx, rby) - mul(b.wy, rbx)
        vn = mul(vbx - vax, nx) + mul(vby - vay, ny) + mul(vbz - vaz, nz)
        if -vn > WAKE_SPEED:
            return True
    # A moving carried cube is player controlled and about to displace
    # whatever it touches, so a sleeper in its way always joins the solve: at
    # a slow walk the hand velocity stays under the neighbour threshold below
    # and the carried cube would slide straight through a resting one. A
    # parked hand is a shelf instead, and a cube resting on it must be allowed
    # to sleep.
    if b.kinematic and _body_speed(b) > 0:
        return True
    # the body it rests on is moving: keep hanging around would leave the
    # sleeper floating once its support slid away
    return _body_speed(b) > WAKE_NEIGHBOUR_SPEED


def _current_pen(a, b, i):
    """Penetration of contact i now, re-derived from the local anchors."""
    ax = _pral[i * 3]
    ay = _pral[i * 3 + 1]
    az = _pral[i * 3 + 2]
    awx = a.px + mul(ax, a.r[0]) + mul(ay, a.r[1]) + mul(az, a.r[2])
    awy = a.py + mul(ax, a.r[3]) + mul(ay, a.r[4]) + mul(az, a.r[5])
    awz = a.pz + mul(ax, a.r[6]) + mul(ay, a.r[7]) + mul(az, a.r[8])
    bx = _prbl[i * 3]
    by = _prbl[i * 3 + 1]
    bz = _prbl[i * 3 + 2]
    bwx = b.px + mul(bx, b.r[0]) + mul(by, b.r[1]) + mul(bz, b.r[2])
    bwy = b.py + mul(bx, b.r[3]) + mul(by, b.r[4]) + mul(bz, b.r[5])
    bwz = b.pz + mul(bx, b.r[6]) + mul(by, b.r[7]) + mul(bz, b.r[8])
    sep = (mul(bwx - awx, _pnx[i]) + mul(bwy - awy, _pny[i])
           + mul(bwz - awz, _pnz[i]))
    return _ppen[i] - sep


def _pair_effective_mass(rx, ry, rz, inv_i, nx, ny, nz):
    """n . ((I^-1 (r x n)) x r) for one body: the angular part of the
    effective mass K along a unit direction (inv_i is that body's world
    inverse inertia tensor, zero for a body that must not rotate)."""
    rnx = mul(ry, nz) - mul(rz, ny)
    rny = mul(rz, nx) - mul(rx, nz)
    rnz = mul(rx, ny) - mul(ry, nx)
    wx = RigidBody._evalI_x(inv_i, rnx, rny, rnz)
    wy = RigidBody._evalI_y(inv_i, rnx, rny, rnz)
    wz = RigidBody._evalI_z(inv_i, rnx, rny, rnz)
    return (mul(mul(wy, rz) - mul(wz, ry), nx)
            + mul(mul(wz, rx) - mul(wx, rz), ny)
            + mul(mul(wx, ry) - mul(wy, rx), nz))


def _solve_pair(a, b, a_static, b_static, count):
    """Sequential impulses for one pair: equal and opposite on both bodies,
    with restitution, Coulomb friction and a converging position projection.
    A static (carried or sleeping) body has zero inverse mass and inertia, so
    it absorbs nothing and only lends its velocity."""
    im_a = 0 if a_static else a.inv_mass
    im_b = 0 if b_static else b.inv_mass
    ii_a = ZERO_I if a_static else a.inv_i_world
    ii_b = ZERO_I if b_static else b.inv_i_world
    # A body the world geometry or its own support holds in place cannot take
    # the impulse. Without this the pair pass shoves the bottom cube of a
    # stack down into the floor and the floor only answers on the next step,
    # so every cube keeps a residual downward velocity, never looks at rest
    # and never falls asleep - the stack jitters and eventually topples.
    a_block = 0
    b_block = 0
    for i in range(count):
        a_held, b_held = _pair_held(a, b, a_static, b_static, i)
        if a_held:
            a_block |= 1 << i
        if b_held:
            b_block |= 1 << i
    avx, avy, avz = _body_vel(a)
    bvx, bvy, bvz = _body_vel(b)
    alx, aly, alz = a.lx, a.ly, a.lz
    blx, bly, blz = b.lx, b.ly, b.lz
    awx, awy, awz = a.wx, a.wy, a.wz
    bwx, bwy, bwz = b.wx, b.wy, b.wz
    for i in range(count):
        _paccn[i] = 0
        _pacct[i] = 0
        _pbias[i] = 0

    # Restitution targets are taken once from the approach velocities, before
    # any impulse is applied, so the sweeps stay mutually consistent.
    for i in range(count):
        nx, ny, nz = _pnx[i], _pny[i], _pnz[i]
        rax, ray, raz = _ppx[i] - a.px, _ppy[i] - a.py, _ppz[i] - a.pz
        rbx, rby, rbz = _ppx[i] - b.px, _ppy[i] - b.py, _ppz[i] - b.pz
        vax = avx + mul(awy, raz) - mul(awz, ray)
        vay = avy + mul(awz, rax) - mul(awx, raz)
        vaz = avz + mul(awx, ray) - mul(awy, rax)
        vbx = bvx + mul(bwy, rbz) - mul(bwz, rby)
        vby = bvy + mul(bwz, rbx) - mul(bwx, rbz)
        vbz = bvz + mul(bwx, rby) - mul(bwy, rbx)
        vn = mul(vbx - vax, nx) + mul(vby - vay, ny) + mul(vbz - vaz, nz)
        _pbias[i] = -mul(BODY_RESTITUTION, vn) if -vn > RESTITUTION_SPEED else 0

    for _ in range(PAIR_IMPULSE_ITERATIONS):
        for i in range(count):
            nx, ny, nz = _pnx[i], _pny[i], _pnz[i]
            a_held = (a_block >> i) & 1 != 0
            b_held = (b_block >> i) & 1 != 0
            im_ac = 0 if a_held else im_a
            im_bc = 0 if b_held else im_b
            ii_ac = ZERO_I if a_held else ii_a
            ii_bc = ZERO_I if b_held else ii_b
            rax, ray, raz = _ppx[i] - a.px, _ppy[i] - a.py, _ppz[i] - a.pz
            rbx, rby, rbz = _ppx[i] - b.px, _ppy[i] - b.py, _ppz[i] - b.pz

            vax = avx + mul(awy, raz) - mul(awz, ray)
            vay = avy + mul(awz, rax) - mul(awx, raz)
            vaz = avz + mul(awx, ray) - mul(awy, rax)
            vbx = bvx + mul(bwy, rbz) - mul(bwz, rby)
            vby = bvy + mul(bwz, rbx) - mul(bwx, rbz)
            vbz = bvz + mul(bwx, rby) - mul(bwy, rbx)
            rvx, rvy, rvz = vbx - vax, vby - vay, vbz - vaz
            vn = mul(rvx, nx) + mul(rvy, ny) + mul(rvz, nz)

            kn = (im_ac + im_bc
                  + _pair_effective_mass(rax, ray, raz, ii_ac, nx, ny, nz)
                  + _pair_effective_mass(rbx, rby, rbz, ii_bc, nx, ny, nz))
            if kn > 0:
                d_n = divq(_pbias[i] - vn, kn)
                new_acc = _paccn[i] + d_n
                if new_acc < 0:
                    new_acc = 0
                d_n = new_acc - _paccn[i]
                _paccn[i] = new_acc
                if d_n != 0:
                    # b takes +j n, a takes -j n
                    imp = mul(d_n, im_bc)
                    bvx += mul(nx, imp)
                    bvy += mul(ny, imp)
                    bvz += mul(nz, imp)
                    imp = mul(d_n, im_ac)
                    avx -= mul(nx, imp)
                    avy -= mul(ny, imp)
                    avz -= mul(nz, imp)
                    blx += mul(rby, mul(nz, d_n)) - mul(rbz, mul(ny, d_n))
                    bly += mul(rbz, mul(nx, d_n)) - mul(rbx, mul(nz, d_n))
                    blz += mul(rbx, mul(ny, d_n)) - mul(rby, mul(nx, d_n))
                    alx -= mul(ray, mul(nz, d_n)) - mul(raz, mul(ny, d_n))
                    aly -= mul(raz, mul(nx, d_n)) - mul(rax, mul(nz, d_n))
                    alz -= mul(rax, mul(ny, d_n)) - mul(ray, mul(nx, d_n))
                    bwx = RigidBody._evalI_x(ii_bc, blx, bly, blz)
                    bwy = RigidBody._evalI_y(ii_bc, blx, bly, blz)
                    bwz = RigidBody._evalI_z(ii_bc, blx, bly, blz)
                    awx = RigidBody._evalI_x(ii_ac, alx, aly, alz)
                    awy = RigidBody._evalI_y(ii_ac, alx, aly, alz)
                    awz = RigidBody._evalI_z(ii_ac, alx, aly, alz)

            if _paccn[i] <= 0:
                continue

            # Friction along the tangent of the relative contact velocity,
            # clamped to mu * accumulated normal impulse.
            vax = avx + mul(awy, raz) - mul(awz, ray)
            vay = avy + mul(awz, rax) - mul(awx, raz)
            vaz = avz + mul(awx, ray) - mul(awy, rax)
            vbx = bvx + mul(bwy, rbz) - mul(bwz, rby)
            vby = bvy + mul(bwz, rbx) - mul(bwx, rbz)
            vbz = bvz + mul(bwx, rby) - mul(bwy, rbx)
            rvx, rvy, rvz = vbx - vax, vby - vay, vbz - vaz
            vnn = mul(rvx, nx) + mul(rvy, ny) + mul(rvz, nz)
            tx = rvx - mul(nx, vnn)
            ty = rvy - mul(ny, vnn)
            tz = rvz - mul(nz, vnn)
            tl = RigidBody._norm3(tx, ty, tz)
            if tl < 1:
                continue
            tx, ty, tz = divq(tx, tl), divq(ty, tl), divq(tz, tl)
            kt = (im_ac + im_bc
                  + _pair_effective_mass(rax, ray, raz, ii_ac, tx, ty, tz)
                  + _pair_effective_mass(rbx, rby, rbz, ii_bc, tx, ty, tz))
            if kt <= 0:
                continue
            vt = mul(rvx, tx) + mul(rvy, ty) + mul(rvz, tz)
            d_t = -divq(vt, kt)
            max_fric = abs_(mul(BODY_FRICTION, _paccn[i]))
            new_acc = _pacct[i] + d_t
            if new_acc > max_fric:
                new_acc = max_fric
            elif new_acc < -max_fric:
                new_acc = -max_fric
            d_t = new_acc - _pacct[i]
            _pacct[i] = new_acc
            if d_t == 0:
                continue
            imp = mul(d_t, im_bc)
            bvx += mul(tx, imp)
            bvy += mul(ty, imp)
            bvz += mul(tz, imp)
            imp = mul(d_t, im_ac)
            avx -= mul(tx, imp)
            avy -= mul(ty, imp)
            avz -= mul(tz, imp)
            blx += mul(rby, mul(tz, d_t)) - mul(rbz, mul(ty, d_t))
            bly += mul(rbz, mul(tx, d_t)) - mul(rbx, mul(tz, d_t))
            blz += mul(rbx, mul(ty, d_t)) - mul(rby, mul(tx, d_t))
            alx -= mul(ray, mul(tz, d_t)) - mul(raz, mul(ty, d_t))
            aly -= mul(raz, mul(tx, d_t)) - mul(rax, mul(tz, d_t))
            alz -= mul(rax, mul(ty, d_t)) - mul(ray, mul(tx, d_t))
            bwx = RigidBody._evalI_x(ii_bc, blx, bly, blz)
            bwy = RigidBody._evalI_y(ii_bc, blx, bly, blz)
            bwz = RigidBody._evalI_z(ii_bc, blx, bly, blz)
            awx = RigidBody._evalI_x(ii_ac, alx, aly, alz)
            awy = RigidBody._evalI_y(ii_ac, alx, aly, alz)
            awz = RigidBody._evalI_z(ii_ac, alx, aly, alz)

    if not a_static:
        a.vx, a.vy, a.vz = avx, avy, avz
        a.lx, a.ly, a.lz = alx, aly, alz
        a.wx, a.wy, a.wz = awx, awy, awz
    if not b_static:
        b.vx, b.vy, b.vz = bvx, bvy, bvz
        b.lx, b.ly, b.lz = blx, bly, blz
        b.wx, b.wy, b.wz = bwx, bwy, bwz

    _correct_pair_positions(a, b, a_static, b_static, im_a, im_b, ii_a, ii_b,
                            count)


def _world_blocked(x, dx, dy, dz):
    """True when the world geometry holds body x against a move along
    (dx, dy, dz): one of the contacts its last step produced pushes back the
    other way. The pair position projection uses this so a cube resting on the
    floor does not pay its half of the correction by being squashed into the
    floor - the world pass would only claw it back slowly, so a stack would
    sink into itself and eventually topple."""
    for i in range(x.num_contacts):
        if mul(x.cnx[i], dx) + mul(x.cny[i], dy) + mul(x.cnz[i], dz) < 0:
            return True
    return False


def _correct_pair_positions(a, b, a_static, b_static, im_a, im_b, ii_a, ii_b,
                            count):
    """Position only de-penetration for one pair. Unlike the world pass this
    re-derives the penetration from the local anchors every iteration, so the
    sweep converges on the slop instead of leaving a fixed fraction of the
    overlap behind (a stack would otherwise sink visibly into itself)."""
    beta = PAIR_BETA[count - 1] if count <= len(PAIR_BETA) else PAIR_BETA[-1]
    for _ in range(PAIR_POSITION_ITERATIONS):
        for i in range(count):
            pen = _current_pen(a, b, i)
            if pen <= POSITION_SLOP << 12:
                continue
            nx, ny, nz = _pnx[i], _pny[i], _pnz[i]
            # whoever the world holds in place gives up its share, so the
            # whole projection goes to the body that can actually move
            a_held, b_held = _pair_held(a, b, a_static, b_static, i)
            if b_held:
                b_move, im_bc, ii_bc = False, 0, ZERO_I
            else:
                b_move, im_bc, ii_bc = not b_static, im_b, ii_b
            if a_held:
                a_move, im_ac, ii_ac = False, 0, ZERO_I
            else:
                a_move, im_ac, ii_ac = not a_static, im_a, ii_a
            # lever arms around the current midpoint of the two anchors
            ax = _pral[i * 3]
            ay = _pral[i * 3 + 1]
            az = _pral[i * 3 + 2]
            awx = a.px + mul(ax, a.r[0]) + mul(ay, a.r[1]) + mul(az, a.r[2])
            awy = a.py + mul(ax, a.r[3]) + mul(ay, a.r[4]) + mul(az, a.r[5])
            awz = a.pz + mul(ax, a.r[6]) + mul(ay, a.r[7]) + mul(az, a.r[8])
            bx = _prbl[i * 3]
            by = _prbl[i * 3 + 1]
            bz = _prbl[i * 3 + 2]
            bwx = b.px + mul(bx, b.r[0]) + mul(by, b.r[1]) + mul(bz, b.r[2])
            bwy = b.py + mul(bx, b.r[3]) + mul(by, b.r[4]) + mul(bz, b.r[5])
            bwz = b.pz + mul(bx, b.r[6]) + mul(by, b.r[7]) + mul(bz, b.r[8])
            cx = (awx + bwx) >> 1
            cy = (awy + bwy) >> 1
            cz = (awz + bwz) >> 1
            rax, ray, raz = cx - a.px, cy - a.py, cz - a.pz
            rbx, rby, rbz = cx - b.px, cy - b.py, cz - b.pz
            k = (im_ac + im_bc
                 + _pair_effective_mass(rax, ray, raz, ii_ac, nx, ny, nz)
                 + _pair_effective_mass(rbx, rby, rbz, ii_bc, nx, ny, nz))
            if k <= 0:
                continue
            dp = divq(mul(pen - (POSITION_SLOP << 12), beta), k)
            if b_move:
                b.px += mul(nx, mul(dp, im_bc))
                b.py += mul(ny, mul(dp, im_bc))
                b.pz += mul(nz, mul(dp, im_bc))
                qx = RigidBody._evalI_x(
                    ii_bc,
                    mul(rby, mul(nz, dp)) - mul(rbz, mul(ny, dp)),
                    mul(rbz, mul(nx, dp)) - mul(rbx, mul(nz, dp)),
                    mul(rbx, mul(ny, dp)) - mul(rby, mul(nx, dp)))
                qy = RigidBody._evalI_y(
                    ii_bc,
                    mul(rby, mul(nz, dp)) - mul(rbz, mul(ny, dp)),
                    mul(rbz, mul(nx, dp)) - mul(rbx, mul(nz, dp)),
                    mul(rbx, mul(ny, dp)) - mul(rby, mul(nx, dp)))
                qz = RigidBody._evalI_z(
                    ii_bc,
                    mul(rby, mul(nz, dp)) - mul(rbz, mul(ny, dp)),
                    mul(rbz, mul(nx, dp)) - mul(rbx, mul(nz, dp)),
                    mul(rbx, mul(ny, dp)) - mul(rby, mul(nx, dp)))
                b._rotate_matrix(qx, qy, qz)
                b._recompute_world_inertia()
            if a_move:
                a.px -= mul(nx, mul(dp, im_ac))
                a.py -= mul(ny, mul(dp, im_ac))
                a.pz -= mul(nz, mul(dp, im_ac))
                qx = RigidBody._evalI_x(
                    ii_ac,
                    mul(ray, mul(nz, dp)) - mul(raz, mul(ny, dp)),
                    mul(raz, mul(nx, dp)) - mul(rax, mul(nz, dp)),
                    mul(rax, mul(ny, dp)) - mul(ray, mul(nx, dp)))
                qy = RigidBody._evalI_y(
                    ii_ac,
                    mul(ray, mul(nz, dp)) - mul(raz, mul(ny, dp)),
                    mul(raz, mul(nx, dp)) - mul(rax, mul(nz, dp)),
                    mul(rax, mul(ny, dp)) - mul(ray, mul(nx, dp)))
                qz = RigidBody._evalI_z(
                    ii_ac,
                    mul(ray, mul(nz, dp)) - mul(raz, mul(ny, dp)),
                    mul(raz, mul(nx, dp)) - mul(rax, mul(nz, dp)),
                    mul(rax, mul(ny, dp)) - mul(ray, mul(nx, dp)))
                a._rotate_matrix(-qx, -qy, -qz)
                a._recompute_world_inertia()
    if not a_static:
        a._fix_matrix()
        a._recompute_world_inertia()
    if not b_static:
        b._fix_matrix()
        b._recompute_world_inertia()


def _collide_pair(a, b):
    count = _generate_contacts(a, b)
    if count == 0:
        return
    _record_support(a, b, count)
    # Wake a sleeper before deciding who is static, otherwise a carried cube
    # would sweep straight through a resting one (both would count as
    # immovable and the pair would be skipped).
    if a.sleeping and not b.sleeping and _should_wake(a, b, count):
        a.wake()
    if b.sleeping and not a.sleeping and _should_wake(b, a, count):
        b.wake()
    a_static = a.kinematic or a.sleeping
    b_static = b.kinematic or b.sleeping
    if a_static and b_static:
        return
    _solve_pair(a, b, a_static, b_static, count)
    a.pair_touched = not a_static
    b.pair_touched = not b_static


def collide_bodies(bodies, count):
    """Resolves every box-box pair for one frame. Called after all bodies
    stepped against the world, so a cube lands on, slides off, pushes and
    stacks on another cube instead of passing through it.

    The whole pair list is walked PAIR_ROUNDS times: contacts are generated
    from scratch each round, so a round sees what the previous one moved and
    support propagates from the ground up. Within a pair, a body the world or
    its own support holds in place gives up its share of the response (see
    _pair_held), which is what lets a stack come to rest."""
    for i in range(count):
        x = bodies[i]
        if x is None:
            continue
        x.prev_support = x.support_body
        x.support_body = None
        x.body_support = False
        x.support_nx = 0
        x.support_ny = 0
        x.support_nz = 0
        x.pair_touched = False
    for round_ in range(PAIR_ROUNDS):
        for i in range(count):
            a = bodies[i]
            if a is None:
                continue
            for j in range(i + 1, count):
                b = bodies[j]
                if b is None:
                    continue
                _collide_pair(a, b)
    for i in range(count):
        x = bodies[i]
        if x is None:
            continue
        # support slid away while it slept: fall again instead of floating
        if x.sleeping and x.prev_support is not None and x.support_body is None:
            x.wake()
        if x.pair_touched:
            x._fix_matrix()
            x._recompute_world_inertia()
            x._clamp_velocity()
            x._compute_vertices()
        # Rest detection for a cube held up by another cube has to happen
        # here: its own step() still saw the gravity this pass cancelled, so
        # a stack would look permanently restless and never fall asleep.
        if x.body_support and not x.kinematic:
            x._update_sleep()
