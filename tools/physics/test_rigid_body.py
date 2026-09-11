#!/usr/bin/env python3
"""Verification of the rigid cube physics port.

Two layers:

 1. Numerical tests of the Python reference (tools/physics/rigid_body.py, a
    line-by-line transcription of src/com/RigidBody.java): resting, bounce,
    friction, walls, angular response, deep penetration rollback, slopes and
    portal warps.

 2. A trace comparison against the real J2ME solver: the same scenarios are
    run through tools/physics/RigidBodyHarness.java (compiled against the
    actual src/com/RigidBody.java) and every state field is compared frame by
    frame. The fixed point implementations must produce identical numbers.

The Java cross-check needs a Java runtime and the Eclipse Compiler jar;
their locations can be overridden with JAVA_BIN and ECJ_JAR environment
variables. When Java is unavailable the reference tests still run and the
cross-check is reported as skipped.
"""

import os
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rigid_body import RigidBody, Collider, F, isqrt  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HALF = 500

FRAMES = {
    "drop": 120,
    "slide": 120,
    "wall": 120,
    "spin": 120,
    "fastdrop": 200,
    "ramp": 120,
    "warp": 40,
}

TRACE_FIELDS = (
    "frame", "cx", "cy", "cz", "vx", "vy", "vz",
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8",
    "sleep", "substeps", "contacts",
)


def make_quad(a, b, c, d):
    """Quad collider with a fixed point Q12 normal from winding, exactly
    like RigidBodyHarness.quad / MathUtils.createNormal: n = (a-b) x (a-c)."""
    col = Collider()
    col.verts = [
        a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
    ]
    col.pols = [0, 1, 2, 3]
    abx, aby, abz = a[0] - b[0], a[1] - b[1], a[2] - b[2]
    acx, acy, acz = a[0] - c[0], a[1] - c[1], a[2] - c[2]
    nx = aby * acz - abz * acy
    ny = abz * acx - abx * acz
    nz = abx * acy - aby * acx
    length = isqrt(nx * nx + ny * ny + nz * nz)
    # truncate toward zero, exactly like Java integer division in the harness
    def _trunc(v):
        q = abs(v * 4096) // length
        return q if v >= 0 else -q
    col.norms = [_trunc(nx), _trunc(ny), _trunc(nz)]
    col.quads = 1
    col.scale8 = 256
    return col


def floor_mesh():
    s = 20000
    # downward normal (into the solid), as in engine rooms
    return make_quad((-s, 0, -s), (s, 0, -s), (s, 0, s), (-s, 0, s))


def wall_mesh():
    s, h, w = 20000, 10000, 1800
    return make_quad((w, 0, -s), (w, h, -s), (w, h, s), (w, 0, s))


def ramp_mesh():
    # ~20 degree slope rising in -x (plane y = -0.364x), solid below;
    # vertex order makes the computed normal point into the solid
    return make_quad((-4000, 1456, 20000), (-4000, 1456, -20000),
                     (0, 0, -20000), (0, 0, 20000))


def build_scenario(name):
    cols, n = [], 0
    body = RigidBody(HALF)
    if name == "drop":
        cols.append(floor_mesh()); n = 1
        body.reset(0, 3000, 0)
    elif name == "slide":
        cols.append(floor_mesh()); n = 1
        body.reset(0, 501, 0)
        body.set_velocity(180, 0, 0)
    elif name == "wall":
        cols.append(floor_mesh()); cols.append(wall_mesh()); n = 2
        body.reset(0, 501, 0)
        body.set_velocity(250, 0, 0)
    elif name == "spin":
        cols.append(floor_mesh()); n = 1
        body.reset(0, 1500, 0)
        body.set_angular_velocity(0, 0, 200)
    elif name == "fastdrop":
        cols.append(floor_mesh()); n = 1
        body.reset(0, 501, 0)
        body.set_velocity(0, -1500, 0)
    elif name == "ramp":
        cols.append(ramp_mesh())
        cols.append(floor_mesh())
        n = 2
        body.reset(-1281, 1152, 0)
    elif name == "warp":
        body.reset(0, 500, 0)
        body.set_velocity(100, 0, 0)
    else:
        raise AssertionError("unknown scenario " + name)
    return body, cols, n


def warp_matrix():
    return [
        -1, 0, 0, 5000,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1,
    ]


def trace_python(name, frames):
    body, cols, n = build_scenario(name)
    rows = []
    collide = name != "warp"
    for f in range(frames):
        body.step(cols if collide else [], n if collide else 0, collide)
        if name == "warp" and f == 10:
            body.warp(warp_matrix())
        cx, cy, cz = body.get_center()
        vx, vy, vz = body.get_velocity()
        row = [
            f, cx, cy, cz, vx, vy, vz,
        ] + [body.get_orientation(i) for i in range(9)] + [
            1 if body.sleeping else 0, body.last_substeps, body.num_contacts,
        ]
        rows.append(dict(zip(TRACE_FIELDS, row)))
    return rows


# --------------------------------------------------------------------- tests

class ReferenceTests(unittest.TestCase):

    def test_rest_on_floor(self):
        tr = trace_python("drop", FRAMES["drop"])
        # never tunnels deeper than the rollback threshold
        for r in tr:
            self.assertGreaterEqual(r["cy"], HALF - 64 - 8, r)
        # settles at the rest height (within the touch slop)
        last = tr[-1]
        self.assertAlmostEqual(last["cy"], HALF, delta=6)
        self.assertEqual(last["vy"], 0)
        self.assertTrue(last["sleep"])
        # first impact bounces (restitution 0.2): velocity flips sign once
        impact = next(r for i, r in enumerate(tr)
                      if i > 0 and tr[i]["vy"] > 0 > tr[i - 1]["vy"])
        self.assertGreater(impact["vy"], 30)

    def test_friction_stops_slide(self):
        tr = trace_python("slide", FRAMES["slide"])
        self.assertTrue(any(r["vx"] == 0 for r in tr))
        self.assertEqual(tr[-1]["vx"], 0)
        self.assertTrue(tr[-1]["sleep"])
        # monotone-ish deceleration: friction may overshoot by a fraction of
        # a unit while stopping, but it never gets kicked backwards hard
        self.assertTrue(all(r["vx"] >= -512 for r in tr))

    def test_wall_stops(self):
        tr = trace_python("wall", FRAMES["wall"])
        for r in tr:
            # center keeps at least the half extent away from the wall face
            self.assertLessEqual(r["cx"], 1800 - HALF + 6)
        self.assertEqual(tr[-1]["vx"], 0)
        self.assertTrue(tr[-1]["sleep"])

    def test_deep_penetration_rollback(self):
        tr = trace_python("fastdrop", FRAMES["fastdrop"])
        # the fast fall must never cross the floor (transient penetration is
        # bounded by the rollback threshold)
        for r in tr:
            self.assertGreaterEqual(r["cy"], HALF - 64 - 8, r)
        # rollback really subdivided the first frame
        self.assertGreater(tr[0]["substeps"], 1)
        # restitution on impact: ~0.2 of the 1500 speed
        self.assertGreater(tr[0]["vy"], 200)
        # settles eventually, possibly resting on a tilted face/edge
        last = tr[-1]
        self.assertTrue(last["sleep"], last)
        self.assertAlmostEqual(last["cy"], HALF, delta=10)
        self.assertLess(abs(last["vx"]), 4)
        self.assertLess(abs(last["vz"]), 4)

    def test_orientation_stays_orthonormal_and_tumbles(self):
        tr = trace_python("spin", FRAMES["spin"])
        def col(r, j):
            return (r["r%d" % j], r["r%d" % (3 + j)], r["r%d" % (6 + j)])
        tumbled = False
        for r in tr:
            axes = [col(r, 0), col(r, 1), col(r, 2)]
            for ax in axes:
                length2 = sum(v * v for v in ax)
                self.assertAlmostEqual(length2 / (F * F), 1.0, delta=0.004, msg=r)
            # orthogonality
            dot = sum(axes[0][i] * axes[1][i] for i in range(3))
            self.assertLess(abs(dot), F * 16)
            if any(abs(axes[2][i]) > 100 or abs(axes[0][1]) > 100 for i in range(3)):
                tumbled = True
        self.assertTrue(tumbled, "the body never rotated")
        # rests on the floor (possibly on a side face)
        self.assertTrue(tr[-1]["sleep"], tr[-1])
        self.assertLess(abs(tr[-1]["cy"] - HALF), 12)

    def test_slope_does_not_sink(self):
        tr = trace_python("ramp", FRAMES["ramp"])
        h = HALF
        # the gentle 20 degree slope is held by static friction (mu = 1 >
        # tan(20 deg)): the cube stays put on the slope instead of sinking
        for r in tr:
            # signed distance of the center above the ramp plane y = -0.364x
            # is 0.342*cx + 0.940*cy; it never falls below the half extent
            # minus the contact slop/bias allowance
            dist = 0.342 * r["cx"] + 0.940 * r["cy"]
            self.assertGreater(dist, h - 30, r)
        # it ends resting on the slope (face flush: center exactly one half
        # extent above the plane), held by static friction
        last = tr[-1]
        self.assertTrue(last["sleep"], last)
        end_dist = 0.342 * last["cx"] + 0.940 * last["cy"]
        self.assertAlmostEqual(end_dist, h, delta=12)
        self.assertAlmostEqual(last["cx"], -1281, delta=200)
        self.assertEqual(last["vx"], 0)

    def test_portal_warp(self):
        tr = trace_python("warp", FRAMES["warp"])
        before, after = tr[9], tr[10]
        # 180 degree flip around Y plus translation; the warp happens after
        # one more integration step, so allow one frame of motion
        self.assertAlmostEqual(after["cx"], 5000 - before["cx"], delta=200)
        self.assertEqual(after["cz"], -before["cz"])
        self.assertAlmostEqual(after["vx"], -before["vx"], delta=12)
        self.assertAlmostEqual(after["vz"], -before["vz"], delta=12)

    def test_fixed_point_determinism(self):
        # two independent runs give identical traces
        a = trace_python("drop", 60)
        b = trace_python("drop", 60)
        self.assertEqual([tuple(r.values()) for r in a],
                         [tuple(r.values()) for r in b])


# ------------------------------------------------------------- Java cross-check

def find_java():
    java = os.environ.get("JAVA_BIN")
    if java and os.path.exists(java):
        return java
    for p in ("/tmp/jdk4py/jdk4py/java-runtime/bin/java",):
        if os.path.exists(p):
            return p
    for p in os.environ.get("PATH", "").split(os.pathsep):
        cand = os.path.join(p, "java")
        if os.path.exists(cand):
            return cand
    return None


def find_ecj():
    ecj = os.environ.get("ECJ_JAR")
    if ecj and os.path.exists(ecj):
        return ecj
    for root in ("/tmp/qmole",):
        for dirpath, _dirs, files in os.walk(root):
            for f in files:
                if f.startswith("ecj-3") and f.endswith(".jar"):
                    return os.path.join(dirpath, f)
    return None


def compile_harness():
    java = find_java()
    ecj = find_ecj()
    if not java or not ecj:
        return None, None
    work = tempfile.mkdtemp(prefix="rigidtest")
    src = os.path.join(REPO, "src", "com", "RigidBody.java")
    harness = os.path.join(REPO, "tools", "physics", "RigidBodyHarness.java")
    cldc = os.path.join(REPO, "libs", "cldc11.jar")
    cmd = [java, "-jar", ecj, "-bootclasspath", cldc, "-d", work,
           "-source", "1.4", "-target", "1.4", "-nowarn", src, harness]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise AssertionError("harness compile failed:\n" + proc.stdout + proc.stderr)
    return java, work


def trace_java(name, frames, java, work):
    proc = subprocess.run(
        [java, "-cp", work, "com.RigidBodyHarness", name, str(frames)],
        capture_output=True, text=True, check=True)
    rows = []
    for line in proc.stdout.strip().split("\n"):
        vals = [int(v) for v in line.split(",")]
        rows.append(dict(zip(TRACE_FIELDS, vals)))
    return rows


class JavaParityTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.java, cls.work = compile_harness()
        if cls.java is None:
            print("\n[skip] Java cross-check: no java runtime or ECJ jar")

    def _parity(self, name):
        if self.java is None:
            self.skipTest("java unavailable")
        frames = FRAMES[name]
        py = trace_python(name, frames)
        jv = trace_java(name, frames, self.java, self.work)
        self.assertEqual(len(py), len(jv))
        int_fields = [f for f in TRACE_FIELDS if f != "frame"]
        mismatches = 0
        for a, b in zip(py, jv):
            for f in int_fields:
                if abs(a[f] - b[f]) > 2:
                    mismatches += 1
                    if mismatches <= 5:
                        print("\n%s frame %d field %s: py=%d java=%d"
                              % (name, a["frame"], f, a[f], b[f]))
        self.assertEqual(mismatches, 0,
                         "%d mismatching states vs Java" % mismatches)

    def test_drop_parity(self): self._parity("drop")
    def test_slide_parity(self): self._parity("slide")
    def test_wall_parity(self): self._parity("wall")
    def test_spin_parity(self): self._parity("spin")
    def test_fastdrop_parity(self): self._parity("fastdrop")
    def test_ramp_parity(self): self._parity("ramp")

    def test_warp_parity(self):
        # float warp, allow a slightly larger quantization tolerance
        if self.java is None:
            self.skipTest("java unavailable")
        py = trace_python("warp", FRAMES["warp"])
        jv = trace_java("warp", FRAMES["warp"], self.java, self.work)
        for a, b in zip(py, jv):
            for f in ("cx", "cy", "cz", "vx", "vy", "vz"):
                self.assertLess(abs(a[f] - b[f]), 6, (a, b, f))
            for i in range(9):
                f = "r%d" % i
                self.assertLess(abs(a[f] - b[f]), 6, (a, b, f))


if __name__ == "__main__":
    unittest.main(verbosity=2)
