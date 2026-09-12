# Rigid body physics verification

These files verify the integer fixed-point OBB rigid-body port in
`src/com/RigidBody.java` (ported from the portalDS `OBB.c` solver).

- `rigid_body.py` — line-by-line Python transcription of the Java solver,
  using the same Q12 (and Q28 for the inverse inertia tensor) fixed-point
  semantics, including Java-style truncating integer division.
- `RigidBodyHarness.java` — dependency-free CLDC Java harness that runs the
  same scenarios through `com.RigidBody` and prints CSV state traces.
- `test_rigid_body.py` — unittest suite: reference behaviour tests against
  the Python solver (resting, friction, walls, tumbling, deep-penetration
  rollback, slopes, fast throws into a concave floor/wall/wall corner,
  portal warp, determinism) plus frame-by-frame numerical parity against
  the compiled Java harness.

The solver also contains safety measures that only trigger on degenerate
manifolds: a separate multi-contact position projection pass (a cube
wedged in a corner is de-penetrated along every contact normal instead of
being snapped once on the deepest point), per-frame linear/angular
velocity clamps, a stable cross-product based matrix
re-orthonormalization that survives very large per-frame spins, an
edge-vs-edge contact pass that catches open wall-end spears missed by the
vertex tests, a history of collision-free poses for cross-frame wedge
rewind, and rest damping/sleep handling for bodies parked in concave
seams.

Run (from the repository root):

```sh
JAVA_BIN=/path/to/java \
ECJ_JAR=/path/to/ecj.jar \
python3 tools/physics/test_rigid_body.py
```

`JAVA_BIN`/`ECJ_JAR` may be omitted if `java` is on `PATH` and an ECJ jar
is in one of the usual locations. The harness is first compiled as Java
1.4 against the CLDC 1.1 boot classpath from `libs/cldc11.jar`; ECJ
builds that dropped the 1.4 source level automatically fall back to a
host J2SE 1.8 compile (the solver only uses `java.lang` math, so numbers
are identical).

CSV columns: frame, cx, cy, cz, vx, vy, vz, nine orientation entries,
sleeping flag, last substep count, contact count.

## Level geometry fuzzing

`level_fuzz.py` throws the cube from randomized free-space points across
the real level 2 geometry (`res/city.3d2`) and rejects tunnelling,
residual deep wedges, matrix distortion, velocity explosions and
failures to settle:

```sh
python3 tools/physics/level_fuzz.py [cases] [seed] [game|stress]
```

`game` uses gameplay-scale speeds, `stress` hammers the solver several
times harder. `level_throw.py [name-substring]` runs the deterministic
anchor throws and reports per-throw results.
