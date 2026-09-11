# Rigid body physics verification

These files verify the integer fixed-point OBB rigid-body port in
`src/com/RigidBody.java` (ported from the portalDS `OBB.c` solver).

- `rigid_body.py` — line-by-line Python transcription of the Java solver,
  using the same Q12 (and Q24 for the inverse inertia tensor) fixed-point
  semantics, including Java-style truncating integer division.
- `RigidBodyHarness.java` — dependency-free CLDC Java harness that runs the
  same scenarios through `com.RigidBody` and prints CSV state traces.
- `test_rigid_body.py` — unittest suite: reference behaviour tests against
  the Python solver (resting, friction, walls, tumbling, deep-penetration
  rollback, slopes, portal warp, determinism) plus frame-by-frame numerical
  parity against the compiled Java harness.

Run (from the repository root):

```sh
JAVA_BIN=/path/to/java \
ECJ_JAR=/path/to/ecj-3.2.0.jar \
python3 tools/physics/test_rigid_body.py
```

`JAVA_BIN`/`ECJ_JAR` may be omitted if `java` is on `PATH` and
`ecj-3.2.0.jar` is in one of the usual locations; the harness is compiled
with the CLDC 1.1 boot classpath from `libs/cldc11.jar`.

CSV columns: frame, cx, cy, cz, vx, vy, vz, nine orientation entries,
sleeping flag, last substep count, contact count.
