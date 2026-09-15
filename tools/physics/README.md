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
  portal warp, determinism, and the cube-vs-cube scenarios below) plus
  frame-by-frame numerical parity against the compiled Java harness.

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

## Cube vs cube

Boxes also collide with each other (`RigidBody.collideBodies`), which is
what lets cubes stack, knock each other over and be shoved aside by a
carried cube. `Cube.collideCubes` runs it once per frame from
`GameScreen.update`, after `Scene.update` has stepped every cube against
the world, and re-syncs the characters afterwards so the next frame does
not read the resolution as a push.

One pair is generated and solved at a time, entirely in static scratch:
a separating axis test over the 6 face and 9 edge-cross axes picks the
shallowest one, then either the incident face is clipped against the
reference face (up to four contacts) or, when an edge-cross axis wins by
a clear margin, the two extreme edges contribute a single closest-points
contact. Contacts are solved with sequential impulses (restitution plus
Coulomb friction) and a position projection, and the whole pair list is
walked `PAIR_ROUNDS` times so a stack settles from the ground up.

Two rules make stacks come to rest instead of jittering and toppling:

- **A held body gives up its share.** A body the world geometry or its own
  support holds in place (a cube on the floor being pushed down, a cube
  being pushed into the cube it rests on) takes no part in that contact's
  impulse or projection, so the whole response goes to the body that can
  actually move. Without it the pair pass squashes the bottom cube of a
  stack into the floor, the floor only answers on the next step, and every
  cube keeps a residual downward velocity that never lets the stack sleep.
  When *neither* body could move — a carried cube pressed onto a floor cube
  — the hold is released again, since an immovable pair has no solution.
- **A body held up by another body may sleep.** `recordSupport` marks the
  supported body (and the normal that supports it), the rest detection runs
  after the pair pass for those bodies, and a sleeper whose support slides
  away wakes up again instead of floating.

Multi-body scenarios (`stack2`, `stack3`, `sweep`, `carry`,
`supportloss`) print one row per body per frame, with the body index as the
second CSV column; `PairTests` checks them against the Python solver and
`JavaParityTests` against the Java one. Known limitation: a cube balanced
exactly on the seam between two cubes keeps rocking and does not fall
asleep, although it stays in place.

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
