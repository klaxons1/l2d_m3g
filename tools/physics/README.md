# Rigid body physics verification

These files verify the integer fixed-point OBB rigid-body solver (ported from
the portalDS `OBB.c` solver). The solver is three files in `src/com`:

- `SolverMath.java` — Q12 arithmetic, the Q24 matrix evaluation both inertia
  tensors need, and the contact micro-ops the two passes share: effective mass
  along a direction, the sliding direction, the clamped accumulated impulse.
  Both solvers extend it so their loops can call `mul` unqualified, because
  source level 1.3 has no static imports and a forwarding wrapper would put a
  second call in front of the innermost operation in the engine.
- `RigidBody.java` — the body itself: state, integration, substep rollback,
  sleep, and the contact pass against the world's triangle soup.
- `BodyPair.java` — body against body: the separating axis test, manifold
  generation by face clipping or closest edge points, and the pair solver.
  `RigidBody.collideBodies` is still the entry point and delegates here, so
  the game and these tests only ever name `RigidBody`.

The authoritative tests are **Java**, run against the real solver:

- `RigidBodyTests.java` — 20 self checking scenarios (9 single body, 10
  cube vs cube, plus a randomized pile fuzz) in `package com`. No JUnit (CLDC has none) and no `assert`
  keyword (Java 1.3 has none), so there is a small check framework at the
  bottom: checks print only when they fail, and `main` exits 1 so the run
  can gate a build.
- `run_tests.sh` — compiles and runs them. See below.
- `fetch_jdk.sh` — gets a Java toolchain onto a box that has none.
- `RigidBodyHarness.java` — dependency-free companion that prints raw CSV
  state traces for the same scenarios, for when a check fails and the
  numbers need eyeballing.

Everything the tests touch is public API (`getCenterX`, `getOrientation`,
`isSleeping`, `getContactCount`, `lastSubsteps`, `collideBodies`, ...), so
adding a test never needs a solver change. Penetration is measured by an
independent separating-axis probe rebuilt in the test file from those
accessors, which means it does not trust the solver's own contact data.

## Running them

```sh
tools/physics/run_tests.sh              # compile + run the self checks
tools/physics/run_tests.sh gate         # only the CLDC 1.1 / Java 1.3 compile
tools/physics/run_tests.sh trace stack3 120
tools/physics/run_tests.sh clean
```

Two compile phases, because the two halves have different requirements:

1. **The gate** compiles the three solver files alone with `-source 1.3
   -target 1.3` and `libs/cldc11.jar:libs/midp21.jar:libs/jsr184.jar` as
   the bootclasspath — exactly what `build.yml` does for the game. This is
   what proves the solver still builds for a phone: it rejects generics,
   for-each, autoboxing, the `assert` keyword, `Math.pow`, anything
   `java.lang` on CLDC does not have.
2. **The tests** compile against it at the compiler's default source level.
   They are a development tool that never ships, so they do not need to be
   1.3 clean and pinning them would only stop them using anything newer.

`--strict` (or `PHYSICS_STRICT=1`) turns "could not run the gate" into an
error instead of a warning, so a run that claims to have tested the solver
cannot quietly skip the half that proves it still builds for a phone.

Nothing here runs in CI: the tests belong to whoever touches the solver, and
the whole suite takes about two seconds. Phone compatibility is still gated
on the way to `main`, because `build.yml` compiles all of `src/` — the solver
included — with the same `-source 1.3 -target 1.3` and bootclasspath on every
push and every pull request. What that does not do is check behaviour, so run
these before pushing.

### No JDK on the box?

```sh
eval "$(tools/physics/fetch_jdk.sh)"    # fetches if needed, exports the paths
tools/physics/run_tests.sh
```

The default install directory is `$HOME/.cache/l2d-physics-java`. Pass a
path to keep it inside the checkout instead — `tools/physics/.jdk` is
gitignored:

```sh
eval "$(tools/physics/fetch_jdk.sh tools/physics/.jdk)"
```

`fetch_jdk.sh` needs no root and no package manager: it installs `jdk4py`
from PyPI (a JRE, ~100 MB) and OpenJDK 8's `tools.jar` from the npm
registry (~5 MB) into `${PHYSICS_JDK_DIR:-$HOME/.cache/l2d-physics-java}`,
checks that the pair really runs, and prints the two exports `run_tests.sh`
reads — `JAVA_BIN` and `JAVA_TOOLS_JAR`. It is idempotent, and it does
nothing at all when a `javac` and a `java` are already on `PATH`.

`tools.jar` is worth the odd detour: driven by that JRE it is a real
`javac 1.8`, and 1.8 is the last javac that accepts `-source 1.3`, so a box
with no JDK can still run the phone compatibility gate rather than only the
tests.

A javac running on a modular JVM like that has no platform classes of its
own to resolve `java.lang` against, so phase two retries with the
bootclasspath — and, because 1.8 turns `"a" + b` into a `StringBuilder`
that CLDC does not have, with `-source 1.3` as well. The tests are written
1.3 clean so both paths work.

The script takes the toolchain from the environment, first match wins:
`JAVAC` (a full compiler command), `javac` from `PATH`, or `JAVA_TOOLS_JAR`
driven by `JAVA_BIN` (default `$JAVA_HOME/bin/java`, else `java`).

CSV columns from `trace`: frame, cx, cy, cz, vx, vy, vz, nine orientation
entries, sleeping flag, last substep count, contact count. The multi body
scenarios (`stack2`, `stack3`, `sweep`, `carry`, `supportloss`) prefix each
row with the body index and print one row per body per frame.

## Solver safety nets

Beyond the ordinary integrate-collide-resolve loop the solver carries
measures that only trigger on degenerate manifolds: a separate multi-contact
position projection pass (a cube wedged in a corner is de-penetrated along
every contact normal instead of being snapped once on the deepest point),
per-frame linear/angular velocity clamps, a stable cross-product based matrix
re-orthonormalization that survives very large per-frame spins, an
edge-vs-edge contact pass that catches open wall-end spears missed by the
vertex tests, a history of collision-free poses for cross-frame wedge
rewind, and rest damping/sleep handling for bodies parked in concave seams.

## Cube vs cube

Boxes also collide with each other (`RigidBody.collideBodies`, implemented in
`BodyPair`), which is what lets cubes stack, knock each other over and be
shoved aside by a carried cube. `Cube.collideCubes` runs it once per frame from
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

### Known limit: no swept test between boxes

`step` substeps a body when it penetrates the world too deeply, but
`collideBodies` runs once per frame after every body has already moved, so a
pair closing fast enough can step clean past each other and never generate a
contact. Measured on a head-on hit between two 1000 unit cubes: transfer is
exact up to about 1300 units/frame of closing speed (the target leaves at
106-108% of the thrower's arrival speed, because the thrower keeps shoving it
over the following frames), from ~1400 the thrower starts to come out the far
side, and from ~1800 the target is never touched at all.

Nothing in the game reaches that. Gravity is 20 units/frame against a drag
divisor of 25, so a falling cube tops out at 500 units/frame, and
`Cube.THROW_SPEED` is 500. Both are an order of magnitude inside the limit,
which is why it is documented rather than fixed: catching it needs the pair
pass to rewind both bodies along their velocity and re-test at earlier poses,
about 15-20 lines with a guard so it never runs for slow pairs, and a cheaper
swept-AABB *detection* alone would only report the miss without doing anything
about it. If a throw or a launch speed ever goes above ~1200 units/frame, that
changes.

Multi body scenarios are `stack2`, `stack3`, `sweep`, `carry` and
`supportloss`; the Java suite covers all of them plus a carried cube that
rides, shoves, is pressed into another cube and then released, and a slowly
carried cube meeting a sleeping one. Known limitation: a cube balanced
exactly on the seam between two cubes keeps rocking and does not fall
asleep, although it stays in place.

## Randomized piles

`randomPiles` in `RigidBodyTests.java` is the randomized half of the suite.
It throws 2 to 5 cubes into a closed floor-and-four-walls arena from random
heights with random velocities and spins, 60 cases of 240 frames, and hammers
the same invariants every frame: no cube driven through the floor or any
wall, no pair left interpenetrated, no velocity past the solver's own
2048 units/frame clamp, every orientation still a rotation (unit columns,
mutually perpendicular). Then it runs the whole pass a second time from the
same seed and compares a checksum of every final state, so determinism is
checked over all 60 cases rather than one.

The random numbers are a plain LCG seeded from a constant, so a failure is
reproducible from the case number printed with it. `FUZZ_STATS` at the top of
that block prints per-case and worst-case numbers, which is how the bounds
were set — measured worst penetration 20 units, lowest center 496, fastest
cube 680 units/frame.

Two allowances are deliberate. Cases start from a clean spawn (cubes may
overlap after they land, but the arena walls keep them in, since a cube
drifting off the edge of a finite floor looks exactly like tunnelling). And
a few cases may end with one cube still awake: a cube balanced exactly on the
seam between two others keeps rocking, so the suite allows up to a tenth of
cases to do that provided the cube is all but motionless.
