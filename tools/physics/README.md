# Rigid body physics verification

These files verify the integer fixed-point OBB rigid-body solver in
`src/com/RigidBody.java` (ported from the portalDS `OBB.c` solver).

The authoritative tests are **Java**, run against the real solver:

- `RigidBodyTests.java` — 18 self checking scenarios (9 single body, 9
  cube vs cube) in `package com`. No JUnit (CLDC has none) and no `assert`
  keyword (Java 1.3 has none), so there is a small check framework at the
  bottom: checks print only when they fail, and `main` exits 1 so the run
  can gate a build.
- `run_tests.sh` — compiles and runs them. See below.
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

1. **The gate** compiles `src/com/RigidBody.java` alone with `-source 1.3
   -target 1.3` and `libs/cldc11.jar:libs/midp21.jar:libs/jsr184.jar` as
   the bootclasspath — exactly what `build.yml` does for the game. This is
   what proves the solver still builds for a phone: it rejects generics,
   for-each, autoboxing, the `assert` keyword, `Math.pow`, anything
   `java.lang` on CLDC does not have.
2. **The tests** compile against it at the compiler's default source level.
   They are a development tool that never ships, so they do not need to be
   1.3 clean and pinning them would only stop them using anything newer.

`--strict` (or `PHYSICS_STRICT=1`) turns "could not run the gate" into an
error instead of a warning, so CI cannot silently skip it. The Physics
tests workflow, `.github/workflows/physics.yml`, runs
`run_tests.sh --strict tests` on every push — on every branch, not just
`main`, because these are the only automated check on the solver.

### No JDK on the box?

The script takes the toolchain from the environment, first match wins:
`JAVAC` (a full compiler command), `javac` from `PATH`, or an OpenJDK 8
`tools.jar` in `JAVA_TOOLS_JAR` driven by the `JAVA_BIN` runtime (default
`$JAVA_HOME/bin/java`, else `java`). A sandbox with neither can be
bootstrapped from PyPI and npm alone, which needs no root:

```sh
pip install --target ~/.cache/jdk4py jdk4py          # a JRE (java, no javac)
npm pack dataslope-tools-jar                          # OpenJDK 8's tools.jar
tar xzf dataslope-tools-jar-1.0.0.tgz

JAVA_BIN=~/.cache/jdk4py/jdk4py/java-runtime/bin/java \
JAVA_TOOLS_JAR=$PWD/package/tools.jar \
tools/physics/run_tests.sh
```

A javac running on a modular JVM like that has no platform classes of its
own to resolve `java.lang` against, so phase two retries with the
bootclasspath — and, because 1.8 turns `"a" + b` into a `StringBuilder`
that CLDC does not have, with `-source 1.3` as well. The tests are written
1.3 clean so both paths work.

CSV columns from `trace`: frame, cx, cy, cz, vx, vy, vz, nine orientation
entries, sleeping flag, last substep count, contact count. The multi body
scenarios (`stack2`, `stack3`, `sweep`, `carry`, `supportloss`) prefix each
row with the body index and print one row per body per frame.

## Python mirror

`rigid_body.py` is a line-by-line Python transcription of the solver, using
the same Q12 (and Q28 for the inverse inertia tensor) fixed-point semantics
including Java-style truncating integer division, and `test_rigid_body.py`
is a unittest suite over it. It exists because it can be run and re-run in
milliseconds while probing a new rule, and because it drives the level
geometry fuzzing below; `JavaParityTests` compares its traces against the
compiled Java harness frame by frame. The Java suite is the one that
matters — the mirror is a scratchpad, and a divergence is a bug in the
mirror until proven otherwise.

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

Multi body scenarios are `stack2`, `stack3`, `sweep`, `carry` and
`supportloss`; the Java suite covers all of them plus a carried cube that
rides, shoves, is pressed into another cube and then released, and a slowly
carried cube meeting a sleeping one. Known limitation: a cube balanced
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
