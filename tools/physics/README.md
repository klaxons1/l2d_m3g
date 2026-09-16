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

- `RigidBodyTests.java` — 24 self checking scenarios (13 single body, 10
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
row with the body index and print one row per body per frame. `blast`, `wind`
and `drive` are the three `applyForceAt` traces, see below.

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
Rest handling also wakes a body whose world contacts have been switched off:
a cube asleep over a freshly opened portal would otherwise hang in mid-air
forever, because `step` skips the whole world pass while it crosses
(`sleepingCubeFallsThroughAPortalOpening` is the regression test). Reaching
that state takes a second fix on the game side: `Cube.update` decides
`world` from `PortalManager.isInOpening`, which a cube at rest on a floor
portal fails — its centre is exactly one radius above the plane and it has no
speed toward it — so it now also tests its feet, the way `Player` does.

## Applying forces

`RigidBody.applyForceAt(x, y, z, dirX, dirY, dirZ, magnitude)` is the one way
to push a body from the game: a force of `magnitude` (Q12, units/frame²) along
an arbitrary direction, applied at an arbitrary world point given in integer
units. Gravity is `20 << 12`, which is the scale to think in.

It lasts exactly one `step()`, and that single lifetime covers both uses:

- **A blast** is one frame of force. `dt` is one frame, so a single call lands
  as an impulse of the same number: `400 << 12` through the centre leaves the
  cube at 400 units/frame, near `Cube.THROW_SPEED`. The `blast` trace fires
  `150 << 12` at the top face, which pops the cube up about 450 units and
  leaves it 720 units downrange, tumbled to rest.
- **Wind or a driven wheel** is the same call repeated every frame. It
  converges on `LINEAR_DRAG × magnitude`: the `wind` trace settles on
  999 units/frame for a push of 40, against gravity's own 500, and re-aimed at
  the body's centre it stays perfectly straight — 200 frames, no drift in z,
  orientation exactly identity, `w` exactly 0.

There is no `dt` argument, because the caller never scales by time: `step()`
owns it, and when it halves `dt` on a substep rollback a held force scales
with it instead of over-applying. The direction is normalised inside, so a Q14
`Vector3D`, a raw delta between two points, or a unit Q12 vector all work
alike. Off centre the push also spins the body, by `w = 3 |r × f| / (2 m h²)`,
so that same 400 unit blast at the top face of a cube tumbles it at
1.2 rad/frame — half the solver's angular clamp, and fast enough that the
floor answers the spin as an impact and the cube hops. Carried bodies ignore
the call (the hand places those), and it wakes a sleeping body, since `step()`
would otherwise throw the velocity away before it ever moved.

Two measured limits, both in the ground contacts rather than the force path:

- **A sustained tangential drive yaws a grounded box.** The `drive` trace
  holds a force 400 units below the centre, on the floor and into a wall: it
  ends up turned about 55°, leaning on a corner at the wall, and stays awake
  while the drive is held. The same drift needs no force API at all — driving
  a resting cube with `setVelocity` every frame yaws it 43° in 60 frames, and
  the `slide` scenario keeps a 4.7° residual yaw from a single shove — so it
  is the contact solver's friction asymmetry under sustained drive. In free
  space the force path is exact. Something that must track straight, a
  vehicle, is better driven kinematically (`setKinematic`/`moveKinematic`).
- **Drive low and it stays down.** Applied at wheel height the box never left
  the floor in 60 frames; the same push through the centre hopped from frame
  13, and 400 units above the centre from frame 3.
- **A character pushing a box along the floor is a pair, not a force.** Capsule
  radius 802 into a resting cube at 200 units/frame: `applyForceAt` compounds
  (300 in three frames, 500 of hop, 23° of tip) and `setVelocity` tumbles above
  ~120. A kinematic capsule box in `collideBodies` slides it at the walk instead
  — 228 peak, upright to 3°, and friction stops it when they do. That is
  `GameObject.pushBody` + `Cube.pushedByCharacters`, which spends only the net
  push of all its characters: the solver cannot balance two kinematic lenders,
  so two on opposite faces would hand it the cube to the last one solved.

`Cube.body` is private, so game code needs a one line forwarder on `Cube`
before `Scene` or `GameScreen` can push a cube.

## Cube vs cube

Boxes also collide with each other (`RigidBody.collideBodies`, implemented in
`BodyPair`), which is what lets cubes stack, knock each other over and be
shoved aside by a carried cube. `Cube.collideCubes` runs it once per frame from
`GameScreen.update`, after `Scene.update` has stepped every cube against
the world, and re-syncs the characters afterwards, so the capsules the next
frame resolves the other characters against are where the bodies ended up.
Walking characters join these pairs with a box of their own
(`Cube.pushedByCharacters`): that is how a walk into a cube becomes a push.

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
