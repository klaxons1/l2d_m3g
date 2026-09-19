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
  sleep, and the contact pass against the world's triangle soup. `step` takes
  the step length in Q12 nominal frames; the three argument form steps one
  nominal frame, which is what the tests and the traces below use.
- `Solver.java` — body against body: the separating axis test, manifold
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
tools/physics/run_tests.sh fps          # frame rate invariance, see the last section
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

The normals that edge pass emits are normalized in Q12 rather than divided by
an integer square root of the squared length: at a seam the vertex offsets are
a unit or two, `isqrt` of a squared length that small truncates to 1, and the
result was corner normals √2 and √3 long — wrong impulse masses, and every
hold test built on them comparing against garbage.

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
  A push only counts where the box's height reaches the cube's. That used to be
  a band of `HALF / 2` around the cube's lowest corner, which left a cube
  resting on a 300 unit step unpushable while `Character.collisionTest` still
  held the character against it: the separation cancels a 200 unit walk exactly,
  so the character walked in place and the cube moved only once it was picked
  up.

`Cube.body` is private, so game code needs a one line forwarder on `Cube`
before `Scene` or `GameScreen` can push a cube.

## Cube vs cube

Boxes also collide with each other (`RigidBody.collideBodies`, implemented in
`Solver`), which is what lets cubes stack, knock each other over and be
shoved aside by a carried cube. `Cube.collideCubes` runs it once per frame from
`GameScreen.update`, after `Scene.update` has stepped every cube against
the world, and re-syncs the characters afterwards, so the capsules the next
frame resolves the other characters against are where the bodies ended up.
Walking characters join these pairs with a box of their own
(`Cube.pushedByCharacters`): that is how a walk into a cube becomes a push.

Every pair of the frame is generated into one batch, entirely in static
scratch: a separating axis test over the 6 face and 9 edge-cross axes picks
the shallowest one, then either the incident face is clipped against the
reference face (up to four contacts) or, when an edge-cross axis wins by
a clear margin, the two extreme edges contribute a single closest-points
contact. The batch is then solved in two phases: sequential impulses
(restitution plus Coulomb friction) for `PAIR_VELOCITY_SWEEPS` sweeps, then
a position projection re-derived from local anchors for
`PAIR_POSITION_SWEEPS` sweeps. Sweeping the whole batch instead of solving
one pair to completion lets a correction reach the pairs that share a body
inside the same frame, and costs about half as much: the randomized pile
scene runs in 47% of the time under `java -Xint`. It converges less per
pair, which is what the penetration bound below records.

Two rules make stacks come to rest instead of jittering and toppling:

- **A held body gives up its share.** A body the world geometry or its own
  support holds in place (a cube on the floor being pushed down, a cube
  being pushed into the cube it rests on) takes no part in that contact's
  impulse or projection, so the whole response goes to the body that can
  actually move. Without it the pair pass squashes the bottom cube of a
  stack into the floor, the floor only answers on the next step, and every
  cube keeps a residual downward velocity that never lets the stack sleep.
  When *neither* body can move the contact is left unsolved, which is what
  keeps a cube from being driven into the world (below).
- **A body held up by another body may sleep.** `recordSupport` marks the
  supported body (and the normal that supports it), the rest detection runs
  after the pair pass for those bodies, and a sleeper whose support slides
  away wakes up again instead of floating.

A sleeper does not get to ignore a body inside it, though. A pair is skipped
whole when both bodies count as immovable, and a sleeping cube with a carried
one parked inside it has nothing to wake it — no closing speed, no moving
neighbour, since a parked hand is a shelf a cube may rest on — so nothing
reports the jam and the hand simply keeps walking. `shouldWake` therefore also
wakes on penetration past `WAKE_PENETRATION`: 100 units, against the few a
settled contact sits at, so it fires on being inside something and not on
touching it.

### A cube cannot be pushed into the world

A pair is solved blind: `Solver` sees two boxes and the contact between
them, never the wall one of them stands against, so a pusher that cannot
itself move — the player's push box or a carried cube, both kinematic — drove
a resting cube into the geometry and left the world to pop it back out. At a
walk (150 units/frame) that measured 54% of a cube through a wall under the
player and 90% under a carried one, coming back out launched. The world's own
contacts now go into the same batch as the box-box ones, as a pair against an
immovable body (`WORLD`), so one set of sweeps converges a wall and a
neighbour together instead of two passes that cannot see each other. The
generation still belongs to the body being moved, and each body's own step
solves its surface contacts first: `step()` refines its own time step alone,
and a surface must be funded by the body's own press rather than by whatever a
pusher is shoving it with. What that leaves for the pair pass to be told:

- **A body the world holds gives up its share** (`pairHeld`, `heldAgainst`).
  The test is on the cosine of the move against the normal, not the raw dot:
  normals reach the pair pass with different magnitudes depending on which
  path emitted them, so a raw threshold blocked a shove *along* a wall as
  readily as one *into* it and cubes stopped sliding.
- **A surface that is only felt through a vertex flickers.** A vertex reaches a
  face within `SURFACE_TOUCH`, 6 units, so a cube pressed into a wall rides up
  out of that reach and reports floor only for a frame at a time. A remembered
  normal used to bridge that; the batch made it unnecessary, and the carried
  squeeze takes 46 more units of wall as a result.
- **When neither body can answer, the contact is left unsolved.** It used to
  be forced through anyway on the grounds that an immovable pair has no
  solution, which is precisely what buried the cube. The cost is that such a
  pair can be left slightly overlapping; the pile fuzz's worst transient
  overlap went from 20 to 32 units, in one frame of one case in 60.
- **A jammed carry slides, then lets go.** The kinematic side of a pair
  records the deepest penetration it sees in `pressPen` and the normal it was
  pressing along in `pressNX..pressNZ`; `Cube.updateHeld` gives up the part of
  the hand step that presses into that normal and lets the rest slide along it,
  so the overlap — and so the shove it can deliver — stays bounded, and
  `HOLD_DROP_DIST` drops the carry. Any press counts rather than a deep one:
  the overlap relaxes the moment the hand stops, so waiting for a threshold let
  the hand alternate between held and full speed, shaking and creeping a step
  at a time into the cube in front. A step back out along the normal is never
  cancelled, so a cube lowered into another one does not stay inside it.

- **A walker's press drags along the surface that holds it** (`drags`,
  `DRAG_PENETRATION`). Leaving that contact unsolved along its normal took the
  friction with it: no approach velocity, no normal impulse, no budget for a
  tangential one. So a cube a wall held answered a diagonal walk with nothing
  at all, while the push box — a box, where the character's own test is a
  sphere, so its corner reaches 1134 against the capsule's 802 — buried itself
  340 units deep. The moment that passed the overlap along the wall, the
  shallowest axis flipped and the pair threw the cube 280 units sideways in one
  frame. A push box now budgets the friction from how deep it is pressed and a
  body held along the normal keeps its own mass for the tangential solve, so
  the cube follows the walker along the wall. Only a push box
  (`RigidBody.drags`): a carried cube doing the same lets its obstacle yield
  sideways and loses the jam `Cube.updateHeld` stops on. Only a shallow press,
  too — past `DRAG_PENETRATION` the pair is buried rather than sliding.

The walker slides as well. `GameObject.slideOutOfPress` gives up the part of
the character's speed pressing into last frame's `pressPen`, the rule
`Cube.updateHeld` already used, and a character now takes a cube's whole
capsule separation (`Character.collisionTest` with `ways = 1`) because
`Cube.syncCharacter` throws the cube's half away.

`aWalkerDragsACubeAlongTheWall` is the regression: the cube follows along the
wall, where the old solver left it at 308 units and threw it sideways. All 16
traces are bit-identical — the drag is gated on `drags`, which no harness body
sets.

`pusherCannotBuryACubeInAWall` and `carriedCubeCannotBuryACubeInAWall` are
the regressions; both fail loudly against the old solver (1842 and 2200 units
of center, past a wall at 1800). `aCarriedCubeSlidesAlongTheJam` covers a hand
moving across a jam: it slides on all 100 frames, where freezing the step
would have stopped it on 83. Traces: 14 of the 16 scenarios are
bit-identical, `corner` comes to rest 1 unit away and `sweep` 18, from the
corner normals above.

### Known limit: no swept test between boxes

`step` substeps a body when it penetrates the world too deeply, but
`collideBodies` runs once per frame after every body has already moved, so a
pair closing fast enough can step clean past each other and never generate a
contact. Measured on a head-on hit between two 1000 unit cubes: transfer is
exact up to about 1300 units/frame of closing speed (the target leaves at
106-108% of the thrower's arrival speed, because the thrower keeps shoving it
over the following frames), from ~1400 the thrower starts to come out the far
side, and from ~1800 the target is never touched at all.

Nothing a cube does on its own reaches that: gravity is 20 units/frame against
a drag divisor of 25, so a falling cube tops out at 500, and `Cube.THROW_SPEED`
is 500. A *carried* one does, because its motion is imposed rather than
integrated - a mouse turn sweeps the hand 2700 units in a frame at `HOLD_DIST`.
So `Cube.updateHeld` limits the follow to `CARRY_STEP_LIMIT`, half the cube,
which is where a contact is still certain to be generated; the cube trails the
hand for a frame or two and catches up. Without the limit a swing at 4000/frame
produces no contact at all and leaves the target untouched, which is what
`carriedCubeCannotSwingCleanThroughAnotherOne` now holds.

The world pass has the same hole at a slow frame, since 500 units/frame over
250 ms is 2500 of travel. `step` refines on penetration, which a body that has
crossed a surface outright never shows, so it also bounds travel per substep up
front (`skipGuard`, the circumradius). Rewinding the *pair* pass the same way
remains the 15-20 line fix judged not worth it; keeping the hand inside its
limit is what keeps that case from arising.

Multi body scenarios are `stack2`, `stack3`, `sweep`, `carry` and
`supportloss`; the Java suite covers all of them plus a carried cube that
rides, shoves, is pressed into another cube and then released, and a slowly
carried cube meeting a sleeping one. Known limitation: a cube balanced
exactly on the seam between two cubes keeps rocking and does not fall
asleep, although it stays in place.

## Box sizes

`RigidBody(halfX, halfY, halfZ)` makes a box that is not a cube; the one
argument form still makes the 500 unit cube the game ships. Everything the
solver does is per axis: the separating axis test, face clipping, the edge
case, the world sweep and the inertia tensor. `getHalfX/Y/Z` answer in units,
`halfExtent(i)` in Q12 for the solver.

Mass follows volume, normalized so a 500 unit cube weighs exactly what it
always did (`UNIT_VOLUME` in `RigidBody`), which is why the traces did not
move. A box too small to weigh anything is clamped to mass 1 instead of being
left immovable.

What settles, measured by dropping each shape on the floor for 300 frames, by
setting it down one unit above it, and by stacking pairs (`nonCubicBoxes`,
`bigBoxesRest`, `gravityIsTheSameForEveryMass`):

| shape | rests at its own height | sleeps |
| --- | --- | --- |
| cube, 100 to 3000 | yes | yes |
| 1000 footprint, 60 to 2000 tall | yes | yes |
| 500 tall, 200 to 1500 footprint | yes | yes |
| crate, barrel, chest, tile, slab, boulder, pillar | yes | yes |
| crate on slab, slab on tall box, boulder on crate, big cube on cube | yes, gap within 2 units | yes |
| plank under a crate, crate on a plank | yes | yes |
| 100x500x100 pole | topples, which is what a 1 to 5 pole does, then lies flat | yes |
| 900x120x250 plank dropped from seven times its height | rocks on two edges | no |

Two things had to change before that table came out flat, and both are exact
no-ops for the 500 unit cube, which is why the traces did not move:

* Gravity and drag went into the integrator as forces and came back out divided
  by the mass, so they were only right for the cube, whose mass is exactly one
  in these units. A 300 unit box fell 4.6 times too fast and a thin tile 8
  times, and both hit the floor hard enough to bounce off it forever. They are
  accelerations now and skip the mass.
* The inverse inertia is Q24, so past roughly 700 units an axis rounded down to
  almost nothing: a 1000 unit cube fell through the floor and a 1500x500x1500
  box was launched out of the level. The tensor is stored 2^`iShift` up now,
  the shift picked per body so the coarsest axis still lands on
  `SPIN_PRECISION` units, and every reader shifts back down. The cube keeps
  shift 0, and `bigBoxesStillTumble` pins the value a 1000 unit box answers an
  off centre shove with.

What is left is impact recovery on the lightest, flattest boxes. A resting
contact is four corner impulses solved one after another, and a box with a big
inverse inertia answers each with a spin the next corner has to undo. Corners
can push but not pull, so a little is left over: one unit of speed on the cube,
thirty on a 900x120x250 plank, which is over `RESTITUTION_SPEED` and bounces
the plank on its own wobble. Gating restitution on the centre of mass instead
of the corner settles the plank and breaks eight of the sixteen traces, because
a struck cube really does bounce off corner speed, so it is not done. The mesh
side is separate: a box body needs a matching `postScale` where the game draws
it.

## Randomized piles

`randomPiles` in `RigidBodyTests.java` is the randomized half of the suite.
It throws 2 to 5 cubes into a closed floor-and-four-walls arena from random
heights with random velocities and spins, 60 cases of 240 frames, and hammers
the same invariants every frame: no cube driven through the floor or any
wall, no pair left deeply interpenetrated, no velocity past the solver's own
2048 units/frame clamp, every orientation still a rotation (unit columns,
mutually perpendicular). Then it runs the whole pass a second time from the
same seed and compares a checksum of every final state, so determinism is
checked over all 60 cases rather than one.

The random numbers are a plain LCG seeded from a constant, so a failure is
reproducible from the case number printed with it. `FUZZ_STATS` at the top of
that block prints per-case and worst-case numbers, which is how the bounds
were set — lowest center 487, fastest cube 680 units/frame. The penetration
bound moved with the pair pass: solving one pair to completion peaked at 32
against a bound of 40, the batched two phase pass peaks at 99 with 9 of the
60 cases over 40, so the bound is 100 and the check says to put it back (see
Cube vs cube).

Two allowances are deliberate. Cases start from a clean spawn (cubes may
overlap after they land, but the arena walls keep them in, since a cube
drifting off the edge of a finite floor looks exactly like tunnelling). And
a few cases may end with one cube still awake: a cube balanced exactly on the
seam between two others keeps rocking, so the suite allows up to a tenth of
cases to do that provided the cube is all but motionless.

## Frame rate independence

The game counted frames: `Scene.update` moved everything by its speed once per
`paint`, the AI thought on `getFrame() % 8`, a weapon counted its cooldown in
rendered frames, and the solver stepped a fixed `dt = F` — so a device that
rendered 60 fps played three times as fast as one that rendered 20.
`src/com/FPS.java` replaced that. It is ticked once per frame, at the top of
`GameScreen.paint`, before anything moves:

- `dt` is the frame length in **Q12 nominal frames**, one `FRAME_MS` (50 ms,
  20 fps) being `F`. The game was written around a 20 fps limit, so every
  per-frame number in it keeps its value and only the integration scales:
  speeds stay units per nominal frame, accelerations units per nominal frame
  squared, and a retune of the whole game is one constant.
- `dtMs` is the same length in whole milliseconds and `ms` the game time so
  far. Durations are counted in those: the bot think and attack cadence, the
  weapon shot time, cooldown and reload, the end-of-level messages, the shard
  and blood sprites. A cadence that was `getFrame() % 8 == 0` is now a stamp
  (`FPS.ms >= thinkAt`), because a frame count fires three times per nominal
  frame at 60 fps and skips beats below 20.
- `fps` is the frame count of the last second, which the HUD prints.
- The length is clamped to 1..250 ms, not saved up. A stall plays a quarter of
  a second and goes on from where it is rather than catching up, and the floor
  keeps `dt` off zero, which the hand velocity of a carried cube divides by.
  A device slower than 4 fps plays slow; one faster than 1000 fps plays fast by
  however much its clock rounds away.

Three things needed more than a multiplication, because a frame is not a
linear unit:

- **Friction is a power of the frame length, not a multiple of it.** The floor
  keeps a quarter of the speed a nominal frame, so over this frame it keeps
  that quarter to the power of `dt`. Scaled linearly instead, a frame of two
  nominal frames bleeds 150% of the speed and flips its sign: a walk at 10 fps
  covered 60% more ground. `SolverMath.powQ(keep, exp)` is the integer power —
  CLDC has no `Math.pow` — by square and multiply over the whole frames and a
  chain of square roots over the fraction, memoised on the last call because
  every object in a frame makes the same one. At a nominal frame it is a single
  multiply and the arithmetic is exactly what it was. The weapon walk bob
  keeps its sprite offset in Q12 for the same reason: a kick is two or five
  pixels a nominal frame, which truncates to nothing on a short one, and
  kicked in whole pixels per rendered frame the bob ran six and a half times
  as fast at 120 fps as at 20.
- **The walk input feeds that bleed**, so it is scaled by the same fraction:
  `Character.groundInput` multiplies by `bleed / 3072`, which is one at a
  nominal frame. Scaling the input by `dt` alone leaves the steady state a
  third short at 60 fps, because the damping is exponential and the input is
  not.
- **Gravity goes into the speed before the speed goes into the position**, so a
  step climbs half a step less than it falls, and over an arc that half step
  grows with the frame: it is what made a jump peak 10% higher at 120 fps. The
  tuned arc is the nominal one, so `Scene.update` gives a step of any other
  length the difference back. It is zero at a nominal frame. The solver's own
  integration gets no such correction: a resting body's gravity is answered by
  contact impulses rather than by a force, and correcting for it lifts the body
  off its own rest — measured, cubes then never came to sleep at any rate.

`RigidBody.step(colliders, count, world, frameDt)` scales the integration, the
drag and gravity accelerations and the sleep timer, and `setKinematicPose` /
`moveKinematic` take the same `frameDt` because the hand velocity is a rate:
the frame to frame delta of a carried cube or of a push box is divided by the
step length, or a swipe shoves other cubes three times weaker at 60 fps. One
more threshold is a per-step quantity: `RESTITUTION_SPEED`, the approach speed
below which a contact does not bounce. A resting contact closes at one step of
gravity, so the threshold grows with the step — unscaled, a frame of two
nominal frames bounced a resting cube for as long as it ran and it never slept.

What does **not** scale, deliberately: impulses (`Character.jump`, the cube
release speed, the knockback in `Bot.damage`), anything positional (the grab
range, the mantle band, the fall limit, the portal crossing tests), and the
solver's drag, which stays linear in the speed and under-damps a long frame by
a couple of percent where the penetration rollback is splitting it anyway.

`FrameRateTests.java` (`run_tests.sh fps`) drives the real `Character`, `FPS`,
`Magazine` and `RigidBody` at 10, 20, 40, 60 and 120 fps through a synthetic
wall clock and compares each scenario with the nominal run: a fall, a walk, a
turn, a jump, a dropped cube coming to rest and to sleep, a thrown cube, a
carried cube shoving a resting one, the weapon walk bob, a reload, the AI
cadence, a one second stall, and a clock finer than it can measure. It compiles all of `src`, so it
runs against the M3G stubs and never touches them at runtime. Worst measured
deviation from the nominal run, at the two extreme rates:

| scenario | 10 fps | 120 fps |
| --- | --- | --- |
| fall in 2 s | 0.5% | 0.2% |
| walk in 2 s | 0.2% | 1.4% |
| jump apex | exact | 0.8% |
| jump air time | exact | 4.6% |
| cube rest height | 0.3% | exact |
| cube sleep time | 3.5% | 7.1% |
| thrown cube in 1 s | 6.0% | 1.4% |
| shove by a carried cube | 1.3% | 0.3% |
| reload of 10 frames | +100 ms | +9 ms |

The two that stay visibly off are both about how a step ends rather than how it
integrates. A jump is noticed to have landed within a step, and the nominal
step covers 180 units of the fall, so a fine rate sees the ground up to 40 ms
sooner. A thrown cube at 10 fps slides 6% further: with no bounce it stays on
the floor instead of hopping along it.

The solver is unchanged at the nominal frame — the three argument `step`,
`moveKinematic` and `setKinematicPose` delegate with `frameDt = F`, `powQ` is a
single multiply there, the step lag and the restitution scaling are zero — so
that rework left `RigidBodyTests` and the CSV traces exactly as they were.
