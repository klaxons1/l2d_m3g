#!/usr/bin/env python3
"""Randomized throws across real level 2 geometry (res/city.3d2).

Spawn points are jittered around known free-space anchors (the named
throws in level_throw.py). Velocities and spins are stressed well beyond
normal gameplay. Every case must settle without tunnelling, residual
deep wedging, matrix distortion or runaway velocities. Departures over
open world/roof edges into a void are not solver faults.

Usage: python3 level_fuzz.py [cases] [seed] [game|stress]
"""
import os
import random
import sys

sys.path.insert(0, os.path.dirname(__file__))
from rigid_body import RigidBody, F
from level_throw import run, THROWS, LEVEL
from load3d2 import load

COLLIDERS = load(LEVEL, 7500.0)


def spawn_is_free(point):
    """True when a fresh cube at the point is not embedded in geometry."""
    b = RigidBody(500)
    b.reset(*point)
    b._backup()
    b._collide_world(COLLIDERS, len(COLLIDERS), True)
    return b.max_penetration <= (200 << 12)


def main():
    cases = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 1234
    # "game" stays around the gameplay throw speed (~450), "stress"
    # (default) hammers the solver several times harder.
    profile = sys.argv[3] if len(sys.argv) > 3 else "stress"
    vlim = 450 if profile == "game" else 1400
    vymin, vymax = (-200, 250) if profile == "game" else (-300, 400)
    wlim = 200 if profile == "game" else 600
    rnd = random.Random(seed)

    anchors = [t[1] for t in THROWS]
    failures = []
    max_sub = 0
    worst_sleep = 0
    rejected = 0
    for i in range(cases):
        start = None
        for _ in range(30):
            ax, ay, az = rnd.choice(anchors)
            cand = (
                ax + rnd.randint(-700, 700),
                ay + rnd.randint(0, 400),
                az + rnd.randint(-700, 700),
            )
            if spawn_is_free(cand):
                start = cand
                break
            rejected += 1
        if start is None:
            print("could not find a free spawn after 30 tries")
            return 2

        vel = (
            rnd.randint(-vlim, vlim),
            rnd.randint(vymin, vymax),
            rnd.randint(-vlim, vlim),
        )
        ang = (
            rnd.randint(-wlim, wlim),
            rnd.randint(-wlim, wlim),
            rnd.randint(-wlim, wlim),
        )
        r = run(start, vel, ang, frames=420, colliders=COLLIDERS)

        problems = []
        if r["tunnel"]:
            problems.append("TUNNEL %s" % r["tunnel"][:3])
        # Short, monotonically recovering residual penetration while
        # skimming a wall seam is self-correcting; flag only a wedge
        # lasting at least four frames.
        longest = []
        cur = []
        for d in r["deep"]:
            if d[1] <= 200:
                continue
            if cur and d[0] - cur[-1][0] <= 2:
                cur.append(d)
            else:
                cur = [d]
            if len(cur) >= len(longest):
                longest = cur
        if len(longest) >= 4:
            problems.append("DEEP %s" % longest[:4])
        if r["matrix"]:
            problems.append("MATRIX %s" % r["matrix"][:2])
        if r["clamp"]:
            problems.append("CLAMP %s" % r["clamp"][:1])
        # Departures through open edges (lateral bounds or a roof edge
        # into a void shaft) are expected at these speeds and never
        # settle; they are not solver faults.
        in_void = bool(r["edge"]) or bool(r["outside"])
        if r["nosettle"] and not in_void:
            problems.append("NO-SETTLE")
        if problems:
            failures.append((i, seed, start, vel, ang, problems))
            print("case %d FAIL %s" % (i, "; ".join(problems)))
            print("   start=%s vel=%s ang=%s end=%s" % (
                start, vel, ang, r["end"]))
        max_sub = max(max_sub, r["max_substeps"])
        if r["sleep_frame"] is not None:
            worst_sleep = max(worst_sleep, r["sleep_frame"])

    print("\n%d cases, seed %d (%s): %d failures (%d spawns rejected), "
          "peak substeps %d, worst sleep frame %s"
          % (cases, seed, profile, len(failures), rejected, max_sub,
             worst_sleep))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
