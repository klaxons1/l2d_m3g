#!/bin/sh
#
# Builds and runs the physics tests for src/com/RigidBody.java.
#
#   tools/physics/run_tests.sh              compile + run the self checks
#   tools/physics/run_tests.sh tests        the same thing
#   tools/physics/run_tests.sh trace drop 120
#                                           compile + dump a raw state trace
#                                           (see RigidBodyHarness.java)
#   tools/physics/run_tests.sh gate         only the CLDC 1.1 / Java 1.3 compile
#   tools/physics/run_tests.sh clean        remove the build output
#
# Two compile phases, because the two halves have different requirements:
#
#   1. the gate - src/com/RigidBody.java alone, with -source 1.3 -target 1.3
#      and the CLDC/MIDP/M3G jars as bootclasspath, exactly like build.yml
#      compiles the game. This is what proves the solver still builds for a
#      phone: it rejects generics, for-each, autoboxing, the assert keyword,
#      Math.pow, anything java.lang on CLDC does not have.
#   2. the tests - RigidBodyTests.java and RigidBodyHarness.java against it at
#      the compiler's default source level. The tests are a development tool
#      that never ships, so they do not need to be 1.3 clean, and pinning them
#      would only stop them using anything newer.
#
# --strict (or PHYSICS_STRICT=1) turns "could not run the 1.3 gate" into an
# error instead of a warning. The Physics tests workflow uses it, so the gate
# can never be silently skipped there.
#
# Toolchain, first match wins:
#   JAVAC          full compiler command, e.g.
#                  "java -cp /path/tools.jar com.sun.tools.javac.Main"
#   javac          from PATH
#   JAVA_TOOLS_JAR an OpenJDK 8 tools.jar, driven by $JAVA_BIN
#   JAVA_BIN       runtime to execute the tests with (default $JAVA_HOME/bin/java
#                  or java from PATH). Must be at least as new as the compiler.
#
# Exits 1 if the gate fails or any check fails.

set -e

STRICT=${PHYSICS_STRICT:-0}
if [ "$1" = "--strict" ]; then
	STRICT=1
	shift
fi

CMD=${1:-tests}

cd `dirname "$0"`/../..
OUT=build/physics
GATE_OUT=build/physics-cldc
BOOT=libs/cldc11.jar:libs/midp21.jar:libs/jsr184.jar
SOLVER=src/com/RigidBody.java
TOOLS="tools/physics/RigidBodyHarness.java tools/physics/RigidBodyTests.java"

if [ "$CMD" = "clean" ]; then
	rm -rf "$OUT" "$GATE_OUT"
	echo "removed $OUT and $GATE_OUT"
	exit 0
fi

# ------------------------------------------------------------- toolchain

if [ -z "$JAVA_BIN" ]; then
	if [ -n "$JAVA_HOME" ] && [ -x "$JAVA_HOME/bin/java" ]; then
		JAVA_BIN="$JAVA_HOME/bin/java"
	elif command -v java > /dev/null 2>&1; then
		JAVA_BIN=java
	fi
fi
if [ -z "$JAVA_BIN" ]; then
	echo "run_tests.sh: no java runtime found (set JAVA_BIN or JAVA_HOME)." >&2
	exit 2
fi

if [ -z "$JAVAC" ]; then
	if command -v javac > /dev/null 2>&1; then
		JAVAC=javac
	elif [ -n "$JAVA_TOOLS_JAR" ] && [ -f "$JAVA_TOOLS_JAR" ]; then
		JAVAC="$JAVA_BIN -cp $JAVA_TOOLS_JAR com.sun.tools.javac.Main"
	fi
fi
if [ -z "$JAVAC" ]; then
	echo "run_tests.sh: no Java compiler found." >&2
	echo "Put javac on PATH, or point JAVAC / JAVA_TOOLS_JAR at one." >&2
	echo "See tools/physics/README.md for bootstrapping a JDK-less box." >&2
	exit 2
fi

# ---------------------------------------------------- phase 1: the 1.3 gate

gate_skipped=""
if [ ! -f libs/cldc11.jar ] || [ ! -f libs/midp21.jar ] || [ ! -f libs/jsr184.jar ]; then
	gate_skipped="the CLDC jars are missing ($BOOT)"
else
	mkdir -p "$GATE_OUT"
	echo "gate: compiling $SOLVER for CLDC 1.1 / Java 1.3"
	GATE_LOG=`mktemp`
	# shellcheck disable=SC2086
	if $JAVAC -source 1.3 -target 1.3 -bootclasspath $BOOT \
			-d "$GATE_OUT" -encoding UTF-8 $SOLVER > "$GATE_LOG" 2>&1; then
		echo "gate: OK"
	else
		if grep -qi "source option\|source release\|invalid source" "$GATE_LOG"; then
			gate_skipped="this javac no longer accepts -source 1.3"
		else
			echo "gate: FAILED - the solver is not phone compatible" >&2
			cat "$GATE_LOG" >&2
			rm -f "$GATE_LOG"
			exit 1
		fi
	fi
	rm -f "$GATE_LOG"
fi

if [ -n "$gate_skipped" ]; then
	if [ "$STRICT" = "1" ]; then
		echo "run_tests.sh: --strict but the 1.3 gate was skipped: $gate_skipped" >&2
		exit 2
	fi
	echo "=================================================================="
	echo "WARNING: the CLDC 1.1 / Java 1.3 gate was SKIPPED ($gate_skipped)."
	echo "The solver is still being compiled and tested below, but nothing"
	echo "here proves it would build for a phone. CI always runs the gate."
	echo "=================================================================="
fi

if [ "$CMD" = "gate" ]; then
	exit 0
fi

# ------------------------------------------------------ phase 2: the tests

mkdir -p "$OUT"
echo "compiling the solver, the harness and the tests"
BUILD_LOG=`mktemp`
# shellcheck disable=SC2086
if $JAVAC -d "$OUT" -encoding UTF-8 $SOLVER $TOOLS > "$BUILD_LOG" 2>&1; then
	:
elif [ -f libs/cldc11.jar ]; then
	# A javac running on a modular JVM (an OpenJDK 8 tools.jar driven by a
	# JDK 9+ java, say) has no platform classes of its own to resolve
	# java.lang against and needs the bootclasspath handed to it. Compiling
	# the tests at 1.3 as well is what makes that work: at 1.8 javac turns
	# "a" + b into StringBuilder, which CLDC does not have. On a normal JDK
	# the branch above is taken and the tests get the full modern language.
	echo "note: this javac needs a bootclasspath, so the tests are being"
	echo "      compiled at Java 1.3 against CLDC as well"
	rm -rf "$OUT"
	mkdir -p "$OUT"
	# shellcheck disable=SC2086
	if ! $JAVAC -source 1.3 -target 1.3 -bootclasspath $BOOT \
			-d "$OUT" -encoding UTF-8 $SOLVER $TOOLS > "$BUILD_LOG" 2>&1; then
		cat "$BUILD_LOG" >&2
		rm -f "$BUILD_LOG"
		exit 1
	fi
else
	cat "$BUILD_LOG" >&2
	rm -f "$BUILD_LOG"
	exit 1
fi
rm -f "$BUILD_LOG"

case "$CMD" in
tests)
	echo
	"$JAVA_BIN" -cp "$OUT" com.RigidBodyTests
	;;
trace)
	shift || true
	echo
	"$JAVA_BIN" -cp "$OUT" com.RigidBodyHarness "$@"
	;;
*)
	echo "run_tests.sh: unknown command '$CMD'" >&2
	echo "usage: run_tests.sh [--strict] [tests | trace <scenario> [frames] | gate | clean]" >&2
	exit 2
	;;
esac
