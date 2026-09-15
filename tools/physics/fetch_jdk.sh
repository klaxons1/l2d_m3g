#!/bin/sh
#
# Fetches a Java toolchain onto a box that has none - no root, no package
# manager, no JDK - so run_tests.sh can compile and run the physics tests.
#
#   eval "$(tools/physics/fetch_jdk.sh)"     # fetch if needed, set env vars
#   tools/physics/fetch_jdk.sh /some/dir     # fetch into a specific directory
#
# Prints the two exports run_tests.sh reads on stdout, and nothing else, so
# eval works. Progress and warnings go to stderr.
#
# What it fetches, into ${PHYSICS_JDK_DIR:-$HOME/.cache/l2d-physics-java}:
#
#   jdk4py (PyPI)            a JRE - java, but no javac. ~100 MB.
#   dataslope-tools-jar      OpenJDK 8's tools.jar, i.e. com.sun.tools.javac,
#   (npm registry)           ~5 MB. Driven by the JRE above it is a working
#                            javac 1.8, which is the last one that accepts
#                            -source 1.3, so it can run the CLDC gate.
#
# Both come over plain HTTPS from pypi.org / files.pythonhosted.org and
# registry.npmjs.org. Nothing is installed system wide and nothing needs
# root; delete the directory to undo it.
#
# Already have a JDK? Then this script does nothing: a javac and a java on
# PATH beat anything it would fetch, and the game build in CI never calls it.

set -e

DIR=${1:-${PHYSICS_JDK_DIR:-$HOME/.cache/l2d-physics-java}}
TOOLS_PKG=dataslope-tools-jar
TOOLS_VERSION=1.0.0
TOOLS_TGZ="$DIR/$TOOLS_PKG-$TOOLS_VERSION.tgz"
TOOLS_JAR="$DIR/package/tools.jar"
PY_TARGET="$DIR/jdk4py"
JRE_BIN="$PY_TARGET/jdk4py/java-runtime/bin/java"

say() { echo "fetch_jdk: $*" >&2; }

# --------------------------------------------------------------- runtime

JAVA_BIN=""
if command -v java > /dev/null 2>&1; then
	JAVA_BIN=`command -v java`
	say "using the java already on PATH"
elif [ -x "$JRE_BIN" ]; then
	JAVA_BIN="$JRE_BIN"
	say "using the cached JRE in $DIR"
else
	mkdir -p "$DIR"
	say "no java on PATH, fetching jdk4py from PyPI into $DIR (~100 MB)"
	PIP=""
	for cand in "python3 -m pip" "pip3" "pip"; do
		if $cand --version > /dev/null 2>&1; then
			PIP="$cand"
			break
		fi
	done
	if [ -z "$PIP" ]; then
		say "no pip either. Install any JDK, or set JAVA_BIN and JAVAC by hand."
		exit 2
	fi
	# shellcheck disable=SC2086
	$PIP install --quiet --disable-pip-version-check --target "$PY_TARGET" jdk4py
	if [ ! -x "$JRE_BIN" ]; then
		say "jdk4py installed but $JRE_BIN is missing - unexpected layout."
		say "Set JAVA_BIN to the java binary it did provide."
		find "$PY_TARGET" -name java -type f >&2 || true
		exit 2
	fi
	JAVA_BIN="$JRE_BIN"
	say "JRE ready"
fi

# ------------------------------------------------------------- compiler

JAVAC_CMD=""
if command -v javac > /dev/null 2>&1; then
	say "using the javac already on PATH"
elif [ -f "$TOOLS_JAR" ]; then
	say "using the cached tools.jar in $DIR"
else
	mkdir -p "$DIR"
	say "no javac on PATH, fetching OpenJDK 8 tools.jar from npm (~5 MB)"
	URL="https://registry.npmjs.org/$TOOLS_PKG/-/$TOOLS_PKG-$TOOLS_VERSION.tgz"
	got=""
	if command -v npm > /dev/null 2>&1; then
		if (cd "$DIR" && npm pack "$TOOLS_PKG@$TOOLS_VERSION" > /dev/null 2>&1); then
			got=yes
		fi
	fi
	if [ -z "$got" ] && command -v python3 > /dev/null 2>&1; then
		if python3 -c "
import shutil, sys, urllib.request
with urllib.request.urlopen('$URL', timeout=120) as r, open('$TOOLS_TGZ', 'wb') as f:
	shutil.copyfileobj(r, f)
" > /dev/null 2>&1; then
			got=yes
		fi
	fi
	if [ -z "$got" ]; then
		for dl in "curl -sSL -o" "wget -q -O"; do
			if command -v `echo "$dl" | cut -d' ' -f1` > /dev/null 2>&1; then
				# shellcheck disable=SC2086
				if $dl "$TOOLS_TGZ" "$URL"; then
					got=yes
					break
				fi
			fi
		done
	fi
	if [ -z "$got" ]; then
		say "could not download $URL"
		say "Fetch it by hand and set JAVA_TOOLS_JAR, or install any JDK."
		exit 2
	fi
	tar xzf "$TOOLS_TGZ" -C "$DIR"
	rm -f "$TOOLS_TGZ"
	if [ ! -f "$TOOLS_JAR" ]; then
		say "the tarball did not contain package/tools.jar"
		exit 2
	fi
	say "tools.jar ready"
fi

if ! command -v javac > /dev/null 2>&1; then
	JAVAC_CMD="$JAVA_BIN -cp $TOOLS_JAR com.sun.tools.javac.Main"
fi

# ---------------------------------------------------------------- verify

if [ -n "$JAVAC_CMD" ]; then
	# shellcheck disable=SC2086
	if ! $JAVAC_CMD -version > /dev/null 2>&1; then
		say "the fetched javac does not run:"
		# shellcheck disable=SC2086
		$JAVAC_CMD -version >&2 || true
		say "A modular JRE cannot host an old tools.jar; install a real JDK."
		exit 2
	fi
	ver=`$JAVAC_CMD -version 2>&1 | head -1`
	say "javac: $ver   java: $JAVA_BIN"
else
	say "javac: `javac -version 2>&1 | head -1`   java: $JAVA_BIN"
fi

# --------------------------------------------------------------- exports

echo "export JAVA_BIN='$JAVA_BIN'"
if [ -n "$JAVAC_CMD" ]; then
	echo "export JAVA_TOOLS_JAR='$TOOLS_JAR'"
fi
