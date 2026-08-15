#!/usr/bin/env bash
# Capture a VICE reference frame and compare our VIC-II renderer against it.
#
#   capture.sh <name> <prg> <halt-address> [outdir]
#
# <halt-address> is the address of the program's terminal `jmp *`, in the form
# 0xNNNN. Read it from the 64tass label file rather than hardcoding it.
#
# VICE is run twice, deliberately:
#
#   1. -limitcycles + -exitscreenshot for the golden image. Trying to take the
#      screenshot from the monitor produced no file, and breaking early caught
#      the loader rather than the program.
#   2. -initbreak <halt> + -moncommands to dump RAM and I/O.
#
# Two runs are safe because a program that halts in a tight loop is
# deterministic: the state at the halt is the state the screenshot shows.
set -euo pipefail

VICE_BIN="${VICE_BIN:-/c/Emulation/Commodore/vice/GTK3VICE-3.9-win64/bin/xscpu64.exe}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHARGEN="${CHARGEN:-$ROOT/ROMs/chargen.rom}"

NAME="${1:?usage: capture.sh <name> <prg> <halt-address> [outdir]}"
PRG="${2:?missing prg}"
HALT="${3:?missing halt address, e.g. 0x089a}"
OUT="${4:-$ROOT/C64Tests/captures/viceconf}"

[ -x "$VICE_BIN" ] || { echo "xscpu64 not found at $VICE_BIN (set VICE_BIN)"; exit 1; }
mkdir -p "$OUT"

# Windows-native paths, because VICE does not understand the MSYS forms.
win() { cygpath -w "$1" 2>/dev/null | sed 's/\\/\//g' || echo "$1"; }
PRG_W="$(win "$PRG")"

echo "[1/3] golden image"
"$VICE_BIN" -default -warp -limitcycles 90000000 \
            -exitscreenshot "$(win "$OUT/$NAME.png")" \
            -autostart "$PRG_W" >/dev/null 2>&1 || true

echo "[2/3] machine state at $HALT"
MON="$OUT/$NAME.mon"
cat > "$MON" <<EOF
bank ram
save "$(win "$OUT/$NAME.ram")" 0 0000 ffff
bank io
save "$(win "$OUT/$NAME.io")" 0 d000 dfff
quit
EOF
"$VICE_BIN" -default -warp -initbreak "$HALT" \
            -moncommands "$(win "$MON")" \
            -autostart "$PRG_W" >/dev/null 2>&1 || true

for f in "$OUT/$NAME.png" "$OUT/$NAME.ram" "$OUT/$NAME.io"; do
	[ -s "$f" ] || { echo "capture failed: $f is missing or empty"; exit 1; }
done

echo "[3/3] render and compare"
# Built by `make viceconf` from the same objects as the test suite, so a fix
# cannot pass here and behave differently on the card.
RENDER="$ROOT/build/host/render_state"
REPLAY_BIN="$ROOT/build/host/replay_state"
[ -x "$RENDER" ] || make -C "$ROOT" viceconf >/dev/null || {
	echo "cannot build the conformance tools (make viceconf)"; exit 1; }

# A program that stores a collision signature at $C000/$C001 gets those checked
# too. $D01E and $D01F clear on read, so the guest must sample them itself.
COLLIDE=""
if [ "${COLLISIONS:-}" = "1" ]; then COLLIDE="--collisions"; fi

RC=0
"$RENDER" "$OUT/$NAME.ram" "$OUT/$NAME.io" "$CHARGEN" "$OUT/$NAME.raw" $COLLIDE || RC=1

# A raster program is several machine states inside one frame, and a state dump
# is one state, so render_state's output cannot match it and its percentage
# means nothing. REPLAY=1 answers that properly: replay_state boots the real
# machine, runs the program, and rebuilds the frame from the genuine write log
# through the shipping band planner -- which IS comparable to the screenshot,
# because a VICE screenshot is the correctly rendered multi-state frame.
if [ "${REPLAY:-}" = "1" ]; then
	"$REPLAY_BIN" "$PRG" "$CHARGEN" "$OUT/$NAME.replay.raw" || RC=1
	python3 "$ROOT/Tools/viceconf/compare.py" \
		"$OUT/$NAME.replay.raw" "$OUT/$NAME.png" "$OUT/$NAME.diff.png" || RC=1
else
	python3 "$ROOT/Tools/viceconf/compare.py" \
		"$OUT/$NAME.raw" "$OUT/$NAME.png" "$OUT/$NAME.diff.png" || RC=1
fi
exit $RC
