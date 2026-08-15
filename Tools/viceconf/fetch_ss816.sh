#!/usr/bin/env bash
# Fetch SingleStepTests 65816 vectors.
#
#   fetch_ss816.sh [dir] [opcode...]
#
# The full set is 512 files (256 opcodes x emulation/native) and about 3GB, so
# this fetches a subset by default: the NATIVE-mode opcodes that
# Tests/CPU/test_w65c816_diff.cpp structurally cannot reach, because that test
# compares our 65816 against our own 6502 and is therefore emulation-mode only.
#
# Run them with:  build/host/ss816 <dir>/*.n.json
#
# Vectors are not committed. They are large, they are someone else's data, and
# they change independently of this repository.
set -euo pipefail

DIR="${1:-/c/tmp/ss816/v1}"
shift || true

# Default set, chosen for what has no external coverage at all:
#   54 44  MVN/MVP block moves       22 6b  JSL/RTL
#   af a7  long and [dp] addressing  3f     long,X
#   c2 e2  REP/SEP                   fb     XCE
#   eb     XBA                       0b 2b  PHD/PLD
#   8b ab  PHB/PLB                   f4 d4 62  PEA/PEI/PER
#   1b 3b  TCS/TSC                   69     ADC immediate, 16-bit
DEFAULT_OPS="54 44 22 6b af a7 c2 e2 fb eb 0b 2b 8b ab f4 d4 62 1b 3b 69 3f"
OPS="${*:-$DEFAULT_OPS}"

BASE="https://raw.githubusercontent.com/SingleStepTests/65816/main/v1"
mkdir -p "$DIR"

echo "fetching into $DIR"
for op in $OPS; do
	if [ -s "$DIR/$op.n.json" ]; then
		echo "  $op.n.json  (already present)"
		continue
	fi
	curl -sL --max-time 300 -o "$DIR/$op.n.json" "$BASE/$op.n.json" &
done
wait

fail=0
for op in $OPS; do
	if [ ! -s "$DIR/$op.n.json" ]; then
		echo "  MISSING $op.n.json"
		fail=1
	fi
done
[ $fail -eq 0 ] || { echo "some downloads failed"; exit 1; }

echo
echo "fetched $(ls "$DIR"/*.n.json | wc -l) files, $(du -sh "$DIR" | cut -f1)"
echo "run: build/host/ss816 $DIR/*.n.json"
