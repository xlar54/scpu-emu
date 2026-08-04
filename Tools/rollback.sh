#!/usr/bin/env bash
# Put a previously built kernel back on the SD card.
#   Tools/rollback.sh            list what is available
#   Tools/rollback.sh <file>     copy that one to D:
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K="$REPO/build/kernels"

if [ $# -eq 0 ]; then
	echo "Available kernels:"
	ls -la "$K"/*.img 2>/dev/null | awk '{printf "  %-34s %8s bytes  %s %s\n", $9, $5, $6, $7}' || echo "  (none)"
	echo
	echo "Usage: Tools/rollback.sh <name>"
	exit 0
fi

SRC="$K/$(basename "$1")"
[ -f "$SRC" ] || { echo "no such kernel: $SRC"; exit 1; }
[ -d /d ] || { echo "SD card not present at D:"; exit 1; }

cp -f "$SRC" /d/kernel8.img
sync
echo "Restored $(basename "$SRC") -> D:/kernel8.img ($(stat -c%s /d/kernel8.img) bytes)"
