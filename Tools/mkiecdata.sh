#!/usr/bin/env bash
# Build the reference file that C64Tests/asm/28-iecload.asm loads and checks.
#
#   mkiecdata.sh [output.prg]
#
# The content is deliberately NOT random and NOT constant. Both hide faults:
# constant data survives a transfer that repeats or drops a byte, and random
# data makes a failure impossible to read by eye. This is a counter pattern with
# a per-page marker, so a corrupt load is diagnosable from a hex dump alone --
# a repeated byte, a dropped byte and a swapped pair each look different.
#
# Prints the CRC-16/XMODEM, which goes in `wantcrc` in the assembly file. The
# CRC here is computed by the same algorithm the 6502 code implements, so a
# mismatch on hardware means the transfer differed, not that the two disagree
# about arithmetic.
set -euo pipefail

OUT="${1:-C64Tests/disk/TESTDATA.prg}"
LENGTH=4096

mkdir -p "$(dirname "$OUT")"

python3 - "$OUT" "$LENGTH" <<'EOF'
import sys

path, length = sys.argv[1], int(sys.argv[2])

# Byte i = (i + page marker) so every byte differs from its neighbours and each
# 256-byte page is distinguishable from every other. A dropped byte shifts the
# whole tail by one and is obvious; a repeated byte breaks the run locally.
data = bytearray()
for i in range(length):
    page = i >> 8
    data.append((i + page * 7 + 0x41) & 0xFF)

# Written as a .prg: two bytes of load address, then the payload. c1541 -write
# stores those first two bytes AS the load address rather than as data, so the
# payload is what actually lands in memory -- and the payload alone is what the
# CRC below covers. Get this wrong and the C64 checksums 4094 bytes while the
# script checksums 4096, which looks exactly like a corrupt transfer.
with open(path, 'wb') as f:
    f.write(bytes([0x00, 0x40]))    # $4000, matching LOADADDR in the test
    f.write(data)

# CRC-16/XMODEM: poly 0x1021, init 0x0000, no reflection.
crc = 0
for b in data:
    crc ^= b << 8
    for _ in range(8):
        crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF

print(f"wrote {path}: {len(data)} bytes")
print(f"  wantcrc = ${crc:04X}")
print(f"  wantlen = ${len(data):04X}")
EOF

echo
echo "Put it on the test disk as TESTDATA, e.g."
echo "  c1541 -attach C64Tests/SCPU-TESTS.d81 -write $OUT testdata"
