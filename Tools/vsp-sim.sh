#!/bin/sh
# Build and run the VSP window simulation on the host.
#
# This does NOT touch the Pi firmware and does NOT need the card. It runs
# Mario's real trigger sequence through the real CPU and memory code against a
# bus model that charges the MEASURED physical transaction costs, and prints the
# DEC-write -> INC-write window the VIC would see.
#
# THE DEPENDENCY-CHECKED REBUILD IS NOT OPTIONAL.
# An earlier version of this script linked whatever .o files happened to be in
# build/host, so a change to c64_memory.cpp or any header could be silently
# ignored: the simulation would score the PREVIOUS source while printing a
# result for the current one. This project has already been bitten by exactly
# that -- an incremental make over a copied build tree reported the suite green
# with a stale c64_memory.o linked in, recorded at length in
# Tests/Integration/test_kernal_boot.cpp. A model that can run against stale
# objects is worse than no model, because its output still looks like a
# measurement. `make` resolves the header dependencies (-MMD -MP) and rebuilds
# what changed; only then are the objects linked.
#
# The object list comes from the Makefile's HOST_SRCS and NOT from every .cpp
# under Source: App/ and Bus/RAD/ are Pi-only and need Circle headers, so a
# blanket rebuild fails there. Taking the list from the Makefile also keeps the
# simulation on exactly the sources and flags the shipping suite uses, which is
# the whole point -- a simulation built from a different set is not simulating
# the thing being shipped.
set -e
cd "$(dirname "$0")/.."

OBJS=$(sed -n '/^HOST_SRCS/,/^$/p' Makefile \
       | grep -o 'Source/[A-Za-z0-9_/]*\.cpp' \
       | sed 's|^|build/host/|; s|\.cpp$|.o|' | tr '\n' ' ')

echo "== rebuilding host objects (dependency-checked) =="
make -j4 build/host/Tests/test_main.o $OBJS >/dev/null

echo "== linking simulation =="
g++ -O1 -g -std=c++14 -fno-rtti -fno-exceptions -DSCPU_HOST_BUILD \
    -o build/host/vsp-sim Tests/Integration/test_vsp_window.cpp \
    build/host/Tests/test_main.o $OBJS

exec ./build/host/vsp-sim "$@"
