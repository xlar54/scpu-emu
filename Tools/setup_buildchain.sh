#!/usr/bin/env bash
#
# SCPU-EMU - assemble the bare-metal build chain.
#
# Fetches an AArch64 bare-metal toolchain and Circle 44.3 into _toolchain/,
# applies the RAD Expansion Unit's Circle build settings, and lays out the tree
# the firmware Makefile expects:
#
#   _toolchain/circle/            <- CIRCLEHOME
#     Config.mk                   RASPPI=3, AARCH=64
#     include/circle/sysconfig.h  <- Firmware/Circle/sysconfig.h (RAD's)
#     Source/
#       Rules.mk                  <- Firmware/Circle/Rules.mk    (RAD's)
#       Firmware/                 <- a copy of this repo's Source/
#
# Everything lands under _toolchain/, which is git-ignored. Re-running is cheap:
# downloads and Circle library builds are skipped if already present.
#
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TC="$REPO/_toolchain"
CIRCLE="$TC/circle"
GCCDIR="$TC/gcc-aarch64"

# Toolchain archive depends on the build host. ARM publishes a separate build
# per platform; picking the wrong one silently produces a toolchain that will
# not execute.
ARM_BASE="https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel"
case "$(uname -s)" in
	MINGW*|MSYS*|CYGWIN*)
		TOOLCHAIN_FILE="arm-gnu-toolchain-13.2.rel1-mingw-w64-i686-aarch64-none-elf.zip"
		TOOLCHAIN_SHA="7d35492cc0255e54b5b58259ccde1b1ab9efc494a70bc6f9ed3e601a2c607605"
		TOOLCHAIN_KIND="zip" ;;
	Linux)
		TOOLCHAIN_FILE="arm-gnu-toolchain-13.2.rel1-x86_64-aarch64-none-elf.tar.xz"
		TOOLCHAIN_SHA=""          # unverified: not the platform this was built on
		TOOLCHAIN_KIND="tar" ;;
	Darwin)
		TOOLCHAIN_FILE="arm-gnu-toolchain-13.2.rel1-darwin-arm64-aarch64-none-elf.tar.xz"
		TOOLCHAIN_SHA=""
		TOOLCHAIN_KIND="tar" ;;
	*)
		echo "ERROR: unrecognised host $(uname -s). Install aarch64-none-elf-gcc"
		echo "       manually and put it on PATH, then re-run."
		exit 1 ;;
esac

TOOLCHAIN_URL="$ARM_BASE/$TOOLCHAIN_FILE"
CIRCLE_URL="https://github.com/rsta2/circle/archive/refs/tags/Step44.3.tar.gz"
CIRCLE_SHA="5373855c5efc633ba0603b07675671436a0ff3e1edce4a2c809bd8d76140d469"

mkdir -p "$TC"

say() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }

# Refuse to unpack an archive whose contents we did not expect. These are large
# binaries fetched over the network and then executed, so an unverified download
# is a supply-chain hole. An empty expected hash means we have no recorded value
# for this platform; warn rather than pretend it was checked.
verify_sha() {
	local file="$1" expected="$2" label="$3"
	if [ -z "$expected" ]; then
		echo "WARNING: no recorded SHA-256 for $label on this platform -- not verified."
		echo "         Compare against ARM's published checksums before trusting it."
		return 0
	fi
	local actual
	actual="$(sha256sum "$file" | cut -d' ' -f1)"
	if [ "$actual" != "$expected" ]; then
		echo "ERROR: checksum mismatch for $label"
		echo "  expected $expected"
		echo "  got      $actual"
		echo "Refusing to continue. Delete $file and retry, or update the"
		echo "expected hash in this script if you changed the pinned version."
		exit 1
	fi
	say "Verified $label (sha256 ok)"
}

# --- toolchain -------------------------------------------------------------
if ! command -v aarch64-none-elf-gcc >/dev/null 2>&1 && [ ! -x "$GCCDIR/bin/aarch64-none-elf-gcc.exe" ] \
   && [ ! -x "$GCCDIR/bin/aarch64-none-elf-gcc" ]; then
	ARCHIVE="$TC/$TOOLCHAIN_FILE"
	if [ ! -f "$ARCHIVE" ]; then
		say "Downloading AArch64 bare-metal toolchain (large)"
		curl -L --retry 2 -o "$ARCHIVE" "$TOOLCHAIN_URL"
	fi
	verify_sha "$ARCHIVE" "$TOOLCHAIN_SHA" "toolchain"

	say "Extracting toolchain"
	rm -rf "$GCCDIR" "$TC/_gccx"
	mkdir -p "$TC/_gccx"
	if [ "$TOOLCHAIN_KIND" = "zip" ]; then
		unzip -q "$ARCHIVE" -d "$TC/_gccx"
	else
		tar xf "$ARCHIVE" -C "$TC/_gccx"
	fi
	# the archive contains a single top-level directory
	mv "$TC/_gccx/"*/ "$GCCDIR"
	rmdir "$TC/_gccx" 2>/dev/null || true
fi

if [ -d "$GCCDIR/bin" ]; then
	export PATH="$GCCDIR/bin:$PATH"
fi

command -v aarch64-none-elf-gcc >/dev/null 2>&1 \
	|| { echo "ERROR: aarch64-none-elf-gcc not on PATH after setup"; exit 1; }
say "Toolchain: $(aarch64-none-elf-gcc --version | head -1)"

# --- circle ----------------------------------------------------------------
if [ ! -d "$CIRCLE" ]; then
	if [ ! -f "$TC/circle-44.3.tar.gz" ]; then
		say "Downloading Circle 44.3"
		curl -L --retry 2 -o "$TC/circle-44.3.tar.gz" "$CIRCLE_URL"
	fi
	verify_sha "$TC/circle-44.3.tar.gz" "$CIRCLE_SHA" "Circle 44.3"
	say "Extracting Circle"
	rm -rf "$TC/_circx"; mkdir -p "$TC/_circx"
	tar xzf "$TC/circle-44.3.tar.gz" -C "$TC/_circx"
	mv "$TC/_circx/"*/ "$CIRCLE"
	rmdir "$TC/_circx" 2>/dev/null || true
fi

# --- RAD's build settings --------------------------------------------------
# RAD is explicit that its Circle settings must be used; the stock ones are not
# expected to work correctly for this kind of hard-real-time bus driving.
say "Applying RAD Circle settings"
cp -f "$REPO/Firmware/Circle/sysconfig.h" "$CIRCLE/include/circle/sysconfig.h"
mkdir -p "$CIRCLE/Source"
cp -f "$REPO/Firmware/Circle/Rules.mk"    "$CIRCLE/Source/Rules.mk"

cat > "$CIRCLE/Config.mk" <<'CFG'
RASPPI = 3
AARCH = 64
PREFIX64 = aarch64-none-elf-
CFG

# --- build Circle libraries ------------------------------------------------

# Circle compiles options from sysconfig.h into libcircle itself.  Do not keep
# an old single-core library after changing those options: the application can
# compile against the new header and then fail at link time (or, worse, retain
# an ABI-incompatible library).  Hash the complete build settings and rebuild
# all linked Circle libraries whenever they change.
CIRCLE_SETTINGS_SHA="$(sha256sum "$CIRCLE/include/circle/sysconfig.h" \
	"$CIRCLE/Config.mk" | sha256sum | cut -d' ' -f1)"
CIRCLE_SETTINGS_STAMP="$CIRCLE/.scpu-circle-settings.sha256"
OLD_CIRCLE_SETTINGS_SHA="$(cat "$CIRCLE_SETTINGS_STAMP" 2>/dev/null || true)"

if [ ! -f "$CIRCLE/lib/libcircle.a" ] \
   || [ "$CIRCLE_SETTINGS_SHA" != "$OLD_CIRCLE_SETTINGS_SHA" ]; then
	say "Building Circle libraries (settings changed or first build)"
	if [ -f "$CIRCLE/lib/libcircle.a" ]; then
		( cd "$CIRCLE/addon/linux"  && make clean )
		( cd "$CIRCLE/addon/fatfs"  && make clean )
		( cd "$CIRCLE/addon/SDCard" && make clean )
		( cd "$CIRCLE/lib/sched"    && make clean )
		( cd "$CIRCLE/lib/fs"       && make clean )
		( cd "$CIRCLE/lib"          && make clean )
	fi
	( cd "$CIRCLE/lib"          && make -j"$(nproc)" )
	( cd "$CIRCLE/lib/fs"       && make -j"$(nproc)" )
	( cd "$CIRCLE/lib/sched"    && make -j"$(nproc)" )
	( cd "$CIRCLE/addon/SDCard" && make -j"$(nproc)" )
	( cd "$CIRCLE/addon/fatfs"  && make -j"$(nproc)" )
	( cd "$CIRCLE/addon/linux"  && make -j"$(nproc)" )
	echo "$CIRCLE_SETTINGS_SHA" > "$CIRCLE_SETTINGS_STAMP"
else
	say "Circle libraries already built"
fi

# --- ARM stub --------------------------------------------------------------
# Must be built and placed on the boot partition, and selected from config.txt
# with armstub=. Without it the GPU firmware runs its default stub, L1 data
# prefetching stays enabled, and the cycle-counted bus windows pick up
# nondeterministic stalls.
say "Building RAD ARM stub"
( cd "$REPO/Firmware/ARMSTUB" && make -f Makefile clean >/dev/null 2>&1 || true )
( cd "$REPO/Firmware/ARMSTUB" && make )
[ -f "$REPO/Firmware/ARMSTUB/rad-prefetch.bin" ] \
	|| { echo "ERROR: rad-prefetch.bin was not produced"; exit 1; }

# --- stage our sources -----------------------------------------------------
say "Staging SCPU-EMU sources into the Circle tree"
rm -rf "$CIRCLE/Source/Firmware"
mkdir -p "$CIRCLE/Source/Firmware"
cp -r "$REPO/Source/." "$CIRCLE/Source/Firmware/"

# --- build the firmware ----------------------------------------------------
say "Building SCPU-EMU firmware"
( cd "$CIRCLE/Source/Firmware" && make "$@" )

if [ -f "$CIRCLE/Source/Firmware/kernel8.img" ]; then
	cp -f "$CIRCLE/Source/Firmware/kernel8.img" "$REPO/Source/kernel8.img"

	# Keep every build, so any of them can be put back on the card instantly.
	# Hardware bring-up means testing changes that cannot be verified here, and
	# "put back the one that worked" needs to be one command, not a rebuild.
	mkdir -p "$REPO/build/kernels"
	N=$(ls "$REPO/build/kernels"/kernel8-*.img 2>/dev/null | wc -l)
	N=$((N + 1))
	ARCHIVED=$(printf "%s/build/kernels/kernel8-%03d.img" "$REPO" "$N")
	cp -f "$REPO/Source/kernel8.img" "$ARCHIVED"

	say "Built $(wc -c < "$REPO/Source/kernel8.img") bytes -> Source/kernel8.img"
	echo "Archived as $(basename "$ARCHIVED")"
	echo "Run 'make sdcard' to stage it for the SD card."
else
	echo "ERROR: kernel8.img was not produced"; exit 1
fi
