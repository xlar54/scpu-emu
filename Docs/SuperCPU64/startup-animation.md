# SuperCPU DOS 2.04 startup animation

The C64 startup animation is stored in `scpu-dos-2.04.bin` as a relocatable
blob. This was traced from the reset path in the local 128 KB image whose SHA-1
is `6aa529a7b1b6de53e8979e407a77b4d5657727f5`.

## Boot path

- The reset vector at ROM offsets `$FFFC-$FFFD` is `$FE00`.
- `$FE00` selects the machine path; the C64 path reaches the long-ROM startup
  code through `$FC90` (`JML $F800FC`).
- At ROM offset `$01E0`, native-mode code performs an `MVN` of `$0E01` bytes
  from `$F8:D000` to bank-0 RAM `$0E00`, then `JML $001400`.
- Consequently ROM `$F8:D600` becomes RAM `$1400`, and ROM `$F8:DC00` becomes
  RAM `$1A00`.

The relevant bytes at ROM offset `$01E0` decode as:

```text
PHB / CLC / XCE / REP #$30
LDA #$0E00             ; MVN count minus one
LDX #$D000             ; source offset in bank $F8
LDY #$0E00             ; destination offset in bank $00
MVN $F8,$00
SEC / XCE / PLB
JML $001400
```

## Animation entry and replay

RAM `$1400` is a trampoline (`JMP $1A19`). `$1A19` handles the machine mode and
jumps to the animation body at `$1406`. The body initializes the VIC, installs
a raster IRQ, prepares the double-buffered graphics and sprites, runs the
animation, and eventually jumps through `$1403` to `$1A00`.

This is not an ordinary subroutine: `$1A00` continues to the appropriate KERNAL
cold-start path rather than returning to its caller.

Calling `$1400` a second time is not sufficient. The animation is self-modifying
and leaves its working copy changed after the initial boot. Re-entering that
dirty copy produces a partially drawn static screen in VICE.

ROM `$F8:01E0` is the copy-and-launch routine described above. The BASIC test
places this five-byte trampoline at `$C000` and calls it with `SYS 49152`:

```text
78             SEI
5C E0 01 F8    JML $F801E0
```

The ROM routine refreshes `$0E00-$1C00`, launches the clean animation at
`$1400`, and cold-starts BASIC when it finishes. It never returns to `$C000`.

The companion `scpu-2.04.info` file records the ranges used with `da65` while
tracing the image.
