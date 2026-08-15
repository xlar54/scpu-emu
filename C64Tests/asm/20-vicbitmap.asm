; 20-vicbitmap.asm - multicolour bitmap and sprite variants, for VICE conformance
;
; One frame that exercises the paths Stunt Car Racer and GEOS depend on:
;
;   * multicolour bitmap with every bit-pair present in every cell. The byte
;     $1B is %00 %01 %10 %11, so a uniform fill still produces all four colour
;     sources side by side;
;   * per-cell variation of both screen nibbles and colour RAM, so pair 01, 10
;     and 11 each resolve from a different place and a swapped source shows;
;   * four sprites covering the attribute combinations that decode differently:
;     plain, multicolour, X-expanded and Y-expanded, plus one behind the
;     graphics so $D01B priority is exercised against real foreground pixels.
;
; Halts at `halt` for Tools/viceconf/capture.sh.

                * = $0801

; BASIC stub: 10 SYS 2061
                .word eob
                .word 10
                .byte $9e
                .text "2061"
                .byte 0
eob             .word 0

start
                sei
                lda $dd02               ; VIC bank 0
                ora #$03
                sta $dd02
                lda $dd00
                ora #$03
                sta $dd00

                lda #$3b                ; DEN, RSEL, YSCROLL=3, BMM
                sta $d011
                lda #$d8                ; CSEL, XSCROLL=0, MCM
                sta $d016
                lda #$18                ; screen $0400, bitmap $2000
                sta $d018
                lda #$00                ; black background = pair 00
                sta $d021
                lda #$0b                ; dark grey border
                sta $d020

                ; Bitmap: every byte $1B so each cell shows 00 01 10 11.
                lda #<$2000
                sta $fb
                lda #>$2000
                sta $fc
                ldx #$20                ; 32 pages = 8000 bytes, rounded up
fillpage        ldy #$00
fillbyte        lda #$1b
                sta ($fb),y
                iny
                bne fillbyte
                inc $fc
                dex
                bne fillpage

                ; Screen nibbles and colour RAM, varied per cell so a
                ; misattributed source cannot hide behind a uniform palette.
                ldx #$00
attrs           txa
                and #$0f
                asl
                asl
                asl
                asl                     ; upper nibble = pair 01 colour
                sta $fd
                txa
                lsr
                lsr
                and #$0f
                ora $fd
                sta $0400,x
                sta $0500,x
                sta $0600,x
                sta $06e8,x
                txa
                eor #$0f
                and #$0f
                sta $d800,x             ; colour RAM = pair 11
                sta $d900,x
                sta $da00,x
                sta $dae8,x
                inx
                bne attrs

                ; Sprite shapes: solid block at $0340, vertical bars at $0380.
                ldx #$00
shapes          lda #$ff
                sta $0340,x
                lda #$cc
                sta $0380,x
                inx
                cpx #63
                bne shapes
                lda #13
                sta $07f8               ; sprite 0 -> $0340
                sta $07f9               ; sprite 1 -> $0340
                lda #14
                sta $07fa               ; sprite 2 -> $0380
                sta $07fb               ; sprite 3 -> $0380

                ; Sprite 0: plain, over graphics.
                lda #40
                sta $d000
                lda #60
                sta $d001
                lda #$01
                sta $d027
                ; Sprite 1: behind graphics, same row, further right.
                lda #90
                sta $d002
                lda #60
                sta $d003
                lda #$07
                sta $d029
                ; Sprite 2: multicolour.
                lda #140
                sta $d004
                lda #60
                sta $d005
                lda #$05
                sta $d02a
                ; Sprite 3: X and Y expanded.
                lda #190
                sta $d006
                lda #60
                sta $d007
                lda #$0a
                sta $d02b

                lda #$00
                sta $d010               ; all X MSBs clear
                lda #$04                ; sprite 2 multicolour
                sta $d01c
                lda #$08                ; sprite 3 X-expanded
                sta $d01d
                sta $d017               ; sprite 3 Y-expanded
                lda #$02                ; sprite 1 behind graphics
                sta $d01b
                lda #$08                ; sprite multicolour 0
                sta $d025
                lda #$0f                ; sprite multicolour 1
                sta $d026
                lda #$0f
                sta $d015               ; enable sprites 0-3

halt            jmp halt
