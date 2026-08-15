; 19-vicgeom.asm - VIC-II geometry ground truth for the VICE conformance harness
;
; Establishes a completely known screen so that Tools/viceconf can compare our
; renderer against VICE pixel for pixel and settle geometry questions by
; measurement rather than by argument:
;
;   * a blank screen in a single background colour, so every lit pixel below
;     comes from something we placed deliberately;
;   * one character cell at the top-left of the display window, which fixes the
;     display origin;
;   * sprite 0 at exactly X=24, Y=50 -- the canonical "top-left of the display"
;     sprite position -- which fixes the sprite-to-display offset.
;
; The program then halts in a tight loop at the label `halt`, whose address the
; capture script reads from the 64tass label file and passes to VICE's
; -initbreak. That makes the captured moment exact and repeatable.

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
                sta $dd00               ; bank bits 11 -> bank 0

                lda #$1b                ; DEN, RSEL, YSCROLL=3 (neutral)
                sta $d011
                lda #$c8                ; CSEL, XSCROLL=0
                sta $d016
                lda #$15                ; screen $0400, character ROM $1000
                sta $d018

                lda #$06                ; background blue
                sta $d021
                lda #$0e                ; border light blue
                sta $d020

                ; Clear the screen to spaces in black so only our marker shows.
                ldx #$00
clear           lda #$20
                sta $0400,x
                sta $0500,x
                sta $0600,x
                sta $06e8,x
                lda #$00
                sta $d800,x
                sta $d900,x
                sta $da00,x
                sta $dae8,x
                inx
                bne clear

                ; One solid marker cell at screen position 0, so the top-left of
                ; the display window is unambiguous. Character $a0 is the
                ; reversed space: all eight bits set on all eight lines.
                lda #$a0
                sta $0400
                lda #$01                ; white
                sta $d800

                ; Sprite 0: shape at $0340 (sprite pointer 13), solid block.
                ldx #$00
                lda #$ff
shape           sta $0340,x
                inx
                cpx #63
                bne shape
                lda #13
                sta $07f8               ; sprite 0 pointer

                lda #24                 ; X = 24: first display column
                sta $d000
                lda #50                 ; Y = 50: first display line
                sta $d001
                lda #$00
                sta $d010               ; X MSB clear
                lda #$02                ; red
                sta $d027
                lda #$00
                sta $d017               ; no Y expand
                sta $d01d               ; no X expand
                sta $d01c               ; not multicolour
                sta $d01b               ; in front of graphics
                lda #$01
                sta $d015               ; enable sprite 0 only

halt            jmp halt
