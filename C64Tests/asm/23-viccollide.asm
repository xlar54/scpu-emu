; 23-viccollide.asm - sprite collision conformance
;
; The collision registers are read-to-clear, so they cannot be sampled from
; VICE's monitor without destroying what they hold. The program therefore reads
; them itself and stores the values in ordinary RAM, which the state dump
; captures alongside everything else. Tools/viceconf then compares those bytes
; against what our renderer derives from the very same frame.
;
; One frame, four scenarios, chosen so every bit is attributable:
;
;   sprites 0+1  overlap in open display, away from any foreground   -> $D01E 0,1
;   sprite  2    over a solid character cell, no other sprite near   -> $D01F 2
;   sprite  3    alone over background, touching nothing             -> neither
;   sprites 4+5  overlap ENTIRELY in the left border                 -> $D01E 4,5
;
; The last is the interesting one. Collision detection lives in the sprite
; sequencer and is independent of the border unit, so real hardware reports it.
; A renderer that only looks inside the display window reports nothing, and a
; game that parks sprites off-screen then behaves differently.
;
; Results land at $C000 ($D01E) and $C001 ($D01F). Halts at `halt`.

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
                lda $dd02
                ora #$03
                sta $dd02
                lda $dd00
                ora #$03
                sta $dd00               ; VIC bank 0

                lda #$1b
                sta $d011
                lda #$c8
                sta $d016
                lda #$15                ; screen $0400, character ROM
                sta $d018
                lda #$06
                sta $d021
                lda #$0e
                sta $d020

                ; Blank screen, so the only foreground anywhere is the one cell
                ; placed below and every sprite/background bit is attributable.
                ldx #$00
clear           lda #$20
                sta $0400,x
                sta $0500,x
                sta $0600,x
                sta $06e8,x
                lda #$01
                sta $d800,x
                sta $d900,x
                sta $da00,x
                sta $dae8,x
                inx
                bne clear

                ; One solid cell at screen row 5, column 5 -> $0400+205.
                lda #$a0
                sta $04cd

                ; Solid 24x21 shape at $0340 (pointer 13) for every sprite.
                ldx #$00
shape           lda #$ff
                sta $0340,x
                inx
                cpx #63
                bne shape
                ldx #$00
ptrs            lda #13
                sta $07f8,x
                inx
                cpx #8
                bne ptrs

                ; 0 and 1: overlapping, open display, clear of the solid cell.
                lda #150
                sta $d000
                lda #60
                sta $d001
                lda #158
                sta $d002
                lda #60
                sta $d003

                ; 2: over the solid cell. Cell (5,5) begins at VIC X 24+40=64,
                ; Y 50+40=90.
                lda #64
                sta $d004
                lda #90
                sta $d005

                ; 3: alone, over background, touching nothing.
                lda #240
                sta $d006
                lda #180
                sta $d007

                ; 4 and 5: overlapping at X=0, which is 24 pixels wide starting
                ; well left of the display window -- entirely inside the border.
                lda #0
                sta $d008
                lda #140
                sta $d009
                lda #0
                sta $d00a
                lda #140
                sta $d00b

                lda #$00
                sta $d010               ; every X MSB clear
                sta $d017               ; no Y expansion
                sta $d01d               ; no X expansion
                sta $d01c               ; none multicolour
                sta $d01b               ; all in front of graphics
                lda #$01
                sta $d027
                sta $d028
                sta $d029
                sta $d02a
                sta $d02b
                sta $d02c
                lda #$3f
                sta $d015               ; enable sprites 0-5

                ; Settle, then measure exactly one frame: clear both latches at
                ; the top of a frame, let a whole frame accumulate, and read.
                jsr waitframe
                jsr waitframe
                lda $d01e
                lda $d01f
                jsr waitframe
                lda $d01e
                sta $c000
                lda $d01f
                sta $c001
                ; Signature, so the harness can tell a real result from
                ; uninitialised RAM. Without it a program that stores nothing
                ; reads back $FF and looks like a spectacular failure.
                lda #$5a
                sta $c002
                lda #$a5
                sta $c003

halt            jmp halt

; Wait for the start of the next frame. Raster 251 is inside the lower border on
; both standards, so this is a frame boundary without depending on line counts.
waitframe
                lda #251
wf1             cmp $d012
                bne wf1
wf2             cmp $d012
                beq wf2
                rts
