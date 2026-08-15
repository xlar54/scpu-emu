; 29-vicsprmux.asm - sprite pointers rewritten mid-frame
;
; Sprite pointers are the most commonly rewritten VIC-visible RAM in real
; software: they live in the last eight bytes of the screen matrix, and every
; sprite multiplexer changes them several times a frame. They are RAM, not
; registers, so they are NOT in the timestamped write log -- the renderer reads
; them from the once-per-frame snapshot and applies whatever value happened to
; be there at snapshot time to the entire frame.
;
; This is the isolation of that, and nothing else. Sprite Y stays fixed, so the
; sprite sequencer and its trigger latching are not involved; the ONLY thing
; that changes mid-frame is a pointer byte.
;
; The VIC re-fetches each sprite's pointer once per raster line while that
; sprite is displaying, so changing a pointer halfway down a sprite changes the
; remaining lines. Both sprites are Y-expanded to 42 lines and each has a raster
; split placed exactly at its halfway point:
;
;   sprite 0  X=100  Y=70   rasters  70..111   split at 91,  ptr 13 -> 14
;   sprite 1  X=200  Y=140  rasters 140..181   split at 161, ptr 13 -> 15
;
; Shapes are chosen so a wrong one is unmistakable rather than subtle:
;
;   13  solid           $FF $FF $FF
;   14  left third      $FF $00 $00
;   15  right third     $00 $00 $FF
;
; Correct output is therefore each sprite solid on top and one-third-wide below.
; A renderer using the snapshot shows the FINAL pointer for the whole sprite --
; both halves one-third wide -- so the error is the top half of each sprite,
; about 500 pixels apiece, and attributable by which half is wrong.
;
; A third split at raster 40 resets both pointers before either sprite starts,
; so the frame is self-contained and repeats identically.

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
                lda #$00
                sta $d021
                lda #$06
                sta $d020

                ; Blank screen: the sprites are the whole subject.
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

                ; --- three shapes ----------------------------------------
                ldx #$00
shapes          lda #$ff                ; 13: solid, at $0340
                sta $0340,x
                inx
                cpx #63
                bne shapes

                ldx #$00
sh14            lda #$ff                ; 14: left third, at $0380
                sta $0380,x
                lda #$00
                sta $0381,x
                sta $0382,x
                inx
                inx
                inx
                cpx #63
                bcc sh14

                ldx #$00
sh15            lda #$00                ; 15: right third, at $03c0
                sta $03c0,x
                sta $03c1,x
                lda #$ff
                sta $03c2,x
                inx
                inx
                inx
                cpx #63
                bcc sh15

                ; --- two sprites, Y-expanded, fixed positions ------------
                lda #100
                sta $d000               ; sprite 0 X
                lda #70
                sta $d001               ; sprite 0 Y
                lda #200
                sta $d002               ; sprite 1 X
                lda #140
                sta $d003               ; sprite 1 Y
                lda #$00
                sta $d010               ; no X MSBs
                sta $d01c               ; not multicolour
                sta $d01d               ; no X expansion
                sta $d01b               ; in front of the background
                lda #$03
                sta $d017               ; Y-expand sprites 0 and 1
                lda #$01
                sta $d027
                lda #$07
                sta $d028
                lda #13
                sta $07f8
                sta $07f9
                lda #$03
                sta $d015               ; sprites 0 and 1 enabled

                ; --- raster chain ----------------------------------------
                lda #<irq
                sta $0314
                lda #>irq
                sta $0315
                lda #$7f
                sta $dc0d               ; no CIA interrupts
                lda $dc0d
                lda #$01
                sta $d01a
                lda #$00
                sta band
                lda #40
                sta $d012
                lda $d011
                and #$7f
                sta $d011
                lda #$01
                sta $d019
                cli

halt            jmp halt

; ---------------------------------------------------------------------------
; Three splits. The pointer write is first in each handler so it lands as early
; in the line as interrupt latency allows.
irq
                ldx band
                cpx #$00
                bne notreset

                ; raster 40: reset both, before either sprite begins
                lda #13
                sta $07f8
                sta $07f9
                jmp advance

notreset        cpx #$01
                bne notfirst
                lda #14                 ; raster 91: sprite 0 halfway
                sta $07f8
                jmp advance

notfirst        lda #15                 ; raster 161: sprite 1 halfway
                sta $07f9

advance
                inx
                cpx #3
                bne setnext
                ldx #$00
setnext         stx band
                lda lines,x
                sta $d012

                lda #$01
                sta $d019               ; explicit acknowledge -- see 24-vicraster
                jmp $ea81

band            .byte 0
lines           .byte 40, 91, 161
