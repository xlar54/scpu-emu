; 21-victext.asm - multicolour text, fine scroll and reduced apertures
;
; Covers three things that are easy to get subtly wrong and that no previous
; capture exercised:
;
;   * multicolour text where colour-RAM bit 3 decides per cell whether the cell
;     is hires or multicolour. Half the cells clear it, so both decoders run in
;     the same frame against the same character data;
;   * non-zero XSCROLL and YSCROLL, which move the matrix relative to the
;     border and are the usual source of one-pixel disagreements;
;   * CSEL=0 and RSEL=0, the 38-column / 24-row apertures, which narrow the
;     border and clip the display on all four sides.
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

                ; RSEL=0 (24 rows), DEN, YSCROLL=5 -- deliberately not the
                ; neutral 3, so a renderer that ignores it is off by two rows.
                lda #$15
                sta $d011
                ; CSEL=0 (38 columns), MCM, XSCROLL=3.
                lda #$13
                sta $d016
                lda #$15                ; screen $0400, character ROM $1000
                sta $d018
                lda #$06                ; background 0
                sta $d021
                lda #$09                ; background 1 -> bit pair 01
                sta $d022
                lda #$02                ; background 2 -> bit pair 10
                sta $d023
                lda #$0b                ; border
                sta $d020

                ; Screen: a rolling character code so glyph fetch varies per
                ; cell rather than repeating one pattern.
                ldx #$00
fill            txa
                sta $0400,x
                clc
                adc #$40
                sta $0500,x
                clc
                adc #$40
                sta $0600,x
                clc
                adc #$40
                sta $06e8,x

                ; Colour RAM: bit 3 alternates every cell, so odd cells decode
                ; as multicolour (bit pairs) and even cells as hires, side by
                ; side with identical glyph data.
                txa
                and #$07
                sta $fb
                txa
                and #$01
                beq hires
                lda $fb
                ora #$08
hires           ora $fb
                sta $d800,x
                sta $d900,x
                sta $da00,x
                sta $dae8,x
                inx
                bne fill

                lda #$00
                sta $d015               ; no sprites: text decode only

halt            jmp halt
