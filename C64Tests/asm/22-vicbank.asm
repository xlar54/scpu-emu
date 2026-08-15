; 22-vicbank.asm - hires bitmap out of VIC bank 1
;
; Two things at once, both previously unverified against a reference:
;
;   * hires bitmap, where the two colours of every cell come from the screen
;     nibbles rather than from colour RAM -- a different source to every other
;     mode, and easy to swap;
;   * VIC bank 1. Bank selection is inverted in CIA2 port A, and the character
;     ROM window at $1000-$1FFF exists only in banks 0 and 2, so a renderer
;     that applies the window unconditionally is wrong exactly here. Bank 1 is
;     also plain RAM from the CPU's point of view, unlike bank 3, so the test
;     can write its own bitmap without banking ROM out.
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
                lda $dd02               ; CIA2 port A bits 0-1 as outputs
                ora #$03
                sta $dd02
                lda $dd00
                and #$fc
                ora #$02                ; %10 -> VIC bank 1 ($4000-$7FFF)
                sta $dd00

                lda #$3b                ; DEN, RSEL, YSCROLL=3, BMM
                sta $d011
                lda #$c8                ; CSEL, XSCROLL=0, no MCM
                sta $d016
                lda #$18                ; screen $4400, bitmap $6000 within bank
                sta $d018
                lda #$0e                ; border
                sta $d020

                ; Bitmap at $6000: a walking pattern so each cell differs from
                ; its neighbour and a one-cell addressing error is visible.
                lda #<$6000
                sta $fb
                lda #>$6000
                sta $fc
                ldx #$20                ; 32 pages covers the 8000-byte bitmap
                ldy #$00
bmpage          tya
                eor $fc
                sta ($fb),y
                iny
                bne bmpage
                inc $fc
                dex
                bne bmpage

                ; Screen at $4400: both nibbles vary, so the set-bit colour and
                ; the clear-bit colour come from visibly different places.
                ldx #$00
scr             txa
                and #$0f
                asl
                asl
                asl
                asl
                sta $fd
                txa
                lsr
                lsr
                lsr
                and #$0f
                eor #$0f
                ora $fd
                sta $4400,x
                sta $4500,x
                sta $4600,x
                sta $46e8,x
                inx
                bne scr

                lda #$00
                sta $d015               ; no sprites

halt            jmp halt
