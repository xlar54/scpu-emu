; 64tass adds the two-byte PRG load address with --cbm-prg.
* = $0801

.word basic_end
.word 10
.byte $9e
.text "2061"
.byte 0
basic_end:
.word 0

print_string .macro text_label
    ldx #0
_loop:
    lda \text_label,x
    beq _done
    jsr $ffd2
    inx
    bra _loop
_done:
.endm
