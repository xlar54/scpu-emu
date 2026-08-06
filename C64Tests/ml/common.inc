.setcpu "65816"
.segment "CODE"

; The first two bytes are the PRG load address. The linked segment begins at
; $07ff so all labels after them have their true in-memory addresses.
.word $0801
.word basic_end
.word 10
.byte $9e
.byte "2061", 0
basic_end:
.word 0

.macro print_string text_label
    .local loop, done
    ldx #0
loop:
    lda text_label,x
    beq done
    jsr $ffd2
    inx
    bra loop
done:
.endmacro
