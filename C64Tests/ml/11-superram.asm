.include "common.inc"

start:
    print_string title
    sei
    clc
    xce
    rep #$10
    .i16
    ldx #0
probe_loop:
    sep #$20
    .a8
    lda addresses+2,x
    sta bank_text
    lda addresses+1,x
    sta z:pointer+1
    lda addresses,x
    sta z:pointer
    lda addresses+2,x
    sta z:pointer+2
    lda [<pointer]
    sta saved
    eor #$a5
    sta expected
    sta [<pointer]
    cmp [<pointer]
    bne probe_fail
    lda saved
    sta [<pointer]
    jsr print_bank_pass
    bra probe_next
probe_fail:
    lda saved
    sta [<pointer]
    jsr print_bank_fail
probe_next:
    inx
    inx
    inx
    cpx #12
    bne probe_loop
    sep #$30
    .i8
    sec
    xce
    cli
    rts

; Long-indirect pointer in direct page: low, high, bank. KERNAL may use this
; workspace while printing, so every probe rebuilds it before dereferencing.
pointer = $fb
saved:    .byte 0
expected: .byte 0
addresses:
    .byte <$011234, >$011234, ^$011234
    .byte <$022345, >$022345, ^$022345
    .byte <$7f3456, >$7f3456, ^$7f3456
    .byte <$f54567, >$f54567, ^$f54567

print_bank_pass:
    phx
    jsr leave_native
    print_string bank_prefix
    lda bank_text
    jsr print_hex
    print_string pass_suffix
    jsr enter_native
    plx
    rts
print_bank_fail:
    phx
    jsr leave_native
    print_string bank_prefix
    lda bank_text
    jsr print_hex
    print_string fail_suffix
    jsr enter_native
    plx
    rts
leave_native:
    sep #$30
    .a8
    .i8
    sec
    xce
    cli
    rts
enter_native:
    sei
    clc
    xce
    rep #$10
    .i16
    rts
print_hex:
    pha
    lsr
    lsr
    lsr
    lsr
    jsr nibble
    pla
    and #$0f
nibble:
    cmp #10
    bcc :+
    adc #6
:
    adc #'0'
    jmp $ffd2

title:       .byte 13,"24-BIT RAM PROBES",13,0
bank_prefix: .byte "BANK $",0
bank_text:   .byte 0
pass_suffix: .byte ": PASS",13,0
fail_suffix: .byte ": FAIL/NOT FITTED",13,0
