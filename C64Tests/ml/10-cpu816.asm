.include "common.inc"

start:
    sei
    clc
    xce                 ; native mode
    rep #$30            ; 16-bit A, X, Y
    .a16
    .i16
    lda #$1234
    clc
    adc #$4321
    cmp #$5555
    bne fail_native
    ldx #$abcd
    phx
    ldx #0
    plx
    cpx #$abcd
    bne fail_native
    sep #$30
    .a8
    .i8
    sec
    xce                 ; back to emulation before KERNAL
    cli
    print_string pass_msg
    rts

fail_native:
    sep #$30
    .a8
    .i8
    sec
    xce
    cli
    print_string fail_msg
    rts

pass_msg: .byte 13,"CPU 65816: PASS",13,0
fail_msg: .byte 13,"CPU 65816: FAIL",13,0

