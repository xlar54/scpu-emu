.include "common.inc"

HWREGS_OPEN  = $d07e
HWREGS_CLOSE = $d07f
SCPU_DETECT  = $d0bc
SYS_RAM      = $d200
USER_RAM     = $d300

start:
    #print_string title_msg
    lda SCPU_DETECT
    bpl scpu_present
    jmp no_scpu
scpu_present:

    php
    sei
    cld

    ; Begin from the documented write-protected state.  The closed-gate
    ; check uses user RAM, never CMD's live $D2xx system scratch page.
    lda #0
    sta HWREGS_CLOSE
    lda USER_RAM
    sta old_byte
    eor #$ff
    sta USER_RAM
    lda USER_RAM
    cmp old_byte
    beq gate_ok
    lda #1
    sta gate_fail
gate_ok:
    ; Restore in case an implementation incorrectly accepted the closed write.
    lda #0
    sta HWREGS_OPEN
    lda old_byte
    sta USER_RAM
    lda #0
    sta HWREGS_CLOSE

    ; $D200 is SuperCPU DOS system RAM.  Keep IRQs masked and expose each
    ; changed byte only inside one short open/write/read/restore/close
    ; transaction, so DOS can never observe the test pattern.
    ldx #0
sys_loop:
    lda SYS_RAM,x
    sta old_byte
    txa
    eor #$a5
    cmp old_byte
    bne sys_pattern_ready
    eor #$ff
sys_pattern_ready:
    sta test_byte
    lda #0
    sta HWREGS_OPEN
    lda test_byte
    sta SYS_RAM,x
    cmp SYS_RAM,x
    beq sys_restore
    lda #1
    sta sys_fail
sys_restore:
    lda old_byte
    sta SYS_RAM,x
    lda #0
    sta HWREGS_CLOSE
    inx
    bne sys_loop

    ; User RAM gets the same atomic test so the register/KERNAL window is
    ; never left open while BASIC or an interrupt handler is running.
    ldx #0
user_loop:
    lda USER_RAM,x
    sta old_byte
    txa
    eor #$5a
    cmp old_byte
    bne user_pattern_ready
    eor #$ff
user_pattern_ready:
    sta test_byte
    lda #0
    sta HWREGS_OPEN
    lda test_byte
    sta USER_RAM,x
    cmp USER_RAM,x
    beq user_restore
    lda #1
    sta user_fail
user_restore:
    lda old_byte
    sta USER_RAM,x
    lda #0
    sta HWREGS_CLOSE
    inx
    bne user_loop

    plp

    #print_string gate_msg
    lda gate_fail
    jsr print_result
    #print_string sys_msg
    lda sys_fail
    jsr print_result
    #print_string user_msg
    lda user_fail
    jsr print_result

    lda gate_fail
    ora sys_fail
    ora user_fail
    bne overall_fail
    #print_string all_pass_msg
    rts

overall_fail:
    #print_string failed_msg
    rts

no_scpu:
    #print_string no_scpu_msg
    rts

print_result:
    beq result_pass
    #print_string fail_msg
    rts
result_pass:
    #print_string pass_msg
    rts

old_byte:  .byte 0
test_byte: .byte 0
gate_fail: .byte 0
sys_fail:  .byte 0
user_fail: .byte 0

title_msg:    .byte 147
              .text "SCPU PRIVATE RAM"
              .byte 13,0
gate_msg:     .text "$D300 CLOSED-WRITE GATE: "
              .byte 0
sys_msg:      .text "$D200 SYSTEM PAGE:       "
              .byte 0
user_msg:     .text "$D300 USER PAGE:         "
              .byte 0
pass_msg:     .text "PASS"
              .byte 13,0
fail_msg:     .text "FAIL"
              .byte 13,0
all_pass_msg: .byte 13
              .text "ALL PRIVATE RAM TESTS PASS"
              .byte 13,0
failed_msg:   .byte 13
              .text "PRIVATE RAM TEST FAILED"
              .byte 13,0
no_scpu_msg:  .text "NO SUPERCPU DETECTED"
              .byte 13,0
