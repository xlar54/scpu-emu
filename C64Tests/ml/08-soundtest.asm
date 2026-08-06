.include "common.inc"

SID = $d400

start:
    print_string title_msg
    jsr silence_sid
    sta aborted
    lda #$0f
    sta SID+$18

    print_string voice1_msg
    ldx #0
voice1_scale:
    lda note_lo,x
    sta SID+$00
    lda note_hi,x
    sta SID+$01
    lda #$24              ; medium attack, decay
    sta SID+$05
    lda #$a8              ; sustain and release
    sta SID+$06
    lda #$11              ; triangle + gate
    sta SID+$04
    lda #6
    jsr wait_jiffies
    lda aborted
    beq voice1_continue
    jmp test_done
voice1_continue:
    inx
    cpx #7
    bne voice1_scale
    lda #$10
    sta SID+$04

    print_string voice2_msg
    ldx #0
voice2_scale:
    lda note_lo,x
    sta SID+$07
    lda note_hi,x
    sta SID+$08
    lda #$14
    sta SID+$0c
    lda #$98
    sta SID+$0d
    lda #$21              ; sawtooth + gate
    sta SID+$0b
    lda #6
    jsr wait_jiffies
    lda aborted
    beq voice2_continue
    jmp test_done
voice2_continue:
    inx
    cpx #7
    bne voice2_scale
    lda #$20
    sta SID+$0b

    print_string noise_msg
    lda #$00
    sta SID+$0e
    lda #$18
    sta SID+$0f
    lda #$08
    sta SID+$13
    lda #$88
    sta SID+$14
    lda #$81              ; noise + gate
    sta SID+$12
    lda #24
    jsr wait_jiffies
    lda aborted
    beq noise_complete
    jmp test_done
noise_complete:
    lda #$80
    sta SID+$12

    print_string chord_msg
    lda #<$1167           ; C4, triangle
    sta SID+$00
    lda #>$1167
    sta SID+$01
    lda #<$15ee           ; E4, sawtooth
    sta SID+$07
    lda #>$15ee
    sta SID+$08
    lda #<$1a14           ; G4, pulse
    sta SID+$0e
    lda #>$1a14
    sta SID+$0f
    lda #0
    sta SID+$10
    lda #8
    sta SID+$11           ; 50 percent pulse width
    lda #$11
    sta SID+$04
    lda #$21
    sta SID+$0b
    lda #$41
    sta SID+$12
    lda #36
    jsr wait_jiffies
    lda aborted
    bne test_done

    print_string filter_msg
    lda #$f7              ; high resonance, route voices 1-3
    sta SID+$17
    lda #$1f              ; low-pass filter, full volume
    sta SID+$18
    lda #0
    sta SID+$15
    ldx #0
filter_sweep:
    stx SID+$16
    txa
    and #7
    bne filter_next
    lda #1
    jsr wait_jiffies
    lda aborted
    bne test_done
filter_next:
    inx
    bne filter_sweep

test_done:
    jsr silence_sid
    lda aborted
    bne early_exit
    print_string complete_msg
early_exit:
    rts

; Wait A jiffies while watching the KERNAL keyboard buffer for Space.
wait_jiffies:
    sta duration
    phx
    phy
    lda $a2
    sta start_time
wait_loop:
    jsr $ffe4
    cmp #' '
    bne check_time
    lda #1
    sta aborted
    ply
    plx
    rts
check_time:
    lda $a2
    sec
    sbc start_time
    cmp duration
    bcc wait_loop
    ply
    plx
    rts

silence_sid:
    lda #0
    ldx #$18
clear_sid_loop:
    sta SID,x
    dex
    bpl clear_sid_loop
    rts

duration:   .byte 0
start_time: .byte 0
aborted:    .byte 0

; Approximate PAL SID frequency words: C3 E3 G3 C4 E4 G4 C5.
note_lo: .byte <$08b4,<$0af7,<$0d0a,<$1167,<$15ee,<$1a14,<$22ce
note_hi: .byte >$08b4,>$0af7,>$0d0a,>$1167,>$15ee,>$1a14,>$22ce

title_msg:    .byte 147,"SID SOUND TEST",13
              .byte "SPACE STOPS AND RETURNS TO BASIC",13,0
voice1_msg:   .byte "VOICE 1: TRIANGLE SCALE",13,0
voice2_msg:   .byte "VOICE 2: SAWTOOTH SCALE",13,0
noise_msg:    .byte "VOICE 3: NOISE",13,0
chord_msg:    .byte "THREE-VOICE CHORD",13,0
filter_msg:   .byte "LOW-PASS FILTER SWEEP",13,0
complete_msg: .byte "SOUND TEST COMPLETE",13,0
