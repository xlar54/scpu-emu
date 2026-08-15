.include "common.inc"

; Compare identical 65816 absolute-long read/write loops in the SuperCPU's
; private bank-$01 SRAM and bank-$02 SuperRAM. The program deliberately stays
; in 6502 emulation mode: long addressing is still available, while the normal
; KERNAL IRQ can keep the jiffy clock at $A0-$A2 running during the benchmark.

SCPU_DETECT = $d0bc
FAST_REG = $d07b
JIFFY_MID = $a1
JIFFY_LOW = $a2
PASSES = 32                    ; 32 * 256 * 256 = 2,097,152 pairs

start:
    #print_string title_msg
    lda SCPU_DETECT
    bpl scpu_present
    #print_string no_scpu_msg
    rts

scpu_present:
    ; Make old/new firmware measurements comparable without requiring a POKE.
    ; The physical speed switch must still permit Turbo mode.
    lda #0
    sta FAST_REG
    #print_string running_msg

    ; Establish identical source data outside the timed region.
    ldx #0
    lda #0
seed_loop:
    sta $012000,x
    sta $022000,x
    inx
    bne seed_loop

    jsr clear_jiffy
    jsr bench_private
    jsr save_private_time

    jsr clear_jiffy
    jsr bench_superram
    jsr save_super_time

    jsr verify_results
    #print_string result_msg
    #print_string private_msg
    lda private_time+1
    jsr print_hex8
    lda private_time
    jsr print_hex8
    #print_string jiffies_msg
    #print_string super_msg
    lda super_time+1
    jsr print_hex8
    lda super_time
    jsr print_hex8
    #print_string jiffies_msg
    lda verify_failed
    bne show_fail
    #print_string pass_msg
    bra done
show_fail:
    #print_string fail_msg
done:
    #print_string hint_msg
    rts

clear_jiffy:
    sei
    lda #0
    sta $a0
    sta JIFFY_MID
    sta JIFFY_LOW
    cli
    rts

save_private_time:
    sei
    lda JIFFY_LOW
    sta private_time
    lda JIFFY_MID
    sta private_time+1
    cli
    rts

save_super_time:
    sei
    lda JIFFY_LOW
    sta super_time
    lda JIFFY_MID
    sta super_time+1
    cli
    rts

; X and Y are eight-bit in emulation mode. Their two nested wraparound loops
; make 65,536 iterations per pass without using any KERNAL zero-page storage.
bench_private:
    lda #PASSES
    sta pass_count
private_pass:
    ldy #0
private_page:
    ldx #0
private_byte:
    lda $012000,x
    eor #$5a
    sta $012100,x
    inx
    bne private_byte
    iny
    bne private_page
    dec pass_count
    bne private_pass
    rts

bench_superram:
    lda #PASSES
    sta pass_count
super_pass:
    ldy #0
super_page:
    ldx #0
super_byte:
    lda $022000,x
    eor #$5a
    sta $022100,x
    inx
    bne super_byte
    iny
    bne super_page
    dec pass_count
    bne super_pass
    rts

verify_results:
    stz verify_failed
    ldx #0
verify_loop:
    lda $012100,x
    cmp #$5a
    bne verify_bad
    lda $022100,x
    cmp #$5a
    bne verify_bad
    inx
    bne verify_loop
    rts
verify_bad:
    lda #1
    sta verify_failed
    rts

print_hex8:
    pha
    lsr
    lsr
    lsr
    lsr
    jsr print_nibble
    pla
    and #$0f
print_nibble:
    cmp #10
    bcc print_digit
    adc #6
print_digit:
    adc #'0'
    jmp $ffd2

pass_count:    .byte 0
private_time:  .word 0
super_time:    .word 0
verify_failed: .byte 0

title_msg: .byte 147
           .text "SUPERRAM SPEED BENCHMARK"
           .byte 13,0
no_scpu_msg: .text "NO SUPERCPU DETECTED"
             .byte 13,0
running_msg: .text "TURBO: 2M READ/WRITE PAIRS"
             .byte 13
             .text "RUNNING..."
             .byte 13,0
result_msg: .text "RESULTS (LOWER IS FASTER)"
            .byte 13,0
private_msg: .text "BANK $01 SRAM:     $"
             .byte 0
super_msg: .text "BANK $02 SUPERRAM: $"
           .byte 0
jiffies_msg: .text " JIFFIES"
              .byte 13,0
pass_msg: .text "MEMORY CHECK: PASS"
          .byte 13,0
fail_msg: .text "MEMORY CHECK: FAIL"
          .byte 13,0
hint_msg: .text "PHOTOGRAPH OLD AND NEW RESULTS"
          .byte 13,0
