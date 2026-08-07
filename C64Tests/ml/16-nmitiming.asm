.include "common.inc"

; CIA2 Timer-A NMI timing/stress diagnostic for a 65816 SuperCPU.
; The KERNAL is banked out while measuring so $FFEA/$FFFA point directly at
; our handlers. $C003 also gets a JML fallback for the SCPU ROM vector route.

SCPU_DETECT = $d0bc
SLOW_REG = $d07a
FAST_REG = $d07b
CIA1_PRA = $dc00
CIA1_PRB = $dc01
CIA1_DDRA = $dc02
CIA2_TALO = $dd04
CIA2_TAHI = $dd05
CIA2_ICR = $dd0d
CIA2_CRA = $dd0e
SWEEP_COUNT = 5
SWEEP_RUNS = 32
NATIVE_RUNS = 64

start:
    print_string title_msg
    lda SCPU_DETECT
    bpl scpu_present
    print_string no_scpu_msg
    rts
scpu_present:
    print_string mode_msg
mode_wait:
    jsr $ffe4
    cmp #'1'
    beq choose_slow
    cmp #'2'
    beq choose_fast
    cmp #'T'
    beq choose_fast
    cmp #'t'
    bne mode_wait
choose_fast:
    lda #0
    sta FAST_REG
    sta selected_slow
    print_string turbo_msg
    bra begin_test
choose_slow:
    lda #0
    sta SLOW_REG
    lda #1
    sta selected_slow
    print_string slow_msg

begin_test:
    print_string running_msg
    sei
    lda $01
    sta old_port
    lda CIA1_PRA
    sta old_dc00
    lda CIA1_DDRA
    sta old_dc02
    lda CIA2_TALO
    sta old_dd04
    lda CIA2_TAHI
    sta old_dd05
    lda CIA2_CRA
    sta old_dd0e
    lda #$7f
    sta CIA2_ICR
    lda #0
    sta CIA2_CRA
    lda CIA2_ICR

    ; $35: RAM at BASIC/KERNAL, I/O remains visible.
    lda #$35
    sta $01
    ldx #0
save_c003:
    lda $c003,x
    sta old_c003,x
    inx
    cpx #4
    bne save_c003
    lda $ffea
    sta old_ffea
    lda $ffeb
    sta old_ffea+1
    lda $fffa
    sta old_fffa
    lda $fffb
    sta old_fffa+1
    lda #<native_nmi
    sta $ffea
    lda #>native_nmi
    sta $ffeb
    lda #<emu_nmi
    sta $fffa
    lda #>emu_nmi
    sta $fffb
    lda #$5c                    ; JML native_nmi
    sta $c003
    lda #<native_nmi
    sta $c004
    lda #>native_nmi
    sta $c005
    lda #^native_nmi
    sta $c006

    ; Direct Space scan: row 7, column 4. IRQs stay masked for clean timing.
    lda #$ff
    sta CIA1_DDRA
    lda #$7f
    sta CIA1_PRA
    stz aborted
    jsr run_sweeps
    bcs test_aborted
    jsr run_native_windows
    bcs test_aborted
    jsr run_stress
    bra test_done
test_aborted:
    lda #1
    sta aborted

test_done:
    lda #$7f
    sta CIA2_ICR
    lda #0
    sta CIA2_CRA
    lda CIA2_ICR
    lda old_ffea
    sta $ffea
    lda old_ffea+1
    sta $ffeb
    lda old_fffa
    sta $fffa
    lda old_fffa+1
    sta $fffb
    ldx #0
restore_c003:
    lda old_c003,x
    sta $c003,x
    inx
    cpx #4
    bne restore_c003
    lda old_port
    sta $01
    lda old_dd04
    sta CIA2_TALO
    lda old_dd05
    sta CIA2_TAHI
    lda old_dd0e
    sta CIA2_CRA
    lda #$90                    ; restore CIA2 FLAG/RESTORE-key NMI mask
    sta CIA2_ICR
    lda old_dc00
    sta CIA1_PRA
    lda old_dc02
    sta CIA1_DDRA
    cli
    jsr print_results
    lda #0
    sta FAST_REG
    rts

; Five latch values, 32 trials each: captured-X min/max, sum and misses.
run_sweeps:
    stz sweep_index
sweep_next:
    ldx sweep_index
    lda #$ff
    sta sweep_min,x
    stz sweep_max,x
    stz sweep_sum_lo,x
    stz sweep_sum_hi,x
    stz sweep_miss,x
    lda #SWEEP_RUNS
    sta repeat_count
sweep_trial:
    ldx sweep_index
    lda sweep_delays,x
    jsr emu_trial
    bcs sweep_missed
    sta captured_temp
    ldx sweep_index
    cmp sweep_min,x
    bcs :+
    sta sweep_min,x
:
    lda captured_temp
    cmp sweep_max,x
    bcc :+
    sta sweep_max,x
:
    lda captured_temp
    clc
    adc sweep_sum_lo,x
    sta sweep_sum_lo,x
    lda sweep_sum_hi,x
    adc #0
    sta sweep_sum_hi,x
    bra sweep_continue
sweep_missed:
    ldx sweep_index
    inc sweep_miss,x
sweep_continue:
    jsr space_pressed
    bcs sweep_abort
    dec repeat_count
    bne sweep_trial
    inc sweep_index
    lda sweep_index
    cmp #SWEEP_COUNT
    bne sweep_next
    clc
    rts
sweep_abort:
    sec
    rts

; Timer expires inside E=0. Kind 2=native vector; kind 1=held until E=1.
run_native_windows:
    stz native_taken
    stz native_deferred
    stz native_missed
    lda #NATIVE_RUNS
    sta repeat_count
native_test_loop:
    jsr native_trial
    bcs native_was_missed
    cmp #2
    beq native_was_taken
    inc native_deferred
    bra native_continue
native_was_taken:
    inc native_taken
    bra native_continue
native_was_missed:
    inc native_missed
native_continue:
    jsr space_pressed
    bcs native_abort
    dec repeat_count
    bne native_test_loop
    clc
    rts
native_abort:
    sec
    rts

; 1024 varying one-shots. Counts and captured-X checksum expose lost edges.
run_stress:
    stz stress_ok_lo
    stz stress_ok_hi
    stz stress_miss_lo
    stz stress_miss_hi
    stz stress_sum_lo
    stz stress_sum_hi
    stz stress_seq
    stz remaining_lo
    lda #4
    sta remaining_hi
stress_loop:
    lda stress_seq
    and #$1f
    clc
    adc #2
    jsr emu_trial
    bcs stress_missed
    pha
    inc stress_ok_lo
    bne :+
    inc stress_ok_hi
:
    pla
    clc
    adc stress_sum_lo
    sta stress_sum_lo
    lda stress_sum_hi
    adc #0
    sta stress_sum_hi
    bra stress_continue
stress_missed:
    inc stress_miss_lo
    bne stress_continue
    inc stress_miss_hi
stress_continue:
    inc stress_seq
    jsr space_pressed
    bcs stress_abort
    lda remaining_lo
    bne :+
    dec remaining_hi
:
    dec remaining_lo
    lda remaining_lo
    ora remaining_hi
    bne stress_loop
    clc
    rts
stress_abort:
    sec
    rts

; A=latch. Returns captured X in A/C=0, or C=1 after a long timeout.
emu_trial:
    sta trial_latch
    lda #$7f
    sta CIA2_ICR
    lda #0
    sta CIA2_CRA
    lda CIA2_ICR
    lda trial_latch
    sta CIA2_TALO
    lda #0
    sta CIA2_TAHI
    sta hit_kind
    sta timeout_hi
    lda #$81
    sta CIA2_ICR
    ldx #0
    lda #$19
    sta CIA2_CRA
emu_wait:
    inx
    bne :+
    inc timeout_hi
    beq emu_timeout
:
    lda hit_kind
    beq emu_wait
    lda captured_x
    clc
    rts
emu_timeout:
    sec
    rts

native_trial:
    lda #$7f
    sta CIA2_ICR
    lda #0
    sta CIA2_CRA
    lda CIA2_ICR
    lda #8
    sta CIA2_TALO
    lda #0
    sta CIA2_TAHI
    sta hit_kind
    lda #$81
    sta CIA2_ICR
    ldx #0
    lda #$19
    sta CIA2_CRA
    clc
    xce
    .a8
    .i8
native_wait:
    inx
    lda hit_kind
    bne native_leave
    cpx #$f0
    bne native_wait
native_leave:
    sec
    xce
    .a8
    .i8
    ldx #0
native_emu_wait:
    lda hit_kind
    bne native_have_result
    inx
    bne native_emu_wait
    sec
    rts
native_have_result:
    lda hit_kind
    clc
    rts

emu_nmi:
    pha
    stx captured_x
    lda CIA2_ICR
    lda #1
    sta hit_kind
    pla
    rti

native_nmi:
    pha
    stx captured_x
    lda CIA2_ICR
    lda #2
    sta hit_kind
    pla
    rti

space_pressed:
    lda CIA1_PRB
    and #$10
    bne :+
    sec
    rts
:
    clc
    rts

print_results:
    print_string result_title
    print_string mode_label
    lda selected_slow
    beq :+
    print_string slow_name
    bra :++
:
    print_string turbo_name
:
    print_string sweep_header
    stz sweep_index
print_sweep_loop:
    ldx sweep_index
    lda sweep_delays,x
    jsr print_hex8
    lda #' '
    jsr $ffd2
    ldx sweep_index
    lda sweep_min,x
    jsr print_hex8
    lda #' '
    jsr $ffd2
    ldx sweep_index
    lda sweep_max,x
    jsr print_hex8
    lda #' '
    jsr $ffd2
    ldx sweep_index
    lda sweep_sum_hi,x
    jsr print_hex8
    ldx sweep_index
    lda sweep_sum_lo,x
    jsr print_hex8
    lda #' '
    jsr $ffd2
    ldx sweep_index
    lda sweep_miss,x
    jsr print_hex8
    lda #13
    jsr $ffd2
    inc sweep_index
    lda sweep_index
    cmp #SWEEP_COUNT
    bne print_sweep_loop
    print_string native_label
    lda native_taken
    jsr print_hex8
    print_string deferred_label
    lda native_deferred
    jsr print_hex8
    print_string miss_label
    lda native_missed
    jsr print_hex8
    lda #13
    jsr $ffd2
    print_string stress_label
    lda stress_ok_hi
    jsr print_hex8
    lda stress_ok_lo
    jsr print_hex8
    print_string miss_label
    lda stress_miss_hi
    jsr print_hex8
    lda stress_miss_lo
    jsr print_hex8
    print_string sum_label
    lda stress_sum_hi
    jsr print_hex8
    lda stress_sum_lo
    jsr print_hex8
    lda #13
    jsr $ffd2
    lda aborted
    beq :+
    print_string aborted_msg
    rts
:
    print_string done_msg
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
    bcc :+
    adc #6
:
    adc #'0'
    jmp $ffd2

old_port: .byte 0
old_dc00: .byte 0
old_dc02: .byte 0
old_dd04: .byte 0
old_dd05: .byte 0
old_dd0e: .byte 0
old_c003: .res 4
old_ffea: .res 2
old_fffa: .res 2
selected_slow: .byte 0
aborted: .byte 0
hit_kind: .byte 0
captured_x: .byte 0
captured_temp: .byte 0
trial_latch: .byte 0
timeout_hi: .byte 0
repeat_count: .byte 0
sweep_index: .byte 0
sweep_delays: .byte 2,4,8,16,32
sweep_min: .res SWEEP_COUNT
sweep_max: .res SWEEP_COUNT
sweep_sum_lo: .res SWEEP_COUNT
sweep_sum_hi: .res SWEEP_COUNT
sweep_miss: .res SWEEP_COUNT
native_taken: .byte 0
native_deferred: .byte 0
native_missed: .byte 0
stress_ok_lo: .byte 0
stress_ok_hi: .byte 0
stress_miss_lo: .byte 0
stress_miss_hi: .byte 0
stress_sum_lo: .byte 0
stress_sum_hi: .byte 0
stress_seq: .byte 0
remaining_lo: .byte 0
remaining_hi: .byte 0

title_msg: .byte 147,"CIA2 NMI TIMING/STRESS",13,0
no_scpu_msg: .byte "NO SUPERCPU DETECTED",13,0
mode_msg: .byte "1=1MHZ  2/T=TURBO",13,0
turbo_msg: .byte "TURBO SELECTED",13,0
slow_msg: .byte "1MHZ SELECTED",13,0
running_msg: .byte "RUNNING - SPACE ABORTS BETWEEN TRIALS",13,0
result_title: .byte 147,"CIA2 NMI RESULTS",13,0
mode_label: .byte "MODE: ",0
slow_name: .byte "1MHZ",13,0
turbo_name: .byte "TURBO",13,0
sweep_header: .byte "LATCH MIN MAX SUM  MISS",13,0
native_label: .byte "NATIVE: ",0
deferred_label: .byte " DEFER: ",0
miss_label: .byte " MISS: ",0
stress_label: .byte "STRESS OK: ",0
sum_label: .byte " SUM: ",0
aborted_msg: .byte "ABORTED - PARTIAL RESULTS",13,0
done_msg: .byte "DONE - PHOTOGRAPH THIS SCREEN",13,0
