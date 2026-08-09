.include "common.inc"

start:
    #print_string title_msg
    sei
    lda $0314
    sta old_irq
    lda $0315
    sta old_irq+1
    lda $d011
    sta old_d011
    lda $d01a
    sta old_d01a
    lda $d012
    sta old_d012
    lda $d020
    sta old_d020
    lda #<irq_handler
    sta $0314
    lda #>irq_handler
    sta $0315
    lda #0
    sta irq_count
    sta irq_phase
    lda raster_lines
    sta $d012
    lda $d011
    and #$7f
    sta $d011
    lda #1
    sta $d019
    sta $d01a
flush_keys:
    jsr $ffe4
    bne flush_keys
    lda #$ff
    sta last_count
    cli
wait_key:
    lda irq_count
    cmp last_count
    beq check_key
    sta last_count
    jsr show_hex
check_key:
    jsr $ffe4
    beq wait_key
    sei
    lda #0
    sta $d01a
    lda #1
    sta $d019
    lda old_irq
    sta $0314
    lda old_irq+1
    sta $0315
    lda old_d011
    sta $d011
    lda old_d012
    sta $d012
    lda old_d01a
    sta $d01a
    lda old_d020
    sta $d020
    cli
    rts

irq_handler:
    lda $d019
    and #1
    beq chain_irq
    sta $d019
    inc irq_count
    lda irq_phase
    eor #1
    sta irq_phase
    tax
    lda colors,x
    sta $d020
    lda raster_lines,x
    sta $d012
    pla
    tay
    pla
    tax
    pla
    rti
chain_irq:
    jmp (old_irq)

show_hex:
    pha
    lda #19
    jsr $ffd2
    #print_string label
    pla
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
    bcc nibble_digit
    adc #6
nibble_digit:
    adc #'0'
    jmp $ffd2

old_irq:   .word 0
old_d011:  .byte 0
old_d01a:  .byte 0
old_d012:  .byte 0
old_d020:  .byte 0
irq_count: .byte 0
last_count:.byte 0
irq_phase: .byte 0
colors:    .byte 2,6
raster_lines: .byte 60,200
title_msg: .byte 147
           .text "RASTER IRQ TEST - PRESS A KEY TO EXIT"
           .byte 13,0
label:     .text "RASTER IRQ COUNT $"
           .byte 0
