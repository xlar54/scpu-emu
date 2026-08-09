.include "common.inc"

start:
    sei
    lda $dd00
    sta old_dd00
    lda $d018
    sta old_d018
    lda $d0b4
    and #$c0
    sta old_opt
    lda #0
    sta bank_no
bank_loop:
    jsr select_optimization
    lda bank_no
    eor #3
    and #3
    sta vic_bits
    lda old_dd00
    and #$fc
    ora vic_bits
    and #$ff
    sta $dd00
    lda bank_no
    asl
    asl
    asl
    asl
    asl
    asl
    clc
    adc #$04
    sta screen_hi
    lda #0
    sta screen_lo
    ldy #0
fill_page:
    lda bank_no
    clc
    ; The solid glyph below is installed in character slots $01-$04.
    ; Using PETSCII "1"-$34 here selected slots $31-$34 instead, whose
    ; untouched bitmap data made every bank appear to be a blank screen.
    adc #1
    sta (screen_lo),y
    iny
    bne fill_page
    inc screen_hi
    lda screen_hi
    and #3
    bne fill_page
    ; Install one solid glyph in RAM at character-set offset $2000. Unlike VIC
    ; banks 0/2, banks 1/3 cannot see the character ROM.
    lda bank_no
    clc
    adc #1
    asl
    asl
    asl
    sta screen_lo
    lda bank_no
    asl
    asl
    asl
    asl
    asl
    asl
    clc
    adc #$20
    sta screen_hi
    ldy #7
    lda #$ff
glyph_loop:
    sta (screen_lo),y
    dey
    bpl glyph_loop
    ; Give each bank a distinct foreground colour.
    lda #0
    sta screen_lo
    lda #$d8
    sta screen_hi
    lda bank_no
    clc
    adc #2
    ldx #4
color_page:
    ldy #0
color_byte:
    sta (screen_lo),y
    iny
    bne color_byte
    inc screen_hi
    dex
    bne color_page
    lda #$18
    sta $d018
    lda #0
    jsr $ffe4
    cli
wait:
    jsr $ffe4
    beq wait
    sei
    inc bank_no
    lda bank_no
    cmp #4
    beq banks_done
    jmp bank_loop
banks_done:
    jsr restore_optimization
    lda old_dd00
    sta $dd00
    lda old_d018
    sta $d018
    cli
    lda #147              ; restore a usable BASIC screen
    jsr $ffd2
    rts

select_optimization:
    lda #0
    sta $d07e             ; open SCPU hardware registers
    lda bank_no
    beq opt_none
    cmp #1
    beq opt_bank1
    cmp #2
    beq opt_bank2
opt_none:
    sta $d077             ; mirror all (banks 0 and 3)
    bra opt_done
opt_bank1:
    sta $d075
    bra opt_done
opt_bank2:
    sta $d074
opt_done:
    sta $d07f             ; promptly restore normal I/O/KERNAL map
    rts

restore_optimization:
    lda #0
    sta $d07e
    lda old_opt
    beq restore_bank2
    cmp #$40
    beq restore_bank1
    cmp #$80
    beq restore_basic
    sta $d077
    bra restore_done
restore_bank2:
    sta $d074
    bra restore_done
restore_bank1:
    sta $d075
    bra restore_done
restore_basic:
    sta $d076
restore_done:
    sta $d07f
    rts

old_dd00:  .byte 0
old_d018:  .byte 0
old_opt:   .byte 0
bank_no:   .byte 0
vic_bits:  .byte 0
screen_lo = $fb
screen_hi = $fc
