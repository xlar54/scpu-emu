.include "common.inc"

BALL_DATA = $3000
BALL_PTR  = BALL_DATA / 64

start:
    sei
    lda $dd00
    sta old_dd00
    lda $d011
    sta old_d011
    lda $d016
    sta old_d016
    lda $d018
    sta old_d018
    lda $d015
    sta old_d015
    lda $d010
    sta old_d010
    lda $d017
    sta old_d017
    lda $d01b
    sta old_d01b
    lda $d01c
    sta old_d01c
    lda $d01d
    sta old_d01d
    lda $d020
    sta old_d020
    lda $d021
    sta old_d021
    ldx #15
save_positions:
    lda $d000,x
    sta old_positions,x
    dex
    bpl save_positions
    ldx #7
save_loop:
    lda $07f8,x
    sta old_ptrs,x
    lda $d027,x
    sta old_colors,x
    dex
    bpl save_loop

    ; VIC bank 0, screen at $0400, ROM character set at $1000.
    lda old_dd00
    ora #3
    sta $dd00
    lda #$14
    sta $d018
    lda #0
    sta $d020
    sta $d021
    sta $d017
    sta $d01b
    sta $d01c
    sta $d01d
    sta $d010

    ; Clear the text screen without relying on KERNAL cursor state.
    lda #$20
    ldx #0
clear_screen:
    sta $0400,x
    sta $0500,x
    sta $0600,x
    sta $06e8,x
    inx
    bne clear_screen

    ; Copy the shared 24x21 ball shape into sprite slot $C0 at $3000.
    ldx #62
copy_ball:
    lda ball_bitmap,x
    sta BALL_DATA,x
    dex
    bpl copy_ball

    ldx #7
sprite_setup:
    lda #BALL_PTR
    sta $07f8,x
    lda ball_colors,x
    sta $d027,x
    dex
    bpl sprite_setup
    lda #$ff
    sta $d015
    cli

main_loop:
    jsr wait_frame
    jsr move_sprites
    jsr draw_sprites
    jsr $ffe4
    beq main_loop

    sei
    lda old_dd00
    sta $dd00
    lda old_d011
    sta $d011
    lda old_d016
    sta $d016
    lda old_d018
    sta $d018
    lda old_d015
    sta $d015
    lda old_d010
    sta $d010
    lda old_d017
    sta $d017
    lda old_d01b
    sta $d01b
    lda old_d01c
    sta $d01c
    lda old_d01d
    sta $d01d
    lda old_d020
    sta $d020
    lda old_d021
    sta $d021
    ldx #15
restore_positions:
    lda old_positions,x
    sta $d000,x
    dex
    bpl restore_positions
    ldx #7
restore_loop:
    lda old_ptrs,x
    sta $07f8,x
    lda old_colors,x
    sta $d027,x
    dex
    bpl restore_loop
    cli
    lda #147
    jsr $ffd2
    rts

wait_frame:
wait_not_line:
    lda $d012
    cmp #$f8
    beq wait_not_line
wait_line:
    lda $d012
    cmp #$f8
    bne wait_line
    rts

move_sprites:
    ldx #7
move_loop:
    lda dx,x
    bmi move_left
move_right:
    clc
    adc xpos_lo,x
    sta xpos_lo,x
    bcc check_right
    inc xpos_hi,x
check_right:
    lda xpos_hi,x
    beq move_y
    lda xpos_lo,x
    cmp #$28                ; 296 = $0128, right edge for 24px sprite
    bcc move_y
    lda #$28
    sta xpos_lo,x
    lda #1
    sta xpos_hi,x
    lda dx,x
    eor #$ff
    clc
    adc #1
    sta dx,x
    bra move_y
move_left:
    clc
    adc xpos_lo,x
    sta xpos_lo,x
    bcs check_left
    dec xpos_hi,x
check_left:
    lda xpos_hi,x
    bpl move_y
    lda #0
    sta xpos_lo,x
    sta xpos_hi,x
    lda dx,x
    eor #$ff
    clc
    adc #1
    sta dx,x

move_y:
    lda dy,x
    bmi move_up
move_down:
    clc
    adc ypos,x
    sta ypos,x
    cmp #$e0                ; bottom edge
    bcc move_next
    lda #$e0
    sta ypos,x
    lda dy,x
    eor #$ff
    clc
    adc #1
    sta dy,x
    bra move_next
move_up:
    clc
    adc ypos,x
    sta ypos,x
    cmp #$32                ; top edge
    bcs move_next
    lda #$32
    sta ypos,x
    lda dy,x
    eor #$ff
    clc
    adc #1
    sta dy,x
move_next:
    dex
    bmi move_done
    jmp move_loop
move_done:
    rts

draw_sprites:
    lda #0
    sta x_msb
    ldx #7
draw_loop:
    txa
    asl
    tay
    lda xpos_lo,x
    sta $d000,y
    lda ypos,x
    sta $d001,y
    lda xpos_hi,x
    beq no_msb
    lda x_msb
    ora bit_table,x
    sta x_msb
no_msb:
    dex
    bpl draw_loop
    lda x_msb
    sta $d010
    rts

x_msb: .byte 0
xpos_lo: .byte 20,58,96,134,172,210,248,30
xpos_hi: .byte 0,0,0,0,0,0,0,1
ypos:    .byte 50,72,94,116,138,160,182,204
dx:      .byte 1,$ff,2,$fe,1,$fe,2,$ff
dy:      .byte 1,2,$ff,$fe,2,$ff,1,$fe
bit_table:   .byte 1,2,4,8,16,32,64,128
ball_colors: .byte 2,3,4,5,7,8,10,13

old_dd00: .byte 0
old_d011: .byte 0
old_d016: .byte 0
old_d018: .byte 0
old_d015: .byte 0
old_d010: .byte 0
old_d017: .byte 0
old_d01b: .byte 0
old_d01c: .byte 0
old_d01d: .byte 0
old_d020: .byte 0
old_d021: .byte 0
old_positions: .res 16
old_ptrs:   .res 8
old_colors: .res 8

; 21 rows x 3 bytes. The 64th sprite byte is unused.
ball_bitmap:
    .byte $00,$7e,$00
    .byte $01,$ff,$80
    .byte $03,$ff,$c0
    .byte $07,$ff,$e0
    .byte $0f,$ff,$f0
    .byte $1f,$ff,$f8
    .byte $3f,$ff,$fc
    .byte $3f,$ff,$fc
    .byte $7f,$ff,$fe
    .byte $7f,$ff,$fe
    .byte $7f,$ff,$fe
    .byte $7f,$ff,$fe
    .byte $7f,$ff,$fe
    .byte $3f,$ff,$fc
    .byte $3f,$ff,$fc
    .byte $1f,$ff,$f8
    .byte $0f,$ff,$f0
    .byte $07,$ff,$e0
    .byte $03,$ff,$c0
    .byte $01,$ff,$80
    .byte $00,$7e,$00
