.include "common.inc"

BITMAP      = $2000
BITMAP_END  = $4000
BITMAP_PTR  = $fb
MAX_ITER    = 40

start:
    #print_string title_msg
    lda $d0bc
    bpl scpu_present
    #print_string no_scpu_msg
    rts
scpu_present:
    lda $d0b8
    and #$80
    sta old_soft_speed
    #print_string mode_msg
choose_mode:
    jsr $ffe4
    cmp #' '
    beq prompt_exit
    cmp #'1'
    beq choose_1mhz
    cmp #'2'
    bne choose_mode
    lda #0
    sta $d07b
    bra mode_selected
choose_1mhz:
    lda #0
    sta $d07a
mode_selected:
    jsr save_and_setup

    sei
    lda $dc00
    sta old_cia_pra
    lda $dc02
    sta old_cia_ddra
    lda $dc03
    sta old_cia_ddrb
    lda #$ff
    sta $dc02
    lda #0
    sta $dc03
    lda #$7f              ; keyboard row 7: Space is column 4
    sta $dc00

    clc
    xce
    rep #$30
    .al
    .xl
    jsr render_mandelbrot
    sep #$30
    .as
    .xs
    sec
    xce

    lda old_cia_pra
    sta $dc00
    lda old_cia_ddra
    sta $dc02
    lda old_cia_ddrb
    sta $dc03
    jsr restore_machine
    cli
    lda #147
    jsr $ffd2
prompt_exit:
    rts

save_and_setup:
    lda $dd00
    sta old_dd00
    lda $d011
    sta old_d011
    lda $d016
    sta old_d016
    lda $d018
    sta old_d018
    lda $d020
    sta old_border
    lda $d021
    sta old_background
    lda $d015
    sta old_sprites
    lda $d0b4
    and #$c0
    sta old_optimization

    sei
    lda #0
    sta $d07e
    sta $d077             ; mirror all bitmap writes to VIC-visible DRAM
    sta $d07f
    sta $d015
    lda old_dd00
    and #$fc
    ora #3
    sta $dd00
    lda old_d011
    and #$3f
    ora #$20
    sta $d011
    lda old_d016
    and #$ef
    sta $d016
    lda #$18
    sta $d018
    lda #0
    sta $d020
    sta $d021
    lda #$10              ; white foreground, black background
    ldx #0
fill_screen:
    sta $0400,x
    sta $0500,x
    sta $0600,x
    sta $06e8,x
    inx
    bne fill_screen
    cli
    rts

restore_machine:
    sei
    lda #0
    sta $d015
    lda old_dd00
    sta $dd00
    lda old_d011
    sta $d011
    lda old_d016
    sta $d016
    lda old_d018
    sta $d018
    lda old_border
    sta $d020
    lda old_background
    sta $d021
    lda old_sprites
    sta $d015
    lda #0
    sta $d07e
    lda old_optimization
    beq restore_bank2
    cmp #$40
    beq restore_bank1
    cmp #$80
    beq restore_basic
    sta $d077
    bra restore_opt_done
restore_bank2:
    sta $d074
    bra restore_opt_done
restore_bank1:
    sta $d075
    bra restore_opt_done
restore_basic:
    sta $d076
restore_opt_done:
    sta $d07f
    lda old_soft_speed
    beq restore_turbo
    lda #0
    sta $d07a
    rts
restore_turbo:
    lda #0
    sta $d07b
    rts

; -------------------------------------------------------------------------
; Native-mode Q8.8 Mandelbrot renderer.
;
; It calculates a 160x100 sample grid and expands every sample to a 2x2 block
; on the 320x200 hires bitmap. Escape-time parity supplies monochrome contour
; bands while points still inside after MAX_ITER iterations are solid white.
; -------------------------------------------------------------------------

.al
.xl
render_mandelbrot:
    phb
    phk
    plb
    phd
    pea $0000
    pld
    jsr build_square_table
    jsr clear_bitmap
    stz abort_flag
    stz row_no
    stz row_fraction
    lda #$fec0            ; -1.25 in Q8.8
    sta cy

render_row:
    stz column_no
    stz column_fraction
    lda #$fdc0            ; -2.25 in Q8.8
    sta cx
render_column:
    stz zx
    stz zy
    stz iteration

iteration_loop:
    lda zx
    jsr square_q8
    sta x_squared
    lda zy
    jsr square_q8
    sta y_squared
    clc
    adc x_squared
    cmp #$0400            ; radius squared >= 4.0
    bcs escaped

    lda zx
    clc
    adc zy
    jsr square_q8
    sec
    sbc x_squared
    sbc y_squared         ; (x+y)^2-x^2-y^2 = 2xy
    clc
    adc cy
    sta next_y
    lda x_squared
    sec
    sbc y_squared
    clc
    adc cx
    sta zx
    lda next_y
    sta zy
    inc iteration
    lda iteration
    cmp #MAX_ITER
    bne iteration_loop
    jsr plot_sample
    bra sample_done

escaped:
    lda iteration
    and #3               ; one thin contour for every four escape bands
    bne sample_done
    jsr plot_sample

sample_done:
    ; Scan Space directly on every sample, independent of KERNAL IRQ service.
    sep #$20
    .as
    lda $dc01
    and #$10
    bne space_not_pressed
    rep #$20
    .al
    jmp render_abort
space_not_pressed:
    rep #$20
    .al

    inc column_no
    lda column_no
    cmp #160
    beq next_row
    lda cx
    clc
    adc #5
    sta cx
    inc column_fraction
    lda column_fraction
    cmp #5
    beq add_column_fraction
    jmp render_column
add_column_fraction:
    stz column_fraction
    inc cx                ; 5.2 Q8.8 units per sample
    jmp render_column

next_row:
    inc row_no
    lda row_no
    cmp #100
    beq render_finished
    lda cy
    clc
    adc #6
    sta cy
    lda row_fraction
    clc
    adc #2
    cmp #5
    bcc store_row_fraction
    sec
    sbc #5
    inc cy                ; 6.4 Q8.8 units per sample
store_row_fraction:
    sta row_fraction
    jmp render_row

render_finished:
wait_for_space:
    sep #$20
    .as
    lda $dc01
    and #$10
    bne wait_for_space
    rep #$20
    .al
    bra render_abort
render_abort:
    pld
    plb
    rts

clear_bitmap:
    lda #0
    ldx #BITMAP
clear_loop:
    sta $0000,x
    inx
    inx
    cpx #BITMAP_END
    bne clear_loop
    rts

; Build square_table[n] = floor(n*n/256), n=0..1024, without multiplication.
; The unshifted square advances by successive odd numbers.
build_square_table:
    stz square_raw_low
    stz square_raw_high
    lda #1
    sta square_delta
    ldx #0
square_build_loop:
    lda square_raw_low
    xba
    and #$00ff
    sta square_value
    sep #$20
    .as
    lda square_raw_high
    sta square_value+1
    rep #$20
    .al
    lda square_value
    sta square_table,x
    lda square_raw_low
    clc
    adc square_delta
    sta square_raw_low
    bcc square_no_carry
    inc square_raw_high
square_no_carry:
    lda square_delta
    clc
    adc #2
    sta square_delta
    inx
    inx
    cpx #2050
    bne square_build_loop
    rts

; Input A is signed Q8.8 in the range -4..4; output is its square in Q8.8.
square_q8:
    bpl square_positive
    eor #$ffff
    inc a
square_positive:
    cmp #$0400
    bcc square_lookup
    lda #$0400            ; |component| >= 4.0 guarantees escape
    rts
square_lookup:
    asl
    tax
    lda square_table,x
    rts

plot_sample:
    lda column_no
    asl
    sta pixel_x
    lda row_no
    asl
    sta pixel_y
    jsr plot_pixel
    inc pixel_x
    jsr plot_pixel
    inc pixel_y
    jsr plot_pixel
    dec pixel_x
    jsr plot_pixel
    rts

plot_pixel:
    lda pixel_y
    lsr
    lsr
    lsr
    sta character_row
    asl
    asl
    asl
    asl
    asl
    asl
    sta pixel_address
    lda character_row
    xba
    clc
    adc pixel_address
    sta pixel_address
    lda pixel_x
    and #$fff8
    clc
    adc pixel_address
    sta pixel_address
    lda pixel_y
    and #7
    clc
    adc pixel_address
    adc #BITMAP
    sta BITMAP_PTR
    lda pixel_x
    and #7
    tax
    sep #$20
    .as
    lda bit_masks,x
    ora (BITMAP_PTR)
    sta (BITMAP_PTR)
    rep #$20
    .al
    rts

.as
.xs
old_soft_speed:   .byte 0
old_dd00:         .byte 0
old_d011:         .byte 0
old_d016:         .byte 0
old_d018:         .byte 0
old_border:       .byte 0
old_background:   .byte 0
old_sprites:      .byte 0
old_optimization: .byte 0
old_cia_pra:      .byte 0
old_cia_ddra:     .byte 0
old_cia_ddrb:     .byte 0

cx:              .word 0
cy:              .word 0
zx:              .word 0
zy:              .word 0
next_y:          .word 0
x_squared:       .word 0
y_squared:       .word 0
iteration:       .word 0
column_no:       .word 0
row_no:          .word 0
column_fraction: .word 0
row_fraction:    .word 0
pixel_x:         .word 0
pixel_y:         .word 0
character_row:   .word 0
pixel_address:   .word 0
abort_flag:      .word 0
square_raw_low:  .word 0
square_raw_high: .word 0
square_delta:    .word 0
square_value:    .word 0

bit_masks: .byte $80,$40,$20,$10,$08,$04,$02,$01

title_msg:   .byte 147
             .text "65816 MANDELBROT"
             .byte 13,0
mode_msg:    .text "PRESS 1 FOR 1 MHZ"
             .byte 13
             .text "PRESS 2 FOR 20 MHZ"
             .byte 13
             .text "SPACE RETURNS TO BASIC"
             .byte 13,0
no_scpu_msg: .text "NO SUPERCPU DETECTED"
             .byte 13,0

; 1025 words, generated before rendering.
square_table: .fill 2050,0

.if * >= BITMAP
    .error "07-MANDELBROT overlaps the $2000 bitmap"
.endif
