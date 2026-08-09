.include "common.inc"

BITMAP       = $2000
FRAME_SIZE   = $2000
FRAME_COUNT  = 16
BITMAP_PTR   = $fb

start:
    #print_string intro_msg
    lda $d0bc
    bpl scpu_present
    jmp no_scpu
scpu_present:
    jsr probe_superram
    lda cache_ok
    bne superram_present
    jmp no_superram
superram_present:

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

    #print_string cache_msg
    sei
    clc
    xce
    rep #$30
    .al
    .xl
    jsr render_all_frames
    sep #$30
    .as
    .xs
    sec
    xce

    ; Mirror all bank-0 writes while cached frames are copied to the VIC
    ; bitmap. Open and close the hardware window as one short transaction.
    lda #0
    sta $d07e
    sta $d077
    sta $d07f

    lda #0
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

    ; Hires colour nybbles: white foreground, black background.
    lda #$10
    ldx #0
fill_screen:
    sta $0400,x
    sta $0500,x
    sta $0600,x
    sta $06e8,x
    inx
    bne fill_screen

flush_keys:
    jsr $ffe4
    bne flush_keys
    lda #0
    sta frame_no
    sta frame_no+1
    cli

play_frame:
    sei
    clc
    xce
    rep #$30
    .al
    .xl
    jsr copy_cached_frame
    sep #$30
    .as
    .xs
    sec
    xce
    cli

    lda $a2
    sta start_tick
wait_frame:
    jsr $ffe4
    bne exit_demo
    lda $a2
    sec
    sbc start_tick
    cmp #3
    bcc wait_frame
    inc frame_no
    lda frame_no
    and #$0f
    sta frame_no
    jmp play_frame

exit_demo:
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
    jsr restore_optimization
    cli
    lda #147
    jsr $ffd2
    rts

no_scpu:
    #print_string no_scpu_msg
    rts
no_superram:
    #print_string no_superram_msg
    rts

; Verify that two full 64 KB SuperRAM banks are present before caching.
probe_superram:
    sei
    clc
    xce
    rep #$10
    .xl
    sep #$20
    .as
    lda $021234
    sta probe_saved
    eor #$a5
    sta probe_expected
    sta $021234
    cmp $021234
    bne probe_failed
    lda probe_saved
    sta $021234
    lda $031234
    sta probe_saved
    eor #$5a
    sta probe_expected
    sta $031234
    cmp $031234
    bne probe_failed_restore3
    lda probe_saved
    sta $031234
    lda #1
    sta cache_ok
    bra probe_done
probe_failed_restore3:
    lda probe_saved
    sta $031234
    bra probe_failed_done
probe_failed:
    lda probe_saved
    sta $021234
probe_failed_done:
    stz cache_ok
probe_done:
    sep #$30
    .xs
    sec
    xce
    cli
    rts

; -------------------------------------------------------------------------
; Cache builder. The line renderer works in bank 0, then MVN stores each
; completed 8 KB bitmap in banks $02/$03 (eight frames per bank).
; -------------------------------------------------------------------------

.al
.xl
render_all_frames:
    stz frame_no
render_frame_loop:
    jsr clear_bitmap
    jsr render_frame
    jsr cache_bitmap
    inc frame_no
    lda frame_no
    cmp #FRAME_COUNT
    bne render_frame_loop
    rts

clear_bitmap:
    lda #0
    ldx #BITMAP
clear_bitmap_loop:
    sta $0000,x
    inx
    inx
    cpx #(BITMAP+FRAME_SIZE)
    bne clear_bitmap_loop
    rts

render_frame:
    lda frame_no
    asl
    asl
    asl
    asl
    clc
    adc #frame_vertices
    sta coord_base
    stz edge_pos
edge_loop:
    stz x0
    stz y0
    stz x1
    stz y1
    ldx edge_pos
    sep #$20
    .as
    lda edges,x
    rep #$20
    .al
    and #$00ff
    asl
    clc
    adc coord_base
    tax
    sep #$20
    .as
    lda $0000,x
    sta x0
    lda $0001,x
    sta y0

    ldx edge_pos
    lda edges+1,x
    rep #$20
    .al
    and #$00ff
    asl
    clc
    adc coord_base
    tax
    sep #$20
    .as
    lda $0000,x
    sta x1
    lda $0001,x
    sta y1
    rep #$20
    .al
    jsr draw_line
    inc edge_pos
    inc edge_pos
    lda edge_pos
    cmp #24
    bne edge_loop
    rts

; Signed 16-bit Bresenham line drawing.
draw_line:
    lda x1
    sec
    sbc x0
    bpl dx_positive
    eor #$ffff
    inc a
    sta dx
    lda #$ffff
    sta sx
    bra dx_done
dx_positive:
    sta dx
    lda #1
    sta sx
dx_done:
    lda y1
    sec
    sbc y0
    bpl dy_positive
    sta dy
    lda #$ffff
    sta sy
    bra dy_done
dy_positive:
    eor #$ffff
    inc a
    sta dy
    lda #1
    sta sy
dy_done:
    lda dx
    clc
    adc dy
    sta line_error
line_loop:
    jsr plot_pixel
    lda x0
    cmp x1
    bne line_continue
    lda y0
    cmp y1
    beq line_done
line_continue:
    lda line_error
    asl
    sta error2

    ; Signed comparison: error2 >= dy.
    lda error2
    sec
    sbc dy
    bvc signed_x_ready
    eor #$8000
signed_x_ready:
    bmi skip_x_step
    lda line_error
    clc
    adc dy
    sta line_error
    lda x0
    clc
    adc sx
    sta x0
skip_x_step:
    ; Signed comparison: error2 <= dx, expressed as dx-error2 >= 0.
    lda dx
    sec
    sbc error2
    bvc signed_y_ready
    eor #$8000
signed_y_ready:
    bmi skip_y_step
    lda line_error
    clc
    adc dx
    sta line_error
    lda y0
    clc
    adc sy
    sta y0
skip_y_step:
    bra line_loop
line_done:
    rts

plot_pixel:
    lda y0
    lsr
    lsr
    lsr
    sta char_row
    asl
    asl
    asl
    asl
    asl
    asl
    sta pixel_address
    lda char_row
    xba
    clc
    adc pixel_address
    sta pixel_address
    lda x0
    and #$fff8
    clc
    adc pixel_address
    sta pixel_address
    lda y0
    and #7
    clc
    adc pixel_address
    adc #BITMAP
    sta BITMAP_PTR
    lda x0
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

cache_bitmap:
    lda frame_no
    and #7
    xba
    asl
    asl
    asl
    asl
    asl
    tay
    lda frame_no
    cmp #8
    bcc cache_bank2
    ldx #BITMAP
    lda #(FRAME_SIZE-1)
    mvn #$00,#$03
    phk
    plb
    rts
cache_bank2:
    ldx #BITMAP
    lda #(FRAME_SIZE-1)
    mvn #$00,#$02
    phk
    plb
    rts

copy_cached_frame:
    lda frame_no
    and #7
    xba
    asl
    asl
    asl
    asl
    asl
    tax
    ldy #BITMAP
    lda frame_no
    cmp #8
    bcc copy_bank2
    lda #(FRAME_SIZE-1)
    mvn #$03,#$00
    phk
    plb
    rts
copy_bank2:
    lda #(FRAME_SIZE-1)
    mvn #$02,#$00
    phk
    plb
    rts

; Called only in emulation mode with IRQs masked.
.as
.xs
restore_optimization:
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
    rts

old_dd00:         .byte 0
old_d011:         .byte 0
old_d016:         .byte 0
old_d018:         .byte 0
old_border:       .byte 0
old_background:   .byte 0
old_sprites:      .byte 0
old_optimization: .byte 0
cache_ok:         .byte 0
probe_saved:      .byte 0
probe_expected:   .byte 0
start_tick:       .byte 0

frame_no:      .word 0
coord_base:    .word 0
edge_pos:      .word 0
x0:            .word 0
y0:            .word 0
x1:            .word 0
y1:            .word 0
dx:            .word 0
dy:            .word 0
sx:            .word 0
sy:            .word 0
line_error:    .word 0
error2:        .word 0
char_row:      .word 0
pixel_address: .word 0

bit_masks: .byte $80,$40,$20,$10,$08,$04,$02,$01
edges: .byte 0,1, 1,2, 2,3, 3,0, 4,5, 5,6, 6,7, 7,4, 0,4, 1,5, 2,6, 3,7

; Sixteen projected vertex sets, one revolution. Each pair is x,y.
frame_vertices:
    .byte 127,67,193,67,193,133,127,133,140,80,180,80,180,120,140,120
    .byte 118,77,182,82,177,145,126,129,149,70,192,73,187,115,150,108
    .byte 117,79,160,113,160,146,130,115,160,60,203,79,190,115,160,95
    .byte 120,73,139,132,148,136,135,98,175,56,206,104,188,119,170,85
    .byte 127,67,127,133,140,120,140,80,193,67,193,133,180,120,180,80
    .byte 139,68,120,127,135,102,148,64,206,96,175,144,170,115,188,81
    .byte 160,87,117,121,130,85,160,54,203,121,160,140,160,105,190,85
    .byte 182,118,118,123,126,71,177,55,192,127,149,130,150,92,187,85
    .byte 193,133,127,133,127,67,193,67,180,120,140,120,140,80,180,80
    .byte 194,129,143,145,138,82,202,77,170,108,133,115,128,73,171,70
    .byte 190,115,160,146,160,113,203,79,160,95,130,115,117,79,160,60
    .byte 185,98,172,136,181,132,200,73,150,85,132,119,114,104,145,56
    .byte 180,80,180,120,193,133,193,67,140,80,140,120,127,133,127,67
    .byte 172,64,185,102,200,127,181,68,132,81,150,115,145,144,114,96
    .byte 160,54,190,85,203,121,160,87,130,85,160,105,160,140,117,121
    .byte 143,55,194,71,202,123,138,118,133,85,170,92,171,130,128,127

intro_msg:       .byte 147
                 .text "3D HIRES CUBE - ML/SUPERRAM"
                 .byte 13,0
cache_msg:       .text "CACHING 16 FRAMES..."
                 .byte 13,0
no_scpu_msg:     .text "NO SUPERCPU DETECTED"
                 .byte 13,0
no_superram_msg: .text "REQUIRES SUPERRAM BANKS $02-$03"
                 .byte 13,0
