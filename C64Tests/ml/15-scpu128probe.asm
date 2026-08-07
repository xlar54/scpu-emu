.setcpu "65816"
.segment "CODE"

; C128 BASIC 7 loader at $1c01: 10 SYS7181 ($1c0d).
.word $1c01
.word basic_end
.word 10
.byte $9e
.byte "7181", 0
basic_end:
.word 0

HWREGS_OPEN  = $d07e
HWREGS_CLOSE = $d07f
MMU_BASE     = $d500
SCPU_BASE    = $d0b0

SETLFS = $ffba
SETNAM = $ffbd
OPEN   = $ffc0
CLOSE  = $ffc3
CHKOUT = $ffc9
CLRCHN = $ffcc
CHROUT = $ffd2
READST = $ffb7

LFN = 2
DEV = 8
SA  = 2

ptr = $fb                       ; three-byte 65816 indirect-long pointer

start:
    cld
    lda #147
    jsr CHROUT
    ldx #0
screen_title_loop:
    lda screen_title,x
    beq collect
    phx
    jsr CHROUT
    plx
    inx
    bra screen_title_loop

open_report:
    lda #report_name_end-report_name
    ldx #<report_name
    ldy #>report_name
    jsr SETNAM
    lda #LFN
    ldx #DEV
    ldy #SA
    jsr SETLFS
    jsr OPEN
    jsr READST
    bne file_failed
    ldx #LFN
    jsr CHKOUT
    bcs file_failed
    lda #1
    sta file_open
    jmp output_report

file_failed:
    jsr CLRCHN
    lda #LFN
    jsr CLOSE
    lda #0
    sta file_open
    ldx #0
file_failed_loop:
    lda file_error_msg,x
    bne :+
    jmp output_report
:
    phx
    jsr CHROUT
    plx
    inx
    bra file_failed_loop

output_report:
    jsr write_report
    jmp finish

collect:
    ; Capture passive machine state before changing the SCPU register gate.
    lda $00
    sta port_regs
    lda $01
    sta port_regs+1
    lda $d030
    sta d030_value
    lda $d02f
    sta d02f_value

    ldx #0
closed_loop:
    lda SCPU_BASE,x
    sta scpu_closed,x
    inx
    cpx #16
    bne closed_loop

    ldx #0
mmu_loop:
    lda MMU_BASE,x
    sta mmu_d5,x
    inx
    cpx #12
    bne mmu_loop

    ldx #0
ff_loop:
    lda $ff00,x
    sta mmu_ff,x
    inx
    cpx #5
    bne ff_loop

    ; The gate changes the visible $d0xx/$d2xx area. Keep IRQ/NMI-visible
    ; time as short as possible and do not write any SCPU register.
    php
    sei
    lda #0
    sta HWREGS_OPEN
    ldx #0
open_loop:
    lda SCPU_BASE,x
    sta scpu_open,x
    inx
    cpx #16
    bne open_loop
    lda #0
    sta HWREGS_CLOSE
    plp

    ; Take all memory fingerprints before disk I/O can disturb accelerator
    ; SRAM or transient MMU state. Only the saved results are logged later.
    jsr collect_checksums
    jmp open_report

finish:
    lda file_open
    beq screen_only_done
    jsr CLRCHN
    lda #LFN
    jsr CLOSE
    ldx #0
done_loop:
    lda done_msg,x
    beq done
    phx
    jsr CHROUT
    plx
    inx
    bra done_loop
screen_only_done:
    ldx #0
screen_only_loop:
    lda screen_only_msg,x
    beq done
    phx
    jsr CHROUT
    plx
    inx
    bra screen_only_loop
done:
    rts

write_report:
    ldx #0
    jsr put_string
    .byte "SCPU128 PROBE V1",13,0

    ldx #0
    jsr put_string
    .byte "READ-ONLY SNAPSHOT; PROGRAM-START STATE",13,13,0

    ldx #0
    jsr put_string
    .byte "CPU PORT 00/01: ",0
    lda port_regs
    jsr put_hex
    lda #' '
    jsr CHROUT
    lda port_regs+1
    jsr put_hex
    jsr put_cr

    ldx #0
    jsr put_string
    .byte "VIC D02F/D030: ",0
    lda d02f_value
    jsr put_hex
    lda #' '
    jsr CHROUT
    lda d030_value
    jsr put_hex
    jsr put_cr
    jsr put_cr

    ldx #0
    jsr put_string
    .byte "SCPU D0B0-D0BF CLOSED",13,0
    lda #<scpu_closed
    sta ptr
    lda #>scpu_closed
    sta ptr+1
    lda #0
    sta ptr+2
    lda #16
    jsr put_buffer

    ldx #0
    jsr put_string
    .byte "SCPU D0B0-D0BF OPEN",13,0
    lda #<scpu_open
    sta ptr
    lda #>scpu_open
    sta ptr+1
    lda #0
    sta ptr+2
    lda #16
    jsr put_buffer

    ldx #0
    jsr put_string
    .byte "MMU D500-D50B",13,0
    lda #<mmu_d5
    sta ptr
    lda #>mmu_d5
    sta ptr+1
    lda #0
    sta ptr+2
    lda #12
    jsr put_buffer

    ldx #0
    jsr put_string
    .byte "MMU FF00-FF04",13,0
    lda #<mmu_ff
    sta ptr
    lda #>mmu_ff
    sta ptr+1
    lda #0
    sta ptr+2
    lda #5
    jsr put_buffer

    ldx #0
    jsr put_string
    .byte "VISIBLE WINDOW CRC16-CCITT",13,0
    ldx #0
    jsr put_string
    .byte "A000-BFFF: ",0
    lda window_crc
    jsr put_hex
    lda window_crc+1
    jsr put_hex
    jsr put_cr
    ldx #0
    jsr put_string
    .byte "C000-CFFF: ",0
    lda window_crc+2
    jsr put_hex
    lda window_crc+3
    jsr put_hex
    jsr put_cr
    ldx #0
    jsr put_string
    .byte "E000-FFFF: ",0
    lda window_crc+4
    jsr put_hex
    lda window_crc+5
    jsr put_hex
    jsr put_cr
    jsr put_cr

    ldx #0
    jsr put_string
    .byte "256-BYTE LONG-ADDRESS CRC16",13,0
    lda #$00
    ldx #0
    jsr report_saved_long_set
    lda #$01
    ldx #12
    jsr report_saved_long_set
    jsr put_cr

    ldx #0
    jsr put_string
    .byte "NOTES: RUN AFTER COLD BOOT IN EACH MODE.",13
    .byte "RECORD 40/80 COLUMN, PAL/NTSC, SWITCH",13
    .byte "POSITION, ROM REVISION, AND SIMM SIZE.",13,0
    rts

collect_checksums:
    ; Visible bank-zero windows.
    lda #0
    sta ptr
    sta ptr+2
    lda #$a0
    sta ptr+1
    lda #$20
    sta page_count
    jsr crc_pages
    lda crc_hi
    sta window_crc
    lda crc_lo
    sta window_crc+1

    lda #0
    sta ptr
    sta ptr+2
    lda #$c0
    sta ptr+1
    lda #$10
    sta page_count
    jsr crc_pages
    lda crc_hi
    sta window_crc+2
    lda crc_lo
    sta window_crc+3

    lda #0
    sta ptr
    sta ptr+2
    lda #$e0
    sta ptr+1
    lda #$20
    sta page_count
    jsr crc_pages
    lda crc_hi
    sta window_crc+4
    lda crc_lo
    sta window_crc+5

    ; One page at six offsets in each of long-address banks $00 and $01.
    lda #0
    sta collect_bank
    sta collect_page
    sta collect_result
collect_long_loop:
    ldx collect_page
    lda long_pages,x
    sta ptr+1
    lda #0
    sta ptr
    lda collect_bank
    sta ptr+2
    jsr crc_long_page
    ldx collect_result
    lda crc_hi
    sta long_crc,x
    inx
    lda crc_lo
    sta long_crc,x
    inx
    stx collect_result
    inc collect_page
    lda collect_page
    cmp #6
    bne collect_long_loop
    lda #0
    sta collect_page
    inc collect_bank
    lda collect_bank
    cmp #2
    bne collect_long_loop
    rts

; A=bank label, X=byte offset into saved CRC array.
report_saved_long_set:
    sta report_bank
    stx report_crc_index
    lda #0
    sta report_page_index
report_long_loop:
    lda report_bank
    jsr put_hex
    lda #':'
    jsr CHROUT
    ldx report_page_index
    lda long_pages,x
    jsr put_hex
    lda #'0'
    jsr CHROUT
    lda #' '
    jsr CHROUT
    ldx report_crc_index
    lda long_crc,x
    jsr put_hex
    ldx report_crc_index
    lda long_crc+1,x
    jsr put_hex
    jsr put_cr
    inc report_crc_index
    inc report_crc_index
    inc report_page_index
    lda report_page_index
    cmp #6
    bne report_long_loop
    rts

; CRC page_count pages through a bank-zero 16-bit pointer.
crc_pages:
    jsr crc_init
crc_page_loop:
    ldy #0
crc_byte_loop:
    lda (ptr),y
    jsr crc_byte
    iny
    bne crc_byte_loop
    inc ptr+1
    dec page_count
    bne crc_page_loop
    rts

; CRC one page through a 65816 24-bit direct-page pointer.
crc_long_page:
    jsr crc_init
    ldy #0
crc_long_loop:
    lda [ptr],y
    jsr crc_byte
    iny
    bne crc_long_loop
    rts

crc_init:
    lda #$ff
    sta crc_lo
    sta crc_hi
    rts

; CRC-16/CCITT-FALSE polynomial $1021, initial value $ffff.
crc_byte:
    eor crc_hi
    sta crc_hi
    phx
    ldx #8
crc_bit_loop:
    asl crc_lo
    rol crc_hi
    bcc crc_no_xor
    lda crc_lo
    eor #$21
    sta crc_lo
    lda crc_hi
    eor #$10
    sta crc_hi
crc_no_xor:
    dex
    bne crc_bit_loop
    plx
    rts

put_crc:
    lda crc_hi
    jsr put_hex
    lda crc_lo
    jmp put_hex

; ptr points at buffer in bank zero, A is byte count.
put_buffer:
    sta buffer_count
    lda ptr
    sta buffer_load+1
    lda ptr+1
    sta buffer_load+2
    lda #0
    sta buffer_index
put_buffer_loop:
    ldx buffer_index
buffer_load:
    lda $ffff,x
    jsr put_hex
    inc buffer_index
    lda buffer_index
    cmp buffer_count
    beq put_buffer_done
    lda #' '
    jsr CHROUT
    bra put_buffer_loop
put_buffer_done:
    jsr put_cr
    jmp put_cr

put_hex:
    pha
    lsr
    lsr
    lsr
    lsr
    jsr put_nibble
    pla
    and #$0f
put_nibble:
    cmp #10
    bcc put_digit
    adc #6
put_digit:
    adc #'0'
    jmp CHROUT

put_cr:
    lda #13
    jmp CHROUT

; Inline zero-terminated string. JSR put_string is followed by the bytes.
put_string:
    pla
    sta string_return+1
    pla
    sta string_return+2
put_string_loop:
    inc string_return+1
    bne string_no_carry
    inc string_return+2
string_no_carry:
string_return:
    lda $ffff
    beq string_done
    jsr CHROUT
    bra put_string_loop
string_done:
    lda string_return+2
    pha
    lda string_return+1
    pha
    rts

report_name:
    .byte "@0:SCPU128 LOG,S,W"
report_name_end:

long_pages: .byte $00,$04,$1c,$a0,$c0,$e0
screen_title: .byte "SCPU128 READ-ONLY PROBE",13,"WRITING SCPU128 LOG TO DEVICE 8...",13,0
file_error_msg: .byte "DEVICE 8 LOG OPEN FAILED",13,"REPORT WILL BE PRINTED TO SCREEN",13,13,0
done_msg: .byte 13,"PROBE COMPLETE - SCPU128 LOG SAVED",13,0
screen_only_msg: .byte 13,"PROBE COMPLETE - SCREEN OUTPUT ONLY",13,0

file_open:   .byte 0
buffer_count:.byte 0
buffer_index:.byte 0
range_start: .byte 0
page_count:  .byte 0
crc_lo:      .byte 0
crc_hi:      .byte 0
collect_bank:.byte 0
collect_page:.byte 0
collect_result:.byte 0
report_bank:.byte 0
report_page_index:.byte 0
report_crc_index:.byte 0
port_regs:   .res 2
d02f_value:  .byte 0
d030_value:  .byte 0
scpu_closed: .res 16
scpu_open:   .res 16
mmu_d5:      .res 12
mmu_ff:      .res 5
window_crc:  .res 6
long_crc:    .res 24
