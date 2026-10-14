; 30-vicvsp.asm - controlled SuperCPU/VIC-II VSP timing diagnostic
;
; This is deliberately a TEST, not a demo.  It reduces Mario-style VSP to a
; deterministic screen, one double-IRQ stabiliser and one selectable variable:
;
;   1  CONTROL       identical timing, but DEC/INC touches harmless RAM
;   2  VSP AT 1MHZ   DEC/INC $D011 while $D07A has selected 1 MHz
;   3  VSP AT TURBO  same trigger without the $D07A/$D07B speed change
;   4  VSP RESULT    mode-2 timing plus a sprite/background-collision probe
;   5..8 SCOPE SEG   four adjacent sub-windows of the mode-2 IRQ path
;
; Mode 2 is the important SuperCPU conformance case.  Mario selects 1 MHz in
; its first IRQ, stabilises a second IRQ, and then reaches the VSP trigger after
; exactly 104 C64 cycles (103 when the one-cycle jitter absorber is not taken).
; The instruction sequence below preserves that count.
;
; The visible matrix contains a row-offset A..Z sequence.  CONTROL must remain
; ordered.  The picture is a useful stability canary, but it is not the primary
; result: VSP can alter the VIC's internal idle/matrix state without producing
; a simple whole-screen shift.  The signature and scope marker below provide
; the unambiguous timing result.
;
; A machine-readable block at $02D0 lets firmware recognise this test without
; depending on a game address:
;
;   +0..3  "VSPT"       signature
;   +4     version       3
;   +5     mode          1..8
;   +6     state         1 while running
;   +7..8  frame count   little endian
;   +9     raster read immediately after the trigger
;   +10    expected $0846-to-trigger cycles (104)
;   +11    flags         bit 0: user-port scope marker enabled
;   +12    expected marker-high cycles (98)
;   +13    $D0B8 sampled after the trigger (bit 6: 1 MHz)
;   +14..15 detected VSP idle address, high byte then low byte
;   +16    detector status: 0 measuring, 1 recognised address, $ff unexpected
;   +17    scope segment: 0 full pulse, otherwise 1..4
;
; Scope marker: CIA2 PB0, user-port pin C.  The line rises six cycles after the
; second IRQ begins and falls immediately before DEC $D011.  It is therefore
; high for exactly 98 C64 cycles when the post-$0846 path really runs at 1 MHz.
; The two LDA/STA pairs replace six NOPs, so the D011 trigger still begins at
; cycle 104.  Modes 5..8 retain that 104-cycle trigger placement but split the
; full 98-cycle interval into expected 44, 30, 18 and 6-cycle pulses.  Their
; five marker stores are identical in every segment mode; only the two values
; that create the visible pulse differ.  Disconnect user-port hardware while
; using this test.

                * = $0801

                .word basic_end
                .word 10
                .byte $9e
                .text format("%4d", start)
                .byte 0
basic_end       .word 0

; ---------------------------------------------------------------------------
; Mario-compatible double-IRQ timing core.  Keeping these entry addresses makes
; traces directly comparable with the game, although the test does not depend
; on the addresses themselves.

                * = $0813

FIRST_IRQ_LINE  = $33
COMPARE_LINE    = $35
SCOPE_MARKER    = 1

irq1
                stx saved_x
                sta saved_a
speed_on        sta $d07a              ; software 1 MHz on; value is ignored
irq1_mode_hook
                nop
                nop
                nop
                lda #<irq2
                sta $fffe              ; both handlers remain in page $08
                lda #$ff
                sta $d019              ; acknowledge first raster IRQ
                inc $d012              ; arm the following raster line
                tsx                     ; remember the first IRQ's stack
                cli
                .rept 24               ; second IRQ lands somewhere in here
                nop
                .endrept
                jmp *                   ; should never be reached

                * = $0846

irq2
.if SCOPE_MARKER
marker_on       lda #$01
                sta $dd01
.endif
                txs                     ; discard the nested IRQ frame
                ldx #$07
delay_loop      dex
                bne delay_loop          ; 34 cycles including LDX
                .rept 6
                nop                     ; 12 cycles
                .endrept
compare_raster  lda #COMPARE_LINE
                cmp $d012
                beq jitter_absorbed     ; 3 cycles when aligned, 2 otherwise
jitter_absorbed jmp fixed_sled

fixed_sled
.if SCOPE_MARKER
                .rept 15
                nop
                .endrept
marker_off      lda #$00
                sta $dd01
.else
                .rept 21
                nop
                .endrept
.endif

; From irq2 through the instruction before this label is 104 C64 cycles when
; BEQ is taken.  The operands are patched to $02F0 in CONTROL mode.
vsp_dec         dec $d011
vsp_inc         inc $d011

irq_post_trigger
                lda $d012
                sta SIG_RASTER
                lda $d0b8
                sta SIG_SPEED
                inc SIG_FRAME_LO
                bne +
                inc SIG_FRAME_HI
+
                ; Mode 3 deliberately removes the 1 MHz window and can become
                ; an IRQ storm on a SuperCPU.  Check SPACE inside the handler
                ; as well as in the foreground so the negative control still
                ; has a reliable escape path.
                lda #$7f
                sta $dc00
                lda $dc01
                and #$10
                bne +
                jmp exit_test
+

                lda #$ff
                sta $d019
irq_next_vector lda #<irq1
                sta $fffe
irq_rearm_line  lda #FIRST_IRQ_LINE
                sta $d012
speed_off       sta $d07b              ; software 1 MHz off; value is ignored
                lda saved_a
                ldx saved_x
                rti

saved_a         .byte 0
saved_x         .byte 0

; ---------------------------------------------------------------------------
; Modes 5..8 duplicate the Mario-compatible handler while exposing four
; adjacent scope intervals.  Five fixed-cost marker slots preserve the same
; 104-cycle irq2-to-DEC placement in every mode.  Every slot performs the same
; CIA2 write, so comparisons between segments do not add a changing observer.
;
;   segment 1: irq2 marker -> after DEX/BNE delay        44 cycles
;   segment 2: delay -> after raster compare/jitter      30 cycles
;   segment 3: compare -> after final NOP sled           18 cycles
;   segment 4: final marker handoff -> DEC $D011          6 cycles

                * = $0900

seg_irq1
                stx saved_x
                sta saved_a
                sta $d07a               ; software 1 MHz on
                lda #<seg_irq2
                sta $fffe               ; both segmented handlers are page $09
                lda #$ff
                sta $d019
                inc $d012
                tsx
                cli
                .rept 24
                nop
                .endrept
                jmp *

seg_irq2
seg_marker_0    lda #$00
                sta $dd01
                txs
                ldx #$07
seg_delay_loop  dex
                bne seg_delay_loop
seg_marker_1    lda #$00
                sta $dd01
                .rept 6
                nop
                .endrept
                lda #COMPARE_LINE
                cmp $d012
                beq seg_jitter_absorbed
seg_jitter_absorbed
                jmp seg_marker_2

seg_marker_2    lda #$00
                sta $dd01
seg_fixed_sled
                .rept 6
                nop
                .endrept
seg_marker_3    lda #$00
                sta $dd01
seg_marker_4    lda #$00
                sta $dd01

; The five six-cycle marker slots plus the shortened sled total exactly the
; same 104 cycles as irq2 above when BEQ is taken.
                dec $d011
                inc $d011
                jmp irq_post_trigger

; Mode 4 uses the timer-stabilised handler from VICE's vsp-tester.  The original
; uses undocumented NMOS opcodes in its one-time synchroniser and NOP sled;
; this port keeps the timing but uses instructions valid on the W65C816.
; Keep both handlers in one page.  The reference changes only $FFFE between
; them; updating $FFFF as well delays the bottom-border $D011 write by six
; cycles and destroys the phase the collision detector is meant to measure.
                * = $0a00
detector_vsp_irq
                sta $20
                lda $dd04
                eor #$3f
                sta detector_int_delay+1
detector_int_delay
                bne *
                ; NMOS $80 is a two-byte/two-cycle NOP.  Repeating $A9 gives
                ; the same byte/cycle geometry on the 65816: wherever the
                ; branch lands, it executes harmless LDA-immediate pairs.
                .fill $3d,$a9
                .byte $a5               ; LDA dp: legal three-cycle tail
                nop                     ; operand consumed by LDA dp
                nop
                lda #$1a
                sta $d011
                inc $d011
                lda #<detector_open_border
                sta $fffe
                lda #$fa
                sta $d012
                dec $d019
                lda $20
                rti

detector_open_border
                sta $20
                lda #<detector_vsp_irq
                sta $fffe
                lda #$32
                sta $d012
                lda #$13
                sta $d011
                dec $d019
                inc SIG_FRAME_LO         ; proves the bottom IRQ chain is live
                lda $20
                rti

                ; Keep the detector sprite below the probed RAM.  The main
                ; program lives at $4000 so mode 4 can fill the same complete
                ; $0B00-$3FFF range as the authoritative VICE test.
                .align 64
detector_sprite
                .byte $80               ; one opaque pixel
                .fill 62,$00

; ---------------------------------------------------------------------------

                * = $4000

SCREEN          = $0400
COLOR           = $d800
CONTROL_CELL    = $02f0

SIG_BASE        = $02d0
SIG_VERSION     = SIG_BASE+4
SIG_MODE        = SIG_BASE+5
SIG_STATE       = SIG_BASE+6
SIG_FRAME_LO    = SIG_BASE+7
SIG_FRAME_HI    = SIG_BASE+8
SIG_RASTER      = SIG_BASE+9
SIG_CYCLES      = SIG_BASE+10
SIG_FLAGS       = SIG_BASE+11
SIG_PULSE       = SIG_BASE+12
SIG_SPEED       = SIG_BASE+13
SIG_DETECT_HI   = SIG_BASE+14
SIG_DETECT_LO   = SIG_BASE+15
SIG_DETECT_OK   = SIG_BASE+16
SIG_SEGMENT     = SIG_BASE+17

start
                sei
                lda #147
                jsr $ffd2
                lda #<menu_text
                ldy #>menu_text
                jsr print_z

choose_mode
                jsr $ffe4
                cmp #'1'
                beq mode_control
                cmp #'2'
                beq mode_vsp_slow
                cmp #'3'
                beq mode_vsp_turbo
                cmp #'4'
                beq mode_vsp_result
                cmp #'5'
                bcc choose_mode
                cmp #'9'
                bcs choose_mode
                sec
                sbc #'4'                ; ASCII 5..8 -> segment 1..4
                jmp mode_scope_segment
                jmp choose_mode

mode_control
                lda #1
                sta selected_mode
                jsr restore_normal_geometry
                jsr restore_slow_strobes
                lda #<CONTROL_CELL
                sta vsp_dec+1
                sta vsp_inc+1
                lda #>CONTROL_CELL
                sta vsp_dec+2
                sta vsp_inc+2
                lda #$1f
                sta CONTROL_CELL
                lda #<control_text
                ldy #>control_text
                jmp mode_ready

mode_vsp_slow
                lda #2
                sta selected_mode
                jsr restore_normal_geometry
                jsr restore_vsp_target
                jsr restore_slow_strobes
                lda #<vsp_slow_text
                ldy #>vsp_slow_text
                jmp mode_ready

mode_vsp_turbo
                lda #3
                sta selected_mode
                jsr restore_normal_geometry
                jsr restore_vsp_target
                ; $D071 is decoded and consumes the same I/O access, but has no
                ; speed side effect.  Patch both speed strobes to that address.
                lda #$71
                sta speed_on+1
                sta speed_off+1
                lda #<vsp_turbo_text
                ldy #>vsp_turbo_text
                jmp mode_ready

mode_vsp_result
                lda #4
                sta selected_mode
                jsr restore_normal_geometry
                jsr restore_vsp_target
                jsr restore_slow_strobes
                lda #<vsp_result_text
                ldy #>vsp_result_text
                jmp mode_ready

mode_scope_segment
                tax
                jsr restore_normal_geometry
                txa
                sta selected_segment
                clc
                adc #4                  ; signature mode 5..8
                sta selected_mode
                jsr restore_vsp_target
                jsr restore_slow_strobes
                lda #<seg_irq1
                sta install_irq_low+1
                lda #>seg_irq1
                sta install_irq_high+1
                lda #<seg_irq1
                sta irq_next_vector+1
                lda segment_text_lo-1,x
                ldy segment_text_hi-1,x
                jmp mode_ready

; Restore every self-modified operand explicitly.  This makes SYS 4096 safe
; after a prior mode has patched the resident PRG.
restore_vsp_target
                lda #$ce
                sta vsp_dec
                lda #<$d011
                sta vsp_dec+1
                sta vsp_inc+1
                lda #>$d011
                sta vsp_dec+2
                sta vsp_inc+2
                lda #$00
                sta marker_off+1
                lda #<$dd01
                sta marker_off+3
                lda #>$dd01
                sta marker_off+4
                rts

restore_slow_strobes
                lda #$7a
                sta speed_on+1
                lda #$7b
                sta speed_off+1
                rts

restore_normal_geometry
                lda #0
                sta selected_segment
                lda #$ea
                sta irq1_mode_hook
                sta irq1_mode_hook+1
                sta irq1_mode_hook+2
                lda #<irq1
                sta irq_next_vector+1
                sta install_irq_low+1
                lda #>irq1
                sta install_irq_high+1
                lda #FIRST_IRQ_LINE
                sta irq_rearm_line+1
                sta install_first_line+1
                lda #COMPARE_LINE
                sta compare_raster+1
                lda #$1f
                sta install_d011+1
                rts

mode_ready
                sta mode_string
                sty mode_string+1
                jsr initialise_signature
                jsr prepare_screen
                lda selected_mode
                cmp #4
                bne +
                jsr detector_install
                jmp detector_measure
+
                jsr install_test

main_loop
                ; SPACE: CIA1 row 7, column 4.  No KERNAL IRQ is active while
                ; the direct hardware vector owns $FFFE.
                lda #$7f
                sta $dc00
                lda $dc01
                and #$10
                bne main_loop

exit_test
                sei
                lda #0
                sta SIG_STATE
                sta $d01a
.if SCOPE_MARKER
                lda saved_port_b
                sta $dd01
                lda saved_ddr_b
                sta $dd03
.endif
                lda #$37
                sta $01
                jmp $fce2               ; clean KERNAL/BASIC restart

; ---------------------------------------------------------------------------

initialise_signature
                ldx #0
-               lda signature,x
                sta SIG_BASE,x
                inx
                cpx #4
                bne -
                lda #3
                sta SIG_VERSION
                lda selected_mode
                sta SIG_MODE
                lda #1
                sta SIG_STATE
                lda #0
                sta SIG_FRAME_LO
                sta SIG_FRAME_HI
                sta SIG_RASTER
                lda #104
                sta SIG_CYCLES
                lda #SCOPE_MARKER
                sta SIG_FLAGS
                lda #98
                sta SIG_PULSE
                lda #0
                sta SIG_SPEED
                sta SIG_DETECT_HI
                sta SIG_DETECT_LO
                sta SIG_DETECT_OK
                lda selected_segment
                sta SIG_SEGMENT
                beq +
                tax
                lda segment_pulse_cycles-1,x
                sta SIG_PULSE
+
                rts

prepare_screen
                lda #147
                jsr $ffd2
                lda #<running_text
                ldy #>running_text
                jsr print_z
                lda mode_string
                ldy mode_string+1
                jsr print_z

                ; Rows 2..24 contain a column sequence offset by three letters
                ; per row.  VSP changes the matrix counter horizontally, so a
                ; row made from one repeated character would hide the very
                ; displacement this test is meant to expose.
                lda #<$0450
                sta $fb
                lda #>$0450
                sta $fc
                lda #<$d850
                sta $fd
                lda #>$d850
                sta $fe
                ldx #0
                stx row_index
row_loop
                txa
                asl
                clc
                adc row_index           ; seed = row * 3
                sta row_seed
                ldy #0
cell_loop
                tya
                clc
                adc row_seed
reduce_code     cmp #26
                bcc code_ready
                sbc #26                 ; carry is set by CMP
                bcs reduce_code
code_ready      clc
                adc #1                  ; screen codes A..Z
                sta ($fb),y
                pha
                txa
                and #7
                clc
                adc #1                  ; colours 1..8 identify row boundaries
                sta ($fd),y
                pla
                iny
                cpy #40
                bne cell_loop
                jsr advance_rows
                inx
                stx row_index
                cpx #23
                bne row_loop

                ; Guard bytes immediately after the visible 1000-byte matrix.
                ; Firmware can compare these without interpreting the picture.
                ldx #0
guard_loop      txa
                eor #$a5
                sta SCREEN+1000,x
                inx
                cpx #24
                bne guard_loop
                rts

advance_rows
                lda $fb
                clc
                adc #40
                sta $fb
                bcc +
                inc $fc
+               lda $fd
                clc
                adc #40
                sta $fd
                bcc +
                inc $fe
+               rts

install_test
                sei
                lda #$7f
                sta $dc0d
                sta $dd0d
                lda $dc0d
                lda $dd0d

                lda #$ff
                sta $dc02               ; keyboard column output
                lda #$00
                sta $dc03               ; keyboard row input

.if SCOPE_MARKER
                lda $dd01
                sta saved_port_b
                and #$fe
                sta marker_off+1
                sta seg_marker_0+1
                sta seg_marker_1+1
                sta seg_marker_2+1
                sta seg_marker_3+1
                sta seg_marker_4+1
                ora #$01
                sta marker_on+1
                ldx selected_segment
                beq scope_values_ready
                dex
                beq scope_segment_1
                dex
                beq scope_segment_2
                dex
                beq scope_segment_3
                ; Segment 4 starts at the final handoff slot.
                sta seg_marker_3+1
                bne scope_values_ready
scope_segment_1
                sta seg_marker_0+1
                bne scope_values_ready
scope_segment_2
                sta seg_marker_1+1
                bne scope_values_ready
scope_segment_3
                sta seg_marker_2+1
scope_values_ready
                lda $dd03
                sta saved_ddr_b
                ora #$01
                sta $dd03               ; CIA2 PB0 output, other bits unchanged
                lda marker_off+1
                sta $dd01
.endif

                lda #$00
                sta $d015               ; sprite DMA would disturb the window
                sta $d020
                sta $d021
                lda #$c8
                sta $d016
                lda #$15                ; screen $0400, character ROM
                sta $d018
install_d011    lda #$1f                ; patched to $1b for detector mode
                sta $d011

                lda #$35                ; RAM vectors, I/O still visible
                sta $01
install_irq_low lda #<irq1
                sta $fffe
install_irq_high
                lda #>irq1
                sta $ffff
install_first_line
                lda #FIRST_IRQ_LINE
                sta $d012
                lda #$ff
                sta $d019
                lda #$01
                sta $d01a
                cli
                rts

; ---------------------------------------------------------------------------
; Mode 4 adapts the result detector from VICE testprogs/VICII/vsp-tester.
; Eight one-pixel sprites sit behind the graphics at the VSP fetch position.
; Filling the upper VIC-bank pages first with each address's high byte and then
; its low byte makes $D01F return the address of the idle byte fetched by the
; VIC on the trigger cycle.  Unlike the row picture, this observes the VSP
; effect itself and therefore provides a numeric fix/no-fix result.

DETECT_FILL_FIRST = $0b             ; official test_area page
DETECT_FILL_END   = $40
DETECT_LINE       = $31
DETECT_TEXT       = SCREEN+80
DETECT_HEX        = DETECT_TEXT+6
DETECT_STATUS     = DETECT_TEXT+11

detector_install
                sei

                ; The detector's synchroniser measures C64/CIA cycles, so it
                ; must run at 1 MHz even when the test was entered in Turbo.
                ; It also uses direct-page $02 as the reference test does.
                ; Entry is through BASIC in 6502-emulation mode, whose direct
                ; page is $0000, so that store is exactly three cycles.  Do
                ; not use PEA/PLD to assert it here: mode 4 must remain valid
                ; on the stock 6510 x64sc reference as well as on a 65816.
                lda #$00
                sta $d07a

                lda $dd02
                ora #$03
                sta $dd02
                lda $dd00
                ora #$03
                sta $dd00               ; VIC bank 0
                lda #$00
                sta $d015
                lda #$c8
                sta $d016
                lda #$15                ; screen $0400, character ROM
                sta $d018

                ; A stock 6510 can use the authoritative NMOS synchroniser
                ; byte for byte.  On a SuperCPU, those bytes are real 65816
                ; opcodes (LDA long/WAI), so select a legal equivalent instead.
                ; $D0BC bit 7 is the same presence test used by 00-detect.
                lda $d0bc
                bmi detector_sync_nmos

                ; Legal 65816 path.  The two $DC04 reads remain nine cycles
                ; apart: STA dp (3) + SBC imm (2) + CMP abs (4).
detector_sync_816
                sec
                lda $dc04
                sta $02
                sbc #51
                cmp $dc04
                bne detector_sync_816
                jmp detector_sync_done

                ; Official vsp-tester loop: LAX $DC04 / SBX #51.  Emit the
                ; undocumented opcodes as data so this file still assembles in
                ; 65816 mode; the presence branch guarantees a 65816 never
                ; executes them.
detector_sync_nmos
                .byte $af,$04,$dc       ; LAX $DC04
                .byte $cb,$33           ; SBX #51
                sta $02
                cpx $dc04
                bne detector_sync_nmos

detector_sync_done
                lda #64
                sta $dd04
                lda #0
                sta $dd05
                ldx #3
-               dex
                bne -
                nop
                nop
                lda #$11
                sta $dd0e

                lda #$35
                sta $01
                lda #$7f
                sta $dc0d
                sta $dd0d
                lda $dc0d
                lda $dd0d

                lda #<detector_open_border
                sta $fffe
                lda #>detector_open_border
                sta $ffff
                lda #$fa
                sta $d012
                lda #$1b
                sta $d011
                lda #$01
                sta $d01a
                dec $d019
                cli
                rts

detector_measure
                ; Modes 1-3 deliberately use a dense A..Z matrix as a visual
                ; displacement canary.  That matrix is invalid input for the
                ; collision detector: its one-pixel sprites sit behind the
                ; foreground, so ordinary glyph pixels would make all eight
                ; collision bits read as $FF.  Give mode 4 the blank probe
                ; field used by the reference test; sprite pointers are
                ; restored immediately afterwards.
                jsr detector_clear_screen
                jsr detector_prepare_sprites

                ; Enable once before measurement so every sprite's MCBASE is
                ; in a known state, matching the authoritative VICE test.
                lda #$ff
                sta $d015
                jsr detector_fill_high
                lda #$00
                sta $d015
                jsr detector_next_top
                lda #$ff
                sta $d015
                lda $d01f              ; clear the collision latch
                jsr detector_wait_bottom
                lda $d01f
                sta SIG_DETECT_HI

                lda #$00
                sta $d015
                jsr detector_fill_low
                jsr detector_next_top
                lda #$ff
                sta $d015
                lda $d01f              ; clear the collision latch
                jsr detector_wait_bottom
                lda $d01f
                sta SIG_DETECT_LO
                lda #$00
                sta $d015

                jsr detector_show_address
                jsr detector_check_address

detector_hold
                lda SIG_DETECT_OK
                cmp #1
                bne detector_no_animate
detector_animate
                inc $3fff              ; operand patched to detected address
                lda #1
                sta $3fff              ; keep ordinary idle byte quiet
detector_no_animate
                ldx #0
-               dex
                bne -
                ldy #0
-               dey
                bne -
                lda #$7f
                sta $dc00
                lda $dc01
                and #$10
                bne detector_hold
                jmp exit_test

detector_clear_screen
                lda #$20                ; screen-code SPACE has no foreground
                ldx #$00
-               sta $0400,x
                sta $0500,x
                sta $0600,x
                sta $0700,x
                inx
                bne -
                rts

detector_prepare_sprites
                lda #(detector_sprite >> 6)
                ldx #7
-               sta $07f8,x
                dex
                bpl -

                lda #$ff
                sta $d01b              ; sprites behind foreground graphics
                sta $d010              ; all sprite X positions are 256+
                lda #$00
                sta $d017
                sta $d01c
                sta $d01d
                ldx #$48
                ldy #14
-               txa
                sta $d000,y
                lda #DETECT_LINE
                sta $d001,y
                inx
                dey
                dey
                bpl -
                rts

detector_next_top
                jsr detector_wait_bottom
detector_wait_top
-               bit $d011
                bmi -
                rts

detector_wait_bottom
-               bit $d011
                bpl -
                rts

detector_fill_high
                lda #DETECT_FILL_FIRST
                sta detector_high_store+2
detector_high_page
                lda detector_high_store+2
                ldx #0
detector_high_byte
detector_high_store
                sta $3000,x
                inx
                bne detector_high_byte
                inc detector_high_store+2
                lda detector_high_store+2
                cmp #DETECT_FILL_END
                bne detector_high_page
                rts

detector_fill_low
                lda #DETECT_FILL_FIRST
                sta detector_low_store+2
detector_low_page
                ldx #0
detector_low_byte
                txa
detector_low_store
                sta $3000,x
                inx
                bne detector_low_byte
                inc detector_low_store+2
                lda detector_low_store+2
                cmp #DETECT_FILL_END
                bne detector_low_page
                rts

detector_show_address
                ldx #0
-               lda detector_label,x
                sta DETECT_TEXT,x
                inx
                cpx #6
                bne -
                lda SIG_DETECT_HI
                ldy #0
                jsr detector_print_byte
                lda SIG_DETECT_LO
                ldy #2
                jsr detector_print_byte
                rts

detector_print_byte
                tax
                lsr
                lsr
                lsr
                lsr
                jsr detector_print_digit
                iny
                txa
                and #$0f
detector_print_digit
                clc
                adc #$30
                cmp #$3a
                bcc +
                sbc #$39              ; screen codes A..F are $01..$06
+               sta DETECT_HEX,y
                rts

detector_check_address
                ; Addresses observed by the reference test on real VIC-IIs:
                ; $3FFF, $38FF, $3807, $38C7 and $38D7.
                lda SIG_DETECT_HI
                cmp #$3f
                bne detector_check_38
                lda SIG_DETECT_LO
                cmp #$ff
                beq detector_pass
                bne detector_fail
detector_check_38
                cmp #$38
                bne detector_fail
                lda SIG_DETECT_LO
                cmp #$ff
                beq detector_pass
                cmp #$07
                beq detector_pass
                cmp #$c7
                beq detector_pass
                cmp #$d7
                bne detector_fail

detector_pass
                lda #1
                sta SIG_DETECT_OK
                lda SIG_DETECT_LO
                sta detector_animate+1
                lda SIG_DETECT_HI
                sta detector_animate+2
                lda #$0d              ; light green
                sta $d020
                ldx #0
-               lda detector_pass_text,x
                sta DETECT_STATUS,x
                inx
                cpx #4
                bne -
                rts

detector_fail
                lda #$ff
                sta SIG_DETECT_OK
                lda #$02              ; red
                sta $d020
                ldx #0
-               lda detector_fail_text,x
                sta DETECT_STATUS,x
                inx
                cpx #4
                bne -
                rts

; A/Y = zero-terminated PETSCII string address.
print_z
                sta $fb
                sty $fc
                ldy #0
-               lda ($fb),y
                beq +
                jsr $ffd2
                iny
                bne -
+               rts

selected_mode   .byte 0
selected_segment .byte 0
mode_string     .word 0
saved_ddr_b     .byte 0
saved_port_b    .byte 0
row_index       .byte 0
row_seed        .byte 0

signature       .text "VSPT"

segment_pulse_cycles
                .byte 44,30,18,6
segment_text_lo
                .byte <scope_seg1_text,<scope_seg2_text
                .byte <scope_seg3_text,<scope_seg4_text
segment_text_hi
                .byte >scope_seg1_text,>scope_seg2_text
                .byte >scope_seg3_text,>scope_seg4_text

menu_text
                .text "VSP TIMING TEST",13,13
                .text "1  CONTROL - NO VIC TRIGGER",13
                .text "2  VSP WITH SCPU 1MHZ",13
                .text "3  VSP WITHOUT SLOW MODE",13,13
                .text "4  VSP COLLISION RESULT",13
                .text "5  SCOPE IRQ DELAY",13
                .text "6  SCOPE RASTER COMPARE",13
                .text "7  SCOPE FINAL SLED",13
                .text "8  SCOPE TRIGGER HANDOFF",13,13
                .text "SPACE EXITS AFTER START",13
                .text "DISCONNECT USER PORT HARDWARE",13,0

running_text
                .text "VSPT V3  SPACE EXITS",13,0
control_text
                .text "CONTROL: ROWS MUST STAY ORDERED",13,0
vsp_slow_text
                .text "VSP: 1MHZ MARIO TIMING",13,0
vsp_turbo_text
                .text "VSP: TURBO NEGATIVE CONTROL",13,0
vsp_result_text
                .text "VSP: COLLISION RESULT",13,0
scope_seg1_text
                .text "SCOPE 1 IRQ-DELAY  EXPECT 44",13,0
scope_seg2_text
                .text "SCOPE 2 COMPARE    EXPECT 30",13,0
scope_seg3_text
                .text "SCOPE 3 SLED       EXPECT 18",13,0
scope_seg4_text
                .text "SCOPE 4 HANDOFF    EXPECT 06",13,0

; Native C64 screen codes used after ROM and KERNAL output are hidden.
detector_label
                .byte 9,4,12,5,32,36       ; "IDLE $"
detector_pass_text
                .byte 16,1,19,19            ; "PASS"
detector_fail_text
                .byte 6,1,9,12              ; "FAIL"
