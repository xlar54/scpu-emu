; 30-vicvsp.asm - VSP ($D011 mid-line) cycle placement conformance
;
; VSP is the one VIC technique whose entire content is WHICH CYCLE a write
; lands on. Changing YSCROLL while the VIC is deciding whether the current line
; is a badline makes it skip that badline; the video matrix counter then does
; not advance and the display below the split repeats a row. Land the same
; write one cycle either side of that window and nothing happens at all.
;
; That makes it the sharpest possible probe for a defect where the physical
; write is performed before the emulator has charged the store, so the edge
; reaches the chip at whatever phase the host happened to reach rather than at
; the cycle the guest intended. Ordinary register writes survive that; VSP
; cannot.
;
; The screen is filled with per-row digits, so the result is unambiguous rather
; than a judgement call:
;
;   VSP TOOK EFFECT   rows below the split are shifted -- a digit repeats, and
;                     every row underneath is renumbered against the reference
;   VSP DID NOT       the digit column runs 0..9 straight down, unbroken
;
; Compare against VICE the way the other VIC tests are compared. A frame that
; differs only in the transition row is a separate and much smaller question
; than a frame where the split did not happen at all.
;
; TUNING NOTE: VSP_DELAY below is the one value that decides whether the write
; lands inside the badline-decision window. It is expressed as added NOPs after
; a stabilised raster interrupt, so it can be walked one cycle at a time against
; a reference emulator without touching anything else. If this test shows "no
; split" on hardware AND in VICE, the delay is wrong, not the machine.

                * = $0801

; BASIC stub: 10 SYS 2061
                .word eob
                .word 10
                .byte $9e
                .text "2061"
                .byte 0
eob             .word 0

VSP_LINE        = $60           ; split line: low 3 bits are 0, so YSCROLL 0 badlines here
VSP_YSCROLL_ON  = $10           ; $D011 with DEN set, YSCROLL = 0  -> badline at VSP_LINE
VSP_YSCROLL_OFF = $13           ; YSCROLL = 3 -> condition no longer met, badline suppressed
VSP_DELAY       = 6             ; NOPs after stabilisation, see TUNING NOTE above

start
                sei
                lda $dd02
                ora #$03
                sta $dd02
                lda $dd00
                ora #$03
                sta $dd00               ; VIC bank 0

                lda #VSP_YSCROLL_ON
                sta $d011               ; DEN, RSEL clear, YSCROLL 0, raster MSB 0
                lda #$c8
                sta $d016
                lda #$15                ; screen $0400, character ROM
                sta $d018
                lda #$00
                sta $d015               ; no sprites -- their DMA would steal cycles
                lda #$00
                sta $d020
                sta $d021

                ; Fill each text row with its own digit. A repeated or missing
                ; digit is the VSP result, readable without a reference image.
                lda #<$0400
                sta $fb
                lda #>$0400
                sta $fc
                lda #<$d800
                sta $fd
                lda #>$d800
                sta $fe
                ldx #$00                ; row counter
rowloop         txa
                and #$09                ; keep digits 0..9 cycling
                clc
                adc #$30                ; screen code for '0'
                ldy #$27
digitloop       sta ($fb),y
                pha
                lda #$01                ; white
                sta ($fd),y
                pla
                dey
                bpl digitloop
                ; advance both pointers by 40
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
+               inx
                cpx #25
                bne rowloop

                ; Raster interrupt one line early. The handler re-arms on the
                ; target line and the SECOND entry is cycle-stable, which is the
                ; only way a mid-line write means anything.
                lda #<irq1
                sta $0314
                lda #>irq1
                sta $0315
                lda #$7f
                sta $dc0d               ; no CIA interrupts
                lda $dc0d
                lda #$01
                sta $d01a               ; raster IRQ on
                lda #VSP_LINE-1
                sta $d012
                lda $d019
                sta $d019               ; clear pending
                cli
halt            jmp halt

; ---------------------------------------------------------------------------
; First interrupt: arrives with up to seven cycles of jitter because the
; instruction in progress must finish. Re-arm for the target line and hand over
; to irq2, which will enter with the jitter removed.
irq1
                lda #<irq2
                sta $0314
                lda #>irq2
                sta $0315
                lda #VSP_LINE
                sta $d012
                lda $d019
                sta $d019
                cli
                ; Burn the rest of this line inside a predictable sled so irq2
                ; is taken from a known instruction boundary.
                .rept 20
                nop
                .endrept
                jmp *

; ---------------------------------------------------------------------------
; Second interrupt: cycle-stable entry on VSP_LINE. Advance to the badline
; decision window, then suppress the badline by moving YSCROLL out from under
; it. Everything below the split shifts.
irq2
                .rept VSP_DELAY
                nop
                .endrept
                lda #VSP_YSCROLL_OFF
                sta $d011               ; <-- the write whose cycle is the test

                ; Restore for the following frame so the split is produced once
                ; per frame at the same place rather than drifting.
                lda #VSP_YSCROLL_ON
                sta $d011

                lda #<irq1
                sta $0314
                lda #>irq1
                sta $0315
                lda #VSP_LINE-1
                sta $d012
                lda $d019
                sta $d019
                jmp $ea31
