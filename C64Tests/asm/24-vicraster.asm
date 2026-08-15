; 24-vicraster.asm - raster split conformance
;
; Four horizontal bands produced by raster interrupts, each changing both the
; border and the background so a misplaced band is unmistakable and a swapped
; register is too. The whole point is to test band RECONSTRUCTION: given
; "register R became V at raster line L", does our replay put the change on the
; same output row VICE does?
;
; The handler writes $D020 and $D021 as its first two actions, so interrupt
; latency is a handful of cycles into the target line rather than a line late.
; Any residual difference should therefore be confined to the single transition
; row, which is a known and separate question from where the band starts.
;
; Splits at raster 60, 120, 180 and 240. Halts at `halt` with interrupts live,
; so the display keeps regenerating while VICE screenshots it.

                * = $0801

; BASIC stub: 10 SYS 2061
                .word eob
                .word 10
                .byte $9e
                .text "2061"
                .byte 0
eob             .word 0

start
                sei
                lda $dd02
                ora #$03
                sta $dd02
                lda $dd00
                ora #$03
                sta $dd00               ; VIC bank 0

                lda #$1b
                sta $d011               ; DEN, RSEL, YSCROLL=3, raster MSB 0
                lda #$c8
                sta $d016
                lda #$15                ; screen $0400, character ROM
                sta $d018

                ; Fill the screen so each band carries visible foreground and a
                ; band placed one row out shows in the glyphs as well as the
                ; background.
                ldx #$00
fill            lda #$a0                ; solid block character
                sta $0400,x
                sta $0500,x
                sta $0600,x
                sta $06e8,x
                txa
                and #$0f
                sta $d800,x
                sta $d900,x
                sta $da00,x
                sta $dae8,x
                inx
                bne fill

                lda #$00
                sta $d015               ; no sprites: bands only

                ; Point IRQ at our handler, VIC as the only source.
                lda #<irq
                sta $0314
                lda #>irq
                sta $0315
                lda #$7f
                sta $dc0d               ; no CIA interrupts
                lda $dc0d
                lda #$01
                sta $d01a               ; enable raster interrupt
                lda #$00
                sta band
                lda #60
                sta $d012
                lda $d011
                and #$7f
                sta $d011               ; compare line 60, MSB clear
                asl $d019               ; acknowledge any pending
                cli

halt            jmp halt

; ---------------------------------------------------------------------------
irq
                ; Colours first, so the change lands as early in the line as
                ; interrupt latency allows.
                ldx band
                lda borders,x
                sta $d020
                lda backs,x
                sta $d021

                inx
                cpx #4
                bne setnext
                ldx #$00
setnext         stx band
                lda lines,x
                sta $d012

                asl $d019               ; acknowledge
                jmp $ea81               ; plain RTI path

band            .byte 0
lines           .byte 60, 120, 180, 240
borders         .byte 2, 5, 7, 11
backs           .byte 0, 6, 11, 12
