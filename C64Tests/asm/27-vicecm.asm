; 27-vicecm.asm - extended background colour mode, and the invalid combinations
;
; ECM is the last valid mode with no coverage. It steals the top two bits of
; every screen code to pick one of four background registers, which leaves only
; 64 usable glyphs and makes $D022-$D024 live for the first time. A renderer
; that ignores those two bits draws the wrong CHARACTER as well as the wrong
; background, so a mistake here is loud -- but only if something looks.
;
; The invalid combinations matter just as much and are easier to get wrong.
; ECM+BMM, ECM+MCM and all three together are not defined modes: the VIC outputs
; BLACK across the whole display window, while the border is unaffected. Real
; software hits this, usually for one raster line during a mode switch, and a
; renderer that instead draws "whatever the nearest valid mode would have" puts
; a bright band where hardware puts nothing.
;
; One frame cannot show four modes, so the mode is chosen at assembly time.
;
; Assemble one variant per mode:
;
;   64tass -D MODE=0 ... 27-vicecm.asm     ECM
;   64tass -D MODE=1 ...                   ECM + BMM       (invalid -> black)
;   64tass -D MODE=2 ...                   ECM + MCM       (invalid -> black)
;   64tass -D MODE=3 ...                   ECM + BMM + MCM (invalid -> black)
;
; The screen is filled with codes that span all four background selectors and
; several glyphs within each, so every one of $D021-$D024 is exercised and the
; 64-glyph masking is visible.

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

                lda #$15                ; screen $0400, character ROM
                sta $d018
                lda #$0e
                sta $d020

                ; Four distinct backgrounds, so which selector was used is
                ; readable straight off the picture.
                lda #$00
                sta $d021               ; selector 0: black
                lda #$02
                sta $d022               ; selector 1: red
                lda #$05
                sta $d023               ; selector 2: green
                lda #$06
                sta $d024               ; selector 3: blue

                ; Fill: code = (cell & 63) | (selector << 6), cycling the
                ; selector every 16 cells. That covers all four selectors many
                ; times over and walks through 64 glyphs, which is exactly the
                ; range ECM leaves addressable.
                ldx #$00
fill            txa
                and #$3f
                sta scratch
                txa
                lsr a
                lsr a
                lsr a
                lsr a
                and #$03
                asl a
                asl a
                asl a
                asl a
                asl a
                asl a
                ora scratch
                sta $0400,x
                sta $0500,x
                sta $0600,x
                sta $06e8,x
                txa
                and #$0f
                ora #$01                ; never 0: black on black hides errors
                sta $d800,x
                sta $d900,x
                sta $da00,x
                sta $dae8,x
                inx
                bne fill

                lda #$00
                sta $d015               ; no sprites: modes only

                ; --- the mode under test ---------------------------------
                ; $D011: DEN + RSEL + YSCROLL 3 = $1B, plus ECM ($40), plus BMM
                ; ($20) when asked. $D016: 40 columns = $C8, plus MCM ($10).
                ; Selected at assembly time so each variant is a distinct,
                ; self-contained program with no memory to patch.
                lda #$1b | $40
.if MODE == 1 || MODE == 3
                ora #$20                ; + BMM  -> invalid
.endif
                sta $d011

                lda #$c8
.if MODE == 2 || MODE == 3
                ora #$10                ; + MCM  -> invalid
.endif
                sta $d016

                ; A bitmap that is never legitimately displayed still has to
                ; hold something recognisable, so an invalid mode that wrongly
                ; falls through to bitmap shows obvious garbage rather than
                ; accidentally-black memory.
                ldx #$00
bmp             lda #$db
                sta $2000,x
                sta $2100,x
                sta $2200,x
                sta $2300,x
                inx
                bne bmp

                jsr waitframe
                jsr waitframe

halt            jmp halt

waitframe
                lda #251
wf1             cmp $d012
                bne wf1
wf2             cmp $d012
                beq wf2
                rts

scratch         .byte 0
