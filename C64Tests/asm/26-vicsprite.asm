; 26-vicsprite.asm - sprite X MSB, expansion and priority
;
; Three things no capture has touched, in one frame.
;
; The X MSB is the one worth being nervous about. Sprite X is NINE bits: the low
; eight in $D000+2n, the ninth for all eight sprites gathered in $D010. Every
; capture so far has written $D010 = 0, so the entire high half of the
; coordinate space -- everything past X=255, which is most of the right of the
; screen -- has never been rendered once. Games live there.
;
; Layout, and what each one is actually worth. Verified against VICE, so the
; observed extents below are the reference's, not a hope:
;
;   sprite 0  X=248  MSB clear, just below the boundary   -> 24 wide
;   sprite 1  X=256  MSB set, low byte ZERO. The case that separates a renderer
;                    which reads $D010 from one that does not: ignore the ninth
;                    bit and this lands at the far LEFT instead of past the
;                    middle. Partly hidden by sprite 0, so 8 wide
;   sprite 2  X=300  MSB set, ordinary                    -> 24 wide
;   sprite 3  X=340  MSB set, at the display's right edge -> clipped to 4 wide
;   sprite 4  X=200  X-expanded                           -> 48 wide
;   sprite 5  X=200  Y-expanded, and deliberately behind sprite 4
;   sprite 6  X=300  BOTH expanded WITH the MSB set       -> 44 wide (clipped
;                    from 48 by the right edge), 42 tall. This is the one that
;                    proves expansion and the ninth bit work together
;   sprite 7  X=120  clear of everything, a plain reference block
;
; Priority falls out of the same picture rather than needing its own scenario.
; Sprites 0 and 1 share rows and overlap in columns, as do 4 and 5, and in both
; pairs the LOWER-numbered sprite must be in front -- which is exactly why 1 is
; 8 wide instead of 24, and why 5 shows only its lower half. That ordering is a
; hardware rule, not a convention, and nothing had checked it.
;
; Distinct colours per sprite so a misplaced one is attributable by colour alone.

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
                sta $d011
                lda #$c8
                sta $d016
                lda #$15                ; screen $0400, character ROM
                sta $d018
                lda #$00
                sta $d021               ; black background
                lda #$06
                sta $d020

                ; Blank screen: sprites are the entire subject here.
                ldx #$00
clear           lda #$20
                sta $0400,x
                sta $0500,x
                sta $0600,x
                sta $06e8,x
                lda #$01
                sta $d800,x
                sta $d900,x
                sta $da00,x
                sta $dae8,x
                inx
                bne clear

                ; Sprite shape at $0340 (pointer 13): a solid 24x21 block, so
                ; every edge is a hard boundary and a one-pixel placement error
                ; is visible.
                ldx #$00
shape           lda #$ff
                sta $0340,x
                inx
                cpx #63
                bne shape
                ldx #$00
ptrs            lda #13
                sta $07f8,x
                inx
                cpx #8
                bne ptrs

                ; --- positions -------------------------------------------
                ldx #$00
                ldy #$00
poslp           lda xlow,x
                sta $d000,y
                lda ytab,x
                sta $d001,y
                iny
                iny
                inx
                cpx #8
                bne poslp

                lda #$4e                ; MSB for sprites 1,2,3,6
                sta $d010

                lda #$50                ; X expansion: sprites 4 and 6
                sta $d01d
                lda #$60                ; Y expansion: sprites 5 and 6
                sta $d017

                lda #$00
                sta $d01c               ; none multicolour
                sta $d01b               ; all in front of the background

                ldx #$00
collp           lda coltab,x
                sta $d027,x
                inx
                cpx #8
                bne collp

                lda #$ff
                sta $d015               ; all eight enabled

                ; One settled frame, then stop with the picture standing.
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

; Low eight bits of X. Sprite 1 is the interesting one: X=256 has a low byte of
; zero, so a renderer that ignores $D010 draws it at the far left.
xlow            .byte 248, 0, 44, 84, 200, 200, 44, 120
;                     0    1  2   3   4    5    6   7
; MSB set for 1 ($100+0=256), 2 ($100+44=300), 3 ($100+84=340), 6 (=300)
; -> bits 1,2,3,6 = %01001110 = $4E

ytab            .byte 60, 60, 60, 60, 120, 120, 180, 130
coltab          .byte 1, 2, 3, 4, 5, 7, 10, 13
