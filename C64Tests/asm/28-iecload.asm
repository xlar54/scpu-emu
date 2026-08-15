; 28-iecload.asm - is the load CORRECT, not just did it finish
;
; Every IEC failure so far has been described rather than measured: "@$ shows
; BASIC keywords at the end of the line", "GEOS warm-resets", "programs won't
; fully load". Those are impressions. This turns the question into a number.
;
; It loads a known file repeatedly and CRCs every byte. A corrupt load, a short
; load and a clean load are then three different results instead of three
; descriptions, and a fault that shows up one time in twenty is countable rather
; than anecdotal -- which is the part that has been missing, because the
; failures are intermittent and nobody can say whether a change made things
; better, worse or neither.
;
; Needs hardware: this is the one thing on the list a host test cannot answer,
; because the whole question is what happens on the physical bus.
;
; It loads TESTDATA, which must be on the disk alongside this program. Any file
; works as long as its CRC is known; c1541 can write one and the expected value
; goes in `wantcrc` below. The default expects the file produced by
;
;   Tools/mkiecdata.sh                (writes TESTDATA and prints its CRC)
;
; Screen output, one line per pass:
;
;   PASS nnn  CRC xxxx  LEN xxxx  OK          bytes and checksum both right
;   PASS nnn  CRC xxxx  LEN xxxx  BAD CRC     right length, wrong content
;   PASS nnn  CRC xxxx  LEN xxxx  SHORT       load ended early
;   PASS nnn  ERROR nn                        the KERNAL refused
;
; and a running total of failures, which is the number that matters. Leave it
; running: an intermittent fault becomes a rate.

                * = $0801

; BASIC stub: 10 SYS 2061
                .word eob
                .word 10
                .byte $9e
                .text "2061"
                .byte 0
eob             .word 0

LOADADDR        = $4000                 ; where TESTDATA lands
wantcrc         = $cca3               ; CRC-16/XMODEM of the reference file
wantlen         = $1000                 ; its length in bytes

start
                lda #$00
                sta passlo
                sta passhi
                sta faillo
                sta failhi

                lda #147                ; clear screen
                jsr $ffd2

mainloop
                ; --- clear the target so a SHORT load cannot be mistaken for
                ; a good one by leaving the previous pass's data behind.
                lda #$00
                tay
clr             sta LOADADDR,y
                sta LOADADDR+$100,y
                sta LOADADDR+$200,y
                sta LOADADDR+$300,y
                sta LOADADDR+$400,y
                sta LOADADDR+$500,y
                sta LOADADDR+$600,y
                sta LOADADDR+$700,y
                sta LOADADDR+$800,y
                sta LOADADDR+$900,y
                sta LOADADDR+$a00,y
                sta LOADADDR+$b00,y
                sta LOADADDR+$c00,y
                sta LOADADDR+$d00,y
                sta LOADADDR+$e00,y
                sta LOADADDR+$f00,y
                iny
                bne clr

                ; --- SETLFS 1,8,0 : secondary 0 means "load where I say",
                ; which keeps the address under our control rather than the
                ; file's, so a corrupt header cannot scatter the data.
                lda #$01
                ldx #$08
                ldy #$00
                jsr $ffba               ; SETLFS

                lda #namelen
                ldx #<fname
                ldy #>fname
                jsr $ffbd               ; SETNAM

                lda #$00                ; 0 = load (not verify)
                ldx #<LOADADDR
                ldy #>LOADADDR
                jsr $ffd5               ; LOAD
                bcs loaderror

                ; X/Y now hold the address one past the last byte loaded.
                stx endlo
                sty endhi

                ; --- length ----------------------------------------------
                sec
                lda endlo
                sbc #<LOADADDR
                sta lenlo
                lda endhi
                sbc #>LOADADDR
                sta lenhi

                ; --- CRC-16/XMODEM over exactly the bytes that arrived ---
                jsr crc

                ; --- verdict ---------------------------------------------
                lda lenlo
                cmp #<wantlen
                bne short
                lda lenhi
                cmp #>wantlen
                bne short

                lda crclo
                cmp #<wantcrc
                bne badcrc
                lda crchi
                cmp #>wantcrc
                bne badcrc

                ldx #<okmsg
                ldy #>okmsg
                jsr verdict
                jmp nextpass

short           ldx #<shortmsg
                ldy #>shortmsg
                jsr verdict
                jsr countfail
                jmp nextpass

badcrc          ldx #<badmsg
                ldy #>badmsg
                jsr verdict
                jsr countfail
                jmp nextpass

loaderror
                ; A is the KERNAL error code.
                pha
                jsr showpass
                lda #<errmsg
                sta $fb
                lda #>errmsg
                sta $fc
                jsr puts
                pla
                jsr hexbyte
                lda #13
                jsr $ffd2
                jsr countfail

nextpass
                inc passlo
                bne nextp1
                inc passhi
nextp1          jmp mainloop

; ---------------------------------------------------------------------------
; CRC-16/XMODEM: poly $1021, init $0000, no reflection. Chosen because it is
; short, catches every single-bit and most burst errors, and -- unlike a plain
; sum -- notices when two bytes swap places, which is exactly what a mistimed
; serial transfer produces.
crc
                lda #$00
                sta crclo
                sta crchi
                lda #<LOADADDR
                sta $fd
                lda #>LOADADDR
                sta $fe
                lda lenlo
                sta cntlo
                lda lenhi
                sta cnthi
                ldy #$00
crcloop
                lda cntlo
                ora cnthi
                beq crcdone
                lda ($fd),y
                eor crchi               ; XMODEM feeds the byte into the HIGH half
                sta crchi
                ldx #$08
crcbit          asl crclo
                rol crchi
                bcc crcnext
                lda crchi
                eor #$10
                sta crchi
                lda crclo
                eor #$21
                sta crclo
crcnext         dex
                bne crcbit

                inc $fd
                bne crcnc
                inc $fe
crcnc
                lda cntlo
                bne crcdec
                dec cnthi
crcdec          dec cntlo
                jmp crcloop
crcdone         rts

; ---------------------------------------------------------------------------
; "PASS nnnn  CRC xxxx  LEN xxxx  " then the verdict in X/Y, then newline.
verdict
                stx $fb
                sty $fc
                jsr showpass

                lda #<crcmsg
                sta $fd
                lda #>crcmsg
                sta $fe
                jsr puts2
                lda crchi
                jsr hexbyte
                lda crclo
                jsr hexbyte

                lda #<lenmsg
                sta $fd
                lda #>lenmsg
                sta $fe
                jsr puts2
                lda lenhi
                jsr hexbyte
                lda lenlo
                jsr hexbyte

                lda #32
                jsr $ffd2
                jsr puts                ; the verdict, via $fb/$fc
                lda #13
                jmp $ffd2

showpass
                lda #<passmsg
                sta $fd
                lda #>passmsg
                sta $fe
                jsr puts2
                lda passhi
                jsr hexbyte
                lda passlo
                jmp hexbyte

countfail
                inc faillo
                bne fcdone
                inc failhi
fcdone
                ; Keep the running failure total in the top-left corner, so it
                ; is readable at a glance across a long unattended run.
                lda failhi
                jsr cornerhex
                lda faillo
                ldx #$02
                stx corner
                jsr cornerhex
                lda #$00
                sta corner
                rts

; Write A as two hex digits directly into screen memory at `corner`.
cornerhex
                pha
                lsr a
                lsr a
                lsr a
                lsr a
                jsr cornernyb
                pla
                and #$0f
cornernyb
                cmp #$0a
                bcc cn1
                adc #$06                ; carry set: +7 total, 'A'..'F'
cn1             adc #$30
                and #$3f                ; ASCII -> screen code
                ldx corner
                sta $0400,x
                inc corner
                rts

corner          .byte 0

; A as two hex digits through CHROUT.
hexbyte
                pha
                lsr a
                lsr a
                lsr a
                lsr a
                jsr hexnyb
                pla
                and #$0f
hexnyb
                cmp #$0a
                bcc hn1
                adc #$06
hn1             adc #$30
                jmp $ffd2

; Print the zero-terminated string at $fb / at $fd.
puts            ldy #$00
puts1           lda ($fb),y
                beq putsdone
                jsr $ffd2
                iny
                bne puts1
putsdone        rts

puts2           ldy #$00
puts2a          lda ($fd),y
                beq puts2done
                jsr $ffd2
                iny
                bne puts2a
puts2done       rts

; ---------------------------------------------------------------------------
fname           .text "testdata"
namelen         = 8

passmsg         .text "pass "
                .byte 0
crcmsg          .text "  crc "
                .byte 0
lenmsg          .text "  len "
                .byte 0
okmsg           .text "ok"
                .byte 0
badmsg          .text "bad crc"
                .byte 0
shortmsg        .text "short"
                .byte 0
errmsg          .text "  error "
                .byte 0

passlo          .byte 0
passhi          .byte 0
faillo          .byte 0
failhi          .byte 0
endlo           .byte 0
endhi           .byte 0
lenlo           .byte 0
lenhi           .byte 0
crclo           .byte 0
crchi           .byte 0
cntlo           .byte 0
cnthi           .byte 0
