; 25-sidtrace.asm - every SID write must reach the chip, in order, unmerged
;
; The audio faults in VIDEO_MODE 1 have no bisection: nobody knows whether the
; wrong sound is a wrong write, a missing write, or a write that arrived at the
; wrong time on the bus. This program supplies the upstream half of that split.
;
; It writes a deterministic sequence to the SID and records what it INTENDED to
; write in ordinary RAM as it goes. Two comparisons then become possible:
;
;   VICE's recorded buffer vs ours   -- did our CPU and memory map execute the
;                                       program the way the reference did?
;   our recorded buffer vs the bus   -- did every intended write actually reach
;                                       the chip, once, in order, unaltered?
;
; If both pass, everything upstream of the bus is exonerated and the fault is in
; delivery: timing, the read/write eye, or the mirror path. If either fails, the
; failing one names the layer.
;
; The sequence is chosen to catch the specific ways a write can be lost:
;
;   * all 25 registers, so none is misrouted or masked off
;   * the SAME value written twice in a row, which a buffer that "drops writes
;     that change nothing" would merge. Legal for DRAM. Fatal for a SID, where
;     re-writing a control register is how a gate is retriggered
;   * a gate off/on pair, the real-world form of the above
;   * $D41D-$D41F, which are unused and must still not disturb anything
;
; Buffer at $C100: two bytes per write, register offset then value.
; Count at $C000/$C001 (little endian). Signature $5A/$A5 at $C002/$C003.

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
                lda #$00
                sta count
                sta index

                ; --- every register once, with a value derived from its own
                ; offset so a misrouted write is visible in the value too.
                ldx #$00
sweep           txa
                eor #$5a
                jsr sidwrite
                inx
                cpx #$19                ; $00..$18, all 25
                bne sweep

                ; --- the unused three. They must reach the bus and change
                ; nothing observable.
                ldx #$1d
                lda #$11
                jsr sidwrite
                ldx #$1e
                lda #$22
                jsr sidwrite
                ldx #$1f
                lda #$33
                jsr sidwrite

                ; --- the same value twice. A merge here is a dropped write.
                ldx #$04
                lda #$21
                jsr sidwrite
                lda #$21
                jsr sidwrite
                lda #$21
                jsr sidwrite

                ; --- a real gate retrigger: off, then on, then on again.
                ldx #$04
                lda #$20
                jsr sidwrite
                lda #$21
                jsr sidwrite
                lda #$21
                jsr sidwrite

                ; --- a full voice set-up, the ordinary case, so the test is not
                ; made entirely of edge cases.
                ldx #$00
                lda #$e0
                jsr sidwrite
                ldx #$01
                lda #$11
                jsr sidwrite
                ldx #$05
                lda #$09
                jsr sidwrite
                ldx #$06
                lda #$00
                jsr sidwrite
                ldx #$18
                lda #$0f
                jsr sidwrite
                ldx #$04
                lda #$11
                jsr sidwrite

                ; Publish the count where the harness looks for it, then the
                ; signature -- in that order, so a reader that sees the
                ; signature is guaranteed to see a finished count.
                lda count
                sta $c000
                lda #$00
                sta $c001
                lda #$5a
                sta $c002
                lda #$a5
                sta $c003

halt            jmp halt

; ---------------------------------------------------------------------------
; Write A to SID register X, and record the pair. X and A are preserved so the
; caller can keep using them.
sidwrite
                sta scratch
                stx scratchx
                sta $d400,x

                ; Record: register offset, then value. Two bytes per write and
                ; well under a hundred writes, so one page and an 8-bit index
                ; are enough -- and an 8-bit index cannot address more anyway.
                txa
                ldx index
                sta $c100,x
                inx
                lda scratch
                sta $c100,x
                inx
                stx index
                inc count
                ldx scratchx
                lda scratch
                rts

count           .byte 0
index           .byte 0
scratch         .byte 0
scratchx        .byte 0
