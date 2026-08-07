# WDC 65C816 — implementation reference

This is the document `Source/CPU/W65C816/w65c816.cpp` is written against.
Everything in it is either measured, quoted from a primary source, or explicitly
flagged as unresolved. Where sources disagree the disagreement is written down
rather than averaged away, because the places sources disagree are exactly the
places a core is silently wrong for a year.

Five bodies of evidence are used throughout, and they are **not** of equal
weight:

| | Source | Weight |
|---|---|---|
| **S1** | WDC **W65C816S datasheet**, rev. 13 Mar 2024, full text | normative, but contains known errors |
| **S2** | Bruce Clark, **"65C816 Opcodes"**, 6502.org | the most precise public statement of behaviour; the opcode table here is machine-extracted from it |
| **S3** | **VICE `_vice/65816core.c`** (the SCPU64 core) | not normative, but it is the only 65816 implementation that has been run against real SuperCPU software for twenty years |
| **S4** | **SingleStepTests/65816** — 20 000 tests/opcode with per-cycle bus activity | **emulator-generated, not silicon** (see §1.4). Strong corroboration, never proof |
| **S5** | Eyes & Lichty, *Programming the 65816* (1986) | normative-historical |

---

## 1. What is verified, and what is not

### 1.1 The opcode map is settled

All 256 encodings are identified. Two independently derived tables agreed on 255
of 256; the remaining one, `$D4`, was researched separately and resolved to
**`PEI (dp)` — Stack (Direct Page Indirect), 2 bytes, not `PEI zp`**. The operand
byte is *dereferenced*: the 16-bit word at `00:(D+dp)` is pushed. The "zp" label
came from carrying the NMOS slot's operand shape (`$D4` is an undocumented
`NOP zp,X`) across into the 65816 column. S1 Table 5-5 writes the operand as
`(d)`, S2 and the SuperFamicom wiki both say "DP Indirect", and undisbeliever's
pseudocode settles it:

```
S <- S-2 ; [S+2] <- [0:D+dp+1] ; [S+1] <- [0:D+dp]
```

As a cross-check the table in §9 was **machine-extracted from S2** and compared
against `Source/CPU/M6502/m6502_opcodes.cpp` entry by entry: 256/256 parsed,
105 encodings undocumented on NMOS, 12 of those JAM. No encoding is unresolved
in *identity*.

### 1.2 Verified by measurement

Each of these was checked against S4 with a model written from the documentation;
the count is the number of tests that agreed, and in every case the number that
disagreed was zero.

| Behaviour | Evidence | Agreed |
|---|---|---|
| Decimal ADC/SBC, 8-bit **and** 16-bit: result, C, Z, N, V | `69.n 69.e e9.n e9.e` | 40 000 / 40 000 |
| Decimal mode costs **no** extra cycle | same, cycle counts by D flag | 40 000 |
| RMW is a dummy **write** when E=1, a dummy **read** when E=0 | `ee.e ee.n 04.e 04.n 1c.n` | 50 000 |
| 16-bit RMW reads lo→hi and writes **hi→lo** | `ee.n`, M=0 subset | 4 971 |
| MVN/MVP object-code operand order, banks, X/Y offsets, DBR:=dest, C-=1, 7 cycles/byte, no flags | `54.n 44.n`, 3 000 tests × 14 iterations each | 42 000 iterations |
| PEI: reads `00:(D+dp)` and `00:(D+dp+1)`, pushes 2 bytes regardless of M, `S-=2`, no flags, +1 cycle if `DL≠0` | `d4.n` | 10 000 |
| XCE: exchanges C and E; entering emulation forces `SH=$01`, `XH=YH=$00`, preserves A **and** B, D, DBR, and all P bits but C | `fb.e fb.n` | 20 000 |
| Emulation mode holds P bits 4 and 5 at 1, always | 12 emulation opcode files | 120 000 |
| `SH` is `$01` while E=1 and stays `$01` after `XCE` into native | `fb.e`, `48.e` | 10 000 |
| XBA is 3 cycles and sets N/Z from the **8-bit** new low byte even when M=0 | `eb.n` (4 981 of them M=0) | 10 000 |
| TCS affects **no** flags; TSC/TCD/TDC set N/Z from all 16 bits | `1b.n 3b.n` | 20 000 |
| `BIT #` affects **Z only** | `89.n` | 10 000 |
| BRK native: 4 pushes (PBR, PCH, PCL, P), P pushed verbatim, PC+2, vector `$00FFE6`, PBR:=0, I set, **D cleared** | `00.n` | 10 000 |
| BRK emulation: 3 pushes, P pushed with bits 4+5 set, stays in page 1, vector `$00FFFE`, **D cleared** | `00.e` | 10 000 |
| BRL and JML: cycles, PC/PBR effects, no push, no flags | `82.n 5c.n` | 20 000 |
| `JMP (abs)`: the 6502 `$xxFF` page-wrap bug is **fixed**, pointer always from bank 0 | `6C.e` distinguishing cases | 36 / 36 |
| WAI and STP are 3 cycles then halt | `cb.n db.n` | 20 000 |
| Direct page confined to bank 0; 8-bit page wrap only when E=1 **and** DL=$00 | `b5.n B5.e a1.e` | 20 000+ |
| Emulation stack: "old" ops stay in page 1, "new" ops spill out | `48.e 20.e f4.e 2b.e 22.e 6b.e` | 60 000 |
| `a,X`/`a,Y`/`(d),Y` data carry into DBR+1 in **both** modes | `BD.e b1.e` | 32 distinguishing cases |

One further result, computed rather than measured, matters more than any single
line above:

> **Every one of the 151 documented 6502 opcodes has the same length and the same
> base cycle count on a 65816 in emulation mode with `D=$0000` as it has in
> `m6502Opcodes[]`.** Evaluating S2's cycle formulas at `e=1, m=1, x=1, w=0`
> reproduces our table exactly — 0 differences out of 151. The page-cross and
> branch penalties match too: `6-m-x+x*p` becomes `4+p`, `2+t+t*e*p` becomes
> `2+t+t*p`.

So the differential test is a fair test. Where the two cores may legitimately
differ is behavioural, not temporal, and §10 enumerates exactly where.

### 1.3 Unresolved — read this section before writing code

**U1. Does the direct-page *pointer* page-wrap for `(d)`, `(d,X)` and `(d),Y` in
emulation mode with `DL=$00`? Sources disagree.** This is the one that will bite,
because it is reachable from ordinary C64 code (`D=$0000` always) and it is a
one-line difference nobody will notice.

The question is concrete: with `E=1`, `D=$0000`, does `LDA ($FF),Y` read the
pointer's high byte from `$0000` (wrap) or `$0100` (no wrap)?

*Page-wrap*, said by three sources:

- S1 §8.2.1: *"When in the Emulation mode, the direct addressing range is 000000
  to 0000FF, **except for [Direct] and [Direct],Y addressing modes and the PEI
  instruction** which will increment from 0000FE or 0000FF into the Stack area."*
  `(d)`, `(d,X)` and `(d),Y` are not in the exception list.
- S2 §5.11, verbatim diagram — the `+1` sits *inside* the 8-bit-truncated box:
  *"When the e flag is 1 and the DL register is $00 (both conditions must be met),
  the address of the pointer is `0 : DH : $LL+X` (pointer lo), `0 : DH : $LL+X+1`
  (pointer hi)."*
- S3 `INDIRECT_X_FUNC`, which special-cases it deliberately:
  ```c
  } else {                                        /* reg_emul && DL == 0 */
      ea2 = ((p1 + reg_x) & 0xff) + reg_dpr;      /* pointer lo */
      ea2 = ((p1 + reg_x + 1) & 0xff) + reg_dpr;  /* pointer hi -- wraps in page */
  ```
  and `INDIRECT_Y_FUNC`, which gates on `p1 != 0xff` for the same reason.

*No wrap*, said by S4. Scanning 924 emulation-mode tests that had `DL=$00` across
24 `(d)`-family opcode files produced exactly one distinguishing case, and it went
the other way — `e1.e`, `SBC ($B0,X)`, `D=$F400`, `X=$4F`, so `dp+X = $FF`:

```
0xba1103 E1  dp-remx-     opcode
0xba1104 B0  -p-remx-     operand
0xba1104     ---remx-     (dp+X internal cycle)
0x00f4ff 2F  d--remx-     pointer lo
0x00f500 3E  d--remx-     pointer hi   <-- $F500, not $F400
0x663e2f B3  d--remx-     data, DBR=$66
```

That is not a stray sample: the test's own RAM setup places the pointer bytes at
`$F4FF`/`$F500`, so the model that *generated* the suite does not wrap here.

**Recommendation: implement the page wrap.** Three sources including WDC's own
exception list beat one unattributed generator, and it is also the reading that
keeps `EA_IZX`/`EA_IZY` in `m6502.cpp:149-153` correct for the differential test.
Put it behind a named constant so it can be flipped in one place, and write the
unit test now so the decision is visible when somebody finally gets a SuperCPU on
a logic analyser.

**U2. Does the SuperCPU's gate array forward the emulation-mode RMW dummy write
to the C64 bus?** The chip drives it (§6), but with VDA and VPA both low. A bus
interface that qualifies on VDA — which is what VDA is *for*, and what any design
that must not waste 1µs on internal cycles would do — would drop it, and
`INC $D019` would stop acknowledging VIC-II interrupts. S1 §8.5 says the address
is valid anyway during writes, and S3 forwards it. The verdict in §6 assumes it
is forwarded; that assumption is testable on hardware in about five minutes and
should be tested.

**U3. The reset value of `SL`.** S1 §2.25 marks it "not initialized"; real silicon
gives `previous_SL - 3`. `$01FD` is a convention chosen to match `CM6502`, not a
hardware fact.

**U4. ABORT is unreachable and untested.** `ABORTB` is tied inactive on every real
65816 system including the SuperCPU. Define the vectors, never fire them, and do
not spend time on the abort semantics in S1 §8.4.

### 1.4 A calibration note on SingleStepTests

Its README does not state provenance, and the project's own description of the
65816 suite is that tests were *"randomly generated in substantial volume, using
an implementation that conforms to all available documentation, official and
third-party"*. It is a careful reimplementation, not a hardware capture — unlike
the 8086/80286 suites in the same organisation, which say "hardware-generated" in
their titles. Treat agreement with it as strong corroboration and disagreement as
a question to escalate, which is exactly what happened in U1.

---

## 2. Address wrapping

### 2.1 The master rule

There are exactly **three** address-generation classes, and every addressing mode
belongs to one. S2 §5.1.2, verbatim:

> Bank boundary wrapping occurs in both native and emulation mode (and does not
> depend on which mode the 65C816 is in). The following are confined to bank 0
> ("confined to" means they address bank 0 and wrap at the bank 0 boundary):
> A. The direct page  B. The stack  C. `[absolute]` and `(absolute)` addressing
> modes. The following are confined to bank K: A. `(absolute,X)` addressing mode
> B. The Program Counter … this means branches wrap at the bank K boundary.
> `source,destination` addressing (i.e. the MVN and MVP instructions) wraps at
> both the source and destination bank boundaries.
> **Otherwise, wrapping does not occur at bank boundaries.**

| Bank 0, 16-bit wrap | Bank K (PBR), 16-bit wrap | 24-bit, no wrap |
|---|---|---|
| `d`, `d,X`, `d,Y` | opcode and operand fetch | `a`, `a,X`, `a,Y` (bank = DBR) |
| the **pointer** of `(d)`, `(d,X)`, `(d),Y`, `[d]`, `[d],Y`, `(d,S),Y` | `r`/`rl` branch targets, PER | `al`, `al,X` (bank = operand byte 4) |
| `d,S`, and every stack push/pull | the **pointer** of `(a,X)` — JMP/JSR | the **data** of every indirect mode |
| the **pointer** of `(a)` and `[a]` — JMP/JML | | MVN/MVP `ss:X` and `dd:Y`, each in its own bank |

Bank registers: **PBR supplies the bank for instruction fetches only**, plus the
`(a,X)` pointer table. **DBR supplies the bank for data** in absolute, absolute
indexed, and the resolved target of `(d)`, `(d,X)`, `(d),Y`, `(d,S),Y`. Direct
page, stack, and long-pointer fetches ignore both and use bank 0.

**DBR and PBR are live in emulation mode.** `E=1` does not force them to zero.
This was confirmed on real silicon in the 6502.org forum and is visible throughout
S4's emulation tests, which run with `DBR` and `PBR` non-zero.

### 2.2 The checklist

Work through this per addressing mode while implementing. Each line is a yes/no
an implementer can check off.

1. **Direct page, all forms** — is the sum `D + dp (+ index)` truncated to 16
   bits and issued in bank 0? *Always, both modes.* S1 §7.2.1.
2. **Direct page, 8-bit sub-wrap** — is the wrap `0 : DH : (dp + index) & $FF`
   applied *only* when `E=1` **and** `DL=$00` **and** the mode is one a 65C02
   had? All three conditions. S2 §5.1.1.
3. **`[d]`, `[d],Y`, `PEI`** — are these excluded from the 8-bit sub-wrap even
   when `E=1, DL=$00`? *Yes, they run off the end of the page.* S1 §8.2.1.
4. **`(d)`, `(d,X)`, `(d),Y` pointer high byte** — see **U1**. Implement the wrap.
5. **Stack, native** — 16-bit S, bank 0, `S` wraps `$0000`↔`$FFFF`.
6. **Stack, emulation** — is `SH` forced to `$01` after *every* push, pull and
   `TCS`? And do the "new" instructions in §2.5 compute their addresses with a
   16-bit S before that forcing is applied?
7. **`d,S` and `(d,S),Y`** — never confined to page 1, in either mode. They are
   "new" modes.
8. **`a,X`, `a,Y`** — is the add a true 24-bit add against DBR, so the carry
   lands in the bank byte? *Yes, in both modes.* S1 §8.3.
9. **`(d),Y` data** — same: pointer + Y carries into DBR+1.
10. **`al`, `al,X`** — 24-bit, no wrap; `al,X` carries into the next bank. There
    is no `al,Y`.
11. **PC** — does `PC` wrap at `$FFFF` without touching PBR? *Yes*, including
    mid-instruction: an `INX` at `$12FFFF` is followed by the instruction at
    `$120000`. Branches, `BRL` and `PER` all wrap in bank K.
12. **`JMP (a)` / `JMP [a]`** — pointer read from **bank 0**, 16-bit increment,
    no `$xxFF` bug.
13. **`JMP (a,X)` / `JSR (a,X)`** — pointer read from **bank K (PBR)**.
14. **`RTL`** — `PC = pulled16 + 1` with **no carry into PBR**.
15. **MVN/MVP** — source and destination each wrap inside their own bank.

### 2.3 Direct page — worked examples

*Native, `D=$6524`, `X=$9A85`, `LDA $D1,X`.* Raw sum `$1_007A`, truncated to 16
bits → **EA `$00007A`**, not `$01007A`. Verified: 2 553 tests in `b5.n` had
`D+dp+X > $FFFF` and all stayed in bank 0.

*16-bit access at the top of bank 0.* S2 §5.7: with `D=$FF00` and `m=0`,
`LDA $FF` takes the low byte from `$00FFFF` and the high byte from `$000000`.

*Emulation, `D=$C700` (so `DL=$00`), `X=$B2`, `LDA $EA,X`.* `($EA+$B2)&$FF = $9C`
→ **EA `$00C79C`**. A plain 16-bit add would give `$00C89C`. Verified 21/21 in
`B5.e`, and 18/18 for `(d,X)` in `a1.e`.

*The same instruction with `DL≠0` switches the 8-bit wrap off entirely*, even in
emulation. S1 §8.2.3, and S2's worked case: `PEA #$3401 / PLD / SEC / XCE /
LDX #1 / LDA $FF,X` reads `$3501`, not `$3401`. Verified: 8 340 / 8 340
emulation-mode `DL≠0` cases in `B5.e` used the full 16-bit add.

`DL≠0` also costs **one extra cycle, in both modes** — the `w` term in every
cycle formula in §9. S1 cycle-table note: *"Direct register low (DL) not equal
zero, add 1 cycle."* Independently visible in `04.e`: 9 947 tests took 6 cycles
and the 53 with `DL=$00` took 5.

*The three exceptions.* S1 §8.2.1 and §8.2.2 name `[Direct]`, `[Direct],Y` and
`PEI` as running off the end of the page. With `E=1`, `D=$2F00`,
`LDA [$FE]` reads its pointer from `$002FFE`, `$002FFF`, **`$003000`**. Verified
in `27.e` — the single distinguishing case found across 80 000 `[d]`-family
emulation tests, and it matched the no-wrap model.

**On a C64 `D` is `$0000` and `E` is 1**, so every rule above collapses to plain
6502 zero-page behaviour and `EA_ZP/EA_ZPX/EA_ZPY/EA_IZX/EA_IZY` in
`Source/CPU/M6502/m6502.cpp:141-153` are already right — modulo **U1**.

### 2.4 Stack — native

Full 16-bit S, always bank 0, 16-bit wrap. Push writes at `$00:S` then
`S = (S-1) & $FFFF`; pull increments first, then reads.

*Native, `M=0`, `PHA` with `S=$0001`*: writes AH at `$000001`, AL at `$000000`,
leaves **`S=$FFFF`**. Verified in `48.n`; 0 of 40 000 native stack accesses across
`48.n/f4.n/20.n/68.n` left bank 0.

### 2.5 Stack — emulation, and the "old"/"new" split

`SH` is hardwired to `$01` (S1 §2.11). Writes to it are ignored; `TCS` transfers
only the low byte. S4 seeds `S` with arbitrary high bytes and the CPU behaves as
`$01:SL` every time — 10 000/10 000 for `LDA d,S`, and 100 % of stack opcodes end
with `SH=$01`.

The split, S1 §8.1 verbatim:

> In the Emulation mode, the Stack address range is 000100 to 0001FF. The
> following OpCodes and addressing modes will increment or decrement beyond this
> range when accessing two or three bytes: **JSL, JSR (a,x), PEA, PEI, PER, PHD,
> PLD, RTL**

| Instruction | class | `S` before | bus accesses | `S` after |
|---|---|---|---|---|
| `PHA` | old, 1 byte | `$0100` | `$000100` | `$01FF` |
| `JSR abs` | old, 2 bytes | `$0100` | `$000100`, **`$0001FF`** | `$01FE` |
| `PEA` | new, 2 bytes | `$0100` | `$000100`, **`$0000FF`** | `$01FE` |
| `JSL` | new, 3 bytes | `$0100` | `$000100`, `$0000FF`, **`$0000FE`** | `$01FD` |
| `PLD` | new, 2 bytes | `$01FE` | `$0001FF`, **`$000200`** | `$0100` |
| `RTL` | new, 3 bytes | `$01FF` | **`$000200`, `$000201`, `$000202`** | `$0102` |
| BRK/IRQ/NMI | interrupt, old | `$0100` | `$000100`, `$0001FF`, `$0001FE` | `$01FD` |

All measured: `48.e` 0/10 000 accesses outside page 1, `20.e` 0/10 000, `f4.e`
51, `2b.e` 99, `22.e` 75, `6b.e` 100. Note the last column — in **both** classes
only `SL` is finally decremented, so `S` always lands back inside page 1.

`d,S` and `(d,S),Y` are "new" modes and are never confined: 4 831 of 10 000
emulation `LDA d,S` accesses landed outside `$0100-$01FF`.

```
// E = 1
pushOld( v ) { wr( 0x000100 | ( m_S & 0xFF ), v ); m_S = 0x0100 | ( ( m_S - 1 ) & 0xFF ); }
pushNew( v ) { wr( m_S & 0xFFFF,              v ); m_S = ( m_S - 1 ) & 0xFFFF;             }
// then force m_S = 0x0100 | ( m_S & 0xFF ) at the end of the instruction either way.
// E = 0: one form, 16-bit, bank 0.
```

"Old" is every 65C02 stack operation — `PHA/PHP/PHX/PHY/PLA/PLP/PLX/PLY`,
`JSR abs`, `RTS`, `RTI`, `BRK` — **plus all hardware interrupts and COP**. "New"
is WDC's list above. `PHB`, `PHK` and `PLB` push one byte and cannot cross, so
their class does not matter.

### 2.6 Absolute indexed — the one place emulation mode is not a 6502

`EA = (DBR<<16) + abs + index`, a true 24-bit add, **in both modes**. S1 §8.3:

> The W65C02S addressing range is 0000 to FFFF. Indexing from page FFXX may
> result in a 00YY data fetch when using the W65C02S. In contrast, **indexing
> from page ZZFFXX may result in ZZ+1,00YY** when using the W65C816S.

*Emulation, `DBR=$7C`, `X=$93`, `LDA $FFA9,X`* → **`$7D003C`**. Verified 17/17
bank-crossing cases in `BD.e` carried into `DBR+1`, 0 wrapped. Same for the data
side of `(d),Y`: 15/15 in `b1.e`.

`EA_ABX_R/EA_ABX_W/EA_ABY_*/EA_IZY_*` in `m6502.cpp:145-153` compute
`(u16)(base + index)`. With the C64's `DBR=$00`, `LDA $FFFF,X` with `X=1` reads
`$000000` on the 6502 and `$010000` on the 65816. Exempt it from the oracle
rather than "fixing" either core.

Cycle interaction: `LDA a,X` is `6-m-x+x*p` — the page-cross penalty applies
**only when `x=1`**; with 16-bit index the extra cycle is unconditional. Stores
(`STA a,X` = `6-m`) always pay it.

### 2.7 Long addressing

`al` and `al,X` are 24-bit with a 24-bit add and no bank wrap. S1 §3.5.6: *"The
effective address is the sum of this 24-bit address and the X Index Register."*
With `X=$000A`, `M=0`, `LDA $12FFFE,X` reads the low byte from **`$130008`** and
the high byte from **`$130009`**. There is no `al,Y`.

---

## 3. Emulation mode, native mode, M and X

### 3.1 E is not a P bit

`m_E` as a separate `bool` is correct. E is changed **only by `XCE`** — not by
`PLP`, not by `RTI`, not by `REP`/`SEP`. RESET forces it to 1.

While `E=1` the hardware **continuously forces** four things:

- `SH = $01`
- `m = 1` (P bit 5)
- `x = 1` (P bit 4)
- `XH = $00`, `YH = $00`

S2: *"Attempting to change the value of the SH register, the XH register, the YH
register, the m flag, or the x flag when the e flag is 1 will have no effect."*
Measured: P bits 4 and 5 were `11` in the initial **and** final state of all
120 000 emulation-mode tests sampled.

Implement it as an invariant re-asserted after **every** write to P, S, X or Y —
not only at the `XCE` site:

```cpp
// Re-assert the emulation-mode invariants. Called after anything that can write
// P, S, X or Y, because in emulation mode the hardware holds these forced --
// writes to them simply do not take.
inline void applyE()
{
	if ( m_E )
	{
		m_P |= ( W65_M | W65_X );
		m_S  = (u16)( 0x0100 | ( m_S & 0x00FF ) );
		m_X &= 0x00FF;
		m_Y &= 0x00FF;
	}
}
```

### 3.2 XCE

`$FB`, implied, 2 cycles. Exchanges C and E and nothing else directly.

```cpp
bool tmp = m_E;
m_E = ( m_P & W65_C ) != 0;
setFlag( W65_C, tmp );
applyE();
```

Going **native → emulation**, S1 §7.10 verbatim:

> When switching from the Native mode to the Emulation mode, the X and M bits of
> the Status Register are set high (logic 1), the high byte of the Stack is set to
> 01, and the high bytes of the X and Y Index Registers are set to 00. To save
> previous values, these bytes must always be stored before changing modes. **Note
> that the low byte of the S, X and Y Registers and the low and high byte of the
> Accumulator (A and B) are not affected by a mode change.**

| Register | Effect |
|---|---|
| `S` | `SH := $01`, `SL` preserved. `$0234` → `$0134`. |
| `X`, `Y` | `XH := $00`, `YH := $00` — **destroyed, saved nowhere** |
| `C` | **A and B both preserved.** B survives and is still reachable via `XBA` |
| `P` | `m := 1`, `x := 1`; no other bit touched |
| `D`, `DBR`, `PBR` | **untouched** — a non-zero `D` or `DBR` survives into emulation mode |
| `PC` | untouched |

The A/B-versus-X/Y asymmetry is the single most-missed detail in this area:
**narrowing the accumulator never loses data; narrowing the index registers always
does.** All of it measured across 20 000 `fb.e`/`fb.n` tests.

Going **emulation → native** there are no side effects beyond E itself. `m` and
`x` stay 1 — they were being *held* at 1 and simply become writable. `SH` stays
`$01` (measured: 2 477 of 2 477 cases whose seeded `SH` was not `$01` came out as
`$01:SL`). This is why real code is `CLC : XCE : REP #$30 : LDA #$xxxx : TCS`.

`$FB` is an undocumented `ISC abs,Y` on NMOS; `CM6502` runs it as a 3-byte NOP.

### 3.3 The status register in each mode

| Bit | Mask | `E=1` | `E=0` |
|---|---|---|---|
| 7 | `$80` | N | N |
| 6 | `$40` | V | V |
| 5 | `$20` | reads and pushes 1 (m held 1) | **M** — 0 = 16-bit accumulator |
| 4 | `$10` | **B**, in the pushed byte only | **X** — 0 = 16-bit index registers |
| 3 | `$08` | D | D |
| 2 | `$04` | I | I |
| 1 | `$02` | Z | Z |
| 0 | `$01` | C | C |

There is no settable B *flag*. B is an internal signal that selects bit 4 of the
**stacked** P byte. S1 §2.8: *"When an interrupt occurs during Emulation mode, the
Break flag is written to stack memory as bit 4 of the Processor Status Register."*

- `BRK` / `COP` in emulation → pushed bit 4 = **1** (measured, `00.e`)
- `IRQ` / `NMI` in emulation → pushed bit 4 = **0**
- `PHP` in emulation → pushes bit 4 = **1**, because x is held at 1

That coincidence is what makes emulation mode agree with NMOS bit-for-bit.
`CM6502::serviceNMI`/`serviceIRQ` push `(m_P | M6502_U) & ~M6502_B`; the 65816
produces the same byte. No change needed there.

**In native mode there is no B bit at all.** Bit 4 is X and is pushed verbatim for
every interrupt including `BRK` — measured on 10 000 `00.n` tests. WDC Table 7-1:
*"X=X on stack always"*. `BRK` is distinguished from `IRQ` only by having its own
vector. A native-mode ISR must not test bit 4.

### 3.4 x transitioning 0 → 1

`XH` and `YH` are **immediately and irrecoverably forced to `$00`**. S1 §2.7: *"If
the Index Select Bit (X) equals one, both registers will be 8 bits wide, and the
high byte is forced to zero."* This applies on every path that can set x —
`SEP #$10`, `PLP`, native `RTI`, and `XCE` into emulation. It is not deferred:
`SEP #$10 : REP #$10 : TXA` yields `$00xx`. While x=1 the high bytes are *held* at
`$00`, so `LDX`, `INX`, `PLX`, `TAX` affect only the low byte.

Contrast: `m` going 0 → 1 does **not** touch B.

### 3.5 REP and SEP

`REP #imm` (`$C2`, 2 bytes, 3 cycles) does `P &= ~imm`; `SEP #imm` (`$E2`, 2 bytes,
3 cycles) does `P |= imm`. Operand bits are P bit positions; zero bits are left
alone. They can modify **any** P bit — `SEP #$04` is a legal, unidiomatic `SEI`.

In emulation mode they cannot touch m or x. S1 §8.7.3.1: *"The REP and SEP
instructions cannot modify the M and X bits when in the Emulation mode. In this
mode the M and X bits will always be high (logic 1)."* They still clear or set
every other bit named — `REP #$3F` in emulation clears C, Z, I and D and leaves
m/x at 1. Implement as: apply the mask, then `applyE()`. `PLP` behaves the same
way.

Because immediate operand widths follow M and X, `REP`/`SEP`/`PLP`/`RTI`/`XCE` are
the points at which a dispatcher templated on `<M,X>` must re-select its template.

`$C2` and `$E2` are undocumented `NOP #` encodings on NMOS — same length, so at
least these two do not desynchronise the stream when misdecoded.

### 3.6 RESET state

S1 §2.25:

| | Value |
|---|---|
| **E** | **1** — always resets into emulation mode |
| **M**, **X** | 1, forced and inaccessible |
| **D** (direct page register) | `$0000` |
| **DBR**, **PBR** | `$00` |
| **S** | `SH = $01`, `SL` indeterminate |
| **P** | `D=0`, `I=1`; N, V, Z indeterminate |
| A/B, XL, YL | indeterminate |
| PC | from `$00FFFC/D` |

`STP` and `WAI` are cleared. RWB stays high during the three stack cycles — they
are *reads*, so `S` drops by 3 and memory is untouched, exactly as on NMOS.

For differential testing, match `CM6502`'s convention (see **U3**):

```cpp
m_C = 0; m_X = m_Y = 0;
m_S = 0x01FD;                       // SH forced to $01; SL matches CM6502
m_D = 0; m_DBR = m_PBR = 0;
m_P = W65_M | W65_X | W65_I;        // no separate "unused" bit -- bit 5 IS M
m_E = true;
m_Stopped = m_Waiting = false;
m_PC = rd16( W65_VEC_EMU_RESET );
```

---

## 4. Interrupts

### 4.1 Vectors

All vectors are read from **bank `$00`** in both modes. The constants already in
`w65c816.h` were checked against S1 Tables 5-2 and 5-3 and are all correct,
including the deliberate absence of a native RESET vector and of an emulation BRK
constant.

| Emulation (E=1) | | Native (E=0) | |
|---|---|---|---|
| `$00FFF4` | COP | `$00FFE4` | COP |
| — | *(BRK shares IRQ)* | `$00FFE6` | **BRK** |
| `$00FFF8` | ABORT | `$00FFE8` | ABORT |
| `$00FFFA` | NMI | `$00FFEA` | NMI |
| `$00FFFC` | **RESET** | — | *(no native RESET)* |
| `$00FFFE` | IRQ **and BRK** | `$00FFEE` | IRQ |

RESET always forces `E=1` first and then uses `$00FFFC`, which is why there is no
native RESET vector to define.

### 4.2 What is pushed

S1 §7.11 verbatim: *"When in the Native mode, the Program Bank register (PBR) is
cleared to 00 when a hardware interrupt, BRK or COP is executed. In the Native
mode, previous PBR contents are automatically saved on Stack. […] In Emulation
Mode the PBR register is cleared to 00 […] previous contents of the PBR are not
automatically saved."*

| | `E=1` | `E=0` |
|---|---|---|
| Bytes pushed | **3**: PCH, PCL, P | **4**: PBR, PCH, PCL, P |
| Push order | PCH, PCL, P | PBR, PCH, PCL, P |
| Bit 4 of the pushed P | 1 for BRK/COP, 0 for IRQ/NMI | the X flag, verbatim, always |
| PBR after | cleared to `$00` | cleared to `$00`, old value on the stack |
| RTI pulls | P, PCL, PCH | P, PCL, PCH, PBR |
| Stack wrap | page 1 ("old" class) | 16-bit, bank 0 |

Measured on 10 000 `00.n` and 10 000 `00.e` tests with zero mismatches, including
the push order and the vector addresses.

In emulation mode `PBR` is still *cleared* even though it is not saved, so an
interrupt taken while executing in a non-zero bank cannot be returned from. S1
§7.11.3 warns that `RTI` must run in the same mode the interrupt was taken in.

### 4.3 The PC that is pushed

- **IRQ / NMI**: the address of the next instruction.
- **BRK / COP**: the signature-byte address, i.e. **opcode address + 2**. Both are
  two-byte instructions and both skip the signature byte, exactly as on NMOS.
  Measured.
- **ABORT**: the address of the *aborted opcode* (S1 §2.13). Not reachable here.

### 4.4 Flag changes on entry

**Push P first, then set I and clear D.** S1 Table 5-3: *"When an interrupt is
executed, D=0 and I=1 in Status Register P."* S1 §7.12: *"The Binary Mode is set
whenever a hardware or software interrupt is executed. The D flag within the
Status Register is cleared to zero."*

Measured on `00.e`: all 5 057 tests that entered with `D=1` came out with `D=0`.

> **This is a divergence from `CM6502` in emulation mode.** The NMOS 6502 does not
> clear D on interrupt entry; `CM6502::serviceIRQ`, `serviceNMI` and the `BRK`
> case set I but leave D alone, correctly for a 6502. The 65816 clears it in both
> modes. Any program that takes an interrupt while in decimal mode behaves
> differently on a SuperCPU than on a stock C64 — that is real hardware behaviour,
> not our bug.

`m`, `x` and `E` are **not** changed by an interrupt: a handler entered in native
mode with `m=0` is still 16-bit and must set its own widths. `RTI` restores `m`
and `x` from the stacked P (and can therefore zero `XH`/`YH`) but never restores
E. `BRK` and `COP` always execute regardless of I. Priority is
RESET > ABORT > NMI > IRQ.

### 4.5 WAI and STP

`WAI` (`$CB`, 1 byte, 3 cycles) pulls RDY low. It is terminated by RESET, ABORT,
NMI or IRQ. Two exits, and the second is the entire point of the instruction:

1. The interrupt will be taken — the normal sequence runs, and because PC has
   already advanced past the `WAI`, `RTI` returns to the *following* instruction.
2. **`I=1` and IRQ asserted** — S1 §7.13: *"the IRQB interrupt will cause the next
   instruction (following the WAI instruction) to be executed without going to the
   IRQB interrupt handler."* No push, no vector.

IRQ is level-sensitive so a held line wakes `WAI` on the next check; NMI is edge
triggered, so a latched pending edge must also wake it.

`STP` (`$DB`, 1 byte, 3 cycles) stops the clock. **Only RESET restarts it** — not
IRQ, not NMI. Model it exactly like `CM6502`'s `m_Jammed` path: burn a cycle, tick
the bus so the C64 keeps running, and clear the flag only in `reset()`. In S4 both
show three real cycles followed by a dead cycle with no valid address.

---

## 5. Decimal mode

### 5.1 The verdict

The 65816's decimal mode is the **65C02's**, not the NMOS 6502's, widened to 16
bits:

- **N, Z and C are valid**, computed from the final BCD result.
- **V is computed the NMOS way** — from the top digit's sum *before* its `+6`
  fixup — and is meaningless for BCD, but it is what the chip produces and a
  differential test will compare it.
- **There is no extra cycle.** S2, verbatim: *"Note that like the NMOS 6502, but
  unlike the 65C02, decimal mode (i.e. when the d flag is 1) takes no additional
  cycles."* Confirmed by measurement — `ADC #` is 2 cycles at `m=1` and 3 at
  `m=0`, with `D=0` and `D=1` alike, across 20 000 tests.
- With `m=0` it is **four-digit BCD**, `$0000`-`$9999`, and C is the carry out of
  the fourth digit.

S2 on the 16-bit case: *"When the d flag is 1, the n, z, and c flags have the same
meaning […] the carry indicates when the result is outside the range 0 to 9999.
The v flag is overwritten, but BCD is really an unsigned representation, so the v
flag can be considered invalid."*

### 5.2 The algorithm

Written once, parameterised on width. `nd` is 2 digits when `m=1`, 4 when `m=0`;
`sb` is the sign bit, `$80` or `$8000`; `mask` is `$FF` or `$FFFF`.

```
ADC:
    carry = C
    for i in 0 .. nd-1:
        dg = digit_i(A) + digit_i(operand) + carry ;  carry = 0
        if i == nd-1:  pre = result-so-far with digit i replaced by the RAW dg
        if dg > 9:     dg += 6 ;  carry = 1
        digit_i(result) = dg & $F
    C = carry
    V = ( ~(A ^ operand) & (A ^ pre) ) & sb        // NMOS rule, at the top digit
    N = result & sb ;   Z = (result == 0)

SBC:
    borrow = 1 - C
    for i in 0 .. nd-1:
        dg = digit_i(A) - digit_i(operand) - borrow ;  borrow = 0
        if dg < 0:  dg -= 6 ;  borrow = 1
        digit_i(result) = dg & $F
    // C and V come from the BINARY subtraction; N and Z from the BCD result.
    binary = A - operand - (1 - C)
    C = (binary >= 0)
    V = ( (A ^ operand) & (A ^ (binary & mask)) ) & sb
    N = result & sb ;   Z = (result == 0)
```

This model was checked against 40 000 `69.n / 69.e / e9.n / e9.e` tests covering
all four combinations of `{ADC, SBC} × {8-bit, 16-bit}` and produced **zero**
mismatches in A, C, Z, N or V.

It also reproduces S2's own worked example: `A=$0001`, `m=0`, `d=1`, `c=1`,
`SBC #$2003` → `A=$7998`, `n=0`, `z=0`, `c=0`.

### 5.3 The divergence from CM6502 — quantified

`CM6502::opADC`/`opSBC` implement the NMOS rules, where Z comes from the binary
result and N from the pre-fixup high nibble. Running that model against the same
8-bit decimal tests:

| | N wrong | Z wrong | A, C, V wrong |
|---|---|---|---|
| `ADC #` emulation, 4 962 decimal tests | **3 204** | 43 | 0 |
| `SBC #` emulation, 5 015 decimal tests | **1 688** | 14 | 0 |

So: **the result byte, the carry and the overflow flag agree; N disagrees roughly
half the time and Z occasionally.** Both cores are right — `CM6502` for a 6510,
the 65816 core for the chip in a SuperCPU. `Tools/trace_compare` must exempt N and
Z after a decimal-mode `ADC`/`SBC`, or it will drown in false positives the moment
BASIC touches a floating-point routine.

---

## 6. Read-modify-write, and the `m_RMWDummyWrite` verdict

### 6.1 What the chip actually does

The 65816 does **not** have one RMW behaviour. It has two, selected by E:

| | bus cycles | middle cycle |
|---|---|---|
| **`E=1`** | `read, write, write` | **write of the original byte**, RWB low, VDA and VPA both **low** |
| **`E=0`, `m=1`** | `read, read, write` | internal cycle, RWB **high** |
| **`E=0`, `m=0`** | `read, read, read, write, write` | reads lo→hi, internal, writes **hi→lo** |

Measured on 50 000 tests across `ee.e`, `ee.n`, `04.e`, `04.n` and `1c.n`: in
emulation mode 10 000 of 10 000 `INC abs` executions wrote the original byte back
to the same address before writing the incremented one. In native mode not one did.

`_vice/65816core.c` encodes exactly this, gated on `reg_emul`, and — importantly —
uses a *real* store, not a dummy:

```c
var = LOAD_LONG(ea);
if (reg_emul) {
    CHECK_INTERRUPT();
    STORE_LONG(ea, var);          /* dummy write, emulation mode only */
} else {
    if (!bits8) { ea = (ea + 1) & 0xffffff; var |= LOAD_LONG(ea) << 8; }
    CHECK_INTERRUPT();
    LOAD_LONG_DUMMY(ea);          /* dummy read, native mode */
}
```

There is no `STORE_LONG_DUMMY` anywhere in that file. On VICE's SCPU64 the write
reaches I/O.

The one thing that makes this look ambiguous is that the dummy-write cycle has
VDA and VPA both low, which normally means "ignore the address bus". S1 §8.5
answers it directly:

> **When VDA or VPA are high and during all write cycles, the Address Bus is
> always valid.** VDA and VPA should be used to qualify all memory cycles. Note
> that when VDA and VPA are both low, invalid addresses may be generated.

Writes are valid regardless. WDC kept the NMOS dummy write in emulation mode
deliberately, because that is what a drop-in 6502 replacement has to do.

### 6.2 The verdict

**`Source/SuperCPU/supercpu.cpp:78`, `m_Core6502.m_RMWDummyWrite = false`, is
wrong, and the comment above it is half wrong.** The 65816 performs the internal
cycle instead of the dummy write only in **native** mode. In emulation mode — the
mode a C64 boots in and the mode `CM6502` is standing in for — it emits the dummy
write exactly like a 6510.

What this costs if left as it is: `INC $D019` and `LSR $D019`, the two standard
VIC-II interrupt acknowledgements, stop acknowledging. `INC $D019` reads `$81`,
writes `$82`; bit 0 of `$82` is clear, so the raster latch is never cleared and
the handler re-enters forever. That is a hang, not a cosmetic difference, and it
is worth far more than the ~1µs the extra bus cycle costs.

Recommended:

1. In `CW65C816`, make the RMW tail mode-dependent: dummy write when `m_E`,
   internal cycle when not. This is not a configuration option; it is what the
   chip does.
2. In `CSuperCPU::init`, delete the `m_RMWDummyWrite = false` line so the
   milestone-1 core keeps 6510 behaviour, which is also 65816-emulation-mode
   behaviour. Keep the flag itself — it is the knob for **U2**.
3. Note for `CWriteBuffer`: an RMW against mirrored RAM now produces two writes to
   the same address in quick succession. Coalescing already handles that; it is a
   free case, not a new one.

The residual risk is **U2** — whether the real SuperCPU's gate array forwards a
cycle with VDA and VPA low. If hardware testing ever shows raster interrupts
misbehaving in a way that points here, this flag is the first thing to flip, and
that experiment is worth doing deliberately rather than by accident.

`MLB` (memory lock) is asserted during the modify and write cycles. It exists for
multiprocessor arbitration and is irrelevant to us.

---

## 7. MVN and MVP

`MVN $54`, `MVP $44`. 3 bytes. **7 cycles per byte moved**, not 7 total — a
65536-byte move costs 458 752 cycles. No flags.

### 7.1 The operand-order trap

```
object code:   $54  dd  ss        dd = DESTINATION bank, ss = SOURCE bank
WDC syntax:    MVN  srcbk,destbk  source first -- the assembler reverses them
```

S1 §3.5.9: *"The second byte of the instruction contains the high-order 8 bits of
the destination address… The third byte of the instruction contains the high-order
8 bits of the source address."* Assembler syntax varies between vendors; **the
object-code order is the invariant**. Decode `[PC+1]` as destination and `[PC+2]`
as source.

Verified directly: in `54.n`, operand byte 1 was `$3D` and byte 2 was `$6D`; every
read came from bank `$6D` and every write went to bank `$3D`, and the final `DBR`
was `$3D`. Across 3 000 tests × 14 iterations each — 42 000 iterations — the
banks, the offsets, the copied byte, the `C` decrement and the "no flags" property
all matched with zero mismatches. The same for `MVP`.

**Second naming trap**: WDC calls `$54` "Block Move **Negative**" and `$44` "Block
Move **Positive**", while `$54` *increments* and `$44` *decrements*. Eyes & Lichty
use the sane names, "Move Next" and "Move Previous". Ignore both; go by
increment/decrement.

### 7.2 Semantics

| | |
|---|---|
| X | source offset within bank `ss` |
| Y | destination offset within bank `dd` |
| C | byte count **minus one** — always the full 16-bit C, regardless of M |

`DBR` is not consulted for addressing; the source EA is `ss:X` and the destination
`dd:Y`, taken from the instruction stream. **`DBR` is set to the destination bank**
(S1 §7.18 exists solely to say this), destroying whatever it held.

```
read ss:X -> write dd:Y
MVN: X++, Y++          MVP: X--, Y--
C--
repeat while C != $FFFF
```

Bytes moved = `C_initial + 1`. `C=0` moves one byte, `C=$FFFF` moves 65 536. On
completion `C=$FFFF`, X and Y sit one past the end (MVN) or one before the start
(MVP), and `PC += 3`. Use MVN when the destination is below the source (X and Y
point at the low end of each block), MVP when it is above (they point at the high
end). Source and destination wrap independently at their own bank boundaries.

### 7.3 Cycle pattern and why the operands are re-fetched

S1 Table 5-7, entries 9a/9b, confirmed cycle-for-cycle in the test data:

```
1   PBR,PC     opcode fetch      <- PC stays parked on the MVN opcode
2   PBR,PC+1   fetch dd
3   PBR,PC+2   fetch ss
4   ss,X       read source       (RWB=1)
5   dd,Y       write destination (RWB=0)
6   dd,Y       internal
7   dd,Y       internal
```

Every iteration re-fetches the opcode and both operand bytes; PC only advances by
3 on the final one. That exists so the instruction is interruptible. Eyes &
Lichty: *"If a block move instruction is interrupted, it may be resumed
automatically via execution of an RTI if all of the registers are restored or
intact. The value pushed onto the stack when a block move is interrupted is the
address of the block move instruction. The current byte-move is completed before
the interrupt is serviced."*

So: recognise interrupts **between** byte moves, never mid-byte; push the address
of the `MVN`/`MVP` opcode itself; let `RTI` re-enter the instruction, which picks
up from X, Y and C. The handler must preserve all three. A side effect worth
knowing is that because `dd` is re-fetched every iteration, `DBR` is effectively
reloaded each time, so a handler that clobbers `DBR` does not break a resumed
move.

### 7.4 With 8-bit index registers

S1 §8.7.3.2: with `x=1` the blocks *"can only move data in the range 0000 to
00FF"*, because `XH` and `YH` are architecturally `$00`. The **bank operands still
apply** on a 65816 (they are ignored on a 65802), so an emulation-mode block move
can cross banks but only within page 0 of each. Verified — the 8-bit-index tests
wrapped X and Y at `$FF`. Practically useless; implement it correctly and move on.

### 7.5 For the write buffer

A block move into shadowed bank 0 can invalidate up to 64KB in one instruction.
`CWriteBuffer` needs a range-invalidate path rather than 65 536 individual
mirrored writes; this is already on the milestone-2 list in `Docs/roadmap.md`.

---

## 8. Traps for the implementer, ranked

Ranked by (probability of getting it wrong) × (how long it takes to find).

1. **Instruction length of the ex-undocumented encodings.** 50 of the 105
   encodings `CM6502` treats as NOPs have a *different length* on the 65816.
   `$22 JSL` is 4 bytes where the 6502 table says 1; `$0B PHD` is 1 where the
   table says 2. Get one wrong and the instruction stream desynchronises, which
   presents as arbitrary corruption a long way from the cause. Build the 65816
   opcode table as its own file with its own length column; never derive lengths
   from `m6502ModeLength`.
2. **The `applyE()` invariant.** Forgetting it after one of `PLP`, `RTI`, `TCS`,
   `TXS`, `PLX`, `PLY`, `SEP`, `REP` leaves an impossible state — a 16-bit X in
   emulation mode, or a stack pointer outside page 1 — that only shows up much
   later. Call it after every write to P, S, X or Y.
3. **`x` 0→1 destroys `XH` and `YH` immediately.** Not lazily, not on next use.
   Whereas `m` 0→1 preserves B. Getting these the same way round is the classic
   65816 bug.
4. **The emulation-mode stack split.** `PHA` wraps inside page 1; `PEA` does not.
   Two push helpers, and the WDC list in §2.5 decides which is used.
5. **Decimal N and Z follow the 65C02 rule, not the NMOS rule.** Copying
   `CM6502::opADC` verbatim produces a wrong N about half the time in decimal
   mode (§5.3).
6. **MVN/MVP operand order is destination-then-source in the object code**, the
   reverse of the assembler syntax (§7.1).
7. **The RMW dummy write is emulation-mode only** (§6). Both getting it backwards
   and applying it in both modes are easy mistakes.
8. **`TCS` sets no flags. `TSC`, `TCD` and `TDC` set N and Z from all 16 bits**,
   even when `m=1` or `E=1`. This is the most commonly mis-implemented group on
   the chip. `TCS` in emulation loads only `SL`; `TSC` in emulation returns the
   full `$01xx`, so B receives `$01`.
9. **Absolute indexed carries into the bank byte** (§2.6) — the one emulation-mode
   addressing difference from a 6502.
10. **Interrupts clear D** in both modes (§4.4). NMOS does not.
11. **`XBA` is 3 cycles**, and sets N and Z from the new 8-bit low byte even when
    `m=0`. It ignores M, X and E entirely.
12. **`JSL` pushes 3 bytes and `RTL` pulls 3**; `JSR`/`RTS` use 2. Mismatching
    them is unrecoverable. `RTL`'s `+1` is a 16-bit increment with **no carry into
    the bank** — a stacked `$xx:FFFF` resumes at `$xx:0000`.
13. **`JSL` pushes PBR *before* fetching the new bank byte** (cycle 4 of 8), and
    `JSR (a,X)` pushes the return address before reading the pointer. Only matters
    if you are chasing a cycle-level trace discrepancy.
14. **Immediate operand width follows M and X.** After `REP #$20` the next `LDA #`
    takes a 16-bit operand. `LDA #` is `3-m` bytes; `LDX #` is `3-x`.
15. **Native BRK has no B bit**; bit 4 of the pushed P is the X flag (§3.3).
16. **`JMP ($xxFF)` is fixed** — no page-wrap bug, and it is 5 cycles, the same as
    NMOS. `rd16WrapPage()` must have no counterpart in the 65816 core.
17. **`JMP (a)`/`JMP [a]` take their pointer from bank 0; `JMP (a,X)`/`JSR (a,X)`
    take theirs from PBR.** S1 §7.9 says exactly this and it is easy to read past.
18. **16-bit RMW writes the high byte first** (`ee.n`, 4 971/4 971), reading lo→hi
    but writing hi→lo. Only observable against I/O, but that is where it matters.
19. **`WDM $42` is 2 bytes.** Treating it as 1 desynchronises the stream. It is a
    reserved no-op — *"WDM are the initials of William D. Mensch, Jr."* Do not
    make it a host trap by default; a plain 2-byte NOP is the hardware behaviour.
    Logging it is worthwhile, since a `WDM` in a C64 program means the CPU is
    executing data.
20. **`PEA` is misnamed** — it pushes its 16-bit operand literally and reads no
    memory. `PEI` reads. `PER` adds a signed 16-bit displacement to the address of
    the *next* instruction. All three push 16 bits regardless of M and X.

---

## 9. The encodings the 6502 leaves undocumented

`CM6502` executes all 105 of these as NOPs of the length in its own table, and
JAMs on 12 of them. The 65816 must execute the real instruction. The `bytes`
column reads *what `CM6502` assumes* → *what the 65816 actually is*; **!** marks a
disagreement, and there are **50** of them.

Cycle formulas use S2's notation: `e`, `m`, `x` are the flags, `p` is 1 on a page
cross, `t` is 1 on a taken branch, `w` is 1 when `DL ≠ $00`.

| Op | NMOS | 65816 | mode | bytes | cycles |
|---|---|---|---|---|---|
| `$02` | JAM imp | `COP #$12` | imm | 1 -> 2 **!** | 8-e |
| `$03` | SLO izx | `ORA $32,S` | stk,S | 2 -> 2 | 5-m |
| `$04` | NOP zp | `TSB $10` | dir | 2 -> 2 | 7-2*m+w |
| `$07` | SLO zp | `ORA [$10]` | [dir] | 2 -> 2 | 7-m+w |
| `$0B` | ANC imm | `PHD` | imp | 2 -> 1 **!** | 4 |
| `$0C` | NOP abs | `TSB $9876` | abs | 3 -> 3 | 8-2*m |
| `$0F` | SLO abs | `ORA $FEDBCA` | long | 3 -> 4 **!** | 6-m |
| `$12` | JAM imp | `ORA ($10)` | (dir) | 1 -> 2 **!** | 6-m+w |
| `$13` | SLO izy | `ORA ($32,S),Y` | (stk,S),Y | 2 -> 2 | 8-m |
| `$14` | NOP zpx | `TRB $10` | dir | 2 -> 2 | 7-2*m+w |
| `$17` | SLO zpx | `ORA [$10],Y` | [dir],Y | 2 -> 2 | 7-m+w |
| `$1A` | NOP imp | `INC` | acc | 1 -> 1 | 2 |
| `$1B` | SLO aby | `TCS` | imp | 3 -> 1 **!** | 2 |
| `$1C` | NOP abx | `TRB $9876` | abs | 3 -> 3 | 8-2*m |
| `$1F` | SLO abx | `ORA $FEDCBA,X` | long,X | 3 -> 4 **!** | 6-m |
| `$22` | JAM imp | `JSL $123456` | long | 1 -> 4 **!** | 8 |
| `$23` | RLA izx | `AND $32,S` | stk,S | 2 -> 2 | 5-m |
| `$27` | RLA zp | `AND [$10]` | [dir] | 2 -> 2 | 7-m+w |
| `$2B` | ANC imm | `PLD` | imp | 2 -> 1 **!** | 5 |
| `$2F` | RLA abs | `AND $FEDBCA` | long | 3 -> 4 **!** | 6-m |
| `$32` | JAM imp | `AND ($10)` | (dir) | 1 -> 2 **!** | 6-m+w |
| `$33` | RLA izy | `AND ($32,S),Y` | (stk,S),Y | 2 -> 2 | 8-m |
| `$34` | NOP zpx | `BIT $10,X` | dir,X | 2 -> 2 | 5-m+w |
| `$37` | RLA zpx | `AND [$10],Y` | [dir],Y | 2 -> 2 | 7-m+w |
| `$3A` | NOP imp | `DEC` | acc | 1 -> 1 | 2 |
| `$3B` | RLA aby | `TSC` | imp | 3 -> 1 **!** | 2 |
| `$3C` | NOP abx | `BIT $9876,X` | abs,X | 3 -> 3 | 6-m-x+x*p |
| `$3F` | RLA abx | `AND $FEDCBA,X` | long,X | 3 -> 4 **!** | 6-m |
| `$42` | JAM imp | `WDM` | imm | 1 -> 2 **!** | 2 |
| `$43` | SRE izx | `EOR $32,S` | stk,S | 2 -> 2 | 5-m |
| `$44` | NOP zp | `MVP #$12,#$34` | src,dest | 2 -> 3 **!** | 7 |
| `$47` | SRE zp | `EOR [$10]` | [dir] | 2 -> 2 | 7-m+w |
| `$4B` | ALR imm | `PHK` | imp | 2 -> 1 **!** | 3 |
| `$4F` | SRE abs | `EOR $FEDBCA` | long | 3 -> 4 **!** | 6-m |
| `$52` | JAM imp | `EOR ($10)` | (dir) | 1 -> 2 **!** | 6-m+w |
| `$53` | SRE izy | `EOR ($32,S),Y` | (stk,S),Y | 2 -> 2 | 8-m |
| `$54` | NOP zpx | `MVN #$12,#$34` | src,dest | 2 -> 3 **!** | 7 |
| `$57` | SRE zpx | `EOR [$10],Y` | [dir],Y | 2 -> 2 | 7-m+w |
| `$5A` | NOP imp | `PHY` | imp | 1 -> 1 | 4-x |
| `$5B` | SRE aby | `TCD` | imp | 3 -> 1 **!** | 2 |
| `$5C` | NOP abx | `JMP $FEDCBA` | long | 3 -> 4 **!** | 4 |
| `$5F` | SRE abx | `EOR $FEDCBA,X` | long,X | 3 -> 4 **!** | 6-m |
| `$62` | JAM imp | `PER LABEL` | imm | 1 -> 3 **!** | 6 |
| `$63` | RRA izx | `ADC $32,S` | stk,S | 2 -> 2 | 5-m |
| `$64` | NOP zp | `STZ $10` | dir | 2 -> 2 | 4-m+w |
| `$67` | RRA zp | `ADC [$10]` | [dir] | 2 -> 2 | 7-m+w |
| `$6B` | ARR imm | `RTL` | imp | 2 -> 1 **!** | 6 |
| `$6F` | RRA abs | `ADC $FEDBCA` | long | 3 -> 4 **!** | 6-m |
| `$72` | JAM imp | `ADC ($10)` | (dir) | 1 -> 2 **!** | 6-m+w |
| `$73` | RRA izy | `ADC ($32,S),Y` | (stk,S),Y | 2 -> 2 | 8-m |
| `$74` | NOP zpx | `STZ $10,X` | dir,X | 2 -> 2 | 5-m+w |
| `$77` | RRA zpx | `ADC [$10],Y` | [dir],Y | 2 -> 2 | 7-m+w |
| `$7A` | NOP imp | `PLY` | imp | 1 -> 1 | 5-x |
| `$7B` | RRA aby | `TDC` | imp | 3 -> 1 **!** | 2 |
| `$7C` | NOP abx | `JMP ($1234,X)` | (abs,X) | 3 -> 3 | 6 |
| `$7F` | RRA abx | `ADC $FEDCBA,X` | long,X | 3 -> 4 **!** | 6-m |
| `$80` | NOP imm | `BRA LABEL` | rel8 | 2 -> 2 | 3+e*p |
| `$82` | NOP imm | `BRL LABEL` | rel16 | 2 -> 3 **!** | 4 |
| `$83` | SAX izx | `STA $32,S` | stk,S | 2 -> 2 | 5-m |
| `$87` | SAX zp | `STA [$10]` | [dir] | 2 -> 2 | 7-m+w |
| `$89` | NOP imm | `BIT #$54` | imm | 2 -> 3-m **!** | 3-m |
| `$8B` | ANE imm | `PHB` | imp | 2 -> 1 **!** | 3 |
| `$8F` | SAX abs | `STA $FEDBCA` | long | 3 -> 4 **!** | 6-m |
| `$92` | JAM imp | `STA ($10)` | (dir) | 1 -> 2 **!** | 6-m+w |
| `$93` | SHA izy | `STA ($32,S),Y` | (stk,S),Y | 2 -> 2 | 8-m |
| `$97` | SAX zpy | `STA [$10],Y` | [dir],Y | 2 -> 2 | 7-m+w |
| `$9B` | TAS aby | `TXY` | imp | 3 -> 1 **!** | 2 |
| `$9C` | SHY abx | `STZ $9876` | abs | 3 -> 3 | 5-m |
| `$9E` | SHX aby | `STZ $9876,X` | abs,X | 3 -> 3 | 6-m |
| `$9F` | SHA aby | `STA $FEDCBA,X` | long,X | 3 -> 4 **!** | 6-m |
| `$A3` | LAX izx | `LDA $32,S` | stk,S | 2 -> 2 | 5-m |
| `$A7` | LAX zp | `LDA [$10]` | [dir] | 2 -> 2 | 7-m+w |
| `$AB` | LXA imm | `PLB` | imp | 2 -> 1 **!** | 4 |
| `$AF` | LAX abs | `LDA $FEDBCA` | long | 3 -> 4 **!** | 6-m |
| `$B2` | JAM imp | `LDA ($10)` | (dir) | 1 -> 2 **!** | 6-m+w |
| `$B3` | LAX izy | `LDA ($32,S),Y` | (stk,S),Y | 2 -> 2 | 8-m |
| `$B7` | LAX zpy | `LDA [$10],Y` | [dir],Y | 2 -> 2 | 7-m+w |
| `$BB` | LAS aby | `TYX` | imp | 3 -> 1 **!** | 2 |
| `$BF` | LAX aby | `LDA $FEDCBA,X` | long,X | 3 -> 4 **!** | 6-m |
| `$C2` | NOP imm | `REP #$12` | imm | 2 -> 2 | 3 |
| `$C3` | DCP izx | `CMP $32,S` | stk,S | 2 -> 2 | 5-m |
| `$C7` | DCP zp | `CMP [$10]` | [dir] | 2 -> 2 | 7-m+w |
| `$CB` | SBX imm | `WAI` | imp | 2 -> 1 **!** | 3 |
| `$CF` | DCP abs | `CMP $FEDBCA` | long | 3 -> 4 **!** | 6-m |
| `$D2` | JAM imp | `CMP ($10)` | (dir) | 1 -> 2 **!** | 6-m+w |
| `$D3` | DCP izy | `CMP ($32,S),Y` | (stk,S),Y | 2 -> 2 | 8-m |
| `$D4` | NOP zpx | `PEI $12` | dir | 2 -> 2 | 6+w |
| `$D7` | DCP zpx | `CMP [$10],Y` | [dir],Y | 2 -> 2 | 7-m+w |
| `$DA` | NOP imp | `PHX` | imp | 1 -> 1 | 4-x |
| `$DB` | DCP aby | `STP` | imp | 3 -> 1 **!** | 3 |
| `$DC` | NOP abx | `JMP [$1234]` | [abs] | 3 -> 3 | 6 |
| `$DF` | DCP abx | `CMP $FEDCBA,X` | long,X | 3 -> 4 **!** | 6-m |
| `$E2` | NOP imm | `SEP #$12` | imm | 2 -> 2 | 3 |
| `$E3` | ISC izx | `SBC $32,S` | stk,S | 2 -> 2 | 5-m |
| `$E7` | ISC zp | `SBC [$10]` | [dir] | 2 -> 2 | 7-m+w |
| `$EB` | SBC imm | `XBA` | imp | 2 -> 1 **!** | 3 |
| `$EF` | ISC abs | `SBC $FEDBCA` | long | 3 -> 4 **!** | 6-m |
| `$F2` | JAM imp | `SBC ($10)` | (dir) | 1 -> 2 **!** | 6-m+w |
| `$F3` | ISC izy | `SBC ($32,S),Y` | (stk,S),Y | 2 -> 2 | 8-m |
| `$F4` | NOP zpx | `PEA #$1234` | imm | 2 -> 3 **!** | 5 |
| `$F7` | ISC zpx | `SBC [$10],Y` | [dir],Y | 2 -> 2 | 7-m+w |
| `$FA` | NOP imp | `PLX` | imp | 1 -> 1 | 5-x |
| `$FB` | ISC aby | `XCE` | imp | 3 -> 1 **!** | 2 |
| `$FC` | NOP abx | `JSR ($1234,X)` | (abs,X) | 3 -> 3 | 8 |
| `$FF` | ISC abx | `SBC $FEDCBA,X` | long,X | 3 -> 4 **!** | 6-m |

### 9.1 Reading the table

The 105 encodings fall into five groups, and knowing the shape is faster than
memorising the list:

- **Columns `$x3`, `$x7`, `$xB`, `$xF` — 64 encodings, every one undocumented on
  NMOS.** `$x3` is stack relative (`d,S` on even rows, `(d,S),Y` on odd), `$x7` is
  long indirect (`[d]` / `[d],Y`), `$xF` is long absolute (`al` / `al,X`), and
  `$xB` is the 16 one-byte oddballs: `PHD TCS PLD TSC PHK TCD RTL TDC PHB TXY PLB
  TYX WAI STP XBA XCE`.
- **Column `$x2` — the 12 NMOS JAMs plus `$82`, `$C2`, `$E2`.** The JAMs become
  `COP`, the eight `(d)` forms, `JSL`, `WDM` and `PER`. `$82` is `BRL` (3 bytes
  where `CM6502` assumes 2), `$C2`/`$E2` are `REP`/`SEP`.
- **The 65C02 additions the NMOS left as multi-byte NOPs**: `TSB`, `TRB`, `STZ`,
  `BIT dp,X`/`BIT abs,X`/`BIT #`, `INC A`, `DEC A`, `PHX/PHY/PLX/PLY`, `BRA`,
  `JMP (a,X)`.
- **The four `NOP abx` slots that became long jumps**: `$5C JMP al`,
  `$7C JMP (a,X)`, `$DC JMP [a]`, `$FC JSR (a,X)`.
- **The block moves and stack pushes hiding in `NOP zp`/`NOP zpx` slots**:
  `$44 MVP`, `$54 MVN`, `$D4 PEI`, `$F4 PEA`.

Note `$9C` and `$9E`, which are the unstable NMOS `SHY`/`SHX`: on the 65816 they
are `STZ abs` and `STZ abs,X`. And `$EB`, the NMOS's duplicate `SBC #`, is `XBA`.

---

## 10. The differential-test contract

`Tools/trace_compare` runs `CM6502` as the oracle in emulation mode. The two cores
agree everywhere except the list below, which is complete as far as the evidence
in this document goes. Encode it as an explicit exemption list, not as tolerance —
anything not on this list that differs is a bug.

| # | Divergence | Trigger | Action |
|---|---|---|---|
| 1 | **The 105 undocumented encodings** (§9) | any of them | Exempt. This is the whole point of the 65816 core. |
| 2 | **Decimal `ADC`/`SBC`: N and Z** (§5.3) | `D=1` | Exempt N and Z. A, C and V must still match. |
| 3 | **Interrupts clear D** (§4.4) | interrupt taken with `D=1` | Exempt D after any interrupt entry. |
| 4 | **Absolute indexed carries into the bank** (§2.6) | `base + index > $FFFF` | Exempt. With `DBR=$00` the 65816 reads bank 1 where the 6502 reads `$0000`. |
| 5 | **`JMP ($xxFF)`** (§8.16) | pointer low byte `$FF` | Exempt. The 65816 fixed the bug. |
| 6 | **P bit 4** | after `PLP` or `RTI` | Normalise. `CM6502` holds bit 4 clear in the live P; the 65816 holds it *set* because x is forced to 1. Both push the same byte, so mask bit 4 when comparing P — never when comparing pushed bytes. |
| 7 | **Direct-page pointer wrap** (§1.3 **U1**) | `dp` (or `dp+X`) `= $FF` with `D=$0000` | Should *not* diverge if U1 is implemented as recommended. If a trace ever diverges here, it is evidence about U1, not a bug — record it. |
| 8 | **RMW dummy write** (§6) | RMW against I/O | Should *not* diverge once `m_RMWDummyWrite` is restored to `true`. Until then it will, on every RMW. |
| 9 | **Reset `S`** (§3.6, **U3**) | reset | Both use `$xxFD`; keep them in step. |

Everything else — all 151 documented opcodes, their lengths, their base cycle
counts, their page-cross and branch penalties, zero-page wrapping, stack
behaviour, the pushed P byte on interrupts, `BRK`'s pushed PC — is identical, and
§1.2 says so with numbers.

---

## Sources

Primary:

- WDC **W65C816S datasheet**, rev. 13 Mar 2024 — §2.7, §2.8, §2.11, §2.13, §2.25,
  §3.4, §3.5.6, §3.5.9, §3.7, §7.9-§7.23, §8.1-§8.7, Tables 5-2/5-3/5-4/5-5/5-7,
  Table 7-1. Full text:
  <https://archive.org/download/WDC_65C816S_Datasheet/WDC_65C816S_Datasheet_djvu.txt>
- Bruce Clark, **65C816 Opcodes**, 6502.org — §5.1.1, §5.1.2, §5.2-§5.22, §6.1.1.1,
  and the complete opcode table, from which §9 here is mechanically derived.
  <https://6502.org/tutorials/65c816opcodes.html>
- Bruce Clark, **Decimal Mode**, 6502.org —
  <https://6502.org/tutorials/decimal_mode.html>
- Eyes & Lichty, **Programming the 65816** (1986) —
  <https://apprize.best/programming/65816/14.html>

Corroborating:

- **VICE**, `src/65816core.c` and `src/scpu64/scpu64mem.c` — local copy in
  `_vice/` (git-ignored). The SCPU64 core, and the only 65816 implementation
  routinely run against real SuperCPU software.
- **SingleStepTests/65816** — <https://github.com/SingleStepTests/65816>.
  20 000 tests per opcode with per-cycle VDA/VPA/RWB/E/M/X. Emulator-generated;
  see §1.4.
- SuperFamicom wiki, 65816 reference — <https://wiki.superfamicom.org/65816-reference>
- undisbeliever, 65816 opcode pseudocode —
  <https://undisbeliever.net/snesdev/65816-opcodes.html>
- 6502.org forum thread 1345 (addressing modes and cycle counts) and thread 7968
  (DBR in emulation mode, checked on real silicon).

Repository cross-references:

- `Source/CPU/W65C816/w65c816.h` — the fixed interface. All ten vector
  constants and all eight flag masks were checked against S1 Tables 5-2/5-3 and
  are correct, including the deliberate absence of `W65_VEC_NATIVE_RESET` and of
  an emulation `BRK` constant.
- `Source/CPU/M6502/m6502.cpp` — the house style and the differential oracle.
  `m6502.cpp:141-153` (addressing helpers), `:50-103` (decimal ALU),
  `:116-132` (interrupts), `:157` (`RMW_DUMMY_WRITE`).
- `Source/CPU/M6502/m6502_opcodes.cpp` — the 151/105 split used in §9.
- `Source/SuperCPU/supercpu.cpp:74-78` — the `m_RMWDummyWrite` setting §6 rules on.

### Working data

The analysis scripts and the downloaded opcode tests live in the session
scratchpad and are not committed:

```
…/scratchpad/dec2.py       decimal-mode model vs 40 000 tests
…/scratchpad/rmw.py        RMW bus patterns, native vs emulation
…/scratchpad/ops.py, ops2.py   XCE, XBA, TSC/TCS, BIT #, BRK, MVN/MVP, PEI, BRL, JML
…/scratchpad/table816.py   opcode table extraction and the 6502 cross-check
…/scratchpad/tests/*.json  77 SingleStepTests opcode files
```

They are cheap to regenerate — every one reads only `bc.txt` (the S2 text) and the
JSON test files — but the numbers quoted throughout this document came from them,
so if a claim here is ever challenged, that is where to start.
