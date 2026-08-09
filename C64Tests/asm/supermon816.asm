; C64/SuperCPU port for 64tass. The original license and monitor core follow.
; RUN enters native mode; X restores the C64 memory map and returns to BASIC.
;* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
;*                                                                                 *
;*      SUPERMON 816 MACHINE LANGUAGE MONITOR FOR THE W65C816S MICROPROCESSOR      *
;* ——————————————————————————————————————————————————————————————————————————————— *
;*      Copyright ©1991-2023 by BCS Technology Limited.  All rights reserved.      *
;*                                                                                 *
;* Redistribution & use of this software in source and/or binary forms, with or    *
;* without modifications, are permitted, provided that the following conditions    *
;* are met:                                                                        *
;*                                                                                 *
;* 1) Redistributions of source code must retain the above copyright notice, this  *
;*    list of conditions & the below disclaimer.                                   *
;*                                                                                 *
;* 2) Redistribution in binary form must reproduce the above copyright notice,     *
;*    this list of conditions & the below disclaimer in the documentation & other  *
;*    materials included with the distribution.                                    *
;*                                                                                 *
;* 3) The names of the copyright holder and/or its contributors shall not be used  *
;*    to endorse or promote any product derived from this software, unless written *
;*    permission to do so has been granted by the copyright holder.                *
;*                                                                                 *
;* DISCLAIMER                                                                      *
;* ——————————                                                                      *
;* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS & CONTRIBUTORS “AS IS”.  Any *
;* express or implied warranties, including, but not limited to, the implied war-  *
;* ranties of merchantability & fitness for a particular purpose are disclaimed.   *
;* In no event shall the copyright owner and/or contributors be liable for any     *
;* direct, indirect, incidental, special, exemplary, or consequential damages      *
;* (including, but not limited to, procurement of substitute goods or services;    *
;* loss of use, data, or profits; or business interruption) however caused & on    *
;* any theory of liability, whether in contract, strict liability, or tort (incl-  *
;* uding negligence or otherwise) arising in any way out of the use of this soft-  *
;* ware, even if advised of the possibility of such damage.                        *
;*                                                                                 *
;* If any provision set forth herein is not acceptable to you, DO NOT USE THIS     *
;* SOFTWARE & immediately delete it & all accompanying documentation from your     *
;* system.                                                                         *
;* ——————————————————————————————————————————————————————————————————————————————— *
;* Supermon 816 is a salute to Jim Butterfield (1936-2007).                        *
;*                                                                                 *
;* Jim, who was the unofficial  spokesman for  Commodore  International during the *  
;* heyday of the company's 8 bit supremacy, was the author of the Supermon machine *
;* language monitor for the PET & CBM computers.   When the best-selling Commodore *
;* 64 was introduced, Jim adapted his software to the new machine & gave the adap- *
;* tation the name Supermon 64.                                                    *
;*                                                                                 *
;* Although Supermon 816 is not an adaptation of Supermon 64,  it was  decided  to *
;* keep the Supermon name alive, since Supermon 816's general operation & user in- *
;* terface is similar to that of Supermon 64.  Supermon 816 is 100 percent native- *
;* mode 65C816 code & was developed from a blank canvas.                           *
;*                                                                                 *
;* Supermon 816's source code was edited and assembled with the assembler in the   *
;* Kowalski simulator, version 1.3.4.  The latest version of the simulator can be  *
;* downloaded at https://rictor.org/sbc/kowalski.html (scroll to the bottom of the *
;* page for the download).  The simulator runs on Microsoft Windows only.          *
;* ——————————————————————————————————————————————————————————————————————————————— *
;* Supermon 816 is a full featured machine language monitor.  It can:              *
;*                                                                                 *
;*     A — Assemble code.                                                          *
;*     C — Compare memory regions.                                                 *
;*     D — Disassemble code.                                                       *
;*     F — Fill memory region (fill cannot span banks in this version).            *
;*     G — Execute code (stops at BRK).                                            *
;*     H — Search (hunt) memory.                                                   *
;*     J — Execute code as a subroutine (stops at BRK or RTS).                     *
;*     L — Load Motorola S-records (function specific to this version).            *
;*     M — Dump & display memory range.                                            *
;*     R — Dump & display 65C816 registers.                                        *
;*     T — Copy (transfer) memory region.                                          *
;*     > — Modify (“poke”) memory.                                                 *
;*     ; — Modify 65C816 registers.                                                *
;*                                                                                 *
;* Supermon 816 accepts binary (%), octal (@), decimal (+) and hexadecimal ($) as  *
;* input for numeric parameters.  Additionally, the H and > operations accept an   *
;* ASCII string in place of numeric values by preceding the string with ', e.g.:   *
;*                                                                                 *
;*     h 042000 042FFF 'BCS Technology Limited                                     *
;*                                                                                 *
;* If no radix symbol is entered hex is assumed.                                   *
;*                                                                                 *
;* Numeric conversion is also available.  For example, typing:                     *
;*                                                                                 *
;*     +1234567 [CR]                                                               *
;*                                                                                 *
;* at the monitor's prompt will display:                                           *
;*                                                                                 *
;*         $12D687                                                                 *
;*         +1234567                                                                *
;*         @04553207                                                               *
;*         %100101101011010000111                                                  *
;*                                                                                 *
;* In the above example, [CR] means the console keyboard's return or enter key.    *
;*                                                                                 *
;* All numeric values are internally processed as 32-bit unsigned integers.  Addr- *
;* esses may be entered as 8-, 16- or 24-bit values.  Instruction assembly follows *
;* 64tass's 65816 width directives.  Enter .AS/.AL to select 8/16-bit accumulator   *
;* and BIT immediates, or .XS/.XL for 8/16-bit index-register immediates.  The       *
;* selected widths persist and are also updated when REP/SEP is assembled:          *
;*                                                                                 *
;*     a 1f2000 .al                                                               *
;*     a 1f2000 lda #4        assembles A9 04 00                                   *
;*     a 1f2003 .xl                                                               *
;*     a 1f2003 ldx #+157     assembles A2 9D 00                                   *
;*                                                                                 *
;* In .AS or .XS mode, an immediate greater than $FF is rejected, as in 64tass.    *
;*                                                                                 *
;* Type X at the Supermon 816 prompt to exit to the operating environment.         *
;* ——————————————————————————————————————————————————————————————————————————————— *
;*                                                                                 *
;* Notes on Instruction Syntax                                                     *
;* ———————————————————————————                                                     *
;* The canonical forms displayed by bundled 64tass 1.60 are used:                  *
;*                                                                                 *
;*     PEA #<word>       PEI <dp>          PER <target>                            *
;*     MVN #<src>,#<dst> MVP #<src>,#<dst>                                        *
;*                                                                                 *
;* Parentheses denote indirect and square brackets indirect-long addressing.       *
;*                                                                                 *
;* The COP instruction is treated as immediate mode for syntactical reasons.  The  *
;* correct syntax for COP is:                                                      *
;*                                                                                 *
;*     COP #<sig>                                                                  *
;*                                                                                 *
;* where <sig> is the 8-bit signature.  Supermon 816 doesn't check the signature's *
;* value, other than to verify it is 8 bits.                                       *
;*                                                                                 *
;* Although the official WDC assembly language syntax for the jump (JMP) instruc-  *
;* tion allows use of a “long” (24-bit) address, Supermon 816 doesn't support that *
;* addressing mode.  Use JML instead.  JML's operand will always be assembled as a *
;* 24-bit address.  JMP's operand will always be treated as a 16-bit address.  An  *
;* attempt to assemble JMP with a 24-bit address will cause an error.              *
;*                                                                                 *
;* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
;
;	* * * * * * * * * * * *
;	* VERSION INFORMATION *
;	* * * * * * * * * * * *
;
softvers .macro
         .text "1";major
         .text "."
         .text "1";minor
         .text "."
         .text "10";revision
         .endm
;
;REVISION TABLE
;
;Ver  Rev Date    Description
;———————————————————————————————————————————————————————————————————————————————
;1.1.10 2026/08/09 Keep the C64 screen editor out of quote mode around every
;                    CHROUT.  Literal quotes in a dump's ASCII column can no
;                    longer turn later control codes into screen glyphs.
;
;1.1.9 2026/08/09 Added a software-managed reverse-block input cursor.  The
;                    monitor keeps IRQs disabled, so the KERNAL cursor cannot
;                    blink on its own.
;
;1.1.8 2026/08/09 Avoided sending CR after each full 40-column memory-dump row.
;                    Full rows already auto-wrap in the C64 screen editor; the
;                    extra CR caused double scrolling and unstable long dumps.
;
;1.1.7 2026/08/09 Fixed workspace references that had assembled as absolute,Y
;                    instead of direct-page indexed accesses after relocating
;                    the monitor workspace to $0200.  Operand display and
;                    numeric output now read the live workspace.
;
;1.1.6 2026/08/09 Set GETCHAR's Z flag explicitly instead of relying on XBA
;                    flag side effects.  Numeric parameters now consume every
;                    digit under SuperCPU hardware and VICE xscpu64.
;
;1.1.5 2026/08/09 Replaced mixed-width numeric conversion with a deterministic
;                    bytewise 32-bit parser for all monitor commands.
;
;1.1.4 2026/08/09 C64/SuperCPU port: native-mode console, stable monitor direct
;                    page, 64tass-style assembler syntax and width directives.
;
;1.1  2015/07/31  A) Added code to accept Motorola S-record loads.  This version
;                    continues with the VT-100 console driver.
;
;     2021/02/22  A) Fix an error in the monitor instruction encode/decode table
;                    that prevented ORA <abs>,Y from being assembled & caused
;                    the instruction to be disassembled as ORA <abs>.
;
;     2021/05/16  A) Fixed an error in the function that generates an offset for
;                    use in assembling BRL & PER.  In cases in which the target
;                    was out of range for a forward branch or reference, the
;                    function was not correctly computing the offset needed &
;                    was instead indicating an out-of-range error.
;
;     2023/08/14  A) Corrected an inadvertent coding error in the GETBYTE sub-
;                    routine.
;———
;1.0  2013/11/01  A) Original derived from the POC V1.1 single-board computer
;                    firmware.
;
;     2013/11/04  A) Fixed a problem where the B-accumulator wasn't always being
;                    be copied to shadow storage after return from execution of
;                    a J command.
;
;     2014/06/03  A) Detail change in the INPUT subroutine; no effect on funct-
;                    ionality.
;
;                 B) Expanded the program notes.
;
;     2014/06/08  A) Special version with VT-100 support.
;———————————————————————————————————————————————————————————————————————————————
;
;
;	COMMENT ABBREVIATIONS
;	—————————————————————————————————————
;	BCD   binary-coded decimal
;	 DP   direct page or page zero
;	EOF   end-of-field
;	EOI   end-of-input
;	LSB   least significant byte/bit
;	LSD   least significant digit
;	LSN   least significant nybble
;	LSW   least significant word
;	MPU   microprocessor
;	MSB   most significant byte/bit
;	MSD   most significant digit
;	MSN   most significant nybble
;	MSW   most-significant word
;	RAM   random access memory
;	 WS   whitespace, i.e., blanks & tabs
;	—————————————————————————————————————
;	A word is defined as 16 bits.
;
;	MPU REGISTER SYMBOLS
;	———————————————————————
;	.A   accumulator LSB
;	.B   accumulator MSB
;	.C   16 bit accumulator
;	.X   X-index
;	.Y   Y-index
;	DB   data bank
;	DP   direct page
;	PB   program bank
;	PC   program counter
;	SP   stack pointer
;	SR   MPU status
;	———————————————————————
;
;	MPU STATUS REGISTER SYMBOLS
;	———————————————————————————
;	C   carry
;	D   decimal mode
;	I   maskable interrupts
;	m   accumulator/memory size
;	N   result negative
;	V   sign overflow
;	x   index registers size
;	Z   result zero
;	———————————————————————————
;
;===============================================================================
;
;SYSTEM INTERFACE DEFINITIONS
;
;	——————————————————————————————————————————————————————————————————
;	This section defines the interface between Supermon 816 & the host
;	system.   Change these definitions to suit your system, but do not
;	change any label names.  All definitions must have valid values in
;	order to assemble & run Supermon 816.
;	——————————————————————————————————————————————————————————————————
;
;	————————————————————————————————————————————————————————
origin_addr =$000a00           ;assembly address...
;
;	Set _ORIGIN_ to Supermon 816's desired assembly address.
;	————————————————————————————————————————————————————————
;
;	————————————————————————————————————————————————————————————————————————
vecexit  =c64_exit              ;exit to environment address...
;
;	Set VECEXIT to where Supermon 816 should go when it exits.  Supermon 816
;	will do a JML (long jump) to this address, which means VECEXIT must be a
;	24 bit address.
;	————————————————————————————————————————————————————————————————————————
;
;	————————————————————————————————————————————————————————————————————————
vecbrki  =brk_vector                ;BRK handler indirect vector...
;
;	Supermon 816 will modify this vector so that execution of a BRK instruc-
;	tion is intercepted & the registers  are  captured.   Your BRK front end
;	should jump through this vector after pushing the registers as follows:
;
;	         phb                   ;save DB
;	         phd                   ;save DP
;	         rep #%00110000        ;16 bit registers
;	         pha
;	         phx
;	         phy
;	         jmp (vecbrki)         ;indirect vector
;
;	When a G or J command is issued, the above sequence will be reversed be-
;	fore a jump is made to the code to be executed.  Upon exit from Supermon
;	816, the original address at VECBRKI will be restored.  Note that this
;	vector must be in bank $00.
;
;	If your BRK front end doesn't conform to the above you will have to mod-
;	ify Supermon 816 to accommodate the differences.  The most likely needed
;	changes will be in the order in which registers are pushed to the stack.
;	————————————————————————————————————————————————————————————————————————
;
;	————————————————————————————————————————————————————————————————————————
hwstack  =$0001ff              ;top of hardware stack...
;
;	Supermon 816 initializes the stack pointer to this address when the cold
;	start at JMON is called to enter the monitor.  The stack pointer will be
;	undisturbed when entry into Supermon 816 is through JMONBRK.
;
;	JMON & JMONBRK are defined in the Supermon 816's jump table.
;	————————————————————————————————————————————————————————————————————————
;
;	————————————————————————————————————————————————————————————————————————
zeropage =$00                  ;Supermon 816's direct page...
;
;	Supermon 816 uses direct page starting at this address.  Be sure that no
;	conflict occurs with other software, as an overwrite of any of these
;	locations may be fatal to Supermon 816.
;	————————————————————————————————————————————————————————————————————————
;
;	————————————————————————————————————————————————————————————————————————
getcha   =c64_getcha              ;get datum from TIA-232 channel A...

;	GETCHA refers to an operating system API call that returns a datum in
;	the 8 bit accumulator.  Supermon 816 assumes that GETCHA is a non-block-
;	ing subroutine & returns with carry clear to indicate that a datum is in
;	.A, or with carry set to indicate that no datum was available.  GETCHA
;	will be called with a JSR instruction.
;
;	Supermon 816 expects .X & .Y to be preserved upon return from GETCHA.
;	You may have to modify Supermon 816 at all calls to GETCHA if your "get
;	datum" routine works differently than described.
;	————————————————————————————————————————————————————————————————————————
;
;	————————————————————————————————————————————————————————————————————————
getchb   =c64_getchb              ;get datum from TIA-232 channel B...
;
;	GETCHB refers to an operating system API call that returns a datum in
;	the 8 bit accumulator.  Supermon 816 assumes that GETCHB is a non-block-
;	ing subroutine & returns with carry clear to indicate that a datum is in
;	.A, or with carry set to indicate that no datum was available.  GETCHB
;	will be called with a JSR instruction.
;
;	Supermon 816 expects .X & .Y to be preserved upon return from GETCHB.
;	You may have to modify Supermon 816 at all calls to GETCHB if your "get
;	datum" routine works differently than described.
;	————————————————————————————————————————————————————————————————————————
;
;	————————————————————————————————————————————————————————————————————————
putcha   =c64_putcha              ;print character on console...
;
;	PUTCHA refers to an operating system API call that prints a character to
;	the console screen.  The character to be printed will be in .A, which
;	will be set to 8-bit width.  Supermon 816 assumes that PUTCHA will block
;	until the character can be processed.  PUTCHA will be called with a JSR
;	instruction.
;
;	Supermon 816 expects .X & .Y to be preserved upon return from PUTCHA.
;	You may have to modify Supermon 816 at all calls to PUTCHA if your "put
;	character" routine works differently than described.
;	————————————————————————————————————————————————————————————————————————
;
;	————————————————————————————————————————————————————————————————————————
chanbctl =c64_chanbctl              ;TIA-232 channel B control...
;
;	CHANBCTL refers to an operating system API call that enables or disables
;	the TIA-232 channel B receiver.  If this call is not present in the tar-
;	get system's API then it will be necessary to comment out references to
;	it.  CHANBCTL is a BCS Technology Limited NXP driver feature.  Refer to
;	the function header comments for details.
;	————————————————————————————————————————————————————————————————————————
;
;	————————————————————————————————————————————————————————————————————————
stopkey  =$03                  ;display abort key...
;
;	Supermon 816 will poll for a "stop key" during display operations, such
;	as code disassembly & memory dumps, so as to abort further processing &
;	return to the command prompt.  STOPKEY must be defined with the ASCII
;	value that the "stop key" will emit when typed.  The polling is via a
;	call to GETCHA (described above).  The default STOPKEY definition of $03
;	is for ASCII <ETX> or [Ctrl-C].  An alternative definition could be $1B,
;	which is ASCII <ESC> or [ESC].
;	————————————————————————————————————————————————————————————————————————
;
ibuffer  =$006000              ;input buffer &...
auxbuf   =ibuffer+s_ibuf+s_byte ;auxiliary buffer...
;
;	———————————————————————————————————————————————————————————————————————
;	Supermon 816 will use the above definitions for workspace in various
;	ways.  These buffers may be located anywhere in RAM that is convenient.
;	The buffers are stateless, which means that unless Supermon 816 has
;	control of your system they may be overwritten without consequence.
;	———————————————————————————————————————————————————————————————————————
;
;===============================================================================
;
;ASCII CONTROL DEFINITIONS (menmonic order)
;
a_bel    =$07                  ;<BEL> alert/ring bell
a_bs     =$08                  ;<BS>  backspace
a_cr     =$0d                  ;<CR>  carriage return
a_del    =$7f                  ;<DEL> delete
a_esc    =$1b                  ;<ESC> escape
a_ht     =$09                  ;<HT>  horizontal tabulation
a_lf     =$0a                  ;<LF>  linefeed
;
;
;	miscellaneous (description order)...
;
a_blank  =' '                  ;blank (whitespace)
a_asclch ='z'                  ;end of lowercase ASCII
a_lctouc =%01011111            ;LC to UC conversion mask
a_asclcl ='a'                  ;start of lowercase ASCII
;
;===============================================================================
;
;GLOBAL ATOMIC CONSTANTS
;
;
;	data type sizes...
;
s_byte   =1                    ;byte
s_word   =2                    ;word (16 bits)
s_xword  =3                    ;extended word (24 bits)
s_dword  =4                    ;double word (32 bits)
s_rampag =$0100                ;65xx RAM page
;
;
;	data type sizes in bits...
;
s_bibyte =8                    ;byte
s_bnybbl =4                    ;nybble
;
;
;	miscellaneous...
;
bitabs   =$2c                  ;absolute BIT opcode
bitzp    =$24                  ;zero page BIT opcode
;
;===============================================================================
;
;W65C816S NATIVE MODE STATUS REGISTER DEFINITIONS
;
s_mpudbx =s_byte               ;data bank size
s_mpudpx =s_word               ;direct page size
s_mpupbx =s_byte               ;program bank size
s_mpupcx =s_word               ;program counter size
s_mpuspx =s_word               ;stack pointer size
s_mpusrx =s_byte               ;status size
;
;
;	status register flags...
;
sr_car   =%00000001            ;C
sr_zer   =sr_car << 1          ;Z
sr_irq   =sr_zer << 1          ;I
sr_bdm   =sr_irq << 1          ;D
sr_ixw   =sr_bdm << 1          ;x
sr_amw   =sr_ixw << 1          ;m
sr_ovl   =sr_amw << 1          ;V
sr_neg   =sr_ovl << 1          ;N
;
;	NVmxDIZC
;	xxxxxxxx
;	||||||||
;	|||||||+———> 1 = carry set/generated
;	||||||+————> 1 = result = zero
;	|||||+—————> 1 = IRQs ignored
;	||||+——————> 0 = binary arithmetic mode
;	||||         1 = decimal arithmetic mode
;	|||+———————> 0 = 16 bit index
;	|||          1 = 8 bit index
;	||+————————> 0 = 16 bit .A & memory
;	||           1 = 8 bit .A & memory
;	|+—————————> 1 = sign overflow
;	+——————————> 1 = result = negative
;
;
;	register size masks (used with REP & SEP)...
;
m_seta   =sr_amw               ;accumulator/memory
m_setx   =sr_ixw               ;index
m_setr   =m_seta|m_setx        ;accumulator/memory & index
;
;===============================================================================
;
;"SIZE-OF" CONSTANTS
;
s_addr   =s_xword              ;24 bit address
s_auxbuf =32                   ;auxiliary buffer
s_ibuf   =69                   ;input buffer
s_mnemon =3                    ;MPU ASCII mnemonic
s_mnepck =2                    ;MPU encoded mnemonic
s_mvinst =3                    ;MVN/MVP instruction
s_opcode =s_byte               ;MPU opcode
s_oper   =s_xword              ;operand
s_pfac   =s_dword              ;primary math accumulator
s_sfac   =s_dword+s_word       ;secondary math accumulators
;
;===============================================================================
;
;"NUMBER-OF" CONSTANTS
;
n_dbytes =21                   ;default disassembly bytes
n_dump   =8                    ;bytes per memory dump line (fits C64 40 columns)
n_mbytes =s_rampag-1           ;default memory dump bytes
n_hccols =4                    ;compare/hunt columns (fits C64 40 columns)
n_opcols =3*s_oper             ;disassembly operand columns
n_opslsr =4                    ;LSRs to extract instruction size
n_shfenc =5                    ;shifts to encode/decode mnemonic
;
;===============================================================================
;
;NUMERIC CONVERSION CONSTANTS
;
a_hexdec ='A'-'9'-2            ;hex to decimal difference
c_bin    ='%'                  ;binary prefix
c_dec    ='+'                  ;decimal prefix
c_hex    ='$'                  ;hexadecimal prefix
c_oct    ='@'                  ;octal prefix
k_hex    ='f'                  ;hex ASCII conversion
m_bits   =s_pfac*s_bibyte      ;operand bit size
m_cbits  =s_sfac*s_bibyte      ;workspace bit size
bcdumask =%00001111            ;isolate BCD units mask
btoamask =%00110000            ;binary to ASCII mask
;
;===============================================================================
;
;ASSEMBLER/DISASSEMBLER CONSTANTS
;
a_mnecvt ='?'                  ;encoded mnemonic conversion base
aimmaska =%00011111            ;.A immediate opcode test #1
aimmaskb =%00001001            ;.A immediate opcode test #2
asmprfx  ='A'                  ;assemble code prefix
ascprmct =9                    ;assembler prompt "size-of"
disprfx  ='.'                  ;disassemble code prefix
flim_a   =%10000000            ;16-bit .A/immediate state
flim_x   =%01000000            ;16-bit .X/.Y immediate state
flimmask =flim_a|flim_x        ;both immediate-width state bits
opc_cpxi =$e0                  ;CPX # opcode
opc_cpyi =$c0                  ;CPY # opcode
opc_ldxi =$a2                  ;LDX # opcode
opc_ldyi =$a0                  ;LDY # opcode
opc_mvn  =$54                  ;MVN opcode
opc_mvp  =$44                  ;MVP opcode
opc_rep  =$c2                  ;REP opcode
opc_sep  =$e2                  ;SEP opcode
pfmxmask =sr_amw | sr_ixw      ;MPU m & x flag bits mask
;
;
;	assembler prompt buffer offsets...
;
apadrbkh =s_word               ;instruction address bank MSN
apadrbkl =apadrbkh+s_byte      ;instruction address bank LSN
apadrmbh =apadrbkl+s_byte      ;instruction address MSB MSN
apadrmbl =apadrmbh+s_byte      ;instruction address MSB LSN
apadrlbh =apadrmbl+s_byte      ;instruction address LSB MSN
apadrlbl =apadrlbh+s_byte      ;instruction address LSB LSN
;
;
;	addressing mode preamble symbols...
;
amp_imm  ='#'                  ;immediate
amp_ind  ='('                  ;indirect
amp_indl ='['                  ;indirect long
;
;
;	addressing mode symbolic translation indices...
;
am_nam   =%0000                ;no symbol
am_imm   =%0001                ;#
am_adrx  =%0010                ;<addr>,X
am_adry  =%0011                ;<addr>,Y
am_ind   =%0100                ;(<addr>)
am_indl  =%0101                ;[<dp>]
am_indly =%0110                ;[<dp>],Y
am_indx  =%0111                ;(<addr>,X)
am_indy  =%1000                ;(<dp>),Y
am_stk   =%1001                ;<offset>,S
am_stky  =%1010                ;(<offset>,S),Y
am_move  =%1011                ;#<sbnk>,#<dbnk>
;
;
;	operand size translation indices...
;
ops0     =%0000 << 4           ;no operand
ops1     =%0001 << 4           ;8 bit operand
ops2     =%0010 << 4           ;16 bit operand
ops3     =%0011 << 4           ;24 bit operand
bop1     =%0101 << 4           ;8 bit relative branch
bop2     =%0110 << 4           ;16 bit relative branch
vops     =%1001 << 4           ;8 or 16 bit operand
;
;
;	operand size & addressing mode extraction masks...
;
amodmask =%00001111            ;addressing mode index
opsmask  =%00110000            ;operand size
vopsmask =%11000000            ;BOPx & VOPS flag bits
;
;
;	instruction mnemonic encoding...
;
mne_adc  =$2144                ;ADC
mne_and  =$2bc4                ;AND
mne_asl  =$6d04                ;ASL
mne_bcc  =$2106                ;BCC
mne_bcs  =$a106                ;BCS
mne_beq  =$9186                ;BEQ
mne_bit  =$aa86                ;BIT
mne_bmi  =$5386                ;BMI
mne_bne  =$33c6                ;BNE
mne_bpl  =$6c46                ;BPL
mne_bra  =$14c6                ;BRA
mne_brk  =$64c6                ;BRK
mne_brl  =$6cc6                ;BRL
mne_bvc  =$25c6                ;BVC
mne_bvs  =$a5c6                ;BVS
mne_clc  =$2348                ;CLC
mne_cld  =$2b48                ;CLD
mne_cli  =$5348                ;CLI
mne_clv  =$bb48                ;CLV
mne_cmp  =$8b88                ;CMP
mne_cop  =$8c08                ;COP
mne_cpx  =$cc48                ;CPX
mne_cpy  =$d448                ;CPY
mne_dec  =$218a                ;DEC
mne_dex  =$c98a                ;DEX
mne_dey  =$d18a                ;DEY
mne_eor  =$9c0c                ;EOR
mne_inc  =$23d4                ;INC
mne_inx  =$cbd4                ;INX
mne_iny  =$d3d4                ;INY
mne_jml  =$6b96                ;JML
mne_jmp  =$8b96                ;JMP
mne_jsl  =$6d16                ;JSL
mne_jsr  =$9d16                ;JSR
mne_lda  =$115a                ;LDA
mne_ldx  =$c95a                ;LDX
mne_ldy  =$d15a                ;LDY
mne_lsr  =$9d1a                ;LSR
mne_mvn  =$7ddc                ;MVN
mne_mvp  =$8ddc                ;MVP
mne_nop  =$8c1e                ;NOP
mne_ora  =$14e0                ;ORA
mne_pea  =$11a2                ;PEA
mne_pei  =$51a2                ;PEI
mne_per  =$99a2                ;PER
mne_pha  =$1262                ;PHA
mne_phb  =$1a62                ;PHB
mne_phd  =$2a62                ;PHD
mne_phk  =$6262                ;PHK
mne_php  =$8a62                ;PHP
mne_phx  =$ca62                ;PHX
mne_phy  =$d262                ;PHY
mne_pla  =$1362                ;PLA
mne_plb  =$1b62                ;PLB
mne_pld  =$2b62                ;PLD
mne_plp  =$8b62                ;PLP
mne_plx  =$cb62                ;PLX
mne_ply  =$d362                ;PLY
mne_rep  =$89a6                ;REP
mne_rol  =$6c26                ;ROL
mne_ror  =$9c26                ;ROR
mne_rti  =$5566                ;RTI
mne_rtl  =$6d66                ;RTL
mne_rts  =$a566                ;RTS
mne_sbc  =$20e8                ;SBC
mne_sec  =$21a8                ;SEC
mne_sed  =$29a8                ;SED
mne_sei  =$51a8                ;SEI
mne_sep  =$89a8                ;SEP
mne_sta  =$1568                ;STA
mne_stp  =$8d68                ;STP
mne_stx  =$cd68                ;STX
mne_sty  =$d568                ;STY
mne_stz  =$dd68                ;STZ
mne_tax  =$c8aa                ;TAX
mne_tay  =$d0aa                ;TAY
mne_tcd  =$292a                ;TCD
mne_tcs  =$a12a                ;TCS
mne_tdc  =$216a                ;TDC
mne_trb  =$1cea                ;TRB
mne_tsb  =$1d2a                ;TSB
mne_tsc  =$252a                ;TSC
mne_tsx  =$cd2a                ;TSX
mne_txa  =$166a                ;TXA
mne_txs  =$a66a                ;TXS
mne_txy  =$d66a                ;TXY
mne_tya  =$16aa                ;TYA
mne_tyx  =$ceaa                ;TYX
mne_wai  =$50b0                ;WAI
mne_wdm  =$7170                ;WDM
mne_xba  =$10f2                ;XBA
mne_xce  =$3132                ;XCE
;
;
;	encoded instruction mnemonic indices...
;
mne_adcx =16                   ;ADC
mne_andx =29                   ;AND
mne_aslx =44                   ;ASL
mne_bccx =15                   ;BCC
mne_bcsx =65                   ;BCS
mne_beqx =59                   ;BEQ
mne_bitx =70                   ;BIT
mne_bmix =36                   ;BMI
mne_bnex =31                   ;BNE
mne_bplx =42                   ;BPL
mne_brax =5                    ;BRA
mne_brkx =39                   ;BRK
mne_brlx =43                   ;BRL
mne_bvcx =23                   ;BVC
mne_bvsx =68                   ;BVS
mne_clcx =20                   ;CLC
mne_cldx =27                   ;CLD
mne_clix =35                   ;CLI
mne_clvx =71                   ;CLV
mne_cmpx =53                   ;CMP
mne_copx =55                   ;COP
mne_cpxx =78                   ;CPX
mne_cpyx =88                   ;CPY
mne_decx =18                   ;DEC
mne_dexx =74                   ;DEX
mne_deyx =84                   ;DEY
mne_eorx =61                   ;EOR
mne_incx =21                   ;INC
mne_inxx =77                   ;INX
mne_inyx =87                   ;INY
mne_jmlx =40                   ;JML
mne_jmpx =54                   ;JMP
mne_jslx =45                   ;JSL
mne_jsrx =63                   ;JSR
mne_ldax =1                    ;LDA
mne_ldxx =73                   ;LDX
mne_ldyx =83                   ;LDY
mne_lsrx =64                   ;LSR
mne_mvnx =48                   ;MVN
mne_mvpx =58                   ;MVP
mne_nopx =56                   ;NOP
mne_orax =6                    ;ORA
mne_peax =2                    ;PEA
mne_peix =33                   ;PEI
mne_perx =60                   ;PER
mne_phax =3                    ;PHA
mne_phbx =10                   ;PHB
mne_phdx =26                   ;PHD
mne_phkx =38                   ;PHK
mne_phpx =51                   ;PHP
mne_phxx =75                   ;PHX
mne_phyx =85                   ;PHY
mne_plax =4                    ;PLA
mne_plbx =11                   ;PLB
mne_pldx =28                   ;PLD
mne_plpx =52                   ;PLP
mne_plxx =76                   ;PLX
mne_plyx =86                   ;PLY
mne_repx =49                   ;REP
mne_rolx =41                   ;ROL
mne_rorx =62                   ;ROR
mne_rtix =37                   ;RTI
mne_rtlx =46                   ;RTL
mne_rtsx =67                   ;RTS
mne_sbcx =14                   ;SBC
mne_secx =19                   ;SEC
mne_sedx =25                   ;SED
mne_seix =34                   ;SEI
mne_sepx =50                   ;SEP
mne_stax =7                    ;STA
mne_stpx =57                   ;STP
mne_stxx =80                   ;STX
mne_styx =89                   ;STY
mne_stzx =91                   ;STZ
mne_taxx =72                   ;TAX
mne_tayx =82                   ;TAY
mne_tcdx =24                   ;TCD
mne_tcsx =66                   ;TCS
mne_tdcx =17                   ;TDC
mne_trbx =12                   ;TRB
mne_tsbx =13                   ;TSB
mne_tscx =22                   ;TSC
mne_tsxx =79                   ;TSX
mne_txax =8                    ;TXA
mne_txsx =69                   ;TXS
mne_txyx =90                   ;TXY
mne_tyax =9                    ;TYA
mne_tyxx =81                   ;TYX
mne_waix =32                   ;WAI
mne_wdmx =47                   ;WDM
mne_xbax =0                    ;XBA
mne_xcex =30                   ;XCE
;
;===============================================================================
;
;MISCELLANEOUS CONSTANTS
;
halftab  =4                    ;1/2 tabulation spacing
m_rectyp ='9'-'0'              ;S-record type range
memprfx  ='>'                  ;memory dump prefix
memsepch =':'                  ;memory dump separator
memsubch ='.'                  ;memory dump non-print char
srecpre  ='S'                  ;S-record preamble char
srinit   =%00000100            ;SR initialization value
;
;===============================================================================
;
;DIRECT PAGE STORAGE
;
reg_pbx  =zeropage             ;PB shadow
reg_pcx  =reg_pbx+s_mpupbx     ;PC shadow
reg_srx  =reg_pcx+s_mpupcx     ;SR shadow
reg_ax   =reg_srx+s_mpusrx     ;.C shadow
reg_xx   =reg_ax+s_word        ;.X shadow
reg_yx   =reg_xx+s_word        ;.Y shadow
reg_spx  =reg_yx+s_word        ;SP shadow
reg_dpx  =reg_spx+s_mpuspx     ;DP shadow
reg_dbx  =reg_dpx+s_mpudpx     ;DB shadow
;
;
;	general workspace...
;
addra    =reg_dbx+s_mpudbx     ;address #1
addrb    =addra+s_addr         ;address #2
faca     =addrb+s_addr         ;primary accumulator
facax    =faca+s_pfac          ;extended primary accumulator
facb     =facax+s_pfac         ;secondary accumulator
facc     =facb+s_sfac          ;tertiary accumulator
operand  =facc+s_sfac          ;instruction operand
auxbufix =operand+s_oper       ;auxiliary buffer index
ibufidx  =auxbufix+s_byte      ;input buffer index
bitsdig  =ibufidx+s_byte       ;bits per numeral
numeral  =bitsdig+s_byte       ;numeral buffer
radix    =numeral+s_byte       ;radix index
admodidx =radix+s_byte         ;addressing mode index
charcnt  =admodidx+s_byte      ;character counter
instsize =charcnt+s_word       ;instruction size
mnepck   =instsize+s_word      ;encoded mnemonic
opcode   =mnepck+s_mnepck      ;current opcode
status   =opcode+s_byte        ;I/O status flag
xrtemp   =status+s_byte        ;temp .X storage
eopsize  =xrtemp+s_byte        ;entered operand size
flimflag =eopsize+s_byte       ;64tass immediate-width state...
;
;	xx000000
;	||
;	|+—————————> 0: .XS (.X/.Y immediate = 8 bits)
;	|            1: .XL (.X/.Y immediate = 16 bits)
;	+——————————> 0: .AS (.A/BIT immediate = 8 bits)
;	             1: .AL (.A/BIT immediate = 16 bits)
;
;	————————————————————————————————————————————————————————————————————————
;	During assembly, FLIMFLAG holds the persistent .AS/.AL and .XS/.XL state.
;	During disassembly it also mirrors the effect of REP and SEP instructions.
;	————————————————————————————————————————————————————————————————————————
;
iopsize  =flimflag+s_byte      ;operand size
range    =iopsize+s_byte       ;allowable radix range
vopsflag =range+s_byte         ;VOPS & ROPS mode bits
getcharps=vopsflag+s_byte      ;GETCHAR's reconstructed caller status
getcharv =getcharps+s_byte     ;GETCHAR's returned character
;
;
;	copy/fill workspace (overlaps some of the above)...
;
mcftwork =faca                 ;start of copy/fill code
mcftopc  =mcftwork+s_byte      ;instruction opcode
mcftbnk  =mcftopc+s_byte       ;banks
;
;
;	S-record load workspace (overlaps some of the above)...
;
endaddr  =addrb                ;load ending address
loadaddr =facc                 ;current record load address
loadbank =loadaddr+s_word      ;current record load bank
strtaddr =addra                ;load starting address
;
datax    =auxbufix             ;datum holding area
cksumcmp =mnepck               ;computed record checksum
nrecs    =eopsize              ;processed S1 record count
pageoff  =iopsize              ;record load page offset
reclen   =range                ;record length
rectype  =vopsflag             ;record type
;
;===============================================================================
;
;C64 CONSOLE DISPLAY CONTROL MACROS
;
bs       .macro                ;destructive backspace
         .byte $14
         .endm
;
cl       .macro                ;clear-to-EOL is not available through CHROUT
         .endm
;
cn       .macro                ;solid reverse-block cursor, then cursor-left
         .byte $12,$20,$92,$9d
         .endm
;
co       .macro                ;erase software cursor, then cursor-left
         .byte $92,$20,$9d
         .endm
;
cr       .macro
         .byte $0d
         .endm
;
lf       .macro                ;C64 RETURN advances to the next line
         .byte $0d
         .endm
;
bf       .macro                ;reverse video on
         .byte $12
         .endm
;
sf       .macro                ;reverse video off
         .byte $92
         .endm
;
rb       .macro                ;PETSCII bell
         .byte $07
         .endm
;
;===============================================================================
;
;SUPERMON 816 JUMP TABLE
;
;===============================================================================
;
;C64 BASIC LOADER AND SUPERCPU NATIVE-MODE INTERFACE
;
* = $0801
         .word c64_basic_end
         .word 10
         .byte $9e
         .text "2061"
         .byte 0
c64_basic_end:
         .word 0
;
; 10 SYS2061
;
c64_entry:
         lda $d0bc
         bpl c64_scpu_present
         ldx #0
c64_no_scpu_loop:
         lda c64_no_scpu,x
         beq c64_no_scpu_done
         jsr $ffd2
         inx
         bne c64_no_scpu_loop
c64_no_scpu_done:
         rts
;
c64_scpu_present:
         sei
         lda $0001
         sta c64_old_port
         lda #$35              ;RAM at $A000-$FFFF, I/O visible
         sta $0001
         lda $ffe6
         sta c64_old_brk
         lda $ffe7
         sta c64_old_brk+1
         lda #<c64_brk_frontend
         sta $ffe6
         lda #>c64_brk_frontend
         sta $ffe7
         clc
         xce                   ;enter 65816 native mode
         rep #$30
         .al
         .xl
         pea #$0200            ;monitor direct page; keeps C64 ZP intact
         pld                   ;avoid depending on TCD across SCPU implementations
         phk
         plb                   ;DB = bank 0
         sep #$30
         .as
         .xs
         jml JMON
;
; Native BRK vector front end expected by the Supermon core.
c64_brk_frontend:
         phb
         phd
         rep #$30
         .al
         .xl
         pha
         phx
         phy
         pea #$0200
         pld
         jmp (brk_vector)
;
; GETIN wrapper. SCNKEY makes input independent of the IRQ keyboard scan.
; Run SCNKEY only once per VIC frame: calling it in Supermon's 20 MHz polling
; loop makes KERNAL-repeatable keys (notably Space) fill the input buffer from
; a single physical key press.
c64_getcha:
         php
         rep #$30
         .al
         .xl
         phx
         phy
         phb
         phd
         pea #0
         pld
         phk
         plb
         sep #$30
         .as
         .xs
         lda #$37
         sta $0001
         lda $d011
         bmi c64_getcha_not_frame0
         lda $d012
         bne c64_getcha_not_frame0
         lda c64_scan_latch
         bne c64_getcha_scanned
         inc c64_scan_latch
         jsr $ff9f             ;SCNKEY, once at raster line 0
         bra c64_getcha_scanned
c64_getcha_not_frame0:
         stz c64_scan_latch
c64_getcha_scanned:
         jsr $ffe4             ;GETIN
         sta c64_char
         lda #$35
         sta $0001
         rep #$30
         .al
         .xl
         pld
         pea #$0200            ;the monitor always owns this direct page
         pld
         plb
         ply
         plx
         plp
         .as
         .xs
         lda c64_char
         beq c64_getcha_empty
         cmp #$14              ;PETSCII DELETE -> ASCII backspace
         bne c64_getcha_ready
         lda #$08
c64_getcha_ready:
         clc
         rts
c64_getcha_empty:
         sec
         rts
;
c64_getchb:
         jmp c64_getcha        ;S-record input uses the keyboard on this port
;
; CHROUT wrapper. Convert the ASCII controls/case used by the monitor.
c64_putcha:
         sta c64_char
         cmp #$09
         bne c64_putcha_not_ht
         lda #' '              ;ASCII tab has no C64 screen-editor equivalent
         bra c64_putcha_mapped
c64_putcha_not_ht:
         cmp #$0a
         bne c64_putcha_not_lf
         lda #$0d
         bra c64_putcha_mapped
c64_putcha_not_lf:
         cmp #$08
         bne c64_putcha_not_bs
         lda #$14
         bra c64_putcha_mapped
c64_putcha_not_bs:
         cmp #'a'
         bcc c64_putcha_mapped
         cmp #'z'+1
         bcs c64_putcha_mapped
         and #$df              ;ASCII lower case -> C64 upper case
c64_putcha_mapped:
         sta c64_char
         php
         rep #$30
         .al
         .xl
         phx
         phy
         phb
         phd
         pea #0
         pld
         phk
         plb
         sep #$30
         .as
         .xs
         lda #$37
         sta $0001
         stz $d4               ;never inherit C64 screen-editor quote mode
         lda c64_char
         jsr $ffd2             ;CHROUT
         stz $d4               ;a printed quote must not leave quote mode set
         lda #$35
         sta $0001
         rep #$30
         .al
         .xl
         pld
         pea #$0200            ;the monitor always owns this direct page
         pld
         plb
         ply
         plx
         plp
         .as
         .xs
         lda c64_char
         rts
;
c64_chanbctl:
         rts
;
c64_exit:
         sei
         pea #0
         pld                   ;CPU port is direct-page address $01
         rep #$20
         .al
         lda c64_old_brk
         sta $ffe6
         sep #$20
         .as
         lda c64_old_port
         sta $0001
         sep #$30
         .as
         .xs
         sec
         xce                   ;return to 6502 emulation mode
         cli
         jmp $a474             ;BASIC warm start
;
brk_vector:  .word 0
c64_old_brk: .word 0
c64_old_port:.byte 0
c64_char:    .byte 0
c64_scan_latch:.byte 0
c64_no_scpu: .byte 13
             .text "REQUIRES A SUPERCPU"
             .byte 13,0
;
         *=origin_addr
;
JMON     bra mon               ;cold start entry
JMONBRK  bra monbrk            ;software interrupt intercept
;
vecbrkia .word 0               ;system indirect BRK vector
;
;===============================================================================
;
;mon: SUPERMON 816 COLD START
;
mon      pea #$0200            ;make cold entry independent of caller DP state
         pld
         rep #m_seta           ;16-bit .A
         .al
         lda vecbrki           ;BRK vector
         cmp #jmonbrk         ;pointing at monitor?
         beq monreg            ;yes, ignore cold start
;
         sta vecbrkia          ;save vector for exit
         lda #jmonbrk         ;Supermon 816 intercepts...
         sta vecbrki           ;BRK handler
         sep #m_setr           ;8 bit registers
         .as
         .xs
         ldx #vopsflag-reg_pbx
;
_0000010 stz reg_pbx,x         ;clear DP storage
         dex
         bpl _0000010
;
;
;	initialize register shadows...
;
         lda #srinit
         sta reg_srx           ;status register
         rep #m_seta      
         .al
         lda #hwstack         ;top of hardware stack
         tcs                   ;set SP...
;
;			—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-
;			The stack pointer initialization is an optional step
;			that could be removed if Supermon 816 is made part
;			of system firware.
;			—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-
;
         tdc                   ;get & save DP...
         sta reg_dpx           ;into shadow register
         lda #0               ;flush .B
         sep #m_seta        
         .as
         phk
         pla                   ;capture PB &...
         sta reg_pbx           ;set
         phb
         pla                   ;capture DB &...
         sta reg_dbx           ;set
;
;
;	print startup banner...
;
         pea #mm_entry         ;"Supermon 816..."
         bra moncom
;
;===============================================================================
;
;monbrk: SOFTWARE INTERRUPT INTERCEPT
;
;	————————————————————————————————————————————————————————————————————————
;	This is the entry point taken when a BRK instruction is executed.  It is
;	assumed that the BRK  handler has pushed the registers to the stack that
;	are not automatically pushed by the MPU in response to BRK.  See the
;	documentation for what Supermon 816 expects to find on the stack follow-
;	ing a BRK.
;	————————————————————————————————————————————————————————————————————————
;
monbrk   pea #$0200            ;restore monitor direct page
         pld
;
;	——————————————————————————————————————————————————————————————————————
;	Uncomment the above instructions to set a definition for the monitor's
;	direct page location.  If left commented, the value in DP will be used
;	as is.
;	——————————————————————————————————————————————————————————————————————
;         
         ply                   ;recover registers
         plx
         pla
         rep #m_setr           ;store 16 bit registers
         .al
         .xl
         sta reg_ax            ;.A
         stx reg_xx            ;.X
         sty reg_yx            ;.Y
         sep #m_setx           ;8 bit index registers
         .xs
         pla                   ;get DP &...
         sta reg_dpx           ;store
         plx                   ;get DB &...
         stx reg_dbx           ;store
         plx                   ;get SR &...
         stx reg_srx           ;store
         pla                   ;get PC &...
         sta reg_pcx           ;store
         sep #m_seta
         .as
         pla                   ;get PB &...
         sta reg_pbx           ;store
         cli                   ;reenable IRQs
         pea #mm_brk           ;"*BRK"
;
;===============================================================================
;
;moncom: COMMON ENTRY POINT
;
;	——————————————————————————————————————
;	DO NOT directly call this entry point!
;	——————————————————————————————————————
;
moncom   jsr sprint            ;print heading
         .as                   ;SPRINT returns with 8-bit A/X/Y
         .xs
         rep #m_seta
         .al
         tsc                   ;get SP &...
         sta reg_spx           ;store
         rep #%11111111        ;clear SR &...
         .al
         .xl
         sep #srinit | sr_car  ;set default state...
;
;			—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—
;			The above implicitly sets carry.  See
;			next step for why carry must be set.
;			—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—
;
;===============================================================================
;
;monreg: DISPLAY MPU REGISTERS
;
;	—————————
;	syntax: R
;	—————————
;
monreg   bcs _0000010          ;okay to proceed
;
         jmp monerr            ;error if called with a parm
;
_0000010 pea #mm_regs
         jsr sprint            ;display register heading
         .as
         .xs
;
;
;	display program bank & counter...
;
         lda reg_pbx           ;PB
         jsr dpyhex            ;display as hex ASCII
         jsr printspc          ;inter-field space
         rep #m_seta
         .al
         lda reg_pcx
         sep #m_seta
         .as
         jsr dpyhexw           ;display PC
         ldx #2
         jsr multspc           ;inter-field spacing
;
;
;	display SR in bitwise fashion...
;
         ldx reg_srx           ;SR
         ldy #s_bibyte         ;bits in a byte
;
_0000020 txa                   ;remaining SR bits
         asl a                   ;grab one of them
         tax                   ;save remainder
         lda #'0'              ;a clear bit but...
         adc #0                ;adjust if set &...
         jsr putcha            ;print
         dey                   ;bit processed
         bne _0000020          ;do another
;
;
;	display .C, .X, .Y, SP & DP...
;
_0000030 jsr printspc          ;spacing
         rep #m_seta
         .al
         lda reg_ax,y          ;get register value
         sep #m_seta
         .as
         jsr dpyhexw           ;convert & display
	.rept s_word
         iny
	.endrept
         cpy #reg_dbx-reg_ax
         bcc _0000030          ;next
;
;
;	display DB...
;
         jsr printspc          ;more spacing
         lda reg_dbx           ;get DB, ...
         jsr dpyhex            ;display it & enter executive
;
;===============================================================================
;
;monce: COMMAND EXECUTIVE
;	
monce    sep #m_seta           ;8-bit .A
         .as
         lda #0                ;default buffer index
;
moncea   sep #m_setr           ;alternate entry point
         .as
         .xs
         sta ibufidx           ;(re)set buffer index
         pea #mm_prmpt
         jsr sprint            ;display input prompt
         .as
         .xs
         jsr input             ;await some input
;
_0000010 jsr getcharc          ;read from buffer
         beq monce             ;terminator, just loop
;
         cmp #a_blank
         beq _0000010          ;strip leading blanks
;
         ldx #n_mpctab-1       ;number of primary commands
;
_0000020 cmp mpctab,x          ;search primary command list
         bne _0000030
;
         txa                   ;get index
         asl a                   ;double for offset
         tax
         rep #m_seta
         .al
         lda mpcextab,x        ;command address -1
         pha                   ;prime the stack
         sep #m_seta
         .as
         jmp getparm           ;evaluate parm & execute command
;
_0000030 dex
         bpl _0000020          ;continue searching primary commands
;
         ldx #n_radix-1        ;number of radices
;
_0000040 cmp radxtab,x         ;search conversion command list
         bne _0000050
;
         jmp monenv            ;convert & display parameter
;
_0000050 dex
         bpl _0000040
;
;===============================================================================
;
;monerr: COMMON ERROR HANDLER
;
monerr   sep #m_setr           ;8 bit registers
         .as
         .xs
;
monerraa jsr dpyerr            ;indicate an error &...
         bra monce             ;return to input loop
;
;===============================================================================
;
;monasc: ASSEMBLE CODE
;
;	———————————————————————————————————————————————————————————————————————
;	syntax: A <addr> <mnemonic> [<argument>]
;
;	After a line of code has been successfully assembled it will be disass-
;	embled & displayed,  & the monitor will prompt with the next address to
;	which code may be assembled.
;	———————————————————————————————————————————————————————————————————————
;
monasc   bcc _0000020          ;assembly address entered
;
_0000010 jmp monerr            ;terminate w/error
;
;
;	evaluate assembly address...
;
_0000020 jsr facasize          ;check address...
         cmp #s_dword          ;range
         bcs _0000010          ;out of range — error
;
         jsr facaddra          ;store assembly address
;
;
;	initialize workspace...
;
         ldx #s_auxbuf-s_byte
;
_0000030 stz auxbuf,x          ;clear addressing mode buffer
         dex
         bne _0000030
;
         lda #a_blank
         sta auxbuf            ;preamble placeholder
         jsr clroper           ;clear operand
         stz auxbufix          ;reset addressing mode index
         stz mnepck            ;clear encoded...
         stz mnepck+s_byte     ;mnemonic workspace
         stz vopsflag          ;clear 8/16 or relative flag
;
;
;	encode mnemonic...
;
         ldy #s_mnemon         ;expected mnemonic size
;
_0000040 jsr getcharw          ;get from buffer wo/whitespace
         bne _0000060          ;gotten
;
         cpy #s_mnemon         ;any input at all?
         bcc _0000050          ;yes
;
         jmp monce             ;no, abort further assembly
;
_0000050 jmp monasc10          ;incomplete mnemonic — error
;
_0000060 cmp #'.'              ;64tass register-width directive?
         bne _0000070
         cpy #s_mnemon         ;directive must replace the mnemonic
         bne _0000050
         jmp monascwd
;
_0000070 sec
         sbc #a_mnecvt         ;ASCII to binary factor
         ldx #n_shfenc         ;shifts required to encode
;
_0000080 lsr a                 ;shift out a bit...
         ror mnepck+s_byte     ;into...
         ror mnepck            ;encoded mnemonic
         dex
         bne _0000080          ;next bit
;
         dey
         bne _0000040          ;get next char
         bra monascmne
;
; Accept 64tass's persistent immediate-width directives as assembler entries:
; .AS/.AL control accumulator/BIT immediates; .XS/.XL control index immediates.
monascwd jsr getcharc
         cmp #'A'
         beq _0000090
         cmp #'X'
         bne monascwderr
         lda #flim_x
         bra _0000100
;
_0000090 lda #flim_a
;
_0000100 sta xrtemp            ;width-state bit to update
         jsr getcharc
         cmp #'L'
         beq _0000110
         cmp #'S'
         bne monascwderr
         jsr getcharw          ;nothing may follow the directive
         bne monascwderr
         lda xrtemp
         eor #$ff
         and flimflag
         sta flimflag
         jmp monasc09          ;prompt again at the same address
;
_0000110 jsr getcharw
         bne monascwderr
         lda xrtemp
         ora flimflag
         sta flimflag
         jmp monasc09
;
monascwderr jmp monasc10
;
;
;	test for copy instruction...
;	————————————————————————————————————————————————————————————————————————
;	The MVN & MVP instructions accept two operands & hence have an irregular
;	syntax.  Therefore, special handling is necessary to assemble either of
;	these instructions.
;
;	As in 64tass's canonical listing form, operands are 8-bit immediate bank
;	values written as MVN #<source>,#<destination> (and likewise for MVP).
;	————————————————————————————————————————————————————————————————————————
;
monascmne rep #m_seta          ;16 bit load
         .al
         lda mnepck            ;packed menmonic
         ldx #opc_mvn          ;MVN opcode
         cmp #mne_mvn         ;is it MVN?
         beq monasc01          ;yes
;
         ldx #opc_mvp          ;MVP opcode
         cmp #mne_mvp         ;is it MVP?
         bne monasc02          ;no
;
;
;	assemble copy instruction...
;
monasc01 stx opcode            ;store relevant opcode
         sep #m_seta
         .as
         jsr instdata          ;get instruction data
         stx eopsize           ;effective operand size
         inx
         stx instsize          ;instruction size
         ldx #s_oper-s_word    ;operand index
         stx xrtemp            ;set it
;
_0000010 jsr getcharw          ;optional 64tass immediate-bank marker
         cmp #amp_imm
         beq _0000015
         dec ibufidx           ;legacy spelling without '#'
_0000015 jsr ascbin            ;evaluate bank
         bcs monasc04          ;conversion error
;
         beq monasc04          ;nothing returned — error
;
         jsr facasize          ;bank must be...
         cmp #s_word           ;8 bits
         bcs monasc04          ;it isn't — error
;
         lda faca              ;bank
         ldx xrtemp            ;operand index
         sta operand,x         ;store
         dec xrtemp            ;index=index-1
         bpl _0000010          ;get destination bank
;
         jsr getcharr          ;should be no more input
         bne monasc04          ;there is — error
;
         jmp monasc08          ;finish MVN/MVP assembly 
;
;
;	continue with normal assembly...
;
monasc02 sep #m_seta           ;back to 8 bits
         .as
;
monasc03 jsr getcharw          ;get next char
         beq monasc06          ;EOI, no argument
;
_0000010 cmp #amp_imm          ;immediate mode?
         beq _0000020          ;yes
;
         cmp #amp_ind          ;indirect mode?
         beq _0000020          ;yes
;
         cmp #amp_indl         ;indirect long mode?
         bne _0000030          ;no
;
_0000020 sta auxbuf            ;set addressing mode preamble
         inc auxbufix          ;bump aux buffer index &...
         bra _0000040          ;evaluate operand
;
_0000030 dec ibufidx           ;position back to char
;
_0000040 jsr ascbin            ;evaluate operand
         bne monasc05          ;evaluated
;
         bcs monasc04          ;conversion error
;
         lda auxbufix          ;no operand...any preamble?
         beq monasc06          ;no, syntax is okay so far
;
monasc04 jmp monasc10          ;abort w/error
;
monasc05 jsr facasize          ;size operand
         cmp #s_dword          ;max is 24 bits
         bcs monasc04          ;too big
;
         sta eopsize           ;save operand size
         jsr facaoper          ;store operand
;
monasc06 dec ibufidx           ;back to last char
         ldx auxbufix          ;mode buffer index
         bne _0000010          ;preamble in buffer
;
         inx                   ;step past preamble position
;
_0000010 jsr getcharc          ;get a char w/forced UC
         beq _0000030          ;EOI
;
         cpx #s_auxbuf         ;mode buffer full?
         bcs monasc04          ;yes, too much input
;
_0000020 sta auxbuf,x          ;store for comparison
         inx
         bne _0000010
;
;
;	evaluate mnemonic...
;
_0000030 ldx #n_mnemon-1       ;starting mnemonic index
;
monasc07 txa                   ;convert index...
         asl a                   ;to offset
         tay                   ;now mnemonic table index
         rep #m_seta           ;16 bit compare
         .al
         lda mnetab,y          ;get mnemonic from table
         cmp mnepck            ;compare to entered mnemonic
         sep #m_seta           ;back to 8 bits
         .as
         beq _0000020          ;match
;
_0000010 dex                   ;try next mnemonic
         bmi monasc04          ;unknown mnemonic — error
;
         bra monasc07          ;keep going
;
_0000020 stx mnepck            ;save mnemonic index
         txa
         ldx #0                ;trial opcode
;
_0000030 cmp mnetabix,x        ;search index table...
         beq _0000050          ;for a match
;
_0000040 inx                   ;keep going until we...
         bne _0000030          ;search entire table
;
         bra monasc04          ;this shouldn't happen!
;
;	—————————————————————————————————————————————————————————————————————
;	If the mnemonic index table search fails then there is a coding error
;	somewhere, as every entry in the mnemonic table is supposed to have a
;	matching cardinal index.
;	—————————————————————————————————————————————————————————————————————
;
;
;	evaluate addressing mode...
;
_0000050 stx opcode            ;save trial opcode
         jsr instdata          ;get related instruction data
         sta vopsflag          ;save 8/16 or relative flag
         stx iopsize           ;operand size
         inx
         stx instsize          ;instruction size
         ldx opcode            ;recover trial opcode
         tya                   ;addressing mode
         asl a                   ;create table index
         tay
         rep #m_seta
         .al
         lda ms_lutab,y        ;mode lookup table
         sta addrb             ;set pointer
         sep #m_seta
         .as
         ldy #0
;
_0000060 lda (addrb),y         ;table addressing mode
         cmp auxbuf,y          ;entered addressing mode
         beq _0000080          ;okay so far
;
_0000070 lda mnepck            ;reload mnemonic index        
         bra _0000040          ;wrong opcode for addresing mode
;
_0000080 ora #0                ;last char the terminator?
         beq _0000090          ;yes, evaluate operand
;
         iny
         bra _0000060          ;keep testing
;
;
;	evaluate operand...
;
_0000090 lda eopsize           ;entered operand size
         bne _0000100          ;non-zero
;
         ora iopsize           ;instruction operand size
         bne _0000070          ;wrong opcode — keep trying
;
         bra monasc08          ;assemble instruction
;
_0000100 bit vopsflag          ;is this a branch?
         bvs _0000160          ;yes, evaluate
;
         lda iopsize           ;instruction operand size
         bit vopsflag          ;variable size operand allowed?
         bmi _0000130          ;yes
;
_0000110 cmp eopsize           ;entered operand size
         bcc _0000070          ;operand too big
;
         sta eopsize           ;new operand size
         bra monasc08          ;assemble, otherwise...
;
_0000120 cmp eopsize           ;exact size match required
         bne _0000070          ;mismatch — wrong opcode
;
         bra monasc08          ;assemble
;
;
;	process variable-size immediate mode operand...
;
_0000130 ldx eopsize           ;entered operand size
         cpx #s_xword          ;check size
         bcs monasc10          ;too big — error
;
         lda opcode            ;choose 64tass .A or .X/.Y width state
         ldy #n_vopidx-1
_0000135 cmp vopidx,y
         beq _0000140
         dey
         bpl _0000135
;
         lda flimflag          ;.AS/.AL state
         and #flim_a
         bra _0000145
;
_0000140 lda flimflag          ;.XS/.XL state
         and #flim_x
;
_0000145 beq _0000150
         ldy #s_word           ;long state always emits a word
         bra _0000155
;
_0000150 ldy #s_byte           ;short state requires a byte value
;
_0000155 cpy eopsize
         bcc _0000070          ;value too large for selected width
         sty eopsize           ;promote small values in long mode
;
         bra monasc08          ;okay, assemble
;
;
;	process relative branch...
;
_0000160 jsr targoff           ;compute branch offset
         bcs monasc10          ;branch out of range
;
         sta eopsize           ;effective operand size
;
;
;	assemble instruction...
;
monasc08 lda opcode            ;opcode
         sta [addra]           ;store at assembly address
         ldx eopsize           ;any operand to process?
         beq _0000020          ;no
;
         txy                   ;also storage offset
;
_0000010 dex
         lda operand,x         ;get operand byte &...
         sta [addra],y         ;poke into memory
         dey
         bne _0000010          ;next
;
_0000020 lda #a_cr
         jsr putcha            ;return to left margin
         lda #asmprfx          ;assembly prefix
         jsr dpycodaa          ;disassemble & display
;
;
;	prompt for next instruction...
;
monasc09 lda #a_blank
         ldx #ascprmct-1
;
_0000010 sta ibuffer,x         ;prepare buffer for...
         dex                   ;next instruction
         bpl _0000010
;
         lda #asmprfx          ;assemble code...
         sta ibuffer           ;prompt prefix
         lda addra+s_word      ;next instruction address bank
         jsr binhex            ;convert to ASCII
         sta ibuffer+apadrbkh  ;store MSN in buffer
         stx ibuffer+apadrbkl  ;store LSN in buffer
         lda addra+s_byte      ;next instruction address MSB
         jsr binhex
         sta ibuffer+apadrmbh
         stx ibuffer+apadrmbl
         lda addra             ;next instruction address LSB
         jsr binhex
         sta ibuffer+apadrlbh
         stx ibuffer+apadrlbl
         lda #ascprmct         ;effective input count
         jmp moncea            ;reenter input loop
;
;
;	process assembly error...
;
monasc10 jsr dpyerr            ;indicate error &...
         bra monasc09          ;prompt w/same assembly address
;
;===============================================================================
;
;mondsc: DISASSEMBLE CODE
;
;	—————————————————————————————
;	syntax: D [<addr1> [<addr2>]]
;	—————————————————————————————
;
mondsc   bcs _0000010          ;no parameters
;
         stz flimflag          ;reset to 8 bit mode
         jsr facasize          ;check starting...
         cmp #s_dword          ;address
         bcs _0000050          ;out of range — error
;
         jsr facaddra          ;copy starting address
         jsr getparm           ;get ending address
         bcc _0000020          ;gotten
;
_0000010 jsr clrfaca           ;clear accumulator
         rep #m_seta
         .al
         clc
         lda addra             ;starting address
         adc #n_dbytes        ;default bytes
         sta faca              ;effective ending address
         sep #m_seta
         .as
         lda addra+s_word      ;starting bank
         adc #0
         sta faca+s_word       ;effective ending bank
         bcs _0000050          ;end address > $FFFFFF
;
_0000020 jsr facasize          ;check ending...
         cmp #s_dword          ;address
         bcs _0000050          ;out of range — error
;
         jsr facaddrb          ;set ending address
         jsr getparm           ;check for excess input
         bcc _0000050          ;present — error
;
         jsr calccnt           ;calculate bytes
         bcc _0000050          ;end < start
;
_0000030 jsr teststop          ;test for display stop
         bcs _0000040          ;stopped
;
         jsr newline           ;next line
         jsr dpycod            ;disassemble & display
         jsr decdcnt           ;decrement byte count
         bcc _0000030          ;not done
;
_0000040 jmp monce             ;back to main loop
;
_0000050 jmp monerr            ;address range error
;
;===============================================================================
;
;monjmp: EXECUTE CODE
;
;	—————————————————————————————————————————————————————————————
;	syntax: G [<dp>]
;
;	If no address is specified, the current values in the PB & PC
;	shadow registers are used.
;	—————————————————————————————————————————————————————————————
;
monjmp   jsr setxaddr          ;set execution address
         bcs monjmpab          ;out of range — error
;
         jsr getparm           ;check for excess input
         bcc monjmpab          ;too much input — error
;
         rep #m_seta           ;16 bit .A
         .al
         lda reg_spx
         tcs                   ;restore SP
;
monjmpaa sep #m_seta
         .as
         lda reg_pbx
         pha                   ;restore PB
         rep #m_seta
         .al
         lda reg_pcx
         pha                   ;restore PC
         sep #m_seta
         .as
         lda reg_srx
         pha                   ;restore SR
         lda reg_dbx
         pha
         plb                   ;restore DB
         rep #m_setr
         .al
         .xl
         lda reg_dpx
         tcd                   ;restore DP
         lda reg_ax            ;restore .C
         ldx reg_xx            ;restore .X
         ldy reg_yx            ;restore .Y
         rti                   ;execute code
;
monjmpab jmp monerr            ;error
;
;===============================================================================
;
;monjsr: EXECUTE CODE AS SUBROUTINE
;
;	————————————————————————————————————————————————————————————
;	syntax: J [<dp>]
;
;	If no address is specified the current values in the PB & PC
;	shadow registers are used.   An RTS at the end of the called
;	subroutine will return control to the monitor,  provided the
;	stack remains in balance.
;	————————————————————————————————————————————————————————————
;
monjsr   jsr setxaddr          ;set execution address
         bcs monjmpab          ;out of range — error
;
         jsr getparm           ;check for excess input
         bcc monjmpab          ;too much input — error
;
         rep #m_seta
         .al
         lda reg_spx
         tcs                   ;restore SP &...
         jsr monjmpaa          ;call subroutine
         php                   ;push SR
         rep #m_setr
         .al
         .xl
         sta reg_ax            ;save...
         stx reg_xx            ;register...
         sty reg_yx            ;returns
         sep #m_setx           ;8 bit .X & .Y
         .xs
         plx                   ;get & save...
         stx reg_srx           ;return SR
         tsc                   ;get & save...
         sta reg_spx           ;return SP
         tdc                   ;get & save...
         sta reg_dpx           ;DP pointer
         sep #m_seta           ;8 bit .A
         .as
         phk                   ;get &...
         pla                   ;save...
         sta reg_pbx           ;return PB
         phb                   ;get &...
         pla                   ;save...
         sta reg_dbx           ;return DB
         pea #mm_rts           ;"*RET"
         jmp moncom            ;return to monitor
;
;===============================================================================
;
;monchm: CHANGE and/or DUMP MEMORY
;
;	————————————————————————————————————————————
;	syntax: > [<addr> <operand> [<operand>]...]
;
;	> <addr> without operands will dump 16 bytes
;	of memory, starting at <addr>.
;	————————————————————————————————————————————
;
monchm   bcs _0000030          ;no address given — quit
;
         jsr facasize          ;size address
         cmp #s_dword
         bcs _0000040          ;address out of range — error
;
         jsr facaddra          ;set starting address
         jsr getpat            ;evaluate change pattern
         bcc _0000010          ;entered
;
         bpl _0000020          ;not entered
;
         bra _0000040          ;evaluation error
;
_0000010 dey                   ;next byte
         bmi _0000020          ;done
;
         lda auxbuf,y          ;write pattern...
         sta [addra],y         ;to memory
         bra _0000010          ;next
;
_0000020 jsr newline           ;next line
         jsr dpymem            ;regurgitate changes
;
_0000030 jmp monce             ;back to command loop
;
_0000040 jmp monerr            ;goto error handler
;
;===============================================================================
;
;moncmp: COMPARE MEMORY
;
;	—————————————————————————————
;	syntax: C <start> <end> <ref>
;	—————————————————————————————
;
moncmp   bcs _0000030          ;start not given — quit
;
         jsr enddest           ;get end & reference addresses
         bcs _0000040          ;range or other error
;
         stz xrtemp            ;column counter
;
_0000010 jsr teststop          ;check for stop
         bcs _0000030          ;abort
;
         lda [addra]           ;get from reference location
         cmp [operand]         ;test against compare location
         beq _0000020          ;match, don't display address
;
         jsr dpycaddr          ;display current location
;
_0000020 jsr nxtaddra          ;next reference location
         bcs _0000030          ;done
;
         rep #m_seta
         .al
         inc operand           ;bump bits 0-15
         sep #m_seta
         .as
         bne _0000010
;
         inc operand+s_word    ;bump bits 16-23
         bra _0000010
;
_0000030 jmp monce             ;return to command exec
;
_0000040 jmp monerr            ;goto error handler
;
;===============================================================================
;
;moncpy: COPY (transfer) MEMORY
;
;	————————————————————————————————
;	syntax: T <start> <end> <target>
;	————————————————————————————————
;
moncpy   bcs _0000040          ;start not given — quit
;
         jsr enddest           ;get end & target addresses
         bcs _0000050          ;range or other error
;
         rep #m_seta
         .al
         sec
         lda addrb             ;ending address
         sbc addra             ;starting address
         bcc _0000050          ;start > end — error
;
         sta facb              ;bytes to copy
         sep #m_seta
         .as
         rep #m_setx
         .xl
         lda operand+s_word    ;target bank
         ldy operand           ;target address
         cmp addra+s_word      ;source bank
         rep #m_seta
         .al
         bne _0000020          ;can use forward copy
;
         cpy addra             ;source address
         bcc _0000020          ;can use forward copy
;
         bne _0000010          ;must use reverse copy
;
         bra _0000050          ;copy in place — error
;
_0000010 lda facb              ;get bytes to copy
         pha                   ;protect
         jsr lodbnk            ;load banks
         jsr cprvsup           ;do reverse copy setup
         pla                   ;get bytes to copy
         tax                   ;save a copy
         clc
         adc operand           ;change target to...
         tay                   ;target end
         txa                   ;recover bytes to copy
         ldx addrb             ;source end
         bra _0000030
;
_0000020 lda facb              ;get bytes to copy
         pha                   ;protect
         jsr lodbnk            ;load banks
         jsr cpfwsup           ;do forward copy setup
         pla                   ;get bytes to copy
         ldx addra             ;source start
;
_0000030 jmp mcftwork          ;copy memory
;
_0000040 jmp monce             ;back to executive
;
_0000050 jmp monerr            ;error
;
;===============================================================================
;
;mondmp: DISPLAY MEMORY RANGE
;
;	—————————————————————————————
;	syntax: M [<addr1> [<addr2>]]
;	—————————————————————————————
;
mondmp   sep #m_setr           ;command parser returns with 8-bit A/X/Y
         .as
         .xs
         bcs _0000010          ;no parameters
;
         jsr facasize          ;check address...
         cmp #s_dword          ;range
         bcs _0000050          ;address out of range
;
         jsr facaddra          ;copy starting address
         jsr getparm           ;get ending address
         bcc _0000020          ;gotten
;
_0000010 jsr clrfaca           ;clear accumulator
         rep #m_seta
         .al
         clc
         lda addra             ;starting address
         adc #n_mbytes        ;default bytes
         sta faca              ;effective ending address
         sep #m_seta
         .as
         lda addra+s_word      ;starting bank
         adc #0
         sta faca+s_word       ;effective ending bank
         bcs _0000050          ;end address > $FFFFFF
;
_0000020 jsr facasize          ;check ending address...
         cmp #s_dword          ;range
         bcs _0000050          ;out of range — error
;
         jsr facaddrb          ;copy ending address
         jsr getparm           ;check for excess input
         bcc _0000050          ;error
;
         jsr calccnt           ;calculate bytes to dump
         bcc _0000050          ;end < start
;
         jsr newline           ;move below the entered command once
_0000030 jsr teststop          ;test for display stop
         bcs _0000040          ;stopped
;
         jsr dpymem            ;display
         jsr decdcnt           ;decrement byte count
         bcc _0000030          ;not done
;
; A full eight-byte row is exactly 40 characters and CHROUT has already moved
; to column zero of the following row.  Only a partial final row needs CR.
         lda charcnt
         cmp #n_dump
         beq _0000040
         jsr newline
;
_0000040 jmp monce             ;back to main loop
;
_0000050 jmp monerr            ;address range error
;
;===============================================================================
;
;monfil: FILL MEMORY
;
;	—————————————————————————————————————————
;	syntax: F <start> <end> <fill>
;
;	<start> & <end> must be in the same bank.
;	—————————————————————————————————————————
;
monfil   bcs _0000010          ;start not given — quit
;
         jsr facasize          ;check size
         cmp #s_dword
         bcs _0000020          ;out of range — error...
;
         jsr facaddra          ;store start
         jsr getparm           ;evaluate end
         bcs _0000020          ;not entered — error
;
         jsr facasize          ;check size
         cmp #s_dword
         bcs _0000020          ;out of range — error
;
         lda faca+s_word       ;end bank
         cmp addra+s_word      ;start bank
         bne _0000020          ;not same — error
;
         jsr facaddrb          ;store <end>
         rep #m_seta
         .al
         sec
         lda addrb             ;ending address
         sbc addra             ;starting address
         bcc _0000020          ;start > end — error
;
         sta facb              ;bytes to copy
         sep #m_seta
         .as
         jsr getparm           ;evaluate <fill>
         bcs _0000020          ;not entered — error
;
         jsr facasize          ;<fill> should be...
         cmp #s_word           ;8 bits
         bcs _0000020          ;it isn't — error
;
         jsr facaoper          ;store <fill>
         jsr getparm           ;should be no more parameters
         bcc _0000020          ;there are — error
;
         lda operand           ;<fill>
         sta [addra]           ;fill 1st location
         rep #m_setr           ;16 bit operations
         .al
         .xl
         lda facb              ;get byte count
         beq _0000010          ;only 1 location — finished
;
         dec a                   ;zero align &...
         pha                   ;protect
         sep #m_seta
         .as
         lda addra+s_word      ;start bank
         xba
         lda addrb+s_word      ;end bank
         jsr cpfwsup           ;do forward copy setup
         pla                   ;recover fill count
         ldx addra             ;fill-from starting location
         txy
         iny                   ;fill-to starting location
         jmp mcftwork          ;fill memory
;
_0000010 jmp monce             ;goto command executive
;
_0000020 jmp monerr            ;goto error handler
;
;===============================================================================
;
;monlsr: LOAD S-RECORD CODE
;
;	————————————————————————————————————————————————————————————————————————
;	syntax: L [<bank> [<offset>]]
;
;	<bank> is the 64KB RAM bank into which code is to be loaded.  If not
;	specified the bank value stored in the MPU's PB shadow register is used.
;	On completion of the load, the PB shadow register will be set to <bank>.
;
;	<offset> is an optional load address page offset.  This offset will be
;	added to each record's starting load address MSB to produce the effect-
;	ive load address.  Address wrap will occur if the sum of the load add-
;	ress MSB & page offset is greater than $FF—the bank address will not
;	wrap.  <bank> must be specified if <offset> is required.
;
;
;	Data Input Requirements
;	—-—-—-—-—-—-—-—-—-—-—-—
;	This function translates Motorola S-records into binary values that are
;	loaded into RAM.  If a non-zero page offset is specified, the load add-
;	ress information in each record will be modified by the offset value,
;	thus permitting a relocating load to a new starting page boundary.
;
;	The data stream is read from TIA-232 channel B until an S9 record has
;	been received & processed without error.  Pressing [Ctrl-C] as a load
;	progresses will immediately abort the process, which will result in a
;	partial & most likely unusable load.
;
;	The end of each record (EOR) is demarcated by an <LF> (UNIX newline) or
;	a <CR><LF> sequence (MS-DOS newline).  A missing EOR demarcation will
;	cause a buffer overrun error & prematurely halt the load.
;
;	The following S-record types are recognized:
;
;	    Type  Description
;	    —-—-—-—-—-—-—-—-—-—
;	      0   Block header
;	      1   Loadable data
;	      5   Record count
;	      9   Termination
;	    —-—-—-—-—-—-—-—-—-—
;
;	The general format of an S-record is as follows:
;
;	    <preamble> <type> <length> <address> <data> <checksum>
;
;	Excepting the preamble & type characters, data in each field is repre-
;	sented by pairs of ASCII characters that are textual equivalents of hex-
;	adecimal values.  For example, "1A" means the binary value $1A or deci-
;	mal 26.  The preamble & type characters taken together are used to des-
;	cribe the record type: S0, S1, S5 or S9.
;
;	Field details are as follows:
;
;	    Field     Size  Description
;	    —-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-
;	    preamble    1   a literal "S" ASCII character
;
;	    type        1   record type, indicated by a literal ASCII numeral
;
;	    length      2   record length, equal to the number of bytes encoded
;	                    in the <address>, <data> & <checksum> fields
;
;	    address     4   S0: always "0000"
;	                    S1: record load address
;	                    S5: number of transmitted S1 records (64K max)
;	                    S9: start address for code (optional)
;
;	    data        *   S0: descriptive information (optional)
;	                *   S1: data to be loaded into RAM, starting at the add-
;	                        ress in the <address> field
;	                0   S5: not used
;	                0   S9: not used
;
;	    checksum    2   8 bit ones complement computed from the <length>,
;	                    <address> & <data> fields
;	    —-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-
;
;	Sizes are in ASCII characters.  A size indicated by * will vary depend-
;	ing on the data in the field.  Values in the <address> field are in big-
;	endian format.  Hence "C4" followed by "21" in the <address> field means
;	the binary value $C421 or decimal 50209.
;
;	This function will attempt to flush all input from TIA-232 channel B be-
;	fore commencing the load.  A data stream should not be sent until the
;	"Ready" prompt appears.
;	————————————————————————————————————————————————————————————————————————
;
monlsr   bcc _0000010          ;bank entered
;
         lda reg_pbx           ;use current bank...
         bra _0000040          ;as default
;
_0000010 jsr facasize          ;check arg size
         cmp #s_word
         bcc _0000030          ;bank in range
;
_0000020 jmp monerr            ;abort w/error
;
_0000030 lda faca              ;entered bank
;
_0000040 sta loadbank          ;set load bank
         stz pageoff           ;reset page offset
         jsr getparm           ;check for page offset
         bcs _0000050          ;none entered
;
         jsr facasize          ;check size
         cmp #s_word
         bcs _0000020          ;out of range
;
         lda faca              ;entered page offset
         sta pageoff           ;save it
         jsr getparm           ;should be no more input
         bcc _0000020          ;error
;
_0000050 rep #m_seta
         .al
         stz nrecs             ;reset S1 record count
         sep #m_setr
         .as
         .xs
         sec
         jsr chanbctl          ;enable channel B receiver
         ldx #20
;
_0000060 wai                   ;flush...
         jsr clrchanb          ;channel B
         dex
         bne _0000060
;
         pea #lr_rtl           ;"Ready"
         jsr sprint            ;print string
         .as
         .xs
;
;	————————————————————
;	MAIN PROCESSING LOOP
;	————————————————————
;
monlsr01 ldx #0                ;buffer index
         ldy #s_ibuf           ;input counter
         stx cksumcmp          ;reset computed...
         stx cksumcmp+s_byte   ;checksum
;
;
;	acquire record...
;
_0000010 jsr getchanb          ;get datum from channel B
         bcc _0000020          ;gotten
;
         jmp monlsr07          ;aborted by user
;
_0000020 beq monlsr02          ;EOL, process record
;
         sta ibuffer,x         ;store datum
         inx                   ;bump index
         dey                   ;decrement counter
         bne _0000010          ;buffer not full, get next
;
         pea #lr_bufor         ;"buffer overrun..."
         jmp monlsr06          ;abort w/error
;
monlsr02 stz ibuffer,x         ;terminate input
         stz ibufidx           ;reset buffer index
         lda nrecs             ;total S1 records LSB
         ora nrecs+s_byte      ;total S1 records MSB
         bne _0000010          ;not 1st record
;
         pea #lr_load          ;"loading..."
         jsr sprint
         .as
         .xs
;
;
;	evaluate record preamble...
;
_0000010 jsr getchar           ;get 1st char from record
         cmp #srecpre          ;preamble char
         bne monlsr03          ;error
;
         jsr getchar           ;get record type
         beq monlsr03          ;unexpected end — error
;
         sec
         sbc #'0'              ;change to binary
         cmp #m_rectyp+1       ;check range
         bcs monlsr03          ;out of range — error
;
         sta rectype           ;set record type
;
;
;	evaluate record length...
;
         jsr getcvt            ;get & convert length
         bcs monlsr03          ;error
;
         tax
	.rept s_word          ;deduct address length
         dex
	.endrept
         stx reclen            ;save record length
         jsr acksum            ;add to checksum
;
;
;	evaluate load address...
;
         jsr getcvt            ;get & convert address MSB
         bcs monlsr03          ;error
;
         sta loadaddr+s_byte   ;save load address MSB
         jsr acksum            ;add to checksum
         jsr getcvt            ;get & convert address LSB
         bcs monlsr03          ;error
;
         sta loadaddr          ;save load address LSB
         jsr acksum            ;add to checksum
;
;
;	evaluate data field...
;
         ldx #0                ;buffer index
         ldy reclen            ;record length
;
_0000020 jsr getcvt            ;get & convert
         bcs monlsr03          ;premature record end
;
         dey                   ;len -= 1
         beq monlsr04          ;.A = checksum byte
;
         sta ibuffer,x         ;store converted byte
         jsr acksum            ;add to checksum
         inx
         bra _0000020          ;get next
;
monlsr03 pea #lr_corup         ;"corrupted record..."
         jmp monlsr06
;
monlsr04 sta datax             ;save record checksum
         lda cksumcmp          ;running checksum
         eor #%11111111        ;get ones complement
         cmp datax             ;check against record checksum
         bne monlsr03          ;bad checksum — error
;
         lda rectype           ;record type
         asl a                   ;generate index
         tax
         jmp (lsrextab,x)      ;process record type
;
;
;	process S0 record...
;
;	—————————————————————————————————
;	S0 data segment is not evaluated.
;	—————————————————————————————————
;
monlsrs0 bra monlsr05          ;get next record
;
;
;	process S1 record...
;
monlsrs1 jsr adjloadr          ;adjust load address
         rep #m_seta
         .al
         lda nrecs             ;any records processed?
         bne _0000010          ;yes
;
         lda loadaddr          ;load address
         sta strtaddr          ;also start address
;
_0000010 sep #m_setr
         .as
         .xs
         dec reclen            ;bytes to copy
         ldx #0                ;buffer index
;
_0000020 lda ibuffer,x         ;get byte &...
         sta [loadaddr]        ;write to RAM
         dec reclen            ;all bytes copied?
         beq _0000030          ;yes
;
         rep #m_seta           ;no
         .al
         inc loadaddr          ;bump load address
         sep #m_seta
         .as
         inx                   ;bump index
         bne _0000020          ;next
;
_0000030 rep #m_seta
         .al
         lda loadaddr
         sta endaddr           ;set working end address
         inc nrecs             ;bump processed record count
         sep #m_seta
         .as
         lda #'.'              ;progress indicator
         jsr putcha            ;print on console
;
monlsr05 sep #m_setr
         .as
         .xs
         jmp monlsr01          ;get next record
;
;
;	process S5 record...
;
monlsrs5 dec reclen            ;data field should be empty
         bne monlsr03          ;it isn't — error
;
         rep #m_seta
         .al
         lda loadaddr          ;expected S1 record count
         cmp nrecs             ;processed S1 record count
         beq monlsr05          ;count okay — continue
;
         pea #lr_recct         ;"invalid S1 count..."
         bra monlsr06          ;abort w/error
;
;
;	process S9 record...
;
monlsrs9 dec reclen            ;data field should be empty
         bne monlsr03          ;it isn't — error
;
         rep #m_seta
         .al
         lda nrecs             ;total S1 records
         bne _0000010          ;at least 1 record loaded
;
         pea #lr_norec         ;"no records..."
         bra monlsr06          ;abort w/error
;
_0000010 sep #m_seta
         .as
         lda loadbank          ;load bank
         sta reg_pbx           ;default for execution
         jsr adjloadr          ;adjust load address
         rep #m_seta
         .al
         lda loadaddr          ;load address from S9
         bne _0000020          ;valid
;
         lda strtaddr          ;initial load address
;
_0000020 sta reg_pcx           ;set execution address
         sep #m_seta
         .as
         pea #lr_addrs         ;"start address..."
         jsr sprint
         .as
         .xs
         jsr dpypbr            ;display bank
         rep #m_seta
         .al
         lda strtaddr          ;initial load address
         sep #m_seta
         .as
         jsr dpyhexw           ;display start address
         pea #lr_addre         ;"end address"
         jsr sprint
         .as
         .xs
         jsr dpypbr            ;display bank
         rep #m_seta
         .al
         lda endaddr           ;final load address
         sep #m_seta
         .as
         jsr dpyhexw           ;display end address
         bra monlsr07          ;back to monitor executive
;
;
;	undefined record type error handler...
;
monlsrut pea #lr_type          ;"invalid..."
;
;
;	general error processing...
;
monlsr06 sep #m_setr
         .as
         .xs
         jsr sprint            ;print error message
         .as
         .xs
;
;
;	terminate S-record processing...
;
monlsr07 clc
         jsr chanbctl          ;disable channel B port
         jmp monce             ;back to monitor executive
;
;===============================================================================
;
;monhnt: SEARCH (hunt) MEMORY
;
;	———————————————————————————————————
;	syntax: H <addr1> <addr2> <pattern>
;	———————————————————————————————————
;
monhnt   bcs _0000050          ;no start address
;
         jsr facasize          ;size starting address
         cmp #s_dword
         bcs _0000060          ;address out of range — error
;
         jsr facaddra          ;store starting address
         jsr getparm           ;evaluate ending address
         bcs _0000060          ;no address — error
;
         jsr facasize          ;size ending address
         cmp #s_dword
         bcs _0000060          ;address out of range — error
;
         jsr facaddrb          ;store ending address
         jsr calccnt           ;calculate byte range
         bcc _0000060          ;end < start
;
         jsr getpat            ;evaluate search pattern
         bcs _0000060          ;error
;
         stz xrtemp            ;clear column counter
;
_0000010 jsr teststop          ;check for stop
         bcs _0000050          ;abort
;
         ldy auxbufix          ;pattern index
;
_0000020 dey
         bmi _0000030          ;pattern match
;
         lda [addra],y         ;get from memory
         cmp auxbuf,y          ;test against pattern
         bne _0000040          ;mismatch, next location
;
         beq _0000020          ;match, keep testing
;
_0000030 jsr dpycaddr          ;display current location
;
_0000040 jsr nxtaddra          ;next location
         bcc _0000010          ;not done
;
_0000050 jmp monce             ;back to executive
;
_0000060 jmp monerr            ;goto error handler
;
;===============================================================================
;
;monenv: CONVERT NUMERIC VALUE
;
;	——————————————————————
;	syntax: <radix><value>
;	——————————————————————
;
monenv   jsr getparmr          ;reread & evaluate parameter
         bcs _0000020          ;none entered
;
         ldx #0                ;radix index
         ldy #n_radix          ;number of radices
;
_0000010 phy                   ;save counter
         phx                   ;save radix index
         jsr newline           ;next line &...
         jsr clearlin          ;clear it
         lda #a_blank
         ldx #halftab
         jsr multspc           ;indent 1/2 tab
         plx                   ;get radix index but...
         phx                   ;put it back
         lda radxtab,x         ;get radix
         jsr binasc            ;convert to ASCII
         phy                   ;string address MSB
         phx                   ;string address LSB
         jsr sprint            ;print
         .as
         .xs
         plx                   ;get index again
         ply                   ;get counter
         inx
         dey                   ;all radices handled?
         bne _0000010          ;no

_0000020 jmp monce             ;back to command exec
;
;===============================================================================
;
;monchr: CHANGE REGISTERS
;
;	——————————————————————————————————————————————————————
;	syntax: ; [PB [PC [.S [.C [.X [.Y [SP [DP [DB]]]]]]]]]
;
;	; with no parameters is the same as the R command.
;	——————————————————————————————————————————————————————
;
monchr   bcs _0000040          ;dump registers & quit
;
         ldy #0                ;register counter
         sty facc              ;initialize register index
;
_0000010 jsr facasize          ;get parameter size
         cmp rcvltab,y         ;check against size table
         bcs _0000050          ;out of range
;
         lda rcvltab,y         ;determine number of bytes...
         cmp #s_word+1         ;to store
         ror facc+s_byte       ;condition flag
         bpl _0000020          ;8 bit register size
;
         rep #m_seta           ;16 bit register size
         .al
;
_0000020 ldx facc              ;get register index
         lda faca              ;get parm
         sta reg_pbx,x         ;put in shadow storage
         sep #m_seta
         .as
         asl facc+s_byte       ;mode flag to carry
         txa                   ;register index
         adc #s_byte           ;at least 1 byte stored
         sta facc              ;save new index
         jsr getparm           ;get a parameter
         bcs _0000040          ;EOI
;
         iny                   ;bump register count
         cpy #n_regchv         ;all registers processed?
         bne _0000010          ;no, keep going
;
_0000030 jsr alert             ;excessive input
;
_0000040 jmp monreg            ;display changes
;
_0000050 jmp monerr            ;goto error handler
;
;===============================================================================
;
;monxit: EXIT TO OPERATING ENVIRONMENT
;
;	—————————
;	syntax: X
;	—————————
;
monxit   bcc _0000020          ;no parameters allowed
;
         rep #m_seta
         .al
         lda vecbrki           ;BRK indirect vector
         cmp #jmonbrk         ;we intercept it?
         bne _0000010          ;no, don't change it
;
         lda vecbrkia          ;old vector
         sta vecbrki           ;restore it
         stz vecbrkia          ;invalidate old vector
;         
_0000010 sep #m_setr
         .as
         .xs
         jml vecexit           ;long jump to exit
;
_0000020 jmp monerr            ;goto error handler
;
;================================================
; * * * * * * * * * * * * * * * * * * * * * * * *
; * * * * * * * * * * * * * * * * * * * * * * * *
; * *                                         * *
; * * S T A R T   o f   S U B R O U T I N E S * *
; * *                                         * *
; * * * * * * * * * * * * * * * * * * * * * * * *
; * * * * * * * * * * * * * * * * * * * * * * * *
;================================================
;
;acksum: ACCUMULATE S-RECORD CHECKSUM
;
;	————————————————————————————————————————————
;	Preparatory Ops: .A: byte to add to checksum
;
;	Returned Values: .A: entry value
;	                 .B: entry value
;	                 .X: entry value
;	                 .Y: entry value
;	————————————————————————————————————————————
;
acksum   pha                   ;protect
         rep #sr_amw|sr_car    ;16 bit .A + clear carry
         .al
         and #%11111111       ;clear .B
         adc cksumcmp          ;checksum
         sta cksumcmp          ;new checksum LSB
         sep #m_seta
         .as
         pla
         rts
;
;===============================================================================
;
;dpycaddr: DISPLAY CURRENT ADDRESS IN COLUMNS
;
dpycaddr ldx xrtemp            ;column count
         bne _0000010          ;not at right side
;
         jsr newline           ;next row
         ldx #n_hccols         ;max columns
;
_0000010 cpx #n_hccols         ;max columns
         beq _0000020          ;at left margin
;
         lda #a_ht
         jsr putcha            ;tab a column
;
_0000020 dex                   ;one less column
         stx xrtemp            ;save column counter
         jmp prntladr          ;print reference address
;
;===============================================================================
;
;dpycod: DISASSEMBLE & DISPLAY CODE
;
;	————————————————————————————————————————————————————————————————————————
;	This function disassembles & displays the machine code at  the  location
;	pointed to by ADDRA.  Upon return, ADDRA will point to the opcode of the
;	next instruction.   The entry point at DPYCODAA  should be called with a
;	disassembly prefix character loaded in .A.   If entered  at  DPYCOD, the
;	default character will be display at the beginning of each  disassembled
;	instruction.
;
;	The disassembly of immediate mode instructions that can take an 8 or  16
;	bit operand is affected by the bit pattern that is  stored  in  FLIMFLAG
;	upon entry to this function:
;
;	    FLIMFLAG: xx000000
;	              ||
;	              |+—————————> 0:  8 bit .X or .Y operand
;	              |            1: 16 bit .X or .Y operand
;	              +——————————> 0:  8 bit .A or BIT # operand
;	                           1: 16 bit .A or BIT # operand
;
;	FLIMFLAG is conditioned according to the operand of  the  most  recently
;	disassembled REP or SEP instruction.   Hence repetitive  calls  to  this
;	subroutine will usually result in the correct disassembly of 16 bit imm-
;	ediate mode instructions.
;	————————————————————————————————————————————————————————————————————————
;
dpycod   lda #disprfx          ;default prefix
;
;
;	alternate prefix display entry point...
;
dpycodaa jsr putcha            ;print prefix
         jsr printspc          ;space
         jsr prntladr          ;print long address
         jsr printspc          ;space to opcode field
         jsr getbyte           ;get opcode
         sta opcode            ;save &...
         jsr printbyt          ;display as hex
;
;
;	decode menmonic & addressing info...
;
         ldx opcode            ;current mnemonic
         lda mnetabix,x        ;get mnemonic index
         asl a                   ;double for...
         tay                   ;mnemonic table offset
         rep #m_seta           ;16 bit load
         .al
         lda mnetab,y          ;copy encoded mnemonic to...
         sta mnepck            ;working storage
         sep #m_seta           ;back to 8 bits
         .as
         jsr instdata          ;extract mode & size data
         sta vopsflag          ;save mode flags
         sty admodidx          ;save mode index
         asl a                   ;variable immediate instruction?
         bcc dpycod01          ;no, effective operand size in .X
;
;
;	determine immediate mode operand size...
;
         lda opcode            ;current opcode
         bit flimflag          ;operand display mode
         bpl _0000010          ;8 bit .A & BIT immediate mode
;
         and #aimmaska         ;determine if...
         cmp #aimmaskb         ;.A or BIT immediate
         beq _0000030          ;display 16 bit operand
;
         lda opcode            ;not .A or BIT immediate
;
_0000010 bvc dpycod01          ;8 bit .X/.Y immediate mode
;
         ldy #n_vopidx-1       ;opcodes to test
;
_0000020 cmp vopidx,y          ;looking for LDX #, CPY #, etc.
         beq _0000040          ;disassemble a 16 bit operand
;
         dey
         bpl _0000020          ;keep trying
;
         bra dpycod01          ;not .X or .Y immediate
;
_0000030 lda opcode            ;reload
;
_0000040 inx                   ;16 bit operand
;
;
;	get & display operand bytes...
;
dpycod01 stx iopsize           ;operand size...
         inx                   ;plus opcode becomes...
         stx instsize          ;instruction size
         stx charcnt           ;total bytes to process
         lda #n_opcols+2       ;total operand columns plus WS
         sta xrtemp            ;initialize counter
         jsr clroper           ;clear operand
         ldy iopsize           ;operand size
         beq _0000020          ;no operand
;
         ldx #0                ;operand index
;
_0000010 jsr getbyte           ;get operand byte
         sta operand,x         ;save
         phx                   ;protect operand index
         jsr printbyt          ;print operand byte
         dec xrtemp            ;3 columns used, 2 for...
         dec xrtemp            ;operand nybbles &...
         dec xrtemp            ;1 for whitespace
         plx                   ;get operand index
         inx                   ;bump it
         dey
         bne _0000010          ;next
;
_0000020 ldx xrtemp            ;operand columns remaining
         jsr multspc           ;space to mnemonic field
;
;
;	display mnemonic...
;
         ldy #s_mnemon         ;size of ASCII mnemonic
;
_0000030 lda #0                ;initialize char
         ldx #n_shfenc         ;shifts to execute
;
_0000040 asl mnepck            ;shift encoded mnemonic
         rol mnepck+s_byte
         rol a
         dex
         bne _0000040
;
         adc #a_mnecvt         ;convert to ASCII &...
         pha                   ;stash
         dey
         bne _0000030          ;continue with mnemonic
;
         ldy #s_mnemon
;
_0000050 pla                   ;get mnenmonic byte
         jsr putcha            ;print it
         dey
         bne _0000050
;
;
;	display operand...
;
         lda iopsize           ;operand size
         beq clearlin          ;zero, disassembly finished
;
         jsr printspc          ;space to operand field
         bit vopsflag          ;check mode flags
         bvc dpycod02          ;not a branch
;
         jsr offtarg           ;compute branch target
         ldx instsize          ;effective instruction size
         dex
         stx iopsize           ;effective operand size
;
dpycod02 stz vopsflag          ;clear
         lda admodidx          ;instruction addressing mode
         cmp #am_move          ;block move instruction?
         bne _0000010          ;no
;
         ror vopsflag          ;yes
;
_0000010 asl a                 ;convert addressing mode to...
         tax                   ;symbology table index
         rep #m_seta           ;do a 16 bit load
         .al
         lda ms_lutab,x        ;addressing symbol pointer
         pha
         sep #m_seta           ;back to 8 bit loads
         .as
         ldy #0
         lda (1,s),y           ;get 1st char
         cmp #a_blank
         beq _0000020          ;no addresing mode preamble
;
         jsr putcha            ;print preamble
;
_0000020 lda #c_hex
         jsr putcha            ;operand displayed as hex
         ldy iopsize           ;operand size = index
;
_0000030 dey
         bmi _0000040          ;done with operand
;
         tyx
         lda operand,x         ;get operand byte through direct page
         jsr dpyhex            ;print operand byte
         bit vopsflag          ;block move?
         bpl _0000030          ;no
;
         stz vopsflag          ;reset
         phy                   ;protect operand index
         pea #ms_move
         jsr sprint            ;display MVN/MVP operand separator
         .as
         .xs
         ply                   ;recover operand index again
         bra _0000030          ;continue
;
_0000040 plx                   ;symbology LSB
         ply                   ;symbology MSB
         inx                   ;move past preamble
         bne _0000050
;
         iny
;
_0000050 phy
         phx
         jsr sprint            ;print postamble, if any
         .as
         .xs
;
;
;	condition immediate mode display format...
;
dpycod03 lda operand           ;operand LSB
         and #pfmxmask         ;isolate M & X bits
         asl a                   ;shift to match...
         asl a                   ;FLIMFLAG alignment
         ldx opcode            ;current instruction
         cpx #opc_rep          ;was it REP?
         bne _0000010          ;no
;
         tsb flimflag          ;set flag bits as required
         bra clearlin
;
_0000010 cpx #opc_sep          ;was it SEP?
         bne clearlin          ;no, just exit
;
         trb flimflag          ;clear flag bits as required
;
;===============================================================================
;
;clearlin: CLEAR DISPLAY LINE
;
clearlin pea #dc_cl
         bra dpyerraa
;
;===============================================================================
;
;dpyibuf: DISPLAY MONITOR INPUT BUFFER CONTENTS
;
dpyibuf  pea #ibuffer
         bra dpyerraa
;
;===============================================================================
;
;dpymem: DISPLAY MEMORY
;
;	————————————————————————————————————————————————————————————
;	This function displays 16 bytes of memory as hex values & as
;	ASCII equivalents.  The starting address for the display is
;	in ADDRA & is expected to be a 24 bit address.  Upon return,
;	ADDRA will point to the start of the next 16 bytes.
;	————————————————————————————————————————————————————————————
;
dpymem   sep #m_setr
         .as
         .xs
         stz charcnt           ;reset
         lda #memprfx
         jsr putcha            ;display prefix
         jsr prntladr          ;print 24 bit address
         ldx #0                ;string buffer index
         ldy #n_dump           ;bytes per line
; Do not read beyond the requested inclusive end address on the final line.
; FACB contains end-start, so a remaining value smaller than N_DUMP means
; exactly FACB+1 bytes remain.
         lda facb+s_word
         bne _dpymem_count_set
         lda facb+1
         bne _dpymem_count_set
         lda facb
         cmp #n_dump
         bcs _dpymem_count_set
         inc a
         tay
_dpymem_count_set:
;
_0000010 jsr getbyte           ;get from RAM, also...
         pha                   ;save for decoding
         phx                   ;save string index
         jsr printbyt          ;display as hex ASCII
         inc charcnt           ;bytes displayed +1
         plx                   ;recover string index &...
         pla                   ;byte
         cmp #a_blank          ;printable?
         bcc _0000020          ;no
;
         cmp #a_del
         bcc _0000030          ;is printable
;
_0000020 lda #memsubch         ;substitute character
;
_0000030 sta ibuffer,x         ;save char
         inx                   ;bump index
         dey                   ;byte count -= 1
         bne _0000010          ;not done
;
         stz ibuffer,x         ;terminate ASCII string
         lda #memsepch
         jsr putcha            ;separate ASCII from bytes
         pea #dc_bf
         jsr sprint            ;select reverse video
         .as
         .xs
         jsr dpyibuf           ;display ASCII equivalents
         pea #dc_sf            ;normal video
         bra dpyerraa
;
;===============================================================================
;
;dpyerr: DISPLAY ERROR SIGNAL
;
dpyerr   pea #mm_err           ;"*ERR"
;
dpyerraa jsr sprint
         .as
         .xs
         rts
;
;===============================================================================
;
;gendbs: GENERATE DESTRUCTIVE BACKSPACE
;
gendbs   pea #dc_bs            ;destructive backspace
         bra dpyerraa
;
;===============================================================================
;
;prntladr: PRINT 24 BIT CURRENT ADDRESS
;
prntladr php                   ;protect register sizes
         sep #m_seta
         .as
         lda addra+s_word      ;get bank byte &...
         jsr dpyhex            ;display it
         rep #m_seta
         .al
         lda addra             ;get 16 bit address
         plp                   ;restore register sizes
;
;===============================================================================
;
;dpyhexw: DISPLAY BINARY WORD AS HEX ASCII
;
;	————————————————————————————————————
;	Preparatory Ops: .C: word to display
;
;	Returned Values: .C: used
;	                 .X: used
;	                 .Y: entry value
;	————————————————————————————————————
;
dpyhexw  php                   ;save register sizes
         rep #m_seta
         .al
         pha                   ;protect value
         sep #m_seta
         .as
         xba                   ;get MSB &...
         jsr dpyhex            ;display
         rep #m_seta
         .al
         pla                   ;recover value
         sep #m_seta           ;only LSB visible
         .as
         plp                   ;reset reg sizes & fall through
;
;===============================================================================
;
;dpyhex: DISPLAY BINARY BYTE AS HEX ASCII
;
;	————————————————————————————————————
;	Preparatory Ops: .A: byte to display
;
;	Returned Values: .A: used
;	                 .X: used
;	                 .Y: entry value
;	————————————————————————————————————
;
dpyhex   jsr binhex            ;convert to hex ASCII
         jsr putcha            ;print MSN
         txa
         jmp putcha            ;print LSN
;
;===============================================================================
;
;dpypbr: DISPLAY PB REGISTER
;
dpypbr   lda reg_pbx           ;PB
         bra dpyhex            ;display as hex ASCII
;
;===============================================================================
;
;multspc: PRINT MULTIPLE BLANKS
;
;	————————————————————————————————————————————————
;	Preparatory Ops : .X: number of blanks to print
;
;	Register Returns: none
;
;	Calling Example : ldx #3
;	                  jsr multspc    ;print 3 spaces
;
;	Notes: This sub will print 1 blank if .X=0.
;	————————————————————————————————————————————————
;
multspc  txa
         bne _0000010          ;blank count specified
;
         inx                   ;default to 1 blank
;
_0000010 lda #a_blank
;
_0000020 jsr putcha
         dex
         bne _0000020
;
         rts
;
;===============================================================================
;
;newline: PRINT NEWLINE (CRLF)
;
newline  pea #dc_lf
         bra dpyerraa
;
;===============================================================================
;
;printbyt: PRINT A BYTE WITH LEADING SPACE
;
printbyt pha                   ;protect byte
         jsr printspc          ;print leading space
         pla                   ;restore &...
         bra dpyhex            ;print byte
;         
;===============================================================================
;
;alert: ALERT USER w/TERMINAL BELL
;
alert    lda #a_bel
         bra printcmn
;
;===============================================================================
;
;printspc: PRINT A SPACE
;
printspc lda #a_blank
;
printcmn jmp putcha
;
;===============================================================================
;
;sprint: PRINT NULL-TERMINATED CHARACTER STRING
;
;	—————————————————————————————————————————————————————————
;	Preparatory Ops : SP+1: string address LSB
;	                  SP+2: string address MSB
;
;	Register Returns: .A: used
;	                  .B: entry value
;	                  .X: used
;	                  .Y: used
;
;	MPU Flags: NVmxDIZC
;	           ||||||||
;	           |||||||+———> 0: okay
;	           |||||||      1: string too long (1)
;	           ||||+++————> not defined
;	           |||+———————> 1
;	           ||+————————> 1
;	           ++—————————> not defined
;
;	Example: PER STRING
;	         JSR SPRINT
;	         BCS TOOLONG
;
;	Notes: 1) Maximum permissible string length including the
;	          terminator is 32,767 bytes.
;	       2) All registers are forced to 8 bits.
;	       3) DO NOT JUMP OR BRANCH INTO THIS FUNCTION!
;	—————————————————————————————————————————————————————————
;
sprint   sep #m_seta           ;8 bit accumulator
         .as
         rep #m_setx           ;16 bit index
         .xl
;
;—————————————————————————————————————————————————————————
_retaddr =1                    ;return address
_src     =_retaddr+s_word      ;string address stack offset
;—————————————————————————————————————————————————————————
;
         ldy #0
         clc                   ;no initial error
;
_0000010 lda (_src,s),y        ;get a byte
         beq _0000020          ;done
;
         jsr putcha            ;write to console port
         iny
         bpl _0000010          ;next
;
         sec                   ;string too long
;
_0000020 plx                   ;pull RTS address
         ply                   ;clear string pointer
         phx                   ;replace RTS
         sep #m_setx
         .xs
         rts
;
;===============================================================================
;
;ascbin: CONVERT NULL-TERMINATED ASCII NUMBER STRING TO BINARY
;
;	———————————————————————————————————————————————————
;	Preparatory Ops: ASCII number string in IBUFFER
;
;	Returned Values: FACA: converted parameter
;	                   .A: used
;	                   .X: used
;	                   .Y: used
;	                   .C: 1 = conversion error
;	                   .Z: 1 = nothing to convert
;
;	Notes: 1) Conversion stops when a non-numeric char-
;	          acter is encountered.
;	       2) Radix symbols are as follows:
;
;	          % binary
;	          @ octal
;	          + decimal
;	          $ hexadecimal
;
;	          Hex is the default if no radix is speci-
;	          fied in the 1st character of the string.
;	———————————————————————————————————————————————————
;
; C64/SuperCPU port: deterministic 32-bit parser.  Keep the entire multiply/add
; path in 8-bit mode so its result cannot depend on an implementation's handling
; of mixed-width direct-page read/modify/write instructions.
ascbin   sep #m_setr
         .as
         .xs
         jsr clrfaca
         stz charcnt
         stz radix

         jsr getcharw
         bne ascbin8_have_char
         clc
         rts

ascbin8_have_char:
         ldx #n_radix-1
ascbin8_radix_loop:
         cmp radxtab,x
         beq ascbin8_radix_done
         dex
         bpl ascbin8_radix_loop
         dec ibufidx
         inx                   ;X=0: hexadecimal default
ascbin8_radix_done:
         lda basetab,x
         sta range

ascbin8_digit_loop:
         jsr getchar
         beq ascbin8_done
         cmp #' '
         beq ascbin8_done
         cmp #','
         beq ascbin8_done
         cmp #a_ht
         beq ascbin8_done
         jsr nybtobin
         bcs ascbin8_error
         cmp range
         bcs ascbin8_error
         sta numeral
         inc charcnt

         ldx #s_pfac-1
ascbin8_copy:
         lda faca,x
         sta facb,x
         stz faca,x
         dex
         bpl ascbin8_copy

         ldx range
ascbin8_multiply:
         clc
         lda faca
         adc facb
         sta faca
         lda faca+1
         adc facb+1
         sta faca+1
         lda faca+2
         adc facb+2
         sta faca+2
         lda faca+3
         adc facb+3
         sta faca+3
         bcs ascbin8_error
         dex
         bne ascbin8_multiply

         clc
         lda faca
         adc numeral
         sta faca
         lda faca+1
         adc #0
         sta faca+1
         lda faca+2
         adc #0
         sta faca+2
         lda faca+3
         adc #0
         sta faca+3
         bcc ascbin8_digit_loop

ascbin8_error:
         sec
         bra ascbin8_exit
ascbin8_done:
         clc
ascbin8_exit:
         lda charcnt
         rts

; Retained temporarily as a reference while validating the C64 port.
ascbin_legacy:
         sep #m_setr
         .as
         .xs
         jsr clrfaca           ;clear accumulator
         stz charcnt           ;zero char count
         stz radix             ;initialize
;
;
;	process radix if present...
;
         jsr getcharw          ;get next non-WS char
         bne _0000010          ;got something
;
         clc                   ;no more input
         rts
;
_0000010 ldx #n_radix-1        ;number of radices
;
_0000020 cmp radxtab,x         ;recognized radix?
         beq _0000030          ;yes
;
         dex
         bpl _0000020          ;try next
;
         dec ibufidx           ;reposition to previous char
         inx                   ;not recognized, assume hex
;
_0000030 cmp #c_dec            ;decimal radix?
         bne _0000040          ;not decimal
;
         ror radix             ;flag decimal conversion
;
_0000040 lda basetab,x         ;number bases table
         sta range             ;set valid numeral range
         lda bitsdtab,x        ;get bits per digit
         sta bitsdig           ;store
;
;
;	process numerals...
;
ascbin01 jsr getchar           ;get next char
         beq ascbin03          ;EOI
;
         cmp #' '
         beq ascbin03          ;blank — EOF
;
         cmp #','
         beq ascbin03          ;comma — EOF
;
         cmp #a_ht
         beq ascbin03          ;tab — EOF
;
         jsr nybtobin          ;change to binary
         bcs ascbin04          ;not a recognized numeral
;
         cmp range             ;check range
         bcs ascbin04          ;not valid for base
;
         sta numeral           ;save processed numeral
         inc charcnt           ;bump numeral count
         bit radix             ;working in base 10?
         bpl _0000030          ;no
;
;
;	compute N*2 for decimal conversion...
;
         ldx #0                ;accumulator index
         ldy #s_pfac/s_word    ;iterations
         rep #m_seta
         .al
         clc
;
_0000020 lda faca,x            ;N
         rol a                   ;N=N*2
         sta facb,x
	.rept s_word
         inx
	.endrept
         dey
         bne _0000020
;
         bcs ascbin04          ;overflow — error
;
         sep #m_seta
         .as
;
;
;	compute N × base for binary, octal or hex,
;	or N × 8 for decimal...
;
_0000030 ldx bitsdig           ;bits per digit
         rep #m_seta           ;16-bit shifts
         .al
;
_0000040 asl faca
         rol faca+s_word
         bcs ascbin04          ;overflow — error
;
         dex
         bne _0000040          ;next shift
;
         sep #m_seta           ;back to 8 bits
         .as
         bit radix             ;check base
         bpl ascbin02          ;not decimal
;
;
;	compute N × 10 for decimal (N × 8 + N × 2)...
;
         ldy #s_pfac
         rep #m_seta
         .al
;
_0000050 lda faca,x            ;N × 8
         adc facb,x            ;N × 2
         sta faca,x            ;now N × 10
	.rept s_word
         inx
	.endrept
         dey
         bne _0000050
;
         bcs ascbin04          ;overflow — error
;
         sep #m_seta
         .as
;
;
;	add current numeral to partial result...
;
ascbin02 lda faca              ;N
         adc numeral           ;N = N + D
         sta faca
         ldx #1
         ldy #s_pfac-1
;
_0000010 lda faca,x
         adc #0                ;account for carry
         sta faca,x
         inx
         dey
         bne _0000010
;
         bcc ascbin01          ;next if no overflow
;
         bcs ascbin04          ;overflow — error
;
;
;	finish up...
;
ascbin03 clc                   ;no error
;
ascbin04 sep #m_seta           ;reset if necessary
         .as
         lda charcnt           ;load char count
         rts                   ;done
;
;===============================================================================
;
;bintonyb: EXTRACT BINARY NYBBLES
;
;	—————————————————————————————————
;	Preparatory Ops: .A: binary value
;
;	Returned Values: .A: MSN
;	                 .X: LSN
;	                 .Y: entry value
;	—————————————————————————————————
;
bintonyb pha                   ;save
         and #bcdumask         ;extract LSN
         tax                   ;save it
         pla
	.rept s_bnybbl        ;extract MSN
         lsr a
	.endrept
         rts
;
;===============================================================================
;
;binasc: CONVERT 32-BIT BINARY TO NULL-TERMINATED ASCII NUMBER STRING
;
;	——————————————————————————————————————————————————————
;	Preparatory Ops: FACA: 32-bit operand
;	                   .A: radix character, w/bit 7 set to
;	                       suppress radix symbol in the
;	                       conversion string
;
;	Returned Values: ibuffer: conversion string
;	                      .A: string length
;	                      .X: string address LSB
;	                      .Y: string address MSB
;
;	Execution Notes: ibufidx & instsize are overwritten.
;	——————————————————————————————————————————————————————
;
binasc   stz ibufidx           ;initialize string index
         stz instsize          ;clear format flag
;
;
;	evaluate radix...
;
         asl a                   ;extract format flag &...
         ror instsize          ;save it
         lsr a                   ;extract radix character
         ldx #n_radix-1        ;total radices
;
_0000010 cmp radxtab,x         ;recognized radix?
         beq _0000020          ;yes
;
         dex
         bpl _0000010          ;try next
;
         inx                   ;assume hex
;
_0000020 stx radix             ;save radix index for later
         bit instsize
         bmi _0000030          ;no radix symbol wanted
;
         lda radxtab,x         ;radix table
         sta ibuffer           ;prepend to string
         inc ibufidx           ;bump string index
;
_0000030 cmp #c_dec            ;converting to decimal?
         bne _0000040          ;no
;
         jsr facabcd           ;convert operand to BCD
         lda #0
         bra _0000070          ;skip binary stuff
;
;
;	prepare for binary, octal or hex conversion...
;
_0000040 ldx #0                ;operand index
         ldy #s_sfac-1         ;workspace index
;
_0000050 lda faca,x            ;copy operand to...
         phx
         tyx
         sta facb,x            ;workspace in direct page...
         plx
         dey                   ;big-endian order
         inx
         cpx #s_pfac
         bne _0000050
;
         lda #0
         tyx
;
_0000060 sta facb,x            ;pad workspace
         dex
         bpl _0000060
;
;
;	set up conversion parameters...
;
_0000070 sta facc              ;initialize byte counter
         ldy radix             ;radix index
         lda numstab,y         ;numerals in string
         sta facc+s_byte       ;set remaining numeral count
         lda bitsntab,y        ;bits per numeral
         sta facc+s_word       ;set
         lda lzsttab,y         ;leading zero threshold
         sta facc+s_xword      ;set
;
;
;	generate conversion string...
;
_0000080 lda #0
         ldy facc+s_word       ;bits per numeral
;
_0000090 ldx #s_sfac-1         ;workspace size
         clc                   ;avoid starting carry
;
_0000100 rol facb,x            ;shift out a bit...
         dex                   ;from the operand or...
         bpl _0000100          ;BCD conversion result
;
         rol a                   ;bit to .A
         dey
         bne _0000090          ;more bits to grab
;
         tay                   ;if numeral isn't zero...
         bne _0000110          ;skip leading zero tests
;
         ldx facc+s_byte       ;remaining numerals
         cpx facc+s_xword      ;leading zero threshold
         bcc _0000110          ;below it, must convert
;
         ldx facc              ;processed byte count
         beq _0000130          ;discard leading zero
;
_0000110 cmp #10               ;check range
         bcc _0000120          ;is 0-9
;
         adc #a_hexdec         ;apply hex adjust
;
_0000120 adc #'0'              ;change to ASCII
         ldy ibufidx           ;string index
         sta ibuffer,y         ;save numeral in buffer
         inc ibufidx           ;next buffer position
         inc facc              ;bytes=bytes+1
;
_0000130 dec facc+s_byte       ;numerals=numerals-1
         bne _0000080          ;not done
;
;
;	terminate string & exit...
;
         ldx ibufidx           ;printable string length
         stz ibuffer,x         ;terminate string
         txa
         ldx #<ibuffer         ;converted string
         ldy #>ibuffer
         clc                   ;all okay
         rts
;
;===============================================================================
;
;binhex: CONVERT BINARY BYTE TO HEX ASCII CHARS
;
;	————————————————————————————————————————————
;	Preparatory Ops: .A: byte to convert
;
;	Returned Values: .A: MSN ASCII char
;	                 .X: LSN ASCII char
;	                 .Y: entry value
;	————————————————————————————————————————————
;
binhex   jsr bintonyb          ;generate binary values
         pha                   ;save MSN
         txa
         jsr _0000010          ;generate ASCII LSN
         tax                   ;save
         pla                   ;get input
;
;
;	convert nybble to hex ASCII equivalent...
;
_0000010 cmp #10
         bcc _0000020          ;in decimal range
;
         adc #k_hex            ;hex compensate
;         
_0000020 eor #'0'              ;finalize nybble
         rts                   ;done
;
;===============================================================================
;
;clrfaca: CLEAR FLOATING ACCUMULATOR A
;
clrfaca  php
         rep #m_seta
         .al
         stz faca
         stz faca+s_word
         plp
         rts
;
;===============================================================================
;
;clrfacb: CLEAR FLOATING ACCUMULATOR B
;
clrfacb  php
         rep #m_seta
         .al
         stz facb
         stz facb+s_word
         plp
         rts
;
;===============================================================================
;
;facabcd: CONVERT FACA INTO BCD
;
facabcd  ldx #s_pfac-1         ;primary accumulator size -1
;
_0000010 lda faca,x            ;value to be converted
         pha                   ;preserve
         dex
         bpl _0000010          ;next
;
         ldx #s_sfac-1         ;workspace size
;
_0000020 stz facb,x            ;clear final result
         stz facc,x            ;clear scratchpad
         dex
         bpl _0000020
;
         inc facc+s_sfac-s_byte
         sed                   ;select decimal mode
         ldy #m_bits-1         ;bits to convert -1
;
_0000030 ldx #s_pfac-1         ;operand size
         clc                   ;no carry at start
;
_0000040 ror faca,x            ;grab LS bit in operand
         dex
         bpl _0000040
;
         bcc _0000060          ;LS bit clear
;
         clc
         ldx #s_sfac-1
;
_0000050 lda facb,x            ;partial result
         adc facc,x            ;scratchpad
         sta facb,x            ;new partial result
         dex
         bpl _0000050
;
         clc
;
_0000060 ldx #s_sfac-1
;
_0000070 lda facc,x            ;scratchpad
         adc facc,x            ;double &...
         sta facc,x            ;save
         dex
         bpl _0000070
;
         dey
         bpl _0000030          ;next operand bit
;
         cld
         ldx #0
         ldy #s_pfac
;
_0000080 pla                   ;operand
         sta faca,x            ;restore
         inx
         dey
         bne _0000080          ;next
;
         rts
;
;===============================================================================
;
;nybtobin: CONVERT ASCII NYBBLE TO BINARY
;
nybtobin sep #m_seta           ;nybble conversion is always 8-bit
         .as
         jsr toupper           ;convert case if necessary
         sec
         sbc #'0'              ;change to binary
         bcc _0000020          ;not a numeral — error
;
         cmp #10
         bcc _0000010          ;numeral is 0-9
;
         sbc #a_hexdec+1       ;10-15 ——> A-F
         clc                   ;no conversion error
;
_0000010 rts
;
_0000020 sec                   ;conversion error
         rts
;
;===============================================================================
;
;calccnt: COMPUTE BYTE COUNT FROM ADDRESS RANGE
;
calccnt  jsr clrfacb           ;clear accumulator
         rep #m_seta
         .al
         sec
         lda addrb             ;ending address
         sbc addra             ;starting address
         sta facb              ;byte count
         sep #m_seta
         .as
         lda addrb+s_word      ;handle banks
         sbc addra+s_word
         sta facb+s_word
         rts
;
;===============================================================================
;
;clroper: CLEAR OPERAND
;
clroper  phx
         ldx #s_oper-1
;
_0000010 stz operand,x
         dex
         bpl _0000010
;
         stz eopsize
         plx
         rts
;
;===============================================================================
;
;cpfwsup: FOWARD COPY MEMORY SETUP
;
cpfwsup  rep #m_setr
         .al
         .xl
         ldx #opc_mvn         ;"move next" opcode
         bra cpsup
;         
;===============================================================================
;
;cprvsup: REVERSE COPY MEMORY SETUP
;
cprvsup  rep #m_setr
         .al
         .xl
         ldx #opc_mvp         ;"move previous" opcode
;         
;===============================================================================
;
;cpsup: COPY MEMORY SETUP
;
cpsup    pha                   ;save banks
         txa                   ;protect...
         xba                   ;opcode
         sep #m_seta
         .as
         ldx #cpcodeee-cpcode-1
;
_0000010 lda cpcode,x          ;transfer copy code to...
         sta mcftwork,x        ;to workspace
         dex
         bpl _0000010
;
         xba                   ;recover opcode &...
         sta mcftopc           ;set it
         rep #m_seta
         .al
         pla                   ;get banks &...
         sta mcftbnk           ;set them
         rts
;
;===============================================================================
;
;decdcnt: DECREMENT DUMP COUNT
;
;	———————————————————————————————————————————
;	Preparatory Ops: bytes to process in FACB
;	                 bytes processed in CHARCNT
;
;	Returned Values: .A: used
;	                 .X: entry value
;	                 .Y: entry value
;	                 .C: 1 = count = zero
;	———————————————————————————————————————————
;
decdcnt  rep #m_seta
         .al
         lda facb+s_word       ;count MSW
         and #%11111111       ;squelch noise in .B
         sec
         ora facb              ;count LSW
         beq _0000020          ;zero, just exit
;
         lda facb
         sbc charcnt           ;bytes processed
         sta facb
         sep #m_seta
         .as
         lda facb+s_word
         sbc #0                ;handle borrow
         bcc _0000010          ;underflow
;
         sta facb+s_word
         clc                   ;count > 0
         rts
;
_0000010 sec
;
_0000020 sep #m_seta
         .as
         rts
;
;===============================================================================
;
;enddest: GET 2ND & 3RD ADDRESSES FOR COMPARE & TRANSFER
;
enddest  jsr facasize          ;check start...
         cmp #s_dword          ;for range
         bcs _0000010          ;out of range — error
;
         jsr facaddra          ;store start
         jsr getparm           ;get end
         bcs _0000010          ;not entered — error
;
         jsr facasize          ;check end...
         cmp #s_dword          ;for range
         bcs _0000010          ;out of range — error
;
         jsr facaddrb          ;store end
         jsr getparm           ;get destination
         bcs _0000010          ;not entered — error
;
         jsr facasize          ;check destination...
         cmp #s_dword          ;for range
         bcc facaoper          ;store dest address
;
_0000010 rts                   ;exit w/error
;
;===============================================================================
;
;facaddra: COPY FACA TO ADDRA
;
facaddra sep #m_setr           ;FACA byte copies require 8-bit A/X
         .as
         .xs
         ldx #s_xword-1
;
_0000010 lda faca,x
         sta addra,x
         dex
         bpl _0000010
;
         rts
;
;===============================================================================
;
;facaddrb: COPY FACA TO ADDRB
;
facaddrb sep #m_setr
         .as
         .xs
         ldx #s_xword-1
;
_0000010 lda faca,x
         sta addrb,x
         dex
         bpl _0000010
;
         rts
;
;===============================================================================
;
;facaoper: COPY FACA TO OPERAND
;
facaoper sep #m_setr
         .as
         .xs
         ldx #s_oper-1
;
_0000010 lda faca,x
         sta operand,x
         dex
         bpl _0000010
;
         rts
;
;===============================================================================
;
;facasize: REPORT OPERAND SIZE IN FACA
;
;	——————————————————————————————————————————
;	Preparatory Ops: operand in FACA
;
;	Returned Values: .A: s_byte  (1)
;	                     s_word  (2)
;	                     s_xword (3)
;	                     s_dword (4)
;
;	Notes: 1) This function will always report
;	          a non-zero result.
;	——————————————————————————————————————————
;
facasize sep #m_setr
         .as
         .xs
         ldx #s_dword-1
;
_0000010 lda faca,x            ;get byte
         bne _0000020          ;done
;
         dex
         bne _0000010          ;next byte
;
_0000020 inx                   ;count=index+1
         txa
         rts
;
;===============================================================================
;
;getparm: GET A PARAMETER
;
;	—————————————————————————————————————————————————
;	Preparatory Ops: null-terminated input in IBUFFER
;
;	Returned Values: .A: chars in converted parameter
;	                 .X: used
;	                 .Y: entry value
;	                 .C: 1 = no parameter entered
;	—————————————————————————————————————————————————
;
getparmr dec ibufidx           ;reread previous char
;
getparm  phy                   ;preserve
         jsr ascbin            ;convert parameter to binary
         bcs _0000040          ;conversion error
;
         jsr getcharr          ;reread last char
         bne _0000010          ;not end-of-input
;
         dec ibufidx           ;reindex to terminator
         lda charcnt           ;get chars processed so far
         beq _0000030          ;none
;
         bne _0000020          ;some
;
_0000010 cmp #a_blank          ;recognized delimiter
         beq _0000020          ;end of parameter
;
         cmp #','              ;recognized delimiter
         bne _0000040          ;unknown delimter
;
_0000020 clc
         .byte bitzp           ;skip SEC below
;
_0000030 sec
         ply                   ;restore
         lda charcnt           ;get count
         rts                   ;done
;
_0000040	.rept 3               ;clean up stack
         pla
	.endrept
         jmp monerr            ;abort w/error
;
;===============================================================================
;
;nxtaddra: TEST & INCREMENT WORKING ADDRESS 'A'
;
;	——————————————————————————————————————————————————
;	Calling syntax: JSR NXTADDRA
;
;	Exit registers: .A: used
;	                .B: used
;	                .X: entry value
;	                .Y: entry value
;	                DB: entry value
;	                DP: entry value
;	                PB: entry value
;	                SR: NVmxDIZC
;	                    ||||||||
;	                    |||||||+———> 0: ADDRA < ADDRB
;	                    |||||||      1: ADDRA >= ADDRB
;	                    ||||||+————> undefined
;	                    |||+++—————> entry value
;	                    ||+————————> 1
;	                    ++—————————> undefined
;	——————————————————————————————————————————————————
;
nxtaddra sep #m_seta
         .as
         lda addra+s_word      ;bits 16-23
         cmp addrb+s_word
         bcc incaddra          ;increment
;
         bne _0000010          ;don't increment
;
         rep #m_seta
         .al
         lda addra             ;bits 0-15
         cmp addrb             ;condition flags
         sep #m_seta
         .as
         bcc incaddra          ;increment
;
_0000010 rts
;
;===============================================================================
;
;getbyte: GET A BYTE FROM MEMORY
;
getbyte  lda [addra]           ;get a byte
;
;===============================================================================
;
;incaddra: INCREMENT WORKING ADDRESS 'A'
;
;	——————————————————————————————————————————————————
;	Calling syntax: JSR INCADDRA
;
;	Exit registers: .A: entry value
;	                .B: entry value
;	                .X: entry value
;	                .Y: entry value
;	                DB: entry value
;	                DP: entry value
;	                PB: entry value
;	                SR: NVmxDIZC
;	                    ||||||||
;	                    ++++++++———> entry value
;	——————————————————————————————————————————————————
;
incaddra php
         rep #m_seta
         .al
         inc addra             ;bump bits 0-15
         bne _0000010
;
         sep #m_seta
         .as
         inc addra+s_word      ;bump bits 16-23
;
_0000010 plp
         rts
;
;===============================================================================
;
;incoper: INCREMENT OPERAND ADDRESS
;
incoper  clc
         php
         rep #m_setr
         .al
         .xl
         pha
         inc operand           ;handle base address
         bne _0000010
;              
         sep #m_seta
         .as
         inc operand+s_word    ;handle bank
         rep #m_seta
         .al
;
_0000010 pla
         plp
         rts
;
;===============================================================================
;
;instdata: GET INSTRUCTION SIZE & ADDRESSING MODE DATA
;
;	——————————————————————————————————
;	Preparatory Ops: .X: 65C816 opcode
;
;	Returned Values: .A: mode flags
;	                 .X: operand size
;	                 .Y: mode index
;	——————————————————————————————————
;
instdata sep #m_setr
         .as
         .xs
         lda mnetabam,x        ;addressing mode data
         pha                   ;save mode flag bits
         pha                   ;save size data
         and #amodmask         ;extract mode index &...
         tay                   ;save
         pla                   ;recover data
         and #opsmask          ;mask mode fields &...
	.rept n_opslsr        ;extract operand size
         lsr a
	.endrept
         tax                   ;operand size
         pla                   ;recover mode flags
         and #vopsmask         ;discard mode & size fields
         rts
;
;===============================================================================
;
;offtarg: CONVERT BRANCH OFFSET TO TARGET ADDRESS
;
;	———————————————————————————————————————————————
;	Preparatory Ops:    ADDRA: base address
;	                 INSTSIZE: instruction size
;	                  OPERAND: offset
;
;	Returned Values:  OPERAND: target address (L/H)
;	                       .A: used
;	                       .X: entry value
;                              .Y: entry value
;	———————————————————————————————————————————————
;
offtarg  rep #m_seta
         .al
         lda addra             ;base address
         sep #m_seta
         .as
         lsr instsize          ;bit 0 will be set if...
         bcs _0000010          ;a long branch
;
         bit operand           ;short forward or backward?
         bpl _0000010          ;forward
;
         xba                   ;expose address MSB
         dec a                   ;back a page
         xba                   ;expose address LSB
;
_0000010 rep #m_seta
         .al
         clc
         adc operand           ;calculate target address
         sta operand           ;new operand
         sep #m_seta
         .as
         lda #s_xword
         sta instsize          ;effective instruction size
         rts
;
;===============================================================================
;
;setxaddr: SET EXECUTION ADDRESS
;
setxaddr bcs _0000010          ;no address given
;
         jsr facasize          ;check address...
         cmp #s_dword          ;range
         bcs _0000020          ;out of range
;
         rep #m_seta
         .al
         lda faca              ;execution address
         sta reg_pcx           ;set new PC value
         sep #m_seta
         .as
         lda faca+s_word
         sta reg_pbx           ;set new PB value
;
_0000010 clc                   ;no error
;
_0000020 rts
;
;===============================================================================
;
;targoff: CONVERT BRANCH TARGET ADDRESS TO BRANCH OFFSET                   
;
;	—————————————————————————————————————————————————
;	Preparatory Ops:   ADDRA: instruction address
;	                 OPERAND: target address
;
;	Returned Values: OPERAND: computed offset
;	                      .A: effective operand size
;	                      .X: entry value
;	                      .Y: entry value
;	                      .C: 1 = branch out of range
;
;	Execution notes: ADDRB is set to the branch base
;	                 address.
;	—————————————————————————————————————————————————
;
targoff  ;this line intentionally has no code
;
;—————————————————————————————————————————————————
_btype   =facc+5               ;branch type flag
;—————————————————————————————————————————————————
;
         lda instsize          ;instruction size will tell...
         lsr a                   ;if long or short branch
         ror _btype            ;set branch type
         rep #m_seta           ;16-bit accumulator
         .al
         clc
         lda addra             ;instruction address
         adc instsize          ;instruction size
         sta addrb             ;offset base address
         sec
         lda operand           ;target address
         sbc addrb             ;base address
         sta operand           ;proposed offset
         sep #m_seta           ;8-bit accumulator
         .as
         bit _btype            ;check branch range
         bmi _0000030          ;long
;
         bcc _0000020          ;short backward branch
;
;
;	process short forward branch...
;
         xba                   ;offset MSB should be zero
         bne _0000040          ;it isn't — out of range
;
         xba                   ;offset LSB should be $00-$7F
         bmi _0000040          ;it isn't — out of range
;
_0000010 lda #s_byte           ;final instruction size
         clc                   ;branch in range
         rts
;
;
;	process short backward branch...
;
_0000020 xba                   ;offset MSB should be negative
         bpl _0000040          ;it isn't — out of range
;
         eor #%11111111        ;complement offset MSB 2s 
         bne _0000040          ;out of range
;
         xba                   ;offset LSB should be $80-$FF
         bmi _0000010          ;it is — branch in range
;
         bra _0000040          ;branch out of range
;
;
;	process long forward branch...
;
_0000030 lda #s_word           ;final instruction size
         clc                   ;never a range error
         rts

_0000040 sec                   ;range error
         rts
;
;===============================================================================
;
;getcharr: GET PREVIOUS INPUT BUFFER CHARACTER
;
getcharr dec ibufidx           ;move back a char
;
;===============================================================================
;
;getchar: GET A CHARACTER FROM INPUT BUFFER
;
;	——————————————————————————————————————————————
;	Preparatory Ops : none
;
;	Register Returns: .A: character or <NUL>
;	                  .B: entry value
;	                  .X: entry value
;	                  .Y: entry value
;
;	MPU Flags: NVmxDIZC
;	           ||||||||
;	           |||||||+———> entry value
;	           ||||||+————> 1: <NUL> gotten
;	           |||||+—————> entry value
;	           ||||+——————> entry value
;	           |||+———————> entry value
;	           ||+————————> entry value
;	           |+—————————> not defined
;	           +——————————> not defined
;	——————————————————————————————————————————————
;
getchar  phx
         phy
         php                   ;save register sizes
         sep #m_setr           ;force 8 bits
         .as
         .xs
         ldx ibufidx           ;buffer index
         lda ibuffer,x         ;get char
         inc ibufidx           ;bump index
         cmp #0                ;condition flags from char, not incremented index
;
; XBA's flag side effects are not dependable on every SuperCPU implementation.
; Rebuild the caller's status with Z taken explicitly from the character while
; preserving A, B, X, Y and the caller's register widths.
         pha                   ;character
         php                   ;status conditioned by 8-bit LDA
         pla                   ;extract its Z bit
         and #sr_zer
         tax                   ;temporarily retain desired Z
         pla                   ;recover character
         tay                   ;temporarily retain character
         pla                   ;caller's saved status
         and #%11111101        ;clear its stale Z bit
         cpx #0
         beq _getchar_nz
         ora #sr_zer           ;character was NUL
_getchar_nz:
         sta getcharps         ;save modified caller status
         tya
         sta getcharv          ;save character while restoring X/Y
         lda getcharps
         pha
         plp                   ;restore widths for matching PLY/PLX
         ply
         plx
         sep #m_seta           ;prepare one-byte final status restore
         lda getcharps
         pha
         lda getcharv          ;return character without disturbing B
         plp                   ;restore caller widths and corrected flags last
         rts
;
;===============================================================================
;
;getpat: GET PATTERN FOR MEMORY CHANGE or SEARCH
;
;	—————————————————————————————————————————————————————
;	Preparatory Ops: Null-terminated pattern in IBUFFER.
;
;	Returned Values: .A: used
;	                 .X: used
;	                 .Y: pattern length if entered
;	                 .C: 0 = pattern valid
;	                     1 = exception:
;	                 .N  0 = no pattern entered
;	                     1 = evaluation error
;
;	Notes: 1) If pattern is preceded by "'" the following
;	          characters are interpreted as ASCII.
;	       2) A maximum of 32 bytes or characters is
;	          accepted.  Excess input will be discarded.
;	—————————————————————————————————————————————————————
;
getpat   stz status            ;clear pattern type indicator
         ldy #0                ;pattern index
         jsr getcharr          ;get last char
         beq _0000070          ;EOS
;
         ldx ibufidx           ;current buffer index
         jsr getcharw          ;get next
         beq _0000070          ;EOS
;
         cmp #$27              ;ASCII apostrophe
         bne _0000010          ;not ASCII input
;
         ror status            ;condition flag
         bra _0000030          ;balance of input is ASCII
;
_0000010 stx ibufidx           ;restore buffer index
;
_0000020 jsr getparm           ;evaluate numeric pattern
         bcs _0000060          ;done w/pattern
;
         jsr facasize          ;size
         cmp #s_word
         bcs _0000070          ;not a byte — error
;
         lda faca              ;get byte &...
         bra _0000040          ;store
;
_0000030 jsr getchar           ;get ASCII char
         beq _0000060          ;done w/pattern
;
_0000040 cpy #s_auxbuf         ;pattern buffer full?
         beq _0000050          ;yes
;
         sta auxbuf,y          ;store pattern
         iny
         bit status
         bpl _0000020          ;get next numeric value
;
         bra _0000030          ;get next ASCII char
;
_0000050 jsr alert             ;excess input
;
_0000060 sty auxbufix          ;save pattern size
         tya                   ;condition .Z
         clc                   ;pattern valid
         rts
;
;
;	no pattern entered...
;
_0000070 rep #%10000000
         sec
         rts
;
;
;	evaluation error...
;
_0000080 sep #%10000001
         rts
;
;===============================================================================
;
;getcharw: GET FROM INPUT BUFFER, DISCARDING WHITESPACE
;
;	——————————————————————————————————————————————————
;	Preparatory Ops: Null-terminated input in IBUFFER.
;
;	Returned Values: .A: char or null
;	                 .X: entry value
;	                 .Y: entry value
;	                 .Z: 1 = null terminator detected
;
;	Notes: Whitespace is defined as a blank ($20) or a
;	       horizontal tab ($09).
;	——————————————————————————————————————————————————
;
getcharw jsr getchar           ;get from buffer
         beq _0000010          ;EOI
;
         cmp #a_blank
         beq getcharw          ;discard whitespace
;
         cmp #a_ht             ;also whitespace
         beq getcharw
;
_0000010 clc
         rts  
;
;===============================================================================
;
;input: INTERACTIVE INPUT FROM CONSOLE CHANNEL
;
;	———————————————————————————————————————————————————————————
;	Preparatory Ops: Zero IBUFIDX or load IBUFFER with default
;	                 input & set IBUFIDX to the number of chars
;	                 loaded into the buffer.
;
;	Returned Values: .A: used
;	                 .X: characters entered
;	                 .Y: used
;
;	Example: STZ IBUFIDX
;	         JSR INPUT
;
;	Notes: Input is collected in IBUFFER & is null-terminated.
;	       IBUFIDX is reset to zero upon exit.
;	———————————————————————————————————————————————————————————
;
input    ldx ibufidx
         stz ibuffer,x         ;be sure buffer is terminated
         jsr dpyibuf           ;print default input if any
         pea #dc_cn
         jsr sprint            ;enable cursor
         .as
         .xs
         ldx ibufidx           ;starting buffer index
;
;
;	main input loop...
;
_0000010 jsr getcha            ;poll for input
         bcs _0000060          ;nothing
;
         cmp #a_del            ;above ASCII range?
         bcs _0000030          ;yes, not allowed
;
         cmp #a_ht             ;horizontal tab?
         bne _0000020          ;no
;
         lda #a_blank          ;replace <HT> w/blank
;
_0000020 cmp #a_blank          ;control char?
         bcc _0000040          ;yes
;
;
;	process QWERTY character...
;
         cpx #s_ibuf           ;room in buffer?
         bcs _0000030          ;no
;
         sta ibuffer,x         ;store char
         inx                   ;bump index
         .byte bitabs          ;echo char
;
_0000030 lda #a_bel            ;alert user
         jsr putcha
         phx
         pea #dc_cn
         jsr sprint            ;show cursor at the new input position
         .as
         .xs
         plx
         bra _0000010          ;get some more
;
;
;	process carriage return...
;
_0000040 cmp #a_cr             ;carriage return?
         bne _0000050          ;no
;
         phx                   ;protect input count
         pea #dc_co
         jsr sprint            ;cursor off
         .as
         .xs
         plx                   ;recover input count
         stz ibuffer,x         ;terminate input
         stz ibufidx           ;reset buffer index
         rts                   ;done
;
;
;	process backspace...
;
_0000050 cmp #a_bs             ;backspace?
         bne _0000010          ;no
;
         txa
         beq _0000010          ;no input, ignore <BS>
;
         phx
         pea #dc_co
         jsr sprint            ;remove cursor before editing the line
         .as
         .xs
         plx
         dex                   ;1 less char
         phx                   ;preserve count
         jsr gendbs            ;destructive backspace
         plx                   ;restore count
         phx
         pea #dc_cn
         jsr sprint            ;show cursor at the edited position
         .as
         .xs
         plx
         bra _0000010          ;get more input
;
;
;	waiting loop...
;
_0000060 nop                   ;wait for datum
         bra _0000010
;
;===============================================================================
;
;lodbnk: LOAD SOURCE & DESTINATION BANKS
;
lodbnk   sep #m_seta
         .as
         lda operand+s_word    ;destination bank
         xba                   ;make it MSB
         lda addra+s_word      ;source bank is LSB
         rts
;         
;===============================================================================
;
;getcharc: GET A CHARACTER FROM INPUT BUFFER & CONVERT CASE
;
;	——————————————————————————————————————————————————
;	Preparatory Ops: Null-terminated input in IBUFFER.
;
;	Returned Values: .A: char or null
;	                 .X: entry value
;	                 .Y: entry value
;	                 .Z: 1 = null terminator detected
;	——————————————————————————————————————————————————
;
getcharc jsr getchar           ;get from buffer
;
;===============================================================================
;
;toupper: FORCE CHARACTER TO UPPER CASE
;
;	————————————————————————————————————————————————
;	Preparatory Ops : .A: 8 bit character to convert
;
;	Register Returns: .A: converted character
;	                  .B: entry value
;	                  .X: entry value
;	                  .Y: entry value
;
;	MPU Flags: no change
;
;	Notes: 1) This subroutine has no effect on char-
;	          acters that are not alpha.
;	————————————————————————————————————————————————
;
toupper  php                   ;protect flags
         cmp #a_asclcl         ;check char range
         bcc _0000010          ;not LC alpha
;
         cmp #a_asclch+s_byte
         bcs _0000010          ;not LC alpha
;
         and #a_lctouc         ;force to UC
;
_0000010 plp                   ;restore flags
;
touppera rts
;
;===============================================================================
;
;teststop: TEST FOR STOP KEY
;
;	——————————————————————————————————————————————
;	Preparatory Ops: none
;
;	Returned Values: .A: detected keypress, if any
;	                 .X: entry value
;	                 .Y: entry value
;
;	MPU Flags: NVmxDIZC
;	           ||||||||
;	           |||||||+———> 0: normal key detected
;	           |||||||      1: <STOP> detected
;	           +++++++————> not defined
;
;	Example: jsr teststop
;	         bcs stopped
;
;	Notes: The symbol STOPKEY defines the ASCII
;	       value of the "stop key."
;	——————————————————————————————————————————————
;
teststop jsr getcha            ;poll console
         bcs _0000010          ;no input
;
         cmp #stopkey          ;stop key pressed?
         beq _0000020          ;yes
;
_0000010 clc
;
_0000020 rts
;
;===============================================================================
;
;cpcode: COPY MEMORY CODE
;
;	———————————————————————————————————————————
;	This code is transfered to workspace when a
;	copy or fill operation is to be performed.
;	———————————————————————————————————————————
;
cpcode   phb                   ;must preserve data bank
	.rept s_mvinst
         nop                   ;placeholder
	.endrept
         plb                   ;restore data bank
         jml monce             ;return to command executive
cpcodeee =*                    ;placeholder — do not delete
;
;===============================================================================
;
;adjloadr: ADJUST S-RECORD LOAD ADDRESS
;
adjloadr lda pageoff           ;page offset
         xba                   ;move to MSB
         lda #0                ;even page boundary
         rep #m_seta
         .al
         clc
         adc loadaddr          ;record load address
         sta loadaddr          ;new load address
         sep #m_seta
         .as
         rts
;
;===============================================================================
;
;clrchanb: CLEAR EXCESS INPUT FROM AUXILIARY CHANNEL
;
clrchanb wai                   ;wait for an IRQ
         jsr getchb            ;read from channel
         bcc clrchanb          ;keep clearing
;
         rts
;
;===============================================================================
;
;getchanb: GET DATUM FROM AUXILIARY CHANNEL W/STOP TEST
;
;	————————————————————————————————————————————————————————————————————
;	Preparatory Ops: NONE
;
;	Returned Values: .A: datum or null
;	                 .B: used
;	                 .X: entry value
;	                 .Y: entry value
;
;	MPU Flags: NVmxDIZC
;	           ||||||||
;	           |||||||+———> 0: .A is valid
;	           |||||||      1: user aborted
;	           ||||||+————> 0: datum in .A
;	           ||||||       1: EOR detected
;	           ||||++—————> undefined
;	           ||++———————> 1
;	           ++—————————> undefined
;
;	Notes: 1) This sub will loop until a datum is gotten or the stop key
;	          is detected on the console channel.
;
;	       2) If <CR>, <LF> or the <CRLF> combination is encountered in
;	          the input stream, $00 will be substituted.  <LFCR> is not
;	          a synonym for <CRLF> & if encountered, may cause process-
;	          ing errors.
;	————————————————————————————————————————————————————————————————————
;
getchanb sep #m_setr           ;8 bit loads
         .as
         .xs
         lda #0
         xba                   ;flush B-accumulator
;
_0000010 jsr teststop          ;check for user abort
         bcs _0000040          ;aborted
;
         jsr getchb            ;get datum from ch B
         bcs _0000050          ;none
;
         cmp #a_lf             ;<LF>?
         beq _0000020          ;yes, EOR delimiter
;
         cmp #a_cr             ;<CR>?
         bne _0000030          ;no
;
         xba                   ;look at previous datum
         cmp #a_cr             ;was it a <CR> as well?
         bne _0000010          ;no, wait for next datum
;
_0000020 lda #0                ;end of record
;
_0000030 clc                   ;valid datum
;
_0000040 rts
;
_0000050 wai                   ;wait for any interrupt &...
         bra _0000010          ;check for data
;
;===============================================================================
;
;getcvt: GET & CONVERT S-RECORD BYTE PAIR
;
;	—————————————————————————————————————————————————————————
;	Preparatory Ops: NONE
;
;	Returned Values: .A: byte converted from ASCII pair
;	                 .X: entry value
;	                 .Y: entry value
;	                 .C: 0: conversion successful
;	                     1: conversion error or premature EOT
;	—————————————————————————————————————————————————————————
;
getcvt   sep #m_setr
         .as
         .xs
         jsr getchar           ;get MSN char
         beq _0000010          ;premature EOT
;
         jsr nybtobin          ;convert to a nybble
         bcs _0000020          ;conversion error
;
	.rept s_bnybbl        ;shift to MSN position
         asl a
	.endrept
         sta datax             ;store MSN
         jsr getchar           ;get LSN char
         beq _0000010          ;premature EOT
;
         jsr nybtobin
         bcs _0000020          ;error
;
         ora datax             ;combine with MSN
         rts                   ;done
;
_0000010 sec                   ;conversion error
;
_0000020 rts
;
;===============================================================================
;
;COMMAND PROCESSING DATA TABLES
;
;
;	monitor commands...
;
mpctab   .text "A";assemble code
         .text "C";compare memory ranges
         .text "D";disassemble code
         .text "F";fill memory
         .text "G";execute code
         .text "H";search memory
         .text "J";execute code as subroutine
         .text "L";load S-records
         .text "M";dump memory range
         .text "R";dump registers
         .text "T";copy memory range
         .text "X";exit from monitor
         .text ">";change memory
         .text ";";change registers
n_mpctab =*-mpctab             ;entries in above table
;
;
;	monitor command jump table...
;
mpcextab .word monasc-s_byte   ; A  assemble code
         .word moncmp-s_byte   ; C  compare memory ranges
         .word mondsc-s_byte   ; D  disassemble code
         .word monfil-s_byte   ; F  fill memory
         .word monjmp-s_byte   ; G  execute code
         .word monhnt-s_byte   ; H  search memory
         .word monjsr-s_byte   ; J  execute code as subroutine
         .word monlsr-s_byte   ; L  load S-records
         .word mondmp-s_byte   ; M  dump memory range
         .word monreg-s_byte   ; R  dump registers
         .word moncpy-s_byte   ; T  copy memory range
         .word monxit-s_byte   ; X  exit from monitor
         .word monchm-s_byte   ; >  change memory
         .word monchr-s_byte   ; ;  change registers
;
;
;	S-record processing jump table...
;
lsrextab .word monlsrs0        ;S0
         .word monlsrs1        ;S1
         .word monlsrut        ;S2 (not implemented)
         .word monlsrut        ;S3 (not implemented)
         .word monlsrut        ;S4 (not implemented)
         .word monlsrs5        ;S5
         .word monlsrut        ;S6 (undefined)
         .word monlsrut        ;S7 (not implemented)
         .word monlsrut        ;S8 (not implemented)
         .word monlsrs9        ;S9
;
;
;	number conversion...
;        
basetab  .byte 16,10,8,2       ;supported number bases
bitsdtab .byte 4,3,3,1         ;bits per binary digit
bitsntab .byte 4,4,3,1         ;bits per ASCII character
lzsttab  .byte 3,2,9,2         ;leading zero suppression thresholds
numstab  .byte 12,12,16,48     ;bin to ASCII conversion numerals
radxtab  .byte c_hex           ;hexadecimal radix
         .byte c_dec           ;decimal radix
         .byte c_oct           ;octal radix
         .byte c_bin           ;binary radix
n_radix  =*-radxtab            ;number of recognized radices
;
;
;	shadow MPU register sizes...
;
rcvltab  .byte s_mpupbx+s_byte ; PB
         .byte s_mpupcx+s_byte ; PC
         .byte s_mpusrx+s_byte ; SR
         .byte s_word+s_byte   ; .C
         .byte s_word+s_byte   ; .X
         .byte s_word+s_byte   ; .Y
         .byte s_mpuspx+s_byte ; SP
         .byte s_mpudpx+s_byte ; DP
         .byte s_mpudbx+s_byte ; DB
n_regchv =*-rcvltab            ;total shadow registers
;
;===============================================================================
;
;ASSEMBLER/DISASSEMBLER DATA TABLES
;
;
;	encoded & sorted 65C816 mnemonics...
;
mnetab   .word mne_xba         ;  0 — XBA
         .word mne_lda         ;  1 — LDA
         .word mne_pea         ;  2 — PEA
         .word mne_pha         ;  3 — PHA
         .word mne_pla         ;  4 — PLA
         .word mne_bra         ;  5 — BRA
         .word mne_ora         ;  6 — ORA
         .word mne_sta         ;  7 — STA
         .word mne_txa         ;  8 — TXA
         .word mne_tya         ;  9 — TYA
         .word mne_phb         ; 10 — PHB
         .word mne_plb         ; 11 — PLB
         .word mne_trb         ; 12 — TRB
         .word mne_tsb         ; 13 — TSB
         .word mne_sbc         ; 14 — SBC
         .word mne_bcc         ; 15 — BCC
         .word mne_adc         ; 16 — ADC
         .word mne_tdc         ; 17 — TDC
         .word mne_dec         ; 18 — DEC
         .word mne_sec         ; 19 — SEC
         .word mne_clc         ; 20 — CLC
         .word mne_inc         ; 21 — INC
         .word mne_tsc         ; 22 — TSC
         .word mne_bvc         ; 23 — BVC
         .word mne_tcd         ; 24 — TCD
         .word mne_sed         ; 25 — SED
         .word mne_phd         ; 26 — PHD
         .word mne_cld         ; 27 — CLD
         .word mne_pld         ; 28 — PLD
         .word mne_and         ; 29 — AND
         .word mne_xce         ; 30 — XCE
         .word mne_bne         ; 31 — BNE
         .word mne_wai         ; 32 — WAI
         .word mne_pei         ; 33 — PEI
         .word mne_sei         ; 34 — SEI
         .word mne_cli         ; 35 — CLI
         .word mne_bmi         ; 36 — BMI
         .word mne_rti         ; 37 — RTI
         .word mne_phk         ; 38 — PHK
         .word mne_brk         ; 39 — BRK
         .word mne_jml         ; 40 — JML
         .word mne_rol         ; 41 — ROL
         .word mne_bpl         ; 42 — BPL
         .word mne_brl         ; 43 — BRL
         .word mne_asl         ; 44 — ASL
         .word mne_jsl         ; 45 — JSL
         .word mne_rtl         ; 46 — RTL
         .word mne_wdm         ; 47 — WDM
         .word mne_mvn         ; 48 — MVN
         .word mne_rep         ; 49 — REP
         .word mne_sep         ; 50 — SEP
         .word mne_php         ; 51 — PHP
         .word mne_plp         ; 52 — PLP
         .word mne_cmp         ; 53 — CMP
         .word mne_jmp         ; 54 — JMP
         .word mne_cop         ; 55 — COP
         .word mne_nop         ; 56 — NOP
         .word mne_stp         ; 57 — STP
         .word mne_mvp         ; 58 — MVP
         .word mne_beq         ; 59 — BEQ
         .word mne_per         ; 60 — PER
         .word mne_eor         ; 61 — EOR
         .word mne_ror         ; 62 — ROR
         .word mne_jsr         ; 63 — JSR
         .word mne_lsr         ; 64 — LSR
         .word mne_bcs         ; 65 — BCS
         .word mne_tcs         ; 66 — TCS
         .word mne_rts         ; 67 — RTS
         .word mne_bvs         ; 68 — BVS
         .word mne_txs         ; 69 — TXS
         .word mne_bit         ; 70 — BIT
         .word mne_clv         ; 71 — CLV
         .word mne_tax         ; 72 — TAX
         .word mne_ldx         ; 73 — LDX
         .word mne_dex         ; 74 — DEX
         .word mne_phx         ; 75 — PHX
         .word mne_plx         ; 76 — PLX
         .word mne_inx         ; 77 — INX
         .word mne_cpx         ; 78 — CPX
         .word mne_tsx         ; 79 — TSX
         .word mne_stx         ; 80 — STX
         .word mne_tyx         ; 81 — TYX
         .word mne_tay         ; 82 — TAY
         .word mne_ldy         ; 83 — LDY
         .word mne_dey         ; 84 — DEY
         .word mne_phy         ; 85 — PHY
         .word mne_ply         ; 86 — PLY
         .word mne_iny         ; 87 — INY
         .word mne_cpy         ; 88 — CPY
         .word mne_sty         ; 89 — STY
         .word mne_txy         ; 90 — TXY
         .word mne_stz         ; 91 — STZ
;
n_mnemon =(*-mnetab)/s_word    ;total mnemonics in above table
;
;
;	mnemonic lookup indices in opcode order...
;
mnetabix .byte mne_brkx        ; $00  BRK
         .byte mne_orax        ; $01  ORA (dp,X)
         .byte mne_copx        ; $02  COP
         .byte mne_orax        ; $03  ORA <offset>,S
         .byte mne_tsbx        ; $04  TSB dp
         .byte mne_orax        ; $05  ORA dp
         .byte mne_aslx        ; $06  ASL dp
         .byte mne_orax        ; $07  ORA [dp]
         .byte mne_phpx        ; $08  PHP
         .byte mne_orax        ; $09  ORA #
         .byte mne_aslx        ; $0A  ASL A
         .byte mne_phdx        ; $0B  PHD
         .byte mne_tsbx        ; $0C  TSB abs
         .byte mne_orax        ; $0D  ORA abs
         .byte mne_aslx        ; $0E  ASL abs
         .byte mne_orax        ; $0F  ORA absl
;
         .byte mne_bplx        ; $10  BPL abs
         .byte mne_orax        ; $11  ORA (<dp>),Y
         .byte mne_orax        ; $12  ORA (dp)
         .byte mne_orax        ; $13  ORA (<offset>,S),Y
         .byte mne_trbx        ; $14  TRB dp
         .byte mne_orax        ; $15  ORA dp,X
         .byte mne_aslx        ; $16  ASL dp,X
         .byte mne_orax        ; $17  ORA [dp],Y
         .byte mne_clcx        ; $18  CLC
         .byte mne_orax        ; $19  ORA abs,Y
         .byte mne_incx        ; $1A  INC A
         .byte mne_tcsx        ; $1B  TCS
         .byte mne_trbx        ; $1C  TRB abs
         .byte mne_orax        ; $1D  ORA abs,X
         .byte mne_aslx        ; $1E  ASL abs,X
         .byte mne_orax        ; $1F  ORA absl,X
;
         .byte mne_jsrx        ; $20  JSR abs
         .byte mne_andx        ; $21  AND (dp,X)
         .byte mne_jslx        ; $22  JSL absl
         .byte mne_andx        ; $23  AND <offset>,S
         .byte mne_bitx        ; $24  BIT dp
         .byte mne_andx        ; $25  AND dp
         .byte mne_rolx        ; $26  ROL dp
         .byte mne_andx        ; $27  AND [dp]
         .byte mne_plpx        ; $28  PLP
         .byte mne_andx        ; $29  AND #
         .byte mne_rolx        ; $2A  ROL A
         .byte mne_pldx        ; $2B  PLD
         .byte mne_bitx        ; $2C  BIT abs
         .byte mne_andx        ; $2D  AND abs
         .byte mne_rolx        ; $2E  ROL abs
         .byte mne_andx        ; $2F  AND absl
;
         .byte mne_bmix        ; $30  BMI abs
         .byte mne_andx        ; $31  AND (<dp>),Y
         .byte mne_andx        ; $32  AND (dp)
         .byte mne_andx        ; $33  AND (<offset>,S),Y
         .byte mne_bitx        ; $34  BIT dp,X
         .byte mne_andx        ; $35  AND dp,X
         .byte mne_rolx        ; $36  ROL dp,X
         .byte mne_andx        ; $37  AND [dp],Y
         .byte mne_secx        ; $38  SEC
         .byte mne_andx        ; $39  AND abs,Y
         .byte mne_decx        ; $3A  DEC A
         .byte mne_tscx        ; $3B  TSC
         .byte mne_bitx        ; $3C  BIT abs,X
         .byte mne_andx        ; $3D  AND abs,X
         .byte mne_rolx        ; $3E  ROL abs,X
         .byte mne_andx        ; $3F  AND absl,X
;
         .byte mne_rtix        ; $40  RTI
         .byte mne_eorx        ; $41  EOR (dp,X)
         .byte mne_wdmx        ; $42  WDM
         .byte mne_eorx        ; $43  EOR <offset>,S
         .byte mne_mvpx        ; $44  MVP #sb,#db
         .byte mne_eorx        ; $45  EOR dp
         .byte mne_lsrx        ; $46  LSR dp
         .byte mne_eorx        ; $47  EOR [dp]
         .byte mne_phax        ; $48  PHA
         .byte mne_eorx        ; $49  EOR #
         .byte mne_lsrx        ; $4A  LSR A
         .byte mne_phkx        ; $4B  PHK
         .byte mne_jmpx        ; $4C  JMP abs
         .byte mne_eorx        ; $4D  EOR abs
         .byte mne_lsrx        ; $4E  LSR abs
         .byte mne_eorx        ; $4F  EOR absl
;
         .byte mne_bvcx        ; $50  BVC abs
         .byte mne_eorx        ; $51  EOR (<dp>),Y
         .byte mne_eorx        ; $52  EOR (dp)
         .byte mne_eorx        ; $53  EOR (<offset>,S),Y
         .byte mne_mvnx        ; $54  MVN #sb,#db
         .byte mne_eorx        ; $55  EOR dp,X
         .byte mne_lsrx        ; $56  LSR dp,X
         .byte mne_eorx        ; $57  EOR [dp],Y
         .byte mne_clix        ; $58  CLI
         .byte mne_eorx        ; $59  EOR abs,Y
         .byte mne_phyx        ; $5A  PHY
         .byte mne_tcdx        ; $5B  TCD
         .byte mne_jmlx        ; $5C  JML absl
         .byte mne_eorx        ; $5D  EOR abs,X
         .byte mne_lsrx        ; $5E  LSR abs,X
         .byte mne_eorx        ; $5F  EOR absl,X
;
         .byte mne_rtsx        ; $60  RTS
         .byte mne_adcx        ; $61  ADC (dp,X)
         .byte mne_perx        ; $62  PER
         .byte mne_adcx        ; $63  ADC <offset>,S
         .byte mne_stzx        ; $64  STZ dp
         .byte mne_adcx        ; $65  ADC dp
         .byte mne_rorx        ; $66  ROR dp
         .byte mne_adcx        ; $67  ADC [dp]
         .byte mne_plax        ; $68  PLA
         .byte mne_adcx        ; $69  ADC #
         .byte mne_rorx        ; $6A  ROR A
         .byte mne_rtlx        ; $6B  RTL
         .byte mne_jmpx        ; $6C  JMP (abs)
         .byte mne_adcx        ; $6D  ADC abs
         .byte mne_rorx        ; $6E  ROR abs
         .byte mne_adcx        ; $6F  ADC absl
;
         .byte mne_bvsx        ; $70  BVS abs
         .byte mne_adcx        ; $71  ADC (<dp>),Y
         .byte mne_adcx        ; $72  ADC (dp)
         .byte mne_adcx        ; $73  ADC (<offset>,S),Y
         .byte mne_stzx        ; $74  STZ dp,X
         .byte mne_adcx        ; $75  ADC dp,X
         .byte mne_rorx        ; $76  ROR dp,X
         .byte mne_adcx        ; $77  ADC [dp],Y
         .byte mne_seix        ; $78  SEI
         .byte mne_adcx        ; $79  ADC abs,Y
         .byte mne_plyx        ; $7A  PLY
         .byte mne_tdcx        ; $7B  TDC
         .byte mne_jmpx        ; $7C  JMP (abs,X)
         .byte mne_adcx        ; $7D  ADC abs,X
         .byte mne_rorx        ; $7E  ROR abs,X
         .byte mne_adcx        ; $7F  ADC absl,X
;
         .byte mne_brax        ; $80  BRA abs
         .byte mne_stax        ; $81  STA (dp,X)
         .byte mne_brlx        ; $82  BRL abs
         .byte mne_stax        ; $83  STA <offset>,S
         .byte mne_styx        ; $84  STY dp
         .byte mne_stax        ; $85  STA dp
         .byte mne_stxx        ; $86  STX dp
         .byte mne_stax        ; $87  STA [dp]
         .byte mne_deyx        ; $88  DEY
         .byte mne_bitx        ; $89  BIT #
         .byte mne_txax        ; $8A  TXA
         .byte mne_phbx        ; $8B  PHB
         .byte mne_styx        ; $8C  STY abs
         .byte mne_stax        ; $8D  STA abs
         .byte mne_stxx        ; $8E  STX abs
         .byte mne_stax        ; $8F  STA absl
;
         .byte mne_bccx        ; $90  BCC abs
         .byte mne_stax        ; $91  STA (<dp>),Y
         .byte mne_stax        ; $92  STA (dp)
         .byte mne_stax        ; $93  STA (<offset>,S),Y
         .byte mne_styx        ; $94  STY dp,X
         .byte mne_stax        ; $95  STA dp,X
         .byte mne_stxx        ; $96  STX dp,Y
         .byte mne_stax        ; $97  STA [dp],Y
         .byte mne_tyax        ; $98  TYA
         .byte mne_stax        ; $99  STA abs,Y
         .byte mne_txsx        ; $9A  TXS
         .byte mne_txyx        ; $9B  TXY
         .byte mne_stzx        ; $9C  STZ abs
         .byte mne_stax        ; $9D  STA abs,X
         .byte mne_stzx        ; $9E  STZ abs,X
         .byte mne_stax        ; $9F  STA absl,X
;
         .byte mne_ldyx        ; $A0  LDY #
         .byte mne_ldax        ; $A1  LDA (dp,X)
         .byte mne_ldxx        ; $A2  LDX #
         .byte mne_ldax        ; $A3  LDA <offset>,S
         .byte mne_ldyx        ; $A4  LDY dp
         .byte mne_ldax        ; $A5  LDA dp
         .byte mne_ldxx        ; $A6  LDX dp
         .byte mne_ldax        ; $A7  LDA [dp]
         .byte mne_tayx        ; $A8  TAY
         .byte mne_ldax        ; $A9  LDA #
         .byte mne_taxx        ; $AA  TAX
         .byte mne_plbx        ; $AB  PLB
         .byte mne_ldyx        ; $AC  LDY abs
         .byte mne_ldax        ; $AD  LDA abs
         .byte mne_ldxx        ; $AE  LDX abs
         .byte mne_ldax        ; $AF  LDA absl
;
         .byte mne_bcsx        ; $B0  BCS abs
         .byte mne_ldax        ; $B1  LDA (<dp>),Y
         .byte mne_ldax        ; $B2  LDA (dp)
         .byte mne_ldax        ; $B3  LDA (<offset>,S),Y
         .byte mne_ldyx        ; $B4  LDY dp,X
         .byte mne_ldax        ; $B5  LDA dp,X
         .byte mne_ldxx        ; $B6  LDX dp,Y
         .byte mne_ldax        ; $B7  LDA [dp],Y
         .byte mne_clvx        ; $B8  CLV
         .byte mne_ldax        ; $B9  LDA abs,Y
         .byte mne_tsxx        ; $BA  TSX
         .byte mne_tyxx        ; $BB  TYX
         .byte mne_ldyx        ; $BC  LDY abs,X
         .byte mne_ldax        ; $BD  LDA abs,X
         .byte mne_ldxx        ; $BE  LDX abs,Y
         .byte mne_ldax        ; $BF  LDA absl,X
;
         .byte mne_cpyx        ; $C0  CPY #
         .byte mne_cmpx        ; $C1  CMP (dp,X)
         .byte mne_repx        ; $C2  REP #
         .byte mne_cmpx        ; $C3  CMP <offset>,S
         .byte mne_cpyx        ; $C4  CPY dp
         .byte mne_cmpx        ; $C5  CMP dp
         .byte mne_decx        ; $C6  DEC dp
         .byte mne_cmpx        ; $C7  CMP [dp]
         .byte mne_inyx        ; $C8  INY
         .byte mne_cmpx        ; $C9  CMP #
         .byte mne_dexx        ; $CA  DEX
         .byte mne_waix        ; $CB  WAI
         .byte mne_cpyx        ; $CC  CPY abs
         .byte mne_cmpx        ; $CD  CMP abs
         .byte mne_decx        ; $CE  DEC abs
         .byte mne_cmpx        ; $CF  CMP absl
;
         .byte mne_bnex        ; $D0  BNE abs
         .byte mne_cmpx        ; $D1  CMP (<dp>),Y
         .byte mne_cmpx        ; $D2  CMP (dp)
         .byte mne_cmpx        ; $D3  CMP (<offset>,S),Y
         .byte mne_peix        ; $D4  PEI dp
         .byte mne_cmpx        ; $D5  CMP dp,X
         .byte mne_decx        ; $D6  DEC dp,X
         .byte mne_cmpx        ; $D7  CMP [dp],Y
         .byte mne_cldx        ; $D8  CLD
         .byte mne_cmpx        ; $D9  CMP abs,Y
         .byte mne_phxx        ; $DA  PHX
         .byte mne_stpx        ; $DB  STP
         .byte mne_jmpx        ; $DC  JMP [abs]
         .byte mne_cmpx        ; $DD  CMP abs,X
         .byte mne_decx        ; $DE  DEC abs,X
         .byte mne_cmpx        ; $DF  CMP absl,X
;
         .byte mne_cpxx        ; $E0  CPX #
         .byte mne_sbcx        ; $E1  SBC (dp,X)
         .byte mne_sepx        ; $E2  SEP #
         .byte mne_sbcx        ; $E3  SBC <offset>,S
         .byte mne_cpxx        ; $E4  CPX dp
         .byte mne_sbcx        ; $E5  SBC dp
         .byte mne_incx        ; $E6  INC dp
         .byte mne_sbcx        ; $E7  SBC [dp]
         .byte mne_inxx        ; $E8  INX
         .byte mne_sbcx        ; $E9  SBC #
         .byte mne_nopx        ; $EA  NOP
         .byte mne_xbax        ; $EB  XBA
         .byte mne_cpxx        ; $EC  CPX abs
         .byte mne_sbcx        ; $ED  SBC abs
         .byte mne_incx        ; $EE  INC abs
         .byte mne_sbcx        ; $EF  SBC absl
;
         .byte mne_beqx        ; $F0  BEQ abs
         .byte mne_sbcx        ; $F1  SBC (<dp>),Y
         .byte mne_sbcx        ; $F2  SBC (dp)
         .byte mne_sbcx        ; $F3  SBC (<offset>,S),Y
         .byte mne_peax        ; $F4  PEA #
         .byte mne_sbcx        ; $F5  SBC dp,X
         .byte mne_incx        ; $F6  INC dp,X
         .byte mne_sbcx        ; $F7  SBC [dp],Y
         .byte mne_sedx        ; $F8  SED
         .byte mne_sbcx        ; $F9  SBC abs,Y
         .byte mne_plxx        ; $FA  PLX
         .byte mne_xcex        ; $FB  XCE
         .byte mne_jsrx        ; $FC  JSR (abs,X)
         .byte mne_sbcx        ; $FD  SBC abs,X
         .byte mne_incx        ; $FE  INC abs,X
         .byte mne_sbcx        ; $FF  SBC absl,X
;
;
;	instruction addressing modes & sizes in opcode order...
;
;	    xxxxxxxx
;	    ||||||||
;	    ||||++++———> Addressing Mode
;	    ||||         ——————————————————————————————————
;	    ||||          0000  dp, abs, absl, implied or A
;	    ||||          0001  #
;	    ||||          0010  dp,X, abs,X or absl,X
;	    ||||          0011  dp,Y or abs,Y
;	    ||||          0100  (dp) or (abs)
;	    ||||          0101  [dp] or [abs]
;	    ||||          0110  [dp],Y
;	    ||||          0111  (dp,X) or (abs,X)
;	    ||||          1000  (<dp>),Y
;	    ||||          1001  <offset>,S
;	    ||||          1010  (<offset>,S),Y
;	    ||||          1011  #sbnk,#dbnk (MVN or MVP)
;	    ||||          —-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—-—
;	    ||||           #    = immediate
;	    ||||           A    = accumulator
;	    ||||           abs  = absolute
;	    ||||           absl = absolute long
;	    ||||           dbnk = destination bank
;	    ||||           dp   = direct (zero) page
;	    ||||           S    = stack relative
;	    ||||           sbnk = source bank
;	    ||||         ——————————————————————————————————
;	    ||||
;	    ||++———————> binary-encoded operand size
;	    |+—————————> 1: relative branch instruction
;	    +——————————> 1: variable operand size...
;
;	    —————————————————————————————————————————————————————————————
;	    Variable operand size refers to an immediate mode instruction
;	    that can accept either an 8 or 16-bit operand. The persistent
;	    .AS/.AL and .XS/.XL directives select that width, matching
;	    64tass. REP and SEP update the same width state.
;	    —————————————————————————————————————————————————————————————
;
mnetabam .byte ops0 | am_nam   ; $00  BRK
         .byte ops1 | am_indx  ; $01  ORA (dp,X)
         .byte ops1 | am_imm   ; $02  COP #<sig>
         .byte ops1 | am_stk   ; $03  ORA <offset>,S
         .byte ops1 | am_nam   ; $04  TSB dp
         .byte ops1 | am_nam   ; $05  ORA dp
         .byte ops1 | am_nam   ; $06  ASL dp
         .byte ops1 | am_indl  ; $07  ORA [dp]
         .byte ops0 | am_nam   ; $08  PHP
         .byte vops | am_imm   ; $09  ORA #
         .byte ops0 | am_nam   ; $0A  ASL A
         .byte ops0 | am_nam   ; $0B  PHD
         .byte ops2 | am_nam   ; $0C  TSB abs
         .byte ops2 | am_nam   ; $0D  ORA abs
         .byte ops2 | am_nam   ; $0E  ASL abs
         .byte ops3 | am_nam   ; $0F  ORA absl
;
         .byte bop1 | am_nam   ; $10  BPL abs
         .byte ops1 | am_indy  ; $11  ORA (<dp>),Y
         .byte ops1 | am_ind   ; $12  ORA (dp)
         .byte ops1 | am_stky  ; $13  ORA (<offset>,S),Y
         .byte ops1 | am_nam   ; $14  TRB dp
         .byte ops1 | am_adrx  ; $15  ORA dp,X
         .byte ops1 | am_adrx  ; $16  ASL dp,X
         .byte ops1 | am_indly ; $17  ORA [dp],Y
         .byte ops0 | am_nam   ; $18  CLC
         .byte ops2 | am_adry  ; $19  ORA abs,Y
         .byte ops0 | am_nam   ; $1A  INC A
         .byte ops0 | am_nam   ; $1B  TCS
         .byte ops2 | am_nam   ; $1C  TRB abs
         .byte ops2 | am_adrx  ; $1D  ORA abs,X
         .byte ops2 | am_adrx  ; $1E  ASL abs,X
         .byte ops3 | am_adrx  ; $1F  ORA absl,X
;
         .byte ops2 | am_nam   ; $20  JSR abs
         .byte ops1 | am_indx  ; $21  AND (dp,X)
         .byte ops3 | am_nam   ; $22  JSL absl
         .byte ops1 | am_stk   ; $23  AND <offset>,S
         .byte ops1 | am_nam   ; $24  BIT dp
         .byte ops1 | am_nam   ; $25  AND dp
         .byte ops1 | am_nam   ; $26  ROL dp
         .byte ops1 | am_indl  ; $27  AND [dp]
         .byte ops0 | am_nam   ; $28  PLP
         .byte vops | am_imm   ; $29  AND #
         .byte ops0 | am_nam   ; $2A  ROL A
         .byte ops0 | am_nam   ; $2B  PLD
         .byte ops2 | am_nam   ; $2C  BIT abs
         .byte ops2 | am_nam   ; $2D  AND abs
         .byte ops2 | am_nam   ; $2E  ROL abs
         .byte ops3 | am_nam   ; $2F  AND absl
;
         .byte bop1 | am_nam   ; $30  BMI abs
         .byte ops1 | am_indy  ; $31  AND (<dp>),Y
         .byte ops1 | am_ind   ; $32  AND (dp)
         .byte ops1 | am_stky  ; $33  AND (<offset>,S),Y
         .byte ops1 | am_adrx  ; $34  BIT dp,X
         .byte ops1 | am_adrx  ; $35  AND dp,X
         .byte ops1 | am_adrx  ; $36  ROL dp,X
         .byte ops1 | am_indly ; $37  AND [dp],Y
         .byte ops0 | am_nam   ; $38  SEC
         .byte ops2 | am_adry  ; $39  AND abs,Y
         .byte ops0 | am_nam   ; $3A  DEC A
         .byte ops0 | am_nam   ; $3B  TSC
         .byte ops2 | am_adrx  ; $3C  BIT abs,X
         .byte ops2 | am_adrx  ; $3D  AND abs,X
         .byte ops2 | am_adrx  ; $3E  ROL abs,X
         .byte ops3 | am_adrx  ; $3F  AND absl,X
;
         .byte ops0 | am_nam   ; $40  RTI
         .byte ops1 | am_indx  ; $41  EOR (dp,X)
         .byte ops0 | am_nam   ; $42  WDM
         .byte ops1 | am_stk   ; $43  EOR <offset>,S
         .byte ops2 | am_move  ; $44  MVP #sb,#db
         .byte ops1 | am_nam   ; $45  EOR dp
         .byte ops1 | am_nam   ; $46  LSR dp
         .byte ops1 | am_indl  ; $47  EOR [dp]
         .byte ops0 | am_nam   ; $48  PHA
         .byte vops | am_imm   ; $49  EOR #
         .byte ops0 | am_nam   ; $4A  LSR A
         .byte ops0 | am_nam   ; $4B  PHK
         .byte ops2 | am_nam   ; $4C  JMP abs
         .byte ops2 | am_nam   ; $4D  EOR abs
         .byte ops2 | am_nam   ; $4E  LSR abs
         .byte ops3 | am_nam   ; $4F  EOR absl
;
         .byte bop1 | am_nam   ; $50  BVC abs
         .byte ops1 | am_indy  ; $51  EOR (<dp>),Y
         .byte ops1 | am_ind   ; $52  EOR (dp)
         .byte ops1 | am_stky  ; $53  EOR (<offset>,S),Y
         .byte ops2 | am_move  ; $54  MVN #sb,#db
         .byte ops1 | am_adrx  ; $55  EOR dp,X
         .byte ops1 | am_adrx  ; $56  LSR dp,X
         .byte ops1 | am_indly ; $57  EOR [dp],Y
         .byte ops0 | am_nam   ; $58  CLI
         .byte ops2 | am_adry  ; $59  EOR abs,Y
         .byte ops0 | am_nam   ; $5A  PHY
         .byte ops0 | am_nam   ; $5B  TCD
         .byte ops3 | am_nam   ; $5C  JML absl
         .byte ops2 | am_adrx  ; $5D  EOR abs,X
         .byte ops2 | am_adrx  ; $5E  LSR abs,X
         .byte ops3 | am_adrx  ; $5F  EOR absl,X
;
         .byte ops0 | am_nam   ; $60  RTS
         .byte ops1 | am_indx  ; $61  ADC (dp,X)
         .byte bop2 | am_nam   ; $62  PER
         .byte ops1 | am_stk   ; $63  ADC <offset>,S
         .byte ops1 | am_nam   ; $64  STZ dp
         .byte ops1 | am_nam   ; $65  ADC dp
         .byte ops1 | am_nam   ; $66  ROR dp
         .byte ops1 | am_indl  ; $67  ADC [dp]
         .byte ops0 | am_nam   ; $68  PLA
         .byte vops | am_imm   ; $69  ADC #
         .byte ops0 | am_nam   ; $6A  ROR A
         .byte ops0 | am_nam   ; $6B  RTL
         .byte ops2 | am_ind   ; $6C  JMP (abs)
         .byte ops2 | am_nam   ; $6D  ADC abs
         .byte ops2 | am_nam   ; $6E  ROR abs
         .byte ops3 | am_nam   ; $6F  ADC absl
;
         .byte bop1 | am_nam   ; $70  BVS abs
         .byte ops1 | am_indy  ; $71  ADC (<dp>),Y
         .byte ops1 | am_ind   ; $72  ADC (dp)
         .byte ops1 | am_stky  ; $73  ADC (<offset>,S),Y
         .byte ops1 | am_adrx  ; $74  STZ dp,X
         .byte ops1 | am_adrx  ; $75  ADC dp,X
         .byte ops1 | am_adrx  ; $76  ROR dp,X
         .byte ops1 | am_indly ; $77  ADC [dp],Y
         .byte ops0 | am_nam   ; $78  SEI
         .byte ops2 | am_adry  ; $79  ADC abs,Y
         .byte ops0 | am_nam   ; $7A  PLY
         .byte ops0 | am_nam   ; $7B  TDC
         .byte ops2 | am_indx  ; $7C  JMP (abs,X)
         .byte ops2 | am_adrx  ; $7D  ADC abs,X
         .byte ops2 | am_adrx  ; $7E  ROR abs,X
         .byte ops3 | am_adrx  ; $7F  ADC absl,X
;
         .byte bop1 | am_nam   ; $80  BRA abs
         .byte ops1 | am_indx  ; $81  STA (dp,X)
         .byte bop2 | am_nam   ; $82  BRL abs
         .byte ops1 | am_stk   ; $83  STA <offset>,S
         .byte ops1 | am_nam   ; $84  STY dp
         .byte ops1 | am_nam   ; $85  STA dp
         .byte ops1 | am_nam   ; $86  STX dp
         .byte ops1 | am_indl  ; $87  STA [dp]
         .byte ops0 | am_nam   ; $88  DEY
         .byte vops | am_imm   ; $89  BIT #
         .byte ops0 | am_nam   ; $8A  TXA
         .byte ops0 | am_nam   ; $8B  PHB
         .byte ops2 | am_nam   ; $8C  STY abs
         .byte ops2 | am_nam   ; $8D  STA abs
         .byte ops2 | am_nam   ; $8E  STX abs
         .byte ops3 | am_nam   ; $8F  STA absl
;
         .byte bop1 | am_nam   ; $90  BCC abs
         .byte ops1 | am_indy  ; $91  STA (<dp>),Y
         .byte ops1 | am_ind   ; $92  STA (dp)
         .byte ops1 | am_stky  ; $93  STA (<offset>,S),Y
         .byte ops1 | am_adrx  ; $94  STY dp,X
         .byte ops1 | am_adrx  ; $95  STA dp,X
         .byte ops1 | am_adry  ; $96  STX dp,Y
         .byte ops1 | am_indly ; $97  STA [dp],Y
         .byte ops0 | am_nam   ; $98  TYA
         .byte ops2 | am_adry  ; $99  STA abs,Y
         .byte ops0 | am_nam   ; $9A  TXS
         .byte ops0 | am_nam   ; $9B  TXY
         .byte ops2 | am_nam   ; $9C  STZ abs
         .byte ops2 | am_adrx  ; $9D  STA abs,X
         .byte ops2 | am_adrx  ; $9E  STZ abs,X
         .byte ops3 | am_adrx  ; $9F  STA absl,X
;
         .byte vops | am_imm   ; $A0  LDY #
         .byte ops1 | am_indx  ; $A1  LDA (dp,X)
         .byte vops | am_imm   ; $A2  LDX #
         .byte ops1 | am_stk   ; $A3  LDA <offset>,S
         .byte ops1 | am_nam   ; $A4  LDY dp
         .byte ops1 | am_nam   ; $A5  LDA dp
         .byte ops1 | am_nam   ; $A6  LDX dp
         .byte ops1 | am_indl  ; $A7  LDA [dp]
         .byte ops0 | am_nam   ; $A8  TAY
         .byte vops | am_imm   ; $A9  LDA #
         .byte ops0 | am_nam   ; $AA  TAX
         .byte ops0 | am_nam   ; $AB  PLB
         .byte ops2 | am_nam   ; $AC  LDY abs
         .byte ops2 | am_nam   ; $AD  LDA abs
         .byte ops2 | am_nam   ; $AE  LDX abs
         .byte ops3 | am_nam   ; $AF  LDA absl
;
         .byte bop1 | am_nam   ; $B0  BCS abs
         .byte ops1 | am_indy  ; $B1  LDA (<dp>),Y
         .byte ops1 | am_ind   ; $B2  LDA (dp)
         .byte ops1 | am_stky  ; $B3  LDA (<offset>,S),Y
         .byte ops1 | am_adrx  ; $B4  LDY dp,X
         .byte ops1 | am_adrx  ; $B5  LDA dp,X
         .byte ops1 | am_adry  ; $B6  LDX dp,Y
         .byte ops1 | am_indly ; $B7  LDA [dp],Y
         .byte ops0 | am_nam   ; $B8  CLV
         .byte ops2 | am_adry  ; $B9  LDA abs,Y
         .byte ops0 | am_nam   ; $BA  TSX
         .byte ops0 | am_nam   ; $BB  TYX
         .byte ops2 | am_adrx  ; $BC  LDY abs,X
         .byte ops2 | am_adrx  ; $BD  LDA abs,X
         .byte ops2 | am_adry  ; $BE  LDX abs,Y
         .byte ops3 | am_adrx  ; $BF  LDA absl,X
;
         .byte vops | am_imm   ; $C0  CPY #
         .byte ops1 | am_indx  ; $C1  CMP (dp,X)
         .byte ops1 | am_imm   ; $C2  REP #
         .byte ops1 | am_stk   ; $C3  CMP <offset>,S
         .byte ops1 | am_nam   ; $C4  CPY dp
         .byte ops1 | am_nam   ; $C5  CMP dp
         .byte ops1 | am_nam   ; $C6  DEC dp
         .byte ops1 | am_indl  ; $C7  CMP [dp]
         .byte ops0 | am_nam   ; $C8  INY
         .byte vops | am_imm   ; $C9  CMP #
         .byte ops0 | am_nam   ; $CA  DEX
         .byte ops0 | am_nam   ; $CB  WAI
         .byte ops2 | am_nam   ; $CC  CPY abs
         .byte ops2 | am_nam   ; $CD  CMP abs
         .byte ops2 | am_nam   ; $CE  DEC abs
         .byte ops3 | am_nam   ; $CF  CMP absl
;
         .byte bop1 | am_nam   ; $D0  BNE abs
         .byte ops1 | am_indy  ; $D1  CMP (<dp>),Y
         .byte ops1 | am_ind   ; $D2  CMP (dp)
         .byte ops1 | am_stky  ; $D3  CMP (<offset>,S),Y
         .byte ops1 | am_nam   ; $D4  PEI dp
         .byte ops1 | am_adrx  ; $D5  CMP dp,X
         .byte ops1 | am_adrx  ; $D6  DEC dp,X
         .byte ops1 | am_indly ; $D7  CMP [dp],Y
         .byte ops0 | am_nam   ; $D8  CLD
         .byte ops2 | am_adry  ; $D9  CMP abs,Y
         .byte ops0 | am_nam   ; $DA  PHX
         .byte ops0 | am_nam   ; $DB  STP
         .byte ops2 | am_indl  ; $DC  JMP [abs]
         .byte ops2 | am_adrx  ; $DD  CMP abs,X
         .byte ops2 | am_adrx  ; $DE  DEC abs,X
         .byte ops3 | am_adrx  ; $DF  CMP absl,X
;
         .byte vops | am_imm   ; $E0  CPX #
         .byte ops1 | am_indx  ; $E1  SBC (dp,X)
         .byte ops1 | am_imm   ; $E2  SEP #
         .byte ops1 | am_stk   ; $E3  SBC <offset>,S
         .byte ops1 | am_nam   ; $E4  CPX dp
         .byte ops1 | am_nam   ; $E5  SBC dp
         .byte ops1 | am_nam   ; $E6  INC dp
         .byte ops1 | am_indl  ; $E7  SBC [dp]
         .byte ops0 | am_nam   ; $E8  INX
         .byte vops | am_imm   ; $E9  SBC #
         .byte ops0 | am_nam   ; $EA  NOP
         .byte ops0 | am_nam   ; $EB  XBA
         .byte ops2 | am_nam   ; $EC  CPX abs
         .byte ops2 | am_nam   ; $ED  SBC abs
         .byte ops2 | am_nam   ; $EE  INC abs
         .byte ops3 | am_nam   ; $EF  SBC absl
;
         .byte bop1 | am_nam   ; $F0  BEQ abs
         .byte ops1 | am_indy  ; $F1  SBC (<dp>),Y
         .byte ops1 | am_ind   ; $F2  SBC (dp)
         .byte ops1 | am_stky  ; $F3  SBC (<offset>,S),Y
         .byte ops2 | am_imm   ; $F4  PEA #<word>
         .byte ops1 | am_adrx  ; $F5  SBC dp,X
         .byte ops1 | am_adrx  ; $F6  INC dp,X
         .byte ops1 | am_indly ; $F7  SBC [dp],Y
         .byte ops0 | am_nam   ; $F8  SED
         .byte ops2 | am_adry  ; $F9  SBC abs,Y
         .byte ops0 | am_nam   ; $FA  PLX
         .byte ops0 | am_nam   ; $FB  XCE
         .byte ops2 | am_indx  ; $FC  JSR (abs,X)
         .byte ops2 | am_adrx  ; $FD  SBC abs,X
         .byte ops2 | am_adrx  ; $FE  INC abs,X
         .byte ops3 | am_adrx  ; $FF  SBC absl,X
;
;
;	.X & .Y immediate mode opcodes...
;
vopidx   .byte opc_cpxi        ;CPX #
         .byte opc_cpyi        ;CPY #
         .byte opc_ldxi        ;LDX #
         .byte opc_ldyi        ;LDY #
n_vopidx =*-vopidx             ;number of opcodes
;
;
;	addressing mode symbology lookup...
;
ms_lutab .word ms_nam          ;no symbol
         .word ms_imm          ;#
         .word ms_addrx        ;<addr>,X
         .word ms_addry        ;<addr>,Y
         .word ms_ind          ;(<addr>)
         .word ms_indl         ;[<dp>]
         .word ms_indly        ;[<dp>],Y
         .word ms_indx         ;(<addr>,X)
         .word ms_indy         ;(<dp>),Y
         .word ms_stk          ;<offset>,S
         .word ms_stky         ;(<offset>,S),Y
         .word ms_imm          ;#<sbnk>,#<dbnk>
;
;
;	addressing mode symbology strings...
;
ms_nam   .text " "
         .byte 0;no symbol
ms_addrx .text " ,X"
         .byte 0;<addr>,X
ms_addry .text " ,Y"
         .byte 0;<addr>,Y
ms_imm   .text "#"
         .byte 0;#
ms_ind   .text "()"
         .byte 0;(<addr>)
ms_indl  .text "[]"
         .byte 0;[<dp>]
ms_indly .text "[],Y"
         .byte 0;[<dp>],Y
ms_indx  .text "(,X)"
         .byte 0;(<addr>,X)
ms_indy  .text "(),Y"
         .byte 0;(<dp>),Y
ms_move  .text ",#$"
         .byte 0;#<sbnk>,#<dbnk>
ms_stk   .text " ,S"
         .byte 0;<offset>,S
ms_stky  .text "(,S),Y"
         .byte 0;(<offset>,S),Y
;
;===============================================================================
;
;CONSOLE DISPLAY CONTROL STRINGS
;
dc_bf    #bf                    ;reverse foreground
         .byte 0
;
dc_bs    #bs                    ;destructive backspace
         .byte 0
;
dc_cl    #cl                    ;clear to end of line 
         .byte 0
;
dc_cn    #cn                    ;cursor on
         .byte 0
;
dc_co    #co                    ;cursor off
         .byte 0
;
dc_lf    #lf                    ;newline
         .byte 0
;
dc_sf    #sf                    ;normal foreground
         .byte 0
;
;===============================================================================
;
;MONITOR STRINGS
;
mm_brk   #rb
         #lf
         .text "*BRK"
         #lf
         .byte 0
;
mm_entry #lf
         .byte a_lf
         .text "Supermon 816 "
         #softvers
         #lf
         .byte 0
;
mm_err   .text " *ERR"
         .byte 0
;
mm_prmpt #lf
         #sf
         .text "."
         .byte 0
;
mm_regs  #lf
         .text "  PB  PC   nvmxdizc  .C   .X   .Y   SP   DP  DB"
         #lf
         .text "; "
         .byte 0
;
mm_rts   #rb
         #lf
         .text "*RTS"
         #lf
         .byte 0
;
;===============================================================================
;
;S-RECORD PROCESSING TEXT STRINGS
;
lr_addre #lf
         .text "  End: $"
         .byte 0
;
lr_addrs #lf
         .text "Start: $"
         #cl
         .byte 0
;
lr_bufor #lf
         .text "Buffer overrun"
         #cl
         #rb
         .byte 0
;
lr_corup #lf
         .text "Bad record"
         #cl
         #rb
         .byte 0
;
lr_load  #co
         #cr
         #cl
         .text "Loading: "
         .byte 0
;
lr_norec #lf
         .text "No S1 records"
         #cl
         #rb
         .byte 0
;
lr_recct #lf
         .text "Bad S1 count"
         #cl
         #rb
         .byte 0
;
lr_rtl   #cr
         #cl
         .text "Ready: "
         #cn
         .byte 0
;
lr_type  #lf
         .text "Invalid record"
         #cl
         #rb
         .byte 0
;
.if * >= ibuffer
         .error "SUPERMON816 overlaps its $6000 input buffer"
.endif
;
text_end =*                     ;end of program text
;
;===============================================================================

