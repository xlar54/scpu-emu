// Boot the CMD ROM on the host, then LOAD"$",10 against a fake IEC drive.
// The drive implements the standard slow protocol, event-driven off the
// host accesses of $DD00 -- as fast as the host polls, which the handshake
// permits. If the LOAD completes here, CMD logic + our emulation agree and
// the hardware stall is timing; if it stalls at the same PC, it is
// reproducible and steppable locally.
#include "Source/SuperCPU/supercpu.h"
#include "Source/Bus/Host/host_bus.h"
#include <cstdio>
#include <cstring>

#define H_ATN  0x08
#define H_CLK  0x10
#define H_DATA 0x20

static const char *stateNames[] = {
    "IDLE","CMD_WAITREADY","CMD_BITS","CMD_ACK",
    "TURNAROUND","SEND_READY","SEND_EOIACK","SEND_BITS","SEND_ACKWAIT","UNTALKED"
};

class CFakeDriveBus : public CHostBus
{
public:
    enum State { IDLE, CMD_WAITREADY, CMD_BITS, CMD_ACK,
                 TURNAROUND, SEND_READY, SEND_EOIACK, SEND_BITS, SEND_ACKWAIT, UNTALKED };

    State st = IDLE;
    u8   hostOut = 0;
    bool drvCLK = false, drvDATA = false;
    u8   shiftIn = 0, bitCount = 0;
    bool atnMode = false, talking = false, listening = false;
    u8   device = 10;
    int  cmdLog[64]; int cmdLogN = 0;
    const u8 *txBuf = 0; u32 txLen = 0, txPos = 0;
    u8   txByte = 0, txBit = 0, txPhase = 0;
    bool eoiPending = false, eoiDone = false;
    bool verbose = false;

    u8 dir[128]; u32 dirLen = 0;
    CFakeDriveBus()
    {
        u32 n = 0;
        dir[n++]=0x01; dir[n++]=0x04;               // load address $0401
        u32 link = 0x0401 + 15;
        dir[n++]=(u8)(link&0xFF); dir[n++]=(u8)(link>>8);
        dir[n++]=0; dir[n++]=0;                     // line number 0
        const char *t = "\"FAKEDISK\"";
        for (const char*p=t;*p;p++) dir[n++]=(u8)*p;
        dir[n++]=0;
        dir[n++]=0; dir[n++]=0;                     // end of program
        dirLen = n;
    }

    void note(const char *what)
    {
        if (verbose) printf("      [drive] %-28s st=%s ATN=%d hCLK=%d hDATA=%d dCLK=%d dDATA=%d\n",
            what, stateNames[st], !!(hostOut&H_ATN), !!(hostOut&H_CLK), !!(hostOut&H_DATA),
            (int)drvCLK, (int)drvDATA);
    }

    u8 lines() const
    {
        u8 v = (u8)(hostOut & 0x3F);
        if (!((hostOut & H_CLK)  || drvCLK))  v |= 0x40;
        if (!((hostOut & H_DATA) || drvDATA)) v |= 0x80;
        return v;
    }

    void hostWrite(u8 v)
    {
        u8 prev = hostOut;
        hostOut = v;
        bool atn = (v & H_ATN) != 0, atnPrev = (prev & H_ATN) != 0;

        if (atn && !atnPrev) {
            atnMode = true; drvDATA = true; drvCLK = false;
            st = CMD_WAITREADY; note("ATN: ack with DATA");
        }
        if (!atn && atnPrev) {
            atnMode = false; note("ATN released");
            if (talking) { st = TURNAROUND; note("await turnaround"); }
            else if (listening) { st = CMD_WAITREADY; }
            else { drvDATA = false; st = IDLE; }
        }
        step(prev);
    }

    void step(u8 prevOut)
    {
        bool hCLKlow = (hostOut & H_CLK) != 0;
        bool hCLKlowPrev = (prevOut & H_CLK) != 0;
        bool hDATAlow = (hostOut & H_DATA) != 0;

        switch (st) {
        case CMD_WAITREADY:
            // The talker's ready signal is a CLK RELEASE EDGE, not CLK merely
            // being high -- the drive must keep holding DATA (device-present)
            // until the talker actually pulls CLK and releases it again.
            if (!hCLKlow && hCLKlowPrev) { drvDATA = false; shiftIn = 0; bitCount = 0; st = CMD_BITS; note("ready for byte"); }
            break;
        case CMD_BITS:
            if (!hCLKlow && hCLKlowPrev) {
                u8 bit = hDATAlow ? 0 : 1;
                shiftIn = (u8)((shiftIn >> 1) | (bit << 7));
                if (++bitCount == 8) { drvDATA = true; st = CMD_ACK; byteDone(); }
            }
            break;
        case CMD_ACK:
            if (hCLKlow) st = (atnMode || listening) ? CMD_WAITREADY : IDLE;
            break;
        case TURNAROUND:
            // Host becomes listener: pulls DATA, releases CLK. The drive must
            // TAKE CLK LOW first -- that is the takeover confirmation the host
            // waits for -- and only then run the normal byte cycle.
            if (hDATAlow && !hCLKlow) {
                drvDATA = false;
                drvCLK = true;                // take the clock line
                takeoverReads = 0;
                st = SEND_READY;              // prepByte-like, but hold first
                txByte = txBuf ? txBuf[txPos] : 0;
                eoiPending = (txPos + 1 >= txLen);
                eoiDone = false;
                holdingTakeover = true;
                note("turnaround: drive took CLK");
            }
            break;
        case SEND_READY:
            if (!hDATAlow) {
                if (eoiPending && !eoiDone) { st = SEND_EOIACK; drvCLK = false; note("EOI wait"); }
                else { st = SEND_BITS; txBit = 0; txPhase = 0; phaseReads = 0;
                       drvCLK = true;   // start-of-byte: pull CLK before bit 0
                       note("sending byte"); }
            }
            break;
        case SEND_EOIACK:
            if (hDATAlow) eoiDone = true;
            else if (eoiDone) { st = SEND_BITS; txBit = 0; txPhase = 0; phaseReads = 0;
                                drvCLK = true; note("EOI acked"); }
            break;
        case SEND_ACKWAIT:
            if (hDATAlow) {
                if (txPos >= txLen) { drvCLK = false; drvDATA = false; st = IDLE; note("transfer done"); }
                else prepByte();
            }
            break;
        default: break;
        }
    }

    void prepByte()
    {
        txByte = txBuf ? txBuf[txPos] : 0;
        eoiPending = (txPos + 1 >= txLen);
        eoiDone = false;
        drvCLK = false;               // ready-to-send: release CLK
        st = SEND_READY;
        note("byte prepped, CLK released");
    }

    // Advance the talker's bit clocking on host polls of $DD00 -- but each
    // half-phase must stay STABLE across several polls, because the KERNAL
    // debounces: it reads the port twice and requires identical values. A
    // drive that flips state between the two reads never passes the debounce.
    u8 phaseReads = 0;
    void talkerAdvance()
    {
        if (st != SEND_BITS) return;
        if (++phaseReads < 5) return;         // hold: at least 2 clean debounce pairs
        phaseReads = 0;
        if (txPhase == 0) {
            u8 bit = (u8)((txByte >> txBit) & 1);
            drvDATA = (bit == 0);     // logical 0 = line pulled low
            drvCLK = false;           // CLK released = bit valid
            txPhase = 1;
        } else {
            drvCLK = true;            // CLK pulled = between bits
            drvDATA = false;
            txPhase = 0;
            if (++txBit == 8) {
                txPos++;
                drvCLK = true;        // byte complete: talker holds CLK
                st = SEND_ACKWAIT;    // and waits for the listener's DATA ack
                note("byte sent, await ack");
            }
        }
    }

    void byteDone()
    {
        u8 b = shiftIn;
        if (cmdLogN < 64) cmdLog[cmdLogN++] = b | (atnMode ? 0x100 : 0);
        if (verbose) printf("      [drive] byte %s: $%02X\n", atnMode ? "ATN" : "data", b);
        if (atnMode) {
            if (b == (u8)(0x20 + device)) listening = true;
            else if (b == 0x3F) listening = false;
            else if (b == (u8)(0x40 + device)) talking = true;
            else if (b == 0x5F) { talking = false; drvCLK = drvDATA = false; st = UNTALKED; }
            else if ((b & 0xF0) == 0x60 && talking) { txBuf = dir; txLen = dirLen; txPos = 0; }
        }
    }

    // --- coarse interrupt synthesis ------------------------------------
    // Once per frame the harness calls onFrame(): a VIC raster interrupt if
    // enabled in $D01A, and a CIA1 timer-A tick if enabled in its mask. Frame
    // granularity is exactly what the CMD splash needs -- its handler
    // decrements a frame counter -- and enough for the KERNAL jiffy.
    bool vicPending = false, ciaPending = false;
    u8   cia1Mask = 0;
    // CIA1 timer B one-shot, used by the KERNAL's EOI timeout: a write to
    // $DC0F with the start bit arms it; a few $DC0D polls later it reports
    // underflow (bit 1).
    bool tbArmed = false; u32 tbCountdown = 0;

    void recomputeIRQ() { m_IRQ = vicPending || ciaPending; }
    void onFrame()
    {
        if (m_Memory[0xD01A] & 1) vicPending = true;
        if (cia1Mask & 1) ciaPending = true;
        recomputeIRQ();
    }

    // Listener-side EOI: when the talker holds its ready state unusually long
    // before the first bit, that is the EOI signal, and the listener must
    // answer with a DATA pulse. Event-driven stand-in for the 200us rule: many
    // consecutive polls with no edge = "unusually long".
    u32 idleReads = 0; bool eoiAckPhase = false; u8 eoiAckReads = 0;
    u32 takeoverReads = 0; bool holdingTakeover = false;

    void listenerEOI(bool hCLKlow)
    {
        if (st != CMD_BITS || bitCount != 0) { idleReads = 0; return; }
        if (hCLKlow) { idleReads = 0; return; }
        if (eoiAckPhase) {
            if (++eoiAckReads > 8) { drvDATA = false; eoiAckPhase = false; note("EOI ack done"); }
            return;
        }
        if (++idleReads > 48) { drvDATA = true; eoiAckPhase = true; eoiAckReads = 0; idleReads = 0; note("EOI ack pulse"); }
    }

    u8 read(u16 addr) override
    {
        if (addr == 0xDD00) {
            if (holdingTakeover && ++takeoverReads > 8) {
                holdingTakeover = false;
                drvCLK = false;                // now signal ready-to-send
                note("takeover hold done, CLK released");
            }
            talkerAdvance(); listenerEOI((hostOut & H_CLK) != 0); return lines();
        }
        if (addr == 0xDC0D) {
            u8 v = ciaPending ? 0x81 : 0x00;
            if (tbArmed && st == SEND_EOIACK && --tbCountdown == 0) { v |= 0x82; tbArmed = false; }
            ciaPending = false; recomputeIRQ(); return v;
        }
        if (addr == 0xD019) { return vicPending ? 0x81 : 0x00; }
        return CHostBus::read(addr);
    }
    void write(u16 addr, u8 v) override
    {
        if (addr == 0xDD00) { hostWrite(v); m_Memory[addr] = v; return; }
        if (addr == 0xDC0D) {
            // set/clear mask semantics
            if (v & 0x80) cia1Mask |= (v & 0x7F); else cia1Mask &= (u8)~(v & 0x7F);
            m_Memory[addr] = v; return;
        }
        if (addr == 0xDC0F) {
            if (v & 0x01) { tbArmed = true; tbCountdown = 24; }
            m_Memory[addr] = v; return;
        }
        if (addr == 0xD019) { if (v & 1) { vicPending = false; recomputeIRQ(); } m_Memory[addr] = v; return; }
        CHostBus::write(addr, v);
    }
};

static bool loadFile(const char *p, u8 *dst, u32 n){
    FILE *f = fopen(p,"rb"); if(!f) return false;
    size_t g = fread(dst,1,n,f); fclose(f); return g==n;
}

int main(int argc, char **argv)
{
    static CFakeDriveBus bus;
    static CSuperCPU scpu;
    static u8 kernal[8192], basic[8192], chargen[4096], rom[131072];

    if(!loadFile("ROMs/kernal.rom",kernal,8192)||!loadFile("ROMs/basic.rom",basic,8192)
     ||!loadFile("ROMs/scpu-dos-1.4.bin",rom,131072)){ printf("rom load failed\n"); return 1; }
    loadFile("ROMs/chargen.rom",chargen,4096);

    scpu.setKernalROM(kernal); scpu.setBasicROM(basic); scpu.setCharROM(chargen);
    for(u32 a=0xDC00;a<=0xDDFF;a++) bus.m_Memory[a]=0xFF;
    bus.m_Memory[0xDD00]=0x3F; bus.m_Memory[0xDD02]=0x3F;

    scpu.memoryMap().setROM(rom,sizeof(rom));
    scpu.setBootmapEnabled(true);
    if(!scpu.init(&bus,SCPU_CORE_65816,SCPU_SIMM_16MB)){ printf("init failed\n"); return 1; }

    // Sanity: does the throttle arm at all, in isolation?
    scpu.memory().read8(0xDD00);
    scpu.memory().write8(0xDD00, (u8)(bus.m_Memory[0xDD00] ^ 0x08));   // toggle ATN
    printf("isolated arm check: events=%llu\n",
           (unsigned long long)scpu.memory().m_IECThrottleEvents);

    for(u32 f=0; f<400; f++) { bus.onFrame(); scpu.runFrame(); }
    printf("after boot: PC=$%06X\n",(unsigned)scpu.cpu()->pc());

    const char *cmd = "LOAD\"$\",10\r";
    for(u32 i=0; cmd[i]; i++) scpu.memory().write8((u16)(0x0277+i),(u8)cmd[i]);
    scpu.memory().write8(0x00C6,(u8)strlen(cmd));

    bus.verbose = (argc > 1);

    u32 lastPC = 0, stuck = 0;
    for(u32 f=0; f<4000; f++){
        bus.onFrame();
        scpu.runFrame();
        u32 pc = (u32)scpu.cpu()->pc();
        if (pc == lastPC) { if (++stuck == 100) break; } else stuck = 0;
        lastPC = pc;
    }

    {
        u32 pc = (u32)scpu.cpu()->pc() & 0xFFFF;
        printf("code at PC-8..PC+15:");
        for (int i=-8;i<16;i++) printf(" %02X", scpu.memory().m_RAM[(u16)((int)pc+i)]);
        printf("\n");
    }

    printf("IEC throttle events: %llu  (hold active now: %s)\n",
           (unsigned long long)scpu.memory().m_IECThrottleEvents,
           scpu.memory().iecThrottleActive() ? "yes" : "no");
    printf("cmd bytes (A:=under ATN): ");
    for(int i=0;i<bus.cmdLogN;i++) printf("%s%02X ", (bus.cmdLog[i]&0x100)?"A:":"", bus.cmdLog[i]&0xFF);
    printf("\nfinal: PC=$%06X drive=%s txPos=%u/%u\n",(unsigned)scpu.cpu()->pc(),
           stateNames[bus.st], bus.txPos, bus.txLen);

    const u8 *ram = scpu.memory().m_RAM;
    for(u32 row=0;row<25;row++){
        char line[41]; bool blank=true;
        for(u32 col=0;col<40;col++){
            u8 c = ram[0x0400+row*40+col];
            char ch=(c==0x20||c==0)?' ':(c>=1&&c<=26)?(char)(65+c-1):(c>=0x30&&c<=0x39)?(char)(48+c-0x30):(c==0x24)?'$':(c==0x22)?'"':'.';
            if(ch!=' ')blank=false; line[col]=ch;
        }
        line[40]=0; if(!blank) printf("  |%s|\n",line);
    }
    return 0;
}
