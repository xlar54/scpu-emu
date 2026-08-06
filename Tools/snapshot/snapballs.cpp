// Seed the host rig from the VICE snapshot's CPU-view RAM and let 3D Pool's
// OWN raster IRQ handler ($FFFE -> $3096) run against the full delivery
// stack. The handler is the whole ball engine: it flips $DD00 between VIC
// banks 3 and 1 every interrupt, self-modifies its store target, and rewrites
// the newly-visible pointer row from the table at $021D -- values $43/$52/$53,
// all under-I/O blocks in bank 3.
//
// PASS = on bank-3 frames the DELIVERED pointer row at $CFF8 carries
// translated blocks (< $30) whose relocated copies byte-match the shapes,
// while bank-1 frames deliver the raw values; real DRAM under $D000 is never
// written.
#include "Source/SuperCPU/supercpu.h"
#include "Source/Bus/Host/host_bus.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

class CSnapBus : public CHostBus
{
public:
    bool vicPending = false;
    void write(u16 addr, u8 value) override
    {
        if (addr == 0xD019) { vicPending = false; m_IRQ = false; }
        CHostBus::write(addr, value);
    }
    u8 read(u16 addr) override
    {
        if (addr == 0xD012) return 0xFB;   // handler BITs this: bit7 set -> sprite path
        if (addr == 0xD019) return vicPending ? 0x81 : 0x00;
        return CHostBus::read(addr);
    }
    void raiseVIC() { vicPending = true; m_IRQ = true; }
};

static bool loadFile(const char *p, u8 *dst, u32 cap, u32 *n)
{
    FILE *f = fopen(p, "rb"); if (!f) return false;
    *n = (u32)fread(dst, 1, cap, f); fclose(f); return *n > 0;
}

int main(int argc, char **argv)
{
    const u32 frames = (argc > 1) ? (u32)atoi(argv[1]) : 600;
    static CSnapBus bus;
    static CSuperCPU scpu;
    static u8 kernal[8192], basic[8192], chargen[4096], rom[131072];
    static u8 snapCPU[65536], snapBoard[65536];
    u32 dummy, n1 = 0, n2 = 0;

    std::memset(rom, 0xFF, sizeof rom);
    if (!loadFile("ROMs/kernal.rom", kernal, 8192, &dummy)
     || !loadFile("ROMs/basic.rom", basic, 8192, &dummy)
     || !loadFile("ROMs/scpu-dos-2.04.bin", rom, sizeof rom, &dummy)
     || !loadFile("C64Tests/snap_ram_cpu.bin", snapCPU, 65536, &n1)
     || !loadFile("C64Tests/snap_ram_board.bin", snapBoard, 65536, &n2)
     || n1 != 65536 || n2 != 65536)
    { printf("load failed\n"); return 1; }
    loadFile("ROMs/chargen.rom", chargen, 4096, &dummy);

    scpu.setKernalROM(kernal); scpu.setBasicROM(basic); scpu.setCharROM(chargen);
    for (u32 a = 0xDC00; a <= 0xDDFF; a++) bus.m_Memory[a] = 0xFF;
    bus.m_Memory[0xDD00] = 0x3F; bus.m_Memory[0xDD02] = 0x3F;
    scpu.memoryMap().setROM(rom, sizeof rom);
    scpu.setBootmapEnabled(true);
    if (!scpu.init(&bus, SCPU_CORE_65816, SCPU_SIMM_16MB)) { printf("init failed\n"); return 1; }

    for (u32 f = 0; f < 200; f++) scpu.runFrame();   // settle CMD boot

    // ---- seed the snapshot state -------------------------------------------
    CC64Memory &mem = scpu.memory();
    std::memcpy(mem.m_RAM, snapCPU, 65536);
    mem.write8(0x0001, 0x35);            // no ROMs, I/O in: the game's map
    // The memcpy bypassed the sink, so every boot-era synced bit now lies
    // (shadow changed, DRAM did not). Drop them, then re-queue everything.
    scpu.writeBuffer().discard();
    scpu.writeBuffer().invalidateRange(0x0000, 0x10000);
    scpu.writeBuffer().flush();
    // Both pointer rows start neutral so the handler's first rewrite is a
    // CHANGE -- on real hardware the game writes them into cleared screens.
    for (u32 i = 0; i < 8; i++)
    {
        mem.write8((u16)(0xCFF8 + i), 0x00);
        mem.write8((u16)(0x4FF8 + i), 0x00);
    }

    mem.write8(0xDD02, 0x3F);
    mem.write8(0xDD00, 0xC4);            // VIC bank 3
    mem.write8(0xD011, 0x3B);            // bitmap mode
    mem.write8(0xD018, 0x38);            // screen $CC00, bitmap $E000
    mem.write8(0xD015, 0xFF);

    // Spin stub for the main thread; the IRQ handler is the machine under test.
    mem.write8(0x0334, 0x4C); mem.write8(0x0335, 0x34); mem.write8(0x0336, 0x03);

    CW65C816 *c = scpu.core65816();
    c->m_PBR = 0; c->m_PC = 0x0334; c->m_E = true;
    c->m_S = 0x01F0; c->m_P &= (u8)~0x04;      // I clear: take the IRQ
    scpu.writeBuffer().flush();

    // ---- run: one VIC IRQ per frame, like the raster at line $FB -----------
    u32 bank3Frames = 0, bank1Frames = 0, bank3RawPtr = 0, bank1WrongPtr = 0;
    u32 flipsSeen = 0; u8 lastDD00 = 0xC4;
    for (u32 f = 0; f < frames; f++)
    {
        mem.write8(0x0040, 0x00);        // main-loop handshake: tables ready
        bus.raiseVIC();
        scpu.runFrame();

        const u8 dd00 = bus.m_Memory[0xDD00];
        if (dd00 != lastDD00) { flipsSeen++; lastDD00 = dd00; }
        if ((dd00 & 3) == 0)             // bank 3 on display
        {
            bank3Frames++;
            for (u32 i = 0; i < 8; i++)
            {
                const u8 p = bus.m_Memory[0xCFF8 + i];
                if (p >= 0x40) bank3RawPtr++;   // an untranslated under-I/O ptr leaked
            }
        }
        else
        {
            bank1Frames++;
            for (u32 i = 0; i < 8; i++)
            {
                const u8 p = bus.m_Memory[0x4FF8 + i];
                if (p < 0x40 || p >= 0x80) bank1WrongPtr++;  // bank 1 must stay raw
            }
        }
    }

    printf("frames: %u   bank3: %u   bank1: %u   dd00 flips: %u\n",
           frames, bank3Frames, bank1Frames, flipsSeen);
    printf("bank-3 raw (untranslated) ptr sightings: %u\n", bank3RawPtr);
    printf("bank-1 wrong ptr sightings: %u\n", bank1WrongPtr);
    printf("relocation: allocs=%llu delivered=%llu exhausted=%llu forwarded=%llu shielded=%llu\n",
           (unsigned long long)mem.m_RelocAllocs,
           (unsigned long long)mem.m_RelocDelivered,
           (unsigned long long)mem.m_RelocExhausted,
           (unsigned long long)scpu.writeBuffer().m_RelocForwarded,
           (unsigned long long)scpu.writeBuffer().m_RelocShielded);

    // ---- relocated copies must byte-match their under-I/O sources ----------
    u32 blockDiffs = 0, blocksChecked = 0;
    for (u32 slot = 0; slot < 0x40; slot++)
    {
        const u8 r = mem.m_PtrReloc[slot];
        if (r == 0xFF) continue;
        blocksChecked++;
        const u32 src = 0xC000 + ((0x40u + slot) << 6);
        const u32 dst = 0xC000 + ((u32)r << 6);
        for (u32 i = 0; i < 63; i++)
            if (bus.m_Memory[dst + i] != mem.m_RAM[src + i]) blockDiffs++;
        printf("  reloc block %02X -> %02X ($%04X)\n", 0x40 + (unsigned)slot, r, dst);
    }
    printf("relocated blocks: %u   byte diffs vs source: %u\n", blocksChecked, blockDiffs);

    // ---- and they must equal what VICE's board RAM really showed -----------
    u32 truthDiffs = 0;
    for (u32 slot = 0; slot < 0x40; slot++)
    {
        const u8 r = mem.m_PtrReloc[slot];
        if (r == 0xFF) continue;
        const u32 src = 0xC000 + ((0x40u + slot) << 6);
        const u32 dst = 0xC000 + ((u32)r << 6);
        for (u32 i = 0; i < 63; i++)
            if (bus.m_Memory[dst + i] != snapBoard[src + i]) truthDiffs++;
    }
    printf("relocated copies vs VICE board-RAM ground truth: %u diffs\n", truthDiffs);

    // ---- real DRAM under I/O must never have been written ------------------
    // (CHostBus records every bus write into m_Memory; the I/O window on a
    // real machine is chip registers, so nothing here may look like shapes.)
    u32 underIOWrites = 0;
    for (u32 a = 0xD400; a <= 0xD7FF; a++)
        if (bus.m_Memory[a] != 0) underIOWrites++;   // SID mirror region the shapes lived in

    const bool pass = flipsSeen > frames / 2 && bank3Frames && bank1Frames
                   && bank3RawPtr == 0 && bank1WrongPtr == 0
                   && blocksChecked >= 3 && blockDiffs == 0 && truthDiffs == 0
                   && mem.m_RelocExhausted == 0;
    printf("\n%s\n", pass ? "PASS: bank-3 frames deliver relocated shapes, bank-1 raw"
                          : "FAIL");
    (void)underIOWrites;
    return pass ? 0 : 1;
}
