# Ball-render forensics: VIC regs, pointer row, shape blocks, IRQ code.
def u32(b, o): return int.from_bytes(b[o:o+4], 'little')
def u16(b, o): return int.from_bytes(b[o:o+2], 'little')

path = r'c:\Users\scott\repos\scpu-emu\C64Tests\vice-snapshot-balls.vsf'
data = open(path, 'rb').read()
off = 37
if data[off:off+13] == b'VICE Version\x1a':
    off += 21
mods = {}
while off + 22 <= len(data):
    name = data[off:off+16].rstrip(b'\x00').decode(errors='replace')
    size = u32(data, off+18)
    mods[name] = (off + 22, size - 22)
    off += size
def mod(name):
    o, n = mods[name]
    return data[o:o+n]

memd = mod('C64MEM')[52:]
X0 = memd[0:65536]          # board DRAM (what the VIC fetches)
X1 = memd[65536:131072]     # SRAM bank 0 (what the 65816 sees)
X2 = memd[131072:196608]    # SRAM bank 1 (KERNAL images)
simm = memd[196608:]

vic = mod('VIC-II')
# Register array starts at +1 (byte 0 looked like a leading count/flag).
# Validate: regs[0x11] should be $3B-ish, regs[0x15]=$FF.
for base in (0, 1, 2):
    r = vic[base:base+0x40]
    print(f'reg base {base}: d011={r[0x11]:02x} d012={r[0x12]:02x} d015={r[0x15]:02x} '
          f'd018={r[0x18]:02x} d01a={r[0x1a]:02x} d010={r[0x10]:02x}')
regs = vic[1:0x41]

print()
print('sprite positions (x9bit,y):')
msb = regs[0x10]
for n in range(8):
    x = regs[2*n] + (256 if (msb >> n) & 1 else 0)
    y = regs[2*n+1]
    print(f'  spr{n}: x={x:3d} y={y:3d}  color={vic[0x28+n] if False else "?"}')

bank = 3
vbase = bank * 0x4000
scr  = vbase + ((regs[0x18] >> 4) & 0x0F) * 0x400
bmp  = vbase + (0x2000 if regs[0x18] & 8 else 0)
prow = scr + 0x3F8
print(f'\nscreen=${scr:04x} bitmap=${bmp:04x} ptr row=${prow:04x} raster_irq_line={regs[0x12] + (256 if regs[0x11]&0x80 else 0)}')

def row(mem, a):
    return ' '.join(f'{mem[a+i]:02x}' for i in range(8))
print(f'ptr row X0 (VIC sees): {row(X0, prow)}')
print(f'ptr row X1 (CPU sees): {row(X1, prow)}')

ptrs = [X0[prow+i] for i in range(8)]
print('\nshape addresses (bank3 + ptr*64):')
for n, p in enumerate(ptrs):
    a = vbase + p * 64
    d0 = X0[a:a+63]
    d1 = X1[a:a+63]
    same = 'SAME' if d0 == d1 else 'DIFF'
    print(f'  spr{n}: ptr={p:02x} addr=${a:04x} X0==X1:{same} X0[0:9]={d0[:9].hex()}')

# Where do X0 and X1 disagree across the VIC bank?
diffs = [a for a in range(0xC000, 0x10000) if X0[a] != X1[a]]
print(f'\nX0 vs X1 diff bytes in $C000-$FFFF: {len(diffs)}')
def ranges(addrs):
    out = []
    for a in addrs:
        if out and a == out[-1][1] + 1:
            out[-1][1] = a
        else:
            out.append([a, a])
    return out
for lo, hi in ranges(diffs)[:40]:
    print(f'  ${lo:04x}-${hi:04x} ({hi-lo+1})')

# Full-64K diff too (game state areas)
alldiffs = [a for a in range(0x0000, 0x10000) if X0[a] != X1[a]]
print(f'X0 vs X1 diff bytes whole 64K: {len(alldiffs)}')

# IRQ vector chain: bank-0 $FFFE as the CPU sees it (X1) + $0314
print(f'\nX1 $FFFE/F = {u16(X1,0xFFFE):04x}   X1 $0314 = {u16(X1,0x314):04x}  X1 $0318 = {u16(X1,0x318):04x}')
print(f'X0 $FFFE/F = {u16(X0,0xFFFE):04x}   X0 $0314 = {u16(X0,0x314):04x}')

# Find writers of the pointer row and $D018 in CPU-visible memory (X1 + SIMM).
def find_all(hay, needle):
    out, i = [], hay.find(needle)
    while i >= 0:
        out.append(i)
        i = hay.find(needle, i+1)
    return out

pats = {}
for lo in range(8):
    a = prow + lo
    pats[f'STA ${a:04x}'] = bytes([0x8D, a & 0xFF, a >> 8])
pats['STA $D018'] = bytes([0x8D, 0x18, 0xD0])
pats['STA $D015'] = bytes([0x8D, 0x15, 0xD0])
pats['STA $DD00'] = bytes([0x8D, 0x00, 0xDD])
pats[f'STA ${prow:04x},X'] = bytes([0x9D, prow & 0xFF, prow >> 8])
pats[f'STA ${prow:04x},Y'] = bytes([0x99, prow & 0xFF, prow >> 8])
pats[f'STA long {prow:04x}'] = bytes([0x8F, prow & 0xFF, prow >> 8, 0x00])

for name, pat in pats.items():
    h0 = find_all(X1, pat)
    hs = find_all(simm, pat)
    if h0 or hs:
        locs = [f'bank0:${a:04x}' for a in h0] + [f'simm:${a:06x}' for a in hs[:10]]
        print(f'{name}: {", ".join(locs)}')
