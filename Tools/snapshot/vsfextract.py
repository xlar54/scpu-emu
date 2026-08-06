# Extract the CPU-view (SRAM bank 0) and board-DRAM 64K images from the vsf.
def u32(b, o): return int.from_bytes(b[o:o+4], 'little')
data = open(r'c:\Users\scott\repos\scpu-emu\C64Tests\vice-snapshot-balls.vsf', 'rb').read()
off = 37
if data[off:off+13] == b'VICE Version\x1a':
    off += 21
while off + 22 <= len(data):
    name = data[off:off+16].rstrip(b'\x00').decode(errors='replace')
    size = u32(data, off+18)
    if name == 'C64MEM':
        body = data[off+22+52 : off+22+52+196608]
        open(r'c:\Users\scott\repos\scpu-emu\C64Tests\snap_ram_cpu.bin', 'wb').write(body[65536:131072])
        open(r'c:\Users\scott\repos\scpu-emu\C64Tests\snap_ram_board.bin', 'wb').write(body[0:65536])
        print('wrote snap_ram_cpu.bin (SRAM bank0) + snap_ram_board.bin (board DRAM)')
        break
    off += size
