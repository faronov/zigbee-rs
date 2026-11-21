#!/usr/bin/env python3
"""Verify a CC26x2 dongle image against its product memory map.

Checks (all hard failures):
* the loadable flash bytes lie in the application window [0, app_end) of
  `products/cc26x2-dongle/link/memory-<chip>.x` -- never in the GP Proxy
  Table, router or security journals or the CCFG sector padding;
* exactly one 88-byte CCFG sits at FLASH_SIZE - 88 and keeps the serial ROM
  bootloader and its backdoor enabled (the only recovery path of a dongle
  without a debug header), the image valid at 0 and no sector protected;
* the vector table has 16 + 38 entries and starts at 0;
* RAM use leaves at least 12 KiB of stack (16 KiB for the NCP).

    verify-layout.py --chip cc2652p  target/.../cc26x2-router
    verify-layout.py --chip cc2652p7 target/p7/.../cc26x2-router
    verify-layout.py --chip cc2652p7 --layout ncp examples/cc26x2-ncp/target/.../cc26x2-ncp

`--layout ncp` checks against `products/cc26x2-ncp/link/memory-<chip>.x`.
"""
import argparse
import struct
import sys

CHIPS = {
    # flash size, application end, RAM end
    "cc2652p": (352 * 1024, 0x46000, 0x20014000),
    "cc2652p7": (704 * 1024, 0x9E000, 0x20024000),
}
# Application end of the ZRHP NCP product (160 KiB of NV journals).
NCP_APP_END = {"cc2652p7": 0x86000}
NCP_MIN_STACK = 16 * 1024
RAM_START = 0x20000000
MIN_STACK = 12 * 1024
VECTORS = (16 + 38) * 4


def fail(msg):
    sys.exit("verify-layout: " + msg)


def sections(data):
    if data[:4] != b"\x7fELF" or data[4] != 1:
        fail("not an ELF32 file")
    shoff, = struct.unpack_from("<I", data, 32)
    shentsize, shnum, shstrndx = struct.unpack_from("<HHH", data, 46)
    hdrs = [struct.unpack_from("<IIIIIIIIII", data, shoff + i * shentsize) for i in range(shnum)]
    strtab = hdrs[shstrndx]
    names = data[strtab[4]:strtab[4] + strtab[5]]
    out = {}
    for h in hdrs:
        name = names[h[0]:names.index(b"\0", h[0])].decode()
        out[name] = h
    return out


def segments(data):
    phoff, = struct.unpack_from("<I", data, 28)
    phentsize, phnum = struct.unpack_from("<HH", data, 42)
    for i in range(phnum):
        p = struct.unpack_from("<IIIIIIII", data, phoff + i * phentsize)
        if p[0] == 1 and p[4]:
            yield p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--chip", choices=sorted(CHIPS), required=True)
    ap.add_argument("--layout", choices=("router", "ncp"), default="router")
    ap.add_argument("elf")
    args = ap.parse_args()
    flash_size, app_end, ram_end = CHIPS[args.chip]
    min_stack = MIN_STACK
    if args.layout == "ncp":
        if args.chip not in NCP_APP_END:
            fail("no NCP layout for %s" % args.chip)
        app_end, min_stack = NCP_APP_END[args.chip], NCP_MIN_STACK
    ccfg_addr = flash_size - 88
    data = open(args.elf, "rb").read()
    secs = sections(data)

    ccfg = None
    flash_end = 0
    for _type, off, _vaddr, paddr, filesz, _memsz, _flags, _align in segments(data):
        if paddr >= 0x10000000:
            continue
        if paddr == ccfg_addr and filesz == 88:
            if ccfg is not None:
                fail("two CCFG segments")
            ccfg = data[off:off + 88]
            continue
        if paddr + filesz > app_end:
            fail("flash segment 0x%05X+%d crosses the application end 0x%05X" % (paddr, filesz, app_end))
        flash_end = max(flash_end, paddr + filesz)
    if ccfg is None:
        fail("no 88-byte CCFG at 0x%05X" % ccfg_addr)
    w = struct.unpack("<22I", ccfg)
    bl = w[12]
    if bl >> 24 != 0xC5 or bl & 0xFF != 0xC5 or (bl >> 8) & 0xFF > 30:
        fail("CCFG BL_CONFIG 0x%08X disables the ROM bootloader/backdoor" % bl)
    if w[17] != 0 or any(x != 0xFFFFFFFF for x in w[18:22]) or w[13] & 0x101 != 0x101:
        fail("CCFG image-valid/protection/erase words are unsafe")
    if (w[2] >> 16) != 88 or not (w[2] & 0x4):
        fail("CCFG size or DIS_GPRAM (cache) flag is wrong")

    vt = secs.get(".vector_table")
    if not vt or vt[3] != 0 or vt[5] != VECTORS:
        fail(".vector_table must be %d bytes at 0" % VECTORS)

    ram_top = RAM_START
    for name in (".data", ".bss", ".uninit"):
        h = secs.get(name)
        if h and h[5]:
            ram_top = max(ram_top, h[3] + h[5])
    if ram_end - ram_top < min_stack:
        fail("RAM ends at 0x%08X: only %d bytes left for the stack" % (ram_top, ram_end - ram_top))
    print("%s: flash %d B of %d (%.1f%%), headroom %d B; RAM static %d B, stack %d B; CCFG BL 0x%08X"
          % (args.chip, flash_end, app_end, 100.0 * flash_end / app_end, app_end - flash_end,
             ram_top - RAM_START, ram_end - ram_top, bl))


if __name__ == "__main__":
    main()
