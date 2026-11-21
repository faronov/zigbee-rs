#!/usr/bin/env python3
"""Program a CC26x2/CC13x2 dongle through its serial ROM bootloader.

The CC26x2 boot ROM's UART bootloader (TI SWCU185 "Bootloader"; protocol as
implemented by JelmerT/cc2538-bsl) is entered through the dongle's USB
bridge: on the Sonoff ZBDongle-P class wiring (measured on the lab unit,
pyserial levels) DTR=0/RTS=1 holds RESET and DTR=1/RTS=0 pulls the DIO15
backdoor low. Both lines are switched with one TIOCMSET so the chip never
leaves reset with the backdoor released.

Safety rules (an image cannot be recovered on a dongle without a debug
header once the bootloader is gone):

* the image's CCFG must enable the ROM bootloader and its backdoor; it is
  written last, after the application verified, and only when it differs;
* only the sectors the application covers are erased; the persistence
  window [app_end, CCFG sector) is never touched unless --erase-persistence;
* every written range is verified with the ROM's CRC32.

    cc26xx-rom-bsl.py identify -p /dev/cu.usbserial-0001
    cc26xx-rom-bsl.py flash -p PORT --app-end 0x46000 image.elf   # CC2652P
    cc26xx-rom-bsl.py read -p PORT --out dump.bin [--addr 0 --len 0x58000]
"""
import argparse
import fcntl
import struct
import sys
import termios
import time
import zlib

import serial

SECTOR = 8192
CCFG_LEN = 88
ACK, NACK = 0xCC, 0x33


class BslError(Exception):
    pass


class Bsl:
    def __init__(self, port, baud):
        self.sp = serial.Serial(port, baud, timeout=1.0)

    def lines(self, dtr, rts):
        v = struct.unpack("I", fcntl.ioctl(self.sp.fd, termios.TIOCMGET, struct.pack("I", 0)))[0]
        v &= ~(termios.TIOCM_DTR | termios.TIOCM_RTS)
        v |= (termios.TIOCM_DTR if dtr else 0) | (termios.TIOCM_RTS if rts else 0)
        fcntl.ioctl(self.sp.fd, termios.TIOCMSET, struct.pack("I", v))

    def enter(self):
        self.lines(True, False)
        time.sleep(0.1)
        self.lines(False, True)
        time.sleep(0.1)
        self.lines(True, False)
        time.sleep(0.5)
        self.sp.reset_input_buffer()
        self.sp.write(b"\x55\x55")
        self.wait_ack("sync")

    def wait_ack(self, what, timeout=2.0):
        end = time.time() + timeout
        prev = None
        while time.time() < end:
            b = self.sp.read(1)
            if not b:
                continue
            b = b[0]
            if prev == 0 and b in (ACK, NACK):
                if b == NACK:
                    raise BslError("NACK on " + what)
                return
            prev = b
        raise BslError("no ACK on " + what)

    def command(self, body, what, timeout=2.0):
        self.sp.write(bytes([len(body) + 2, sum(body) & 0xFF]) + body)
        self.wait_ack(what, timeout)

    def packet(self):
        hdr = self.sp.read(2)
        if len(hdr) < 2:
            raise BslError("short response header")
        size, chk = hdr
        data = self.sp.read(size - 2)
        if sum(data) & 0xFF != chk:
            self.sp.write(b"\x00\x33")
            raise BslError("response checksum")
        self.sp.write(b"\x00\xCC")
        return data

    def status(self, what):
        self.command(b"\x23", "GET_STATUS")
        s = self.packet()
        if s[0] != 0x40:
            raise BslError("%s failed: status 0x%02X" % (what, s[0]))

    def read_words(self, addr, count):
        self.command(b"\x2A" + struct.pack(">I", addr) + bytes([1, count]), "MEMORY_READ")
        data = self.packet()
        self.status("MEMORY_READ")
        return data

    def word(self, addr):
        return struct.unpack("<I", self.read_words(addr, 1))[0]

    def read(self, addr, length):
        out = bytearray()
        while len(out) < length:
            n = min(63, (length - len(out) + 3) // 4)
            out += self.read_words(addr + len(out), n)
        return bytes(out[:length])

    def crc32(self, addr, length):
        self.command(b"\x27" + struct.pack(">III", addr, length, 0), "CRC32", timeout=10)
        data = self.packet()
        self.status("CRC32")
        return struct.unpack(">I", data)[0]

    def erase_sector(self, addr):
        self.command(b"\x26" + struct.pack(">I", addr), "SECTOR_ERASE", timeout=5)
        self.status("SECTOR_ERASE 0x%05X" % addr)

    def program(self, addr, data):
        self.command(b"\x21" + struct.pack(">II", addr, len(data)), "DOWNLOAD")
        self.status("DOWNLOAD 0x%05X" % addr)
        for i in range(0, len(data), 252):
            chunk = data[i:i + 252]
            self.command(b"\x24" + chunk, "SEND_DATA", timeout=5)
            self.status("SEND_DATA 0x%05X" % (addr + i))

    def reset(self):
        self.lines(True, True)  # release the backdoor before the reset
        time.sleep(0.05)
        self.command(b"\x25", "RESET")
        self.sp.close()


def identify(b):
    ice = b.word(0x50001318)
    user = b.word(0x50001294)
    sectors = b.word(0x4003002C) & 0xFF
    mac = struct.pack("<II", b.word(0x500012F0), b.word(0x500012F4))[::-1]
    info = {
        "wafer_id": (ice >> 12) & 0xFFFF,
        "pg_rev": ice >> 28,
        "high_pa": bool(user & (1 << 25)),
        "cc13": bool(user & (1 << 23)),
        "package": (user >> 16) & 7,
        "flash_size": sectors * SECTOR,
        "eui64": ":".join("%02X" % x for x in mac),
    }
    return info


def ccfg_ok(ccfg):
    words = struct.unpack("<22I", ccfg)
    bl = words[12]
    if bl >> 24 != 0xC5 or bl & 0xFF != 0xC5:
        return "CCFG disables the ROM bootloader or its backdoor (BL_CONFIG 0x%08X)" % bl
    if (bl >> 8) & 0xFF > 30:
        return "CCFG backdoor pin is not a DIO"
    if words[17] != 0:
        return "CCFG IMAGE_VALID_CONF is not 0"
    if any(w != 0xFFFFFFFF for w in words[18:22]):
        return "CCFG write-protects sectors"
    if words[13] & 0x101 != 0x101:
        return "CCFG disables chip/bank erase"
    return None


def elf_segments(path):
    data = open(path, "rb").read()
    if data[:4] != b"\x7fELF" or data[4] != 1:
        raise BslError("not an ELF32 file")
    phoff, = struct.unpack_from("<I", data, 28)
    phentsize, phnum = struct.unpack_from("<HH", data, 42)
    segs = []
    for i in range(phnum):
        p_type, off, _va, pa, filesz, _memsz, _f, _a = struct.unpack_from(
            "<IIIIIIII", data, phoff + i * phentsize)
        if p_type == 1 and filesz:
            segs.append((pa, data[off:off + filesz]))
    return segs


def flash(b, info, path, app_end, erase_persistence):
    size = info["flash_size"]
    ccfg_addr = size - CCFG_LEN
    ccfg_sector = size - SECTOR
    image = bytearray(b"\xff" * size)
    used_end = 0
    ccfg = None
    for addr, seg in elf_segments(path):
        if addr == ccfg_addr and len(seg) == CCFG_LEN:
            ccfg = bytes(seg)
            continue
        if addr >= 0x10000000:
            continue
        if addr + len(seg) > app_end:
            raise BslError("segment 0x%05X+%d crosses the application end 0x%05X" % (addr, len(seg), app_end))
        image[addr:addr + len(seg)] = seg
        used_end = max(used_end, addr + len(seg))
    if ccfg is None:
        raise BslError("ELF has no CCFG at 0x%05X (wrong chip/flash size?)" % ccfg_addr)
    reason = ccfg_ok(ccfg)
    if reason:
        raise BslError(reason + " -- refusing to flash")
    app_len = (used_end + 3) & ~3
    sectors = (app_len + SECTOR - 1) // SECTOR
    print("application 0x00000..0x%05X (%d sectors), CCFG at 0x%05X" % (app_len, sectors, ccfg_addr))
    for s in range(sectors):
        b.erase_sector(s * SECTOR)
    for off in range(0, app_len, 4096):
        chunk = bytes(image[off:min(off + 4096, app_len)])
        if chunk.strip(b"\xff"):
            b.program(off, chunk)
        sys.stderr.write("\r  programmed %d/%d KiB" % (min(off + 4096, app_len) >> 10, app_len >> 10))
    sys.stderr.write("\n")
    want = zlib.crc32(bytes(image[:app_len])) & 0xFFFFFFFF
    got = b.crc32(0, app_len)
    if want != got:
        raise BslError("application CRC mismatch: 0x%08X != 0x%08X (CCFG left untouched)" % (got, want))
    print("application verified, CRC32 0x%08X" % got)
    if erase_persistence:
        for a in range(app_end, ccfg_sector, SECTOR):
            b.erase_sector(a)
        print("persistence 0x%05X..0x%05X erased" % (app_end, ccfg_sector))
    current = b.read(ccfg_addr, CCFG_LEN)
    if current == ccfg:
        print("CCFG already identical; not rewritten")
    else:
        b.erase_sector(ccfg_sector)
        b.program(ccfg_addr, ccfg)
        if b.read(ccfg_addr, CCFG_LEN) != ccfg:
            raise BslError("CCFG read-back mismatch (the ROM bootloader stays reachable: erased CCFG = no valid image)")
        print("CCFG written and verified")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["identify", "flash", "read"])
    ap.add_argument("image", nargs="?")
    ap.add_argument("-p", "--port", required=True)
    ap.add_argument("-b", "--baud", type=int, default=460800)
    ap.add_argument("--app-end", type=lambda s: int(s, 0), help="end of the application window")
    ap.add_argument("--erase-persistence", action="store_true")
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=0)
    ap.add_argument("--len", type=lambda s: int(s, 0))
    ap.add_argument("--out")
    ap.add_argument("--expect-eui64", help="refuse to program a different chip")
    args = ap.parse_args()
    b = Bsl(args.port, args.baud)
    try:
        b.enter()
        info = identify(b)
        print(info)
        if args.expect_eui64 and info["eui64"].lower() != args.expect_eui64.lower():
            raise BslError("EUI-64 %s is not the expected %s" % (info["eui64"], args.expect_eui64))
        if args.cmd == "flash":
            if not args.image or args.app_end is None:
                raise BslError("flash needs an ELF and --app-end")
            flash(b, info, args.image, args.app_end, args.erase_persistence)
        elif args.cmd == "read":
            length = args.len or info["flash_size"]
            data = b.read(args.addr, length)
            open(args.out, "wb").write(data)
            crc = b.crc32(args.addr, length)
            ok = crc == zlib.crc32(data) & 0xFFFFFFFF
            print("read %d bytes, CRC32 0x%08X %s" % (length, crc, "verified" if ok else "MISMATCH"))
            if not ok:
                raise BslError("read CRC mismatch")
    finally:
        try:
            b.reset()
        except Exception as e:  # noqa: BLE001
            print("reset failed:", e, file=sys.stderr)


if __name__ == "__main__":
    try:
        main()
    except BslError as e:
        sys.exit("error: %s" % e)
