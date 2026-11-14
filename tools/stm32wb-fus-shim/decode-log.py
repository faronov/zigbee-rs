#!/usr/bin/env python3
"""Decode the STM32WB FUS sequencer's flash log (see README.md).

Usage: decode-log.py <dump.bin> [offset]

<dump.bin> is a flash read that contains the 4 KiB log page; offset is the
page's byte offset inside the dump (default 0, i.e. a dump read from
0x080C0000).
"""
import struct
import sys

KINDS = {1: "Boot", 2: "Ready", 3: "FusVersion", 4: "StackVersion",
         5: "State", 6: "Action", 7: "Stop", 8: "Error"}
ACTIONS = {1: "EnterFus", 2: "UpgradeFus", 3: "DeleteStack", 4: "InstallMac",
           5: "StartWs", 6: "Reset"}
STOPS = {0: "Done", 1: "WaitMacStaging", 2: "NeedFusImage", 3: "Cap",
         4: "Unexpected"}
STAGED = {0: "Empty", 1: "FusFor053", 2: "Fus120", 3: "Other"}
ERRORS = {0x02: "IPCCBR", 0x03: "SRAM2A secure", 0x04: "C2BOOT set",
          0x05: "no ready event", 0x06: "ready code", 0x07: "command busy",
          0x08: "command timeout", 0x09: "bad response", 0x0A: "device info",
          0x0C: "refused", 0x0D: "FUS error", 0x0E: "timeout",
          0x0F: "event queue", 0x10: "release", 0x11: "no reset"}


def ver(v):
    return f"{v >> 24}.{(v >> 16) & 0xFF}.{(v >> 8) & 0xFF}"


def main():
    data = open(sys.argv[1], "rb").read()
    off = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0
    page = data[off:off + 4096]
    for slot in range(len(page) // 8):
        w0, w1 = struct.unpack_from("<II", page, slot * 8)
        if w0 == 0xFFFFFFFF and w1 == 0xFFFFFFFF:
            break
        if w0 & 0xFF != 0x5A:
            print(f"{slot:3}: foreign {w0:08x} {w1:08x}")
            continue
        kind, arg = (w0 >> 8) & 0xFF, w0 >> 16
        name = KINDS.get(kind, f"kind{kind}")
        if kind == 1:
            text = f"#{arg} RCC_CSR=0x{w1:08x}"
        elif kind == 2:
            ready = {0: "wireless stack", 1: "FUS"}.get(arg & 0xFF, hex(arg & 0xFF))
            text = f"{ready}, SFSA=0x{arg >> 8:02x}, devinfo w0=0x{w1:08x}"
        elif kind == 3:
            text = (f"FUS {ver(w1)}, staged {STAGED.get(arg & 0xFF, arg & 0xFF)}, "
                    f"MAC staged {bool(arg >> 8)}")
        elif kind == 4:
            text = f"stack {ver(w1)} (0x{w1:08x}), InfoStack 0x{arg:02x}"
        elif kind == 5:
            text = f"state 0x{arg & 0xFF:02x} error 0x{arg >> 8:02x} after {w1} ms"
        elif kind == 6:
            text = ACTIONS.get(arg, hex(arg))
        elif kind == 7:
            text = STOPS.get(arg, hex(arg))
        elif kind == 8:
            text = f"{ERRORS.get(arg, hex(arg))} detail=0x{w1:08x}"
        else:
            text = f"arg=0x{arg:04x} val=0x{w1:08x}"
        print(f"{slot:3}: {name:12} {text}")


if __name__ == "__main__":
    main()
