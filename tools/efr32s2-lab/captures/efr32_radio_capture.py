#!/usr/bin/env python3
"""EFR32 Series 2 radio register captures from a RAIL reference program.

The pure-Rust radio driver (`efr32s2-hal/src/radio`) needs register values
that only silicon running Silicon Labs RAIL can give: the static PHY setup,
the per-state analog values the sequencer microcode writes, the PA
programming and the RSSI offset. This tool keeps that work reproducible.

``experiment``
    Write an hwlab experiment (YAML) from a scenario JSON. The experiment
    drives a RAILtest image over the WSTK VCOM and, between commands, dumps
    every radio register block over SWD into ``<out>/<state>/<block>.bin``.
    This script never touches hardware; run the file with
    ``hwlab run --yes <file>`` (or the hwlab MCP ``experiment_run``).

``reduce``
    Load the dumps, compare them with the GSDK radio configurator register
    map (reset values and access types, read from a local GSDK; the map is
    not copied into this repository) and print the static table, the
    per-state differences and the registers outside the map.

``gsdk-phy``
    Apply a GSDK ``sl_rail_ieee802154_config_*.c`` PHY table (the radio
    configurator's compressed address/value list, Zlib licence) to the
    register-map reset values and print the resulting register values, or
    the difference between two crystal variants (``--against``), or the
    difference with a captured state (``--capture``).

``manifest``
    SHA-256 of every dump under a capture directory.

``stock-gbl``
    Look for GSDK's 802.15.4 PHY tables (every crystal variant of
    ``rail_util_built_in_phys/<chip>``) in the application of a vendor GBL,
    print the matching ``phyInfo`` and the per-channel ``SRCRATIO2`` /
    ``DIGMIXFREQ`` RAIL derives from it (``xg24::channel_words``). Used to
    show that the stock SONOFF Dongle-M EmberZNet image runs GSDK's
    38.4 MHz PHY (``captures/dongle-m-stock``). The GBL is only read.

The RAILtest image, the GSDK and the register map stay outside the
repository (lab-only tools under the Silicon Labs licence). Only our own
measurements, the raw register dumps, are committed next to this script.
"""

from __future__ import annotations

import argparse
import glob
import hashlib
import json
import os
import re
import struct
import sys
from dataclasses import dataclass

DEFAULT_GSDK = os.path.expanduser("~/SimplicityStudio/SDKs/gecko_sdk_4.5.0")

# ---------------------------------------------------------------------------
# Register blocks dumped in every state (xG24). Read-to-pop registers are
# skipped: FRC.DATABUFFER (0x18), FRC.RXRAWDATA (0x90) and BUFC
# BUFn.READDATA / READDATA32 (+0x14 / +0x30). (name, address, length)
# ---------------------------------------------------------------------------
XG24_BLOCKS = [
    ("FRC0", 0xB800_4000, 0x18),
    ("FRC1", 0xB800_401C, 0x74),
    ("FRC2", 0xB800_4094, 0xF0),
    ("AGC", 0xB800_C000, 0x164),
    ("RFCRC", 0xB801_0000, 0x24),
    ("MODEM", 0xB801_4000, 0x404),
    ("SYNTH", 0xB801_8000, 0xC0),
    ("RAC0", 0xB802_0000, 0x1D4),
    ("RAC1", 0xB802_03E0, 0x20),
    ("RAC2", 0xB802_07E8, 0x18),
    ("BUFC", 0xBA00_0000, 0x0C),
    ("BUF0", 0xBA00_000C, 0x14),
    ("BUF0S", 0xBA00_002C, 0x08),
    ("BUF1", 0xBA00_004C, 0x14),
    ("BUF1S", 0xBA00_006C, 0x08),
    ("BUF2", 0xBA00_008C, 0x14),
    ("BUF2S", 0xBA00_00AC, 0x08),
    ("BUF3", 0xBA00_00CC, 0x14),
    ("BUF3S", 0xBA00_00EC, 0x08),
    ("B8030", 0xB803_0000, 0x200),
    ("B8034", 0xB803_4000, 0x200),
    # Non-secure aliases: RAILtest leaves every peripheral but the SMU and
    # the SE mailbox non-secure (SMU.PPUSATD0/1), and a debugger read
    # through the secure alias raises SMU PPUSEC (IRQ 0), which RAILtest
    # does not handle (observed: HFXO0, peripheral ID 46).
    ("HFXO0", 0x5A00_4000, 0x84),
    ("CMU", 0x5000_8000, 0x110),
    ("DCDC", 0x5009_4000, 0x60),
]

# Small set for power sweeps (TX state only).
XG24_PA_BLOCKS = [
    ("RAC0", 0xB802_0000, 0x1D4),
    ("MODEMRAMP", 0xB801_40D8, 0x0C),
]

# TX transient samples (RAC.STATUS and RAC.TX in one SWD transfer).
XG24_TX_BLOCKS = [
    ("RAC0", 0xB802_0000, 0x1D4),
    ("MODEMPADEBUG", 0xB801_41B0, 0x04),
    ("MODEMRAMP", 0xB801_40D8, 0x0C),
]

# Packet-TX samples: one contiguous RAC window from STATUS2 (0xB0, current
# RSM state at the start of the transfer) to PACTRL (0x1C8, end), short
# enough (~1 ms over SWD) to sit inside one 4 ms frame.
XG24_TXW_BLOCKS = [
    ("RACW", 0xB802_0098, 0x134),
    ("MODEMPADEBUG", 0xB801_41B0, 0x04),
]

# Channel-dependent registers (RX on each channel).
XG24_CH_BLOCKS = [
    ("MODEMSRCCHF", 0xB801_411C, 0x04),
    ("MODEMDIGMIX", 0xB801_413C, 0x04),
    ("SYNTH", 0xB801_8000, 0xC0),
    ("RACRETIME", 0xB802_0190, 0x10),
    ("FRCWCNT", 0xB800_4020, 0x0C),
]

# AGC status and RSSI (`RSSI` 0x18, `FRAMERSSI` 0x1C).
XG24_AGC_BLOCKS = [("AGCRSSI", 0xB800_C000, 0x24)]

BLOCK_SETS = {
    "dump": XG24_BLOCKS,
    "dump-pa": XG24_PA_BLOCKS,
    "dump-tx": XG24_TX_BLOCKS,
    "dump-agc": XG24_AGC_BLOCKS,
    "dump-txw": XG24_TXW_BLOCKS,
    "dump-ch": XG24_CH_BLOCKS,
}


def _q(text: str) -> str:
    return json.dumps(text)


def cmd_experiment(args: argparse.Namespace) -> int:
    """Emit an hwlab experiment from a scenario JSON.

    Scenario format::

        {"name": "...", "duration": "300s", "out": "/abs/dir",
         "steps": [["cli", "rx 0"], ["sleep", 0.5], ["dump", "idle-ch26"],
                   ["dump-pa", "tx-tone-200"], ...]}
    """
    with open(args.scenario) as f:
        sc = json.load(f)
    out = os.path.expanduser(sc["out"])
    lines = [
        f"name: {sc['name']}",
        f"duration: {sc.get('duration', '300s')}",
        "devices:",
        f"  vcom: {{plugin: uart, device: {sc.get('uart', 'cu.usbmodem0004402062861')},"
        " baud: 115200, lines: true}",
        f"  dbg: {{plugin: jlink, device: \"{sc.get('jlink', '440206286')}\","
        f" target: {sc.get('target', 'EFR32MG24BXXXF1536')}, control: true}}",
    ]
    if "sniffer" in sc:
        lines.append(f"  inj: {{plugin: nrf802154, device: {sc['sniffer']}, control: true}}")
    lines.append("steps:")
    t_ms = int(sc.get("start_ms", 1500))
    gap = int(sc.get("cli_gap_ms", 400))
    n = 0
    for step in sc["steps"]:
        kind = step[0]
        n += 1
        if kind == "cli":
            lines.append(
                f"  - {{name: c{n:03d}, at: {t_ms}ms, device: vcom,"
                f" call: SerialTransport.write, args: {{data: {_q(step[1])}, eol: crlf}}}}"
            )
            t_ms += gap
        elif kind == "inject":
            # ["inject", psdu_hex, channel, power_dbm]: one frame from the
            # zigbee-rs radio-lab firmware on the nRF sniffer.
            lines.append(
                f"  - {{name: i{n:03d}, at: {t_ms}ms, device: inj,"
                f" call: PacketInjection.transmit, args: {{psdu: {_q(step[1])},"
                f" channel: {int(step[2])}, power_dbm: {int(step[3])}, note: {_q('capture ' + sc['name'])}}}}}"
            )
            t_ms += int(sc.get("inject_ms", 1500))
        elif kind == "sleep":
            t_ms += int(float(step[1]) * 1000)
        elif kind in BLOCK_SETS:
            state = step[1]
            for name, addr, length in BLOCK_SETS[kind]:
                path = os.path.join(out, state, f"{name}.bin")
                lines.append(
                    f"  - {{name: d{n:03d}-{state}-{name}, at: {t_ms}ms, device: dbg,"
                    f" call: MemoryAccess.read, args: {{address: \"0x{addr:08X}\","
                    f" length: {length}, path: {_q(path)}, overwrite: true}}}}"
                )
            # Steps run back to back, so a late step fires at once. Budget
            # the dump time so that CLI commands keep their spacing (the
            # RAILtest VCOM receive buffer is only 32 bytes).
            t_ms += int(sc.get("dump_ms_per_block", 400)) * len(BLOCK_SETS[kind]) + gap
        else:
            raise SystemExit(f"unknown step kind {kind!r}")
    # hwlab does not create directories for `path`.
    for step in sc["steps"]:
        if step[0] in BLOCK_SETS:
            os.makedirs(os.path.join(out, step[1]), exist_ok=True)
    text = "\n".join(lines) + "\n"
    if args.output == "-":
        sys.stdout.write(text)
    else:
        with open(args.output, "w") as f:
            f.write(text)
    return 0


# ---------------------------------------------------------------------------
# GSDK radio configurator register map (local GSDK only)
# ---------------------------------------------------------------------------

_RM_PAT = re.compile(
    r"super\(RM_Register_(\w+?)_NS_(\w+), self\).__init__\(rmio, label,\s*"
    r"(0x[0-9a-fA-F]+), (0x[0-9a-fA-F]+),\s*'(\w+)', '([\w.]+)', '([\w-]+)',\s*"
    r"u\"[^\"]*\",\s*(0x[0-9a-fA-F]+), (0x[0-9a-fA-F]+)",
    re.S,
)


@dataclass(frozen=True)
class Reg:
    block: str
    name: str
    address: int
    access: str
    reset: int
    mask: int

    @property
    def label(self) -> str:
        return f"{self.block}.{self.name}"


def load_regmap(gsdk: str, chip: str, rev: str) -> dict[int, Reg]:
    root = os.path.join(
        gsdk,
        "platform/radio/efr32_multiphy_configurator/host_py_rm_studio_internal/full",
        chip,
        rev,
    )
    files = sorted(glob.glob(os.path.join(root, "*_NS_register.py")))
    if not files:
        raise SystemExit(f"no register map under {root}")
    regs: dict[int, Reg] = {}
    for path in files:
        with open(path) as f:
            text = f.read()
        for m in _RM_PAT.finditer(text):
            blk, name, base, off, _, _, acc, rst, mask = m.groups()
            addr = int(base, 16) + int(off, 16)
            regs[addr] = Reg(blk, name, addr, acc, int(rst, 16), int(mask, 16))
    return regs


# Registers outside the configurator map that the driver programs or that
# the capture documents: address -> label.
EXTRA_REGS = {0xBA00_0000: "BUFC.IPVERSION", 0xBA00_0004: "BUFC.EN", 0xBA00_0008: "BUFC.?08"}
for _i in range(4):
    _b = 0xBA00_000C + 0x40 * _i
    EXTRA_REGS[_b + 0x00] = f"BUFC.BUF{_i}_CTRL"
    EXTRA_REGS[_b + 0x04] = f"BUFC.BUF{_i}_ADDR"
    EXTRA_REGS[_b + 0x08] = f"BUFC.BUF{_i}_WRITEOFFSET"
    EXTRA_REGS[_b + 0x0C] = f"BUFC.BUF{_i}_READOFFSET"
    EXTRA_REGS[_b + 0x10] = f"BUFC.BUF{_i}_WRITESTART"
    EXTRA_REGS[_b + 0x20] = f"BUFC.BUF{_i}_STATUS"
    EXTRA_REGS[_b + 0x24] = f"BUFC.BUF{_i}_THRESHOLDCTRL"
EXTRA_REGS.update(
    {
        0x5A00_4010: "HFXO0.XTALCFG",
        0x5A00_4018: "HFXO0.XTALCTRL",
        0x5A00_401C: "HFXO0.XTALCTRL1",
        0x5A00_4020: "HFXO0.CFG",
        0x5A00_4028: "HFXO0.CTRL",
        0x5A00_4058: "HFXO0.STATUS",
        0x5000_8064: "CMU.CLKEN0",
        0x5000_8068: "CMU.CLKEN1",
    }
)


def load_state(directory: str) -> dict[int, int]:
    """All 32-bit words of one state's dumps, keyed by address."""
    words: dict[int, int] = {}
    seen = set()
    for name, addr, length in XG24_BLOCKS + XG24_PA_BLOCKS + XG24_TX_BLOCKS + XG24_AGC_BLOCKS + XG24_TXW_BLOCKS + XG24_CH_BLOCKS:
        if name in seen:
            continue
        seen.add(name)
        path = os.path.join(directory, f"{name}.bin")
        if not os.path.exists(path):
            continue
        with open(path, "rb") as f:
            data = f.read()
        if len(data) != length:
            raise SystemExit(f"{path}: {len(data)} bytes, expected {length}")
        for i in range(0, length, 4):
            words[addr + i] = struct.unpack_from("<I", data, i)[0]
    return words


def label_of(regs: dict[int, Reg], addr: int) -> str:
    if addr in regs:
        return regs[addr].label
    return EXTRA_REGS.get(addr, f"?{addr:08X}")


def cmd_reduce(args: argparse.Namespace) -> int:
    regs = load_regmap(args.gsdk, args.chip, args.rev)
    states = {}
    for s in args.states:
        states[s] = load_state(os.path.join(args.capture, s))
        if not states[s]:
            raise SystemExit(f"state {s}: no dumps under {args.capture}/{s}")
    static, per_state, outside = [], [], []
    for addr in sorted(regs):
        r = regs[addr]
        vals = [states[s].get(addr) for s in args.states]
        if any(v is None for v in vals):
            continue
        same = all(v == vals[0] for v in vals)
        if r.access != "read-write":
            if not same and args.all:
                per_state.append((addr, vals, r.label + " (ro)"))
            continue
        if same and (vals[0] & r.mask) != (r.reset & r.mask):
            static.append((addr, vals[0], r.label))
        elif not same:
            per_state.append((addr, vals, r.label))
    all_addrs = sorted(set().union(*[set(v) for v in states.values()]))
    for addr in all_addrs:
        if addr in regs:
            continue
        vals = [states[s].get(addr) for s in args.states]
        if any(v is None for v in vals):
            continue
        if not all(v == vals[0] for v in vals) or vals[0] != 0:
            outside.append((addr, vals, label_of(regs, addr)))
    if args.json:
        json.dump(
            {
                "states": args.states,
                "static": [[f"0x{a:08X}", f"0x{v:08X}", lbl] for a, v, lbl in static],
                "per_state": [[f"0x{a:08X}", [f"0x{v:08X}" for v in vs], lbl] for a, vs, lbl in per_state],
                "outside_map": [[f"0x{a:08X}", [f"0x{v:08X}" for v in vs], lbl] for a, vs, lbl in outside],
            },
            sys.stdout,
            indent=1,
        )
        print()
        return 0
    print(f"# static (read-write, != reset, same in all {len(args.states)} states): {len(static)}")
    for addr, val, lbl in static:
        print(f"    (0x{addr:08X}, 0x{val:08X}), // {lbl}")
    print(f"# per-state ({' '.join(args.states)}): {len(per_state)}")
    for addr, vals, lbl in per_state:
        print(f"  0x{addr:08X} {lbl:30s} " + " ".join(f"{v:08X}" for v in vals))
    print(f"# outside the configurator map (non-zero or changing): {len(outside)}")
    for addr, vals, lbl in outside:
        print(f"  0x{addr:08X} {lbl:30s} " + " ".join(f"{v:08X}" for v in vals))
    return 0


# ---------------------------------------------------------------------------
# GSDK PHY tables (radio configurator output)
# ---------------------------------------------------------------------------
# Word list: a header ``OBCC_OOOO`` followed by CC values. O (bits 31:28) is
# the operation (0 write, 1 clear the value's bits, 3 set them), B (27:24)
# the block group (0 = 0xB800_xxxx, 1 = 0xB801_xxxx, 2 = 0xB802_xxxx,
# 3 = RAIL-internal pointer), OOOO the offset. 0xFFFF_FFFF ends the list.

_PHY_BASES = {0: 0xB800_0000, 1: 0xB801_0000, 2: 0xB802_0000}


def phy_arrays(path: str) -> dict[str, list[str]]:
    with open(path) as f:
        text = f.read()
    out = {}
    for m in re.finditer(r"const uint32_t (\w+)\[\] = \{(.*?)\};", text, re.S):
        body = re.sub(r"/\*.*?\*/", "", m.group(2))
        out[m.group(1)] = [t.strip() for t in body.split(",") if t.strip()]
    return out


def phy_ops(tokens: list[str]):
    i = 0
    while i < len(tokens):
        word = tokens[i]
        i += 1
        if word.startswith("(uint32_t)"):
            continue
        v = int(word.rstrip("UL"), 16)
        if v == 0xFFFF_FFFF:
            return
        op, blk, cnt, off = v >> 28, (v >> 24) & 0xF, (v >> 16) & 0xFF, v & 0xFFFF
        for k in range(cnt):
            val = tokens[i]
            i += 1
            if blk == 3:
                continue
            yield op, _PHY_BASES[blk] + off + 4 * k, int(val.rstrip("UL"), 16)


def phy_registers(path: str, regs: dict[int, Reg], variant: str = "") -> dict[int, int]:
    arrays = phy_arrays(path)
    xtal = re.search(r"config_(\w+?)\.c$", path).group(1)
    names = [
        f"sl_rail_ieee802154_phy_2G4Hz_{xtal}_modemConfigBase",
        f"sl_rail_ieee802154_phy_2G4Hz{variant}_{xtal}_modemConfig",
    ]
    state: dict[int, int] = {}
    for name in names:
        for op, addr, val in phy_ops(arrays[name]):
            cur = state.get(addr, regs[addr].reset if addr in regs else 0)
            if op == 0:
                cur = val
            elif op == 1:
                cur &= ~val
            elif op == 3:
                cur |= val
            else:
                raise SystemExit(f"unknown op {op} at 0x{addr:08X}")
            state[addr] = cur & 0xFFFF_FFFF
    return state


def cmd_gsdk_phy(args: argparse.Namespace) -> int:
    regs = load_regmap(args.gsdk, args.chip, args.rev)
    a = phy_registers(args.file, regs)
    if args.against:
        b = phy_registers(args.against, regs)
        for addr in sorted(set(a) | set(b)):
            if a.get(addr) != b.get(addr):
                print(f"  0x{addr:08X} {label_of(regs, addr):30s} {a.get(addr, 0):08X} -> {b.get(addr, 0):08X}")
        return 0
    if args.capture:
        cap = load_state(args.capture)
        same = differ = 0
        for addr in sorted(a):
            got = cap.get(addr)
            if got is None:
                continue
            if got == a[addr]:
                same += 1
            else:
                differ += 1
                print(f"  0x{addr:08X} {label_of(regs, addr):30s} gsdk {a[addr]:08X} silicon {got:08X}")
        print(f"# {same} equal, {differ} differ")
        return 0
    for addr in sorted(a):
        print(f"  0x{addr:08X} 0x{a[addr]:08X} {label_of(regs, addr)}")
    return 0


def cmd_manifest(args: argparse.Namespace) -> int:
    entries = []
    for path in sorted(glob.glob(os.path.join(args.capture, "**", "*.bin"), recursive=True)):
        with open(path, "rb") as f:
            data = f.read()
        entries.append(
            {
                "file": os.path.relpath(path, args.capture),
                "bytes": len(data),
                "sha256": hashlib.sha256(data).hexdigest(),
            }
        )
    json.dump(entries, sys.stdout, indent=1)
    print()
    return 0


# ---------------------------------------------------------------------------
# Vendor GBL: which GSDK PHY table does the application carry?
# ---------------------------------------------------------------------------

_GBL_PROG_TAGS = (0xFD0303FD, 0xFE0101FE)
_PHY_ARRAY = re.compile(r"const uint32_t (\w+modemConfig(?:Base)?)\[\] = \{(.*?)\};", re.S)


def gbl_application(path: str) -> tuple[int, bytes]:
    """Program data of a plain GBL3 merged at its lowest address."""
    data = open(path, "rb").read()
    chunks, offset = [], 0
    while offset + 8 <= len(data):
        tag, length = struct.unpack_from("<II", data, offset)
        body = data[offset + 8 : offset + 8 + length]
        if tag in _GBL_PROG_TAGS:
            chunks.append((struct.unpack_from("<I", body)[0], body[4:]))
        offset += 8 + length
        if tag == 0xFC0404FC:
            break
    base = min(c[0] for c in chunks)
    end = max(c[0] + len(c[1]) for c in chunks)
    image = bytearray(b"\xff" * (end - base))
    for address, body in chunks:
        image[address - base : address - base + len(body)] = body
    return base, bytes(image)


def phy_arrays(path: str) -> dict[str, list[int | None]]:
    """The radio-configurator word lists of a GSDK PHY file (pointers as None)."""
    text = open(path).read()
    out = {}
    for m in _PHY_ARRAY.finditer(text):
        body = re.sub(r"/\*.*?\*/", "", m.group(2), flags=re.S)
        body = re.sub(r"//[^\n]*", "", body)
        words: list[int | None] = []
        for token in (t.strip() for t in body.split(",")):
            if token:
                words.append(None if token.startswith("(uint32_t)") else int(token.rstrip("UL"), 0))
        out[m.group(1)] = words
    return out


def best_match(words: list[int | None], image_words: tuple[int, ...]) -> tuple[int, int]:
    """(matching concrete words, word index) of the best alignment."""
    first = next(i for i, w in enumerate(words) if w is not None)
    best = (0, -1)
    for i, w in enumerate(image_words):
        start = i - first
        if w != words[first] or start < 0 or start + len(words) > len(image_words):
            continue
        same = sum(1 for j, x in enumerate(words) if x is not None and image_words[start + j] == x)
        best = max(best, (same, start))
    return best


def lo_word(channel: int, hfxo: int) -> int:
    return ((2_405_000_000 + 5_000_000 * (channel - 11)) << 19) // hfxo


def channel_words(hfxo: int, iffreq: int, src2_denominator: int, dec0: int) -> list[tuple[int, int]]:
    """RAIL's per-channel SRCRATIO2 / DIGMIXFREQ (efr32s2-hal xg24::channel_words)."""
    out = []
    for channel in range(11, 27):
        lo = lo_word(channel, hfxo) + iffreq
        den = 2048 * src2_denominator
        out.append(((2 * lo * hfxo + den) // (2 * den), (2 * ((iffreq * dec0) << 26) + lo) // (2 * lo)))
    return out


def cmd_stock_gbl(args: argparse.Namespace) -> int:
    base, image = gbl_application(args.gbl)
    words = struct.unpack_from(f"<{len(image) // 4}I", image)
    print(f"# {os.path.basename(args.gbl)}: sha256 {hashlib.sha256(open(args.gbl, 'rb').read()).hexdigest()}")
    print(f"# application {base:#010x}..{base + len(image):#010x}")
    root = os.path.join(args.gsdk, "platform/radio/rail_lib/plugin/rail_util_built_in_phys", args.chip)
    for path in sorted(glob.glob(os.path.join(root, "sl_rail_ieee802154_config_*.c"))):
        for name, phy in phy_arrays(path).items():
            concrete = sum(1 for w in phy if w is not None)
            same, at = best_match(phy, words)
            print(f"  {name:62s} {same:4d}/{concrete:4d} at {base + 4 * at:#010x}")
            if same != concrete or not name.endswith("_modemConfig"):
                continue
            # 0x03014FFC, &phyInfo is the first entry of a modemConfig.
            info_at = words[at + 1] - base
            info = struct.unpack_from("<20I", image, info_at)
            hfxo = 38_400_000 if "38M4Hz" in path else int(re.search(r"_(\d+)MHz", path).group(1)) * 1_000_000
            cache = info[10]
            dec0 = {0: 3, 1: 4, 3: 8}[(cache >> 22) & 3]
            print(f"    phyInfo at {base + info_at:#010x}: freqOffsetFactor {info[1]:#010x}, "
                  f"src2Denominator {info[7]}, synthCache {cache:#010x} (IFFREQ {cache & 0xFFFFF:#x}, dec0 {dec0})")
            for channel, (src, mix) in zip(range(11, 27), channel_words(hfxo, cache & 0xFFFFF, info[7], dec0)):
                print(f"    ch{channel}: SRCRATIO2 {src:#06x} DIGMIXFREQ {mix:#07x}")
    # RAIL_TxPowerConfig_t {mode, voltage mV, ramp us} with GSDK defaults
    for m in re.finditer(rb"[\x00-\x0f]\x00\xe4\x0c\x0a\x00", image):
        print(f"  PA config candidate at {base + m.start():#010x}: mode {image[m.start()]}, 3300 mV, 10 us")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    e = sub.add_parser("experiment", help="write an hwlab experiment from a scenario JSON")
    e.add_argument("scenario")
    e.add_argument("-o", "--output", default="-")
    e.set_defaults(func=cmd_experiment)

    def regmap_args(sp):
        sp.add_argument("--gsdk", default=DEFAULT_GSDK)
        sp.add_argument("--chip", default="efr32xg24")
        sp.add_argument("--rev", default="revA1")

    r = sub.add_parser("reduce", help="reduce dumps against the GSDK register map")
    regmap_args(r)
    r.add_argument("--capture", required=True)
    r.add_argument("--json", action="store_true")
    r.add_argument("--all", action="store_true", help="also list changing read-only registers")
    r.add_argument("states", nargs="+")
    r.set_defaults(func=cmd_reduce)

    g = sub.add_parser("gsdk-phy", help="registers of a GSDK 802.15.4 PHY table")
    regmap_args(g)
    g.add_argument("file")
    g.add_argument("--against", help="second PHY file (other crystal) to diff with")
    g.add_argument("--capture", help="state directory to compare with")
    g.set_defaults(func=cmd_gsdk_phy)

    m = sub.add_parser("manifest", help="sha256 of every dump")
    m.add_argument("capture")
    m.set_defaults(func=cmd_manifest)

    v = sub.add_parser("stock-gbl", help="find GSDK PHY tables in a vendor GBL")
    v.add_argument("gbl")
    v.add_argument("--gsdk", default=DEFAULT_GSDK)
    v.add_argument("--chip", default="efr32xg24")
    v.set_defaults(func=cmd_stock_gbl)

    args = p.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
