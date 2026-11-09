#!/usr/bin/env python3
"""Post-link check of the `esp32h2-sleep` RAM-resident sleep/wake path.

The crate's PMU sequence runs while the flash cache is suspended, and with
`top-pd` its restore stub runs before the cache and MMU are usable again.
Code reached from it must therefore live in internal SRAM, call only other
SRAM code or mask ROM, and never materialise a flash (`.rodata`/`.text`)
address. Its retention buffers must sit below the ROM's wake stack.

Checks, on the linked ELF:

* every crate function in `.rwtext` (the roots) and everything they call
  directly stays in SRAM; ROM calls through `lui rX, 0x40000` are allowed;
* no instruction in that call graph builds an address in the flash window
  (`lui`/`auipc` reaching 0x42000000..0x44000000);
* a flash-resident function reached from the roots is reported (a missing
  `.rwtext` placement or a non-inlined helper);
* the retention buffers end below the ROM wake stack (0x4084D380);
* `.dram2_uninit` holds nothing.

Usage: esp32h2-sleep-ram-check.py [--objdump PATH] [--nm PATH] ELF...
Exit status 1 on any violation.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

SRAM = (0x4080_0000, 0x4085_0000)
ROM = (0x4000_0000, 0x4006_0000)
FLASH = (0x4200_0000, 0x4400_0000)
ROM_WAKE_STACK = 0x4084_D380
CRATE = "esp32h2_sleep"
BUFFERS = ("ESP32H2_SLEEP_CPU_FRAME", "esp32h2_sleep::hw::top::FRAMES")

FUNC_RE = re.compile(r"^([0-9a-f]+) <(.+)>:$")
INSN_RE = re.compile(r"^\s*([0-9a-f]+):\s+(\S+)\s*(.*)$")
TARGET_RE = re.compile(r"<(.+)>\s*$")


def tool(name: str, explicit: str | None) -> str:
    if explicit:
        return explicit
    found = shutil.which(name)
    if found:
        return found
    sysroot = subprocess.run(
        ["rustc", "--print", "sysroot"], check=True, capture_output=True, text=True
    ).stdout.strip()
    for path in Path(sysroot).rglob(name):
        return str(path)
    sys.exit(f"{name} not found (install the llvm-tools rustup component)")


def in_range(value: int, window: tuple[int, int]) -> bool:
    return window[0] <= value < window[1]


def signed20(imm: int) -> int:
    imm &= 0xFFFFF
    return imm - (1 << 20) if imm & 0x80000 else imm


def parse_imm(text: str) -> int | None:
    try:
        return int(text, 0)
    except ValueError:
        return None


def disassemble(objdump: str, elf: Path) -> dict[str, tuple[int, list[tuple[int, str, str]]]]:
    out = subprocess.run(
        [objdump, "-d", "--no-show-raw-insn", "-C", str(elf)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    functions: dict[str, tuple[int, list[tuple[int, str, str]]]] = {}
    current: str | None = None
    for line in out.splitlines():
        if m := FUNC_RE.match(line):
            current = m.group(2)
            # Local labels (`.Lpcrel_hi0`) continue the enclosing function.
            if current.startswith(".L") and functions:
                current = next(reversed(functions))
                continue
            functions.setdefault(current, (int(m.group(1), 16), []))
        elif current and (m := INSN_RE.match(line)):
            functions[current][1].append((int(m.group(1), 16), m.group(2), m.group(3)))
    return functions


def check_code(functions, errors: list[str]) -> int:
    roots = [
        name
        for name, (addr, _) in functions.items()
        if CRATE in name and in_range(addr, SRAM)
    ]
    if not roots:
        errors.append("no esp32h2-sleep function found in SRAM (.rwtext)")
    seen: set[str] = set()
    stack = list(roots)
    while stack:
        name = stack.pop()
        if name in seen:
            continue
        seen.add(name)
        addr, insns = functions[name]
        if not in_range(addr, SRAM):
            errors.append(f"{name} @0x{addr:08x} is reached from the sleep path but not in SRAM")
            continue
        for pc, op, args in insns:
            regs = [a.strip() for a in args.split(",")]
            if op == "lui" and len(regs) == 2:
                imm = parse_imm(regs[1])
                if imm is not None and in_range((imm << 12) & 0xFFFF_FFFF, FLASH):
                    errors.append(f"{name} @0x{pc:08x}: lui builds flash address 0x{imm << 12:08x}")
            elif op == "auipc" and len(regs) == 2:
                imm = parse_imm(regs[1])
                if imm is not None:
                    base = (pc + (signed20(imm) << 12)) & 0xFFFF_FFFF
                    # auipc+addi/jalr spans +-2 KiB around the page.
                    if in_range(base, (FLASH[0] - 0x800, FLASH[1] + 0x800)):
                        errors.append(f"{name} @0x{pc:08x}: auipc reaches flash 0x{base:08x}")
            if op in ("jal", "jalr", "j", "call", "tail") and (t := TARGET_RE.search(args)):
                target = t.group(1)
                if target.startswith(".L") or target == name or "+0x" in target:
                    continue
                if target in functions:
                    stack.append(target)
    return len(seen)


def symbols(nm: str, elf: Path) -> dict[str, tuple[int, int]]:
    out = subprocess.run(
        [nm, "-S", "-C", str(elf)], check=True, capture_output=True, text=True
    ).stdout
    table = {}
    for line in out.splitlines():
        parts = line.split(maxsplit=3)
        if len(parts) == 4:
            table[parts[3]] = (int(parts[0], 16), int(parts[1], 16))
    return table


def check_layout(objdump: str, nm: str, elf: Path, errors: list[str]) -> int:
    syms = symbols(nm, elf)
    total = 0
    for name in BUFFERS:
        if name not in syms:
            continue
        addr, size = syms[name]
        total += size
        if not in_range(addr, SRAM) or addr + size > ROM_WAKE_STACK:
            errors.append(
                f"{name} 0x{addr:08x}+{size} overlaps the ROM wake stack (0x{ROM_WAKE_STACK:08x})"
            )
    headers = subprocess.run(
        [objdump, "-h", str(elf)], check=True, capture_output=True, text=True
    ).stdout
    for line in headers.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[1] == ".dram2_uninit" and int(parts[2], 16) != 0:
            errors.append(f".dram2_uninit holds 0x{int(parts[2], 16):x} bytes (ROM wake stack)")
    return total


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--objdump")
    parser.add_argument("--nm")
    parser.add_argument("elf", nargs="+", type=Path)
    args = parser.parse_args()
    objdump = tool("llvm-objdump", args.objdump)
    nm = tool("llvm-nm", args.nm)
    failed = False
    for elf in args.elf:
        errors: list[str] = []
        reached = check_code(disassemble(objdump, elf), errors)
        buffers = check_layout(objdump, nm, elf, errors)
        status = "FAIL" if errors else "ok"
        print(f"{elf}: {status}: {reached} sleep-path functions in SRAM, retention buffers {buffers} B")
        for error in errors:
            print(f"  {error}")
        failed |= bool(errors)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
