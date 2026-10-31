#!/usr/bin/env python3
"""Build and measure the MAC footprint of every production firmware image.

Two sub-commands:

``build``   rebuilds the selected images exactly as CI does (same working
            directory, toolchain, features and ``--locked``) and copies each
            linked ELF to ``<out>/<image>.elf``.

``report``  measures ELF files (``<out>/*.elf`` or ``NAME=PATH`` arguments):

  flash       file-backed allocated bytes (PROGBITS|ALLOC: .text, .rodata,
              .data load image, vector tables, RAM-resident code)
  text        allocated executable PROGBITS bytes
  data        allocated writable PROGBITS bytes (.data initialisers)
  bss         allocated NOBITS bytes, excluding sections named *stack*/*heap*
  pool        largest Embassy task pool / arena / main-task static (nm)
  stack       static worst-case stack over *direct* calls, from any root,
              derived from function prologues in the linked image
  mac_chain   deepest direct-call chain, from its outermost caller, that
              executes ``zigbee_mac::`` code: the stack needed while the MAC
              runs, independent of how the inliner split MAC and runtime
  mac_stack   the ``stack`` metric restricted to roots inside ``zigbee_mac::``
              (the MAC call tree alone; shifts when the inliner moves a MAC
              wrapper into or out of its runtime caller)
  mac_frame   largest single stack frame of a ``zigbee_mac::`` function

The stack numbers are a static upper bound for direct-call chains.
Indirect calls (executor polls through vtables, function pointers) and
interrupt nesting are not followed; ``indirect`` counts the indirect call
sites and ``dyn`` lists functions with a dynamic stack adjustment, so a
change in either is visible.  Use the numbers to compare a build against a
baseline, not as a runtime high-water mark.

``compare BASE.json NEW.json`` prints per-image deltas and exits non-zero if
any flash/data/bss/pool/stack metric grew.

Examples::

    tools/mac-footprint.py build --out target/mac-footprint/base nrf52840-sensor
    tools/mac-footprint.py report --out target/mac-footprint/base --json base.json
    tools/mac-footprint.py compare base.json after.json
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUST_TOOLCHAIN = "nightly-2026-03-23"  # .github/workflows/ci.yml RUST_TOOLCHAIN
ESP_PHY_TOOLCHAIN = "nightly-2026-08-01"  # ci.yml ESP_PHY_RUST_TOOLCHAIN
TC32_TOOLCHAIN = os.environ.get(
    "TC32_TOOLCHAIN", str(ROOT / ".toolchains/tc32-1.98.1-20261003-31a272")
)


@dataclass
class Image:
    name: str
    workdir: str
    elf: str  # relative to workdir
    args: list[str] = field(default_factory=list)
    toolchain: str = RUST_TOOLCHAIN
    env_unset: list[str] = field(default_factory=list)
    # Non-cargo build command (Telink uses its layout-checking wrapper).
    command: list[str] | None = None


def _nrf(name: str, crate: str, features: str = "") -> Image:
    args = ["--features", features] if features else []
    return Image(name, f"examples/{crate}",
                 f"target/thumbv7em-none-eabihf/release/{crate}", args)


def _uf2(board: str, feature: str) -> Image:
    return Image(
        f"nrf52840-sensor-uf2-{board}", "examples/nrf52840-sensor-uf2",
        "target/thumbv7em-none-eabihf/release/nrf52840-sensor-uf2",
        ["--no-default-features", "--features", feature])


def _esp(chip: str, light_sleep: bool) -> Image:
    args = ["-Z", "build-std=core,alloc"]
    if light_sleep:
        args += ["--features", "light-sleep"]
    return Image(
        f"{chip}-sensor{'-light-sleep' if light_sleep else ''}",
        f"examples/{chip}-sensor",
        f"target/riscv32imac-unknown-none-elf/release/{chip}-sensor",
        args, ESP_PHY_TOOLCHAIN)


def _telink(name: str, mode: str, crate: str) -> Image:
    return Image(name, f"examples/{crate}",
                 f"target/tc32-unknown-none-elf/release/{crate}",
                 command=["./scripts/tlsr8258.sh", "build", mode])


# PHY62x2 images build in an isolated target directory so measurements never
# race with lab builds that share the example's own target directory.
_PHY_TARGET = "../../target/mac-footprint/build"

IMAGES: list[Image] = [
    _nrf("nrf52840-sensor", "nrf52840-sensor"),
    _nrf("nrf52840-sensor-bme280", "nrf52840-sensor", "sensor-bme280"),
    _nrf("nrf52840-sensor-sht31", "nrf52840-sensor", "sensor-sht31"),
    _nrf("nrf52833-sensor", "nrf52833-sensor"),
    _nrf("nrf52833-sensor-bme280", "nrf52833-sensor", "sensor-bme280"),
    _nrf("nrf52833-sensor-sht31", "nrf52833-sensor", "sensor-sht31"),
    _nrf("nrf52840-router", "nrf52840-router"),
    _uf2("promicro", "board-promicro"),
    _uf2("mdk", "board-mdk"),
    _uf2("pca10059", "board-nrf-dongle"),
    _uf2("dk", "board-nrf-dk"),
    Image("bl702-sensor", "examples/bl702-sensor",
          "target/riscv32imc-unknown-none-elf/release/bl702-sensor"),
    Image("cc2340-sensor-fallback", "examples/cc2340-sensor",
          "target/fallback/thumbv6m-none-eabi/release/cc2340-sensor",
          ["--target-dir", "target/fallback"], env_unset=["CC2340_SDK_DIR"]),
    Image("efr32mg1-sensor", "examples/efr32mg1-sensor",
          "target/thumbv7em-none-eabi/release/efr32mg1-sensor"),
    Image("efr32mg21-sensor", "examples/efr32mg21-sensor",
          "target/thumbv8m.main-none-eabihf/release/efr32mg21-sensor"),
    _esp("esp32c6", False),
    _esp("esp32c6", True),
    _esp("esp32h2", False),
    _esp("esp32h2", True),
    Image("phy6222-sensor", "examples/phy6222-sensor",
          f"{_PHY_TARGET}/phy6222/thumbv6m-none-eabi/release/phy6222-sensor",
          ["--target-dir", f"{_PHY_TARGET}/phy6222"], ESP_PHY_TOOLCHAIN),
    Image("phy6252-sensor", "examples/phy6222-sensor",
          f"{_PHY_TARGET}/phy6252/thumbv6m-none-eabi/release/phy6222-sensor",
          ["--no-default-features", "--features", "phy6252",
           "--target-dir", f"{_PHY_TARGET}/phy6252"], ESP_PHY_TOOLCHAIN),
    Image("pb03f-zigbee", "examples/pb03f-kit",
          f"{_PHY_TARGET}/pb03f/thumbv6m-none-eabi/release/pb03f-zigbee",
          ["--bin", "pb03f-zigbee", "--target-dir", f"{_PHY_TARGET}/pb03f"],
          ESP_PHY_TOOLCHAIN),
    _telink("tlsr8258-sensor", "sensor", "telink-tlsr8258-sensor"),
    _telink("tlsr8258-sensor-retention", "sensor-retention",
            "telink-tlsr8258-sensor"),
    _telink("tlsr8258-sensor-retention-10s", "sensor-retention-10s",
            "telink-tlsr8258-sensor"),
    _telink("tlsr8258-router", "router", "telink-tlsr8258-router"),
]


def select_images(patterns: list[str]) -> list[Image]:
    if not patterns:
        return IMAGES
    chosen = [i for i in IMAGES if any(re.fullmatch(p, i.name) for p in patterns)]
    if not chosen:
        sys.exit(f"no image matches {patterns}; known: {[i.name for i in IMAGES]}")
    return chosen


def build(images: list[Image], out: Path, keep_going: bool) -> int:
    out.mkdir(parents=True, exist_ok=True)
    failed = []
    for image in images:
        workdir = ROOT / image.workdir
        env = os.environ.copy()
        for name in image.env_unset:
            env.pop(name, None)
        if image.command:
            cmd = [str(ROOT / image.command[0].lstrip("./"))] + image.command[1:]
            cwd = ROOT
        else:
            cmd = ["cargo", f"+{image.toolchain}", "build", "--release",
                   "--locked", *image.args]
            cwd = workdir
        print(f"== {image.name}: {' '.join(cmd)} (in {cwd})", flush=True)
        result = subprocess.run(cmd, cwd=cwd, env=env)
        elf = workdir / image.elf
        if result.returncode != 0 or not elf.is_file():
            print(f"!! {image.name}: build failed", file=sys.stderr, flush=True)
            failed.append(image.name)
            if not keep_going:
                break
            continue
        shutil.copy2(elf, out / f"{image.name}.elf")
    if failed:
        print(f"failed: {' '.join(failed)}", file=sys.stderr)
    return 1 if failed else 0


# ── ELF analysis ──────────────────────────────────────────────────────────


def llvm_tool(tool: str, elf: Path) -> str:
    override = os.environ.get("LLVM_BIN_DIR")
    if override:
        return str(Path(override) / tool)
    if "tlsr8258" in elf.name or "telink" in str(elf):
        return str(Path(TC32_TOOLCHAIN) / "llvm/bin" / tool)
    sysroot = subprocess.run(
        ["rustc", f"+{RUST_TOOLCHAIN}", "--print", "sysroot"],
        capture_output=True, text=True, check=True).stdout.strip()
    found = list(Path(sysroot).glob(f"lib/rustlib/*/bin/{tool}"))
    if not found:
        sys.exit(f"{tool} not found; install llvm-tools for {RUST_TOOLCHAIN}")
    return str(found[0])


def run(tool: str, elf: Path, *args: str) -> str:
    return subprocess.run([llvm_tool(tool, elf), *args, str(elf)],
                          capture_output=True, text=True, check=True).stdout


SECTION_RE = re.compile(
    r"^\s*\[\s*\d+\]\s+(\S+)\s+(\S+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)"
    r"\s+[0-9a-f]+\s+([A-Za-z]*)")


def sections(elf: Path) -> dict[str, int]:
    text = data = flash = bss = 0
    reserved = 0
    listing = run("llvm-readobj", elf, "--elf-output-style=GNU", "-S", "--wide")
    for line in listing.splitlines():
        m = SECTION_RE.match(line)
        if not m:
            continue
        name, kind, _addr, _off, size, flags = m.groups()
        size = int(size, 16)
        if "A" not in flags or size == 0:
            continue
        if kind == "NOBITS":
            if re.search(r"stack|heap", name, re.I):
                reserved += size
            else:
                bss += size
            continue
        flash += size
        if "X" in flags:
            text += size
        elif "W" in flags:
            data += size
    return {"flash": flash, "text": text, "data": data, "bss": bss,
            "reserved_stack_heap": reserved}


POOL_RE = re.compile(r"POOL|ARENA|__embassy_main|main_task|TASK", re.I)


def pools(elf: Path) -> list[tuple[str, int]]:
    found = []
    for line in run("llvm-nm", elf, "-S", "-C", "--size-sort").splitlines():
        parts = line.split(maxsplit=3)
        if len(parts) < 4 or parts[2] not in "bBdD":
            continue
        size = int(parts[1], 16)
        if size >= 256 and POOL_RE.search(parts[3]):
            found.append((parts[3], size))
    return sorted(found, key=lambda item: -item[1])


FUNC_RE = re.compile(r"^([0-9a-f]+) <(.+)>:$")
INSN_RE = re.compile(r"^\s*([0-9a-f]+):\s+(\S+)\s*(.*)$")
TARGET_RE = re.compile(r"(?:\b(?:0x)?([0-9a-f]+)\s+)?<(.+)>\s*$")
# Thumb / RISC-V / TC32 direct-call mnemonics.  Any other instruction whose
# operand names the *start* of another function (``<name>`` without ``+0x``)
# is treated as a tail call.  ``jalr``/``blx``/``tjex`` without a symbolic
# target are indirect.
CALLS = {"bl", "blx", "jal", "jalr", "call", "tjl"}
NOT_BRANCH = re.compile(r"^(ldr|adr|auipc|lui|la|tloadr|tadd|add|addi|mov|lw|sw)")
INDIRECT = {"blx", "jalr", "c.jalr", "tjex", "jr", "bx"}
REG_LIST_RE = re.compile(r"\{([^}]*)\}")


def _reg_count(operands: str) -> int:
    m = REG_LIST_RE.search(operands)
    if not m:
        return 0
    count = 0
    for part in m.group(1).split(","):
        part = part.strip()
        if "-" in part:
            lo, hi = (re.sub(r"\D", "", p) for p in part.split("-"))
            count += int(hi) - int(lo) + 1 if lo and hi else 1
        elif part:
            count += 1
    return count


def _imm(operands: str) -> int | None:
    m = re.search(r"#(-?(?:0x[0-9a-f]+|\d+))", operands)
    if not m:
        m = re.search(r",\s*(-?(?:0x[0-9a-f]+|\d+))\s*$", operands)
    return int(m.group(1), 0) if m else None


@dataclass
class Func:
    name: str
    frame: int = 0
    dynamic: bool = False
    calls: set[int] = field(default_factory=set)
    tails: set[int] = field(default_factory=set)
    indirect: int = 0


# Panic/abort paths never return to the MAC, so they would otherwise dominate
# every chain that contains a bounds check and hide real MAC changes.
PANIC_RE = re.compile(r"panic|rust_begin_unwind|slice_index_fail|unwrap_failed|"
                      r"expect_failed|::fail\b|HardFault|_defmt_|defmt::export")


def analyse_stack(elf: Path) -> dict:
    funcs: dict[int, Func] = {}
    order: list[int] = []
    current: Func | None = None
    cur_addr = 0
    prologue = 0
    disasm = run("llvm-objdump", elf, "-d", "-C", "--no-show-raw-insn")
    for line in disasm.splitlines():
        m = FUNC_RE.match(line)
        if m and not m.group(2).startswith((".L", "$")):
            cur_addr = int(m.group(1), 16)
            current = funcs.setdefault(cur_addr, Func(m.group(2)))
            order.append(cur_addr)
            prologue = 0
            continue
        if current is None:
            continue
        m = INSN_RE.match(line)
        if not m:
            continue
        mnemonic = m.group(2).lower()
        operands = re.split(r"\s(?://|;|@)\s", m.group(3))[0].strip()
        prologue += 1
        if prologue <= 24:
            if mnemonic in ("push", "push.w", "tpush") or \
                    (mnemonic == "stmdb" and operands.strip().startswith("sp!")):
                current.frame += 4 * _reg_count(operands)
            elif mnemonic in ("vpush", "vpush.64"):
                current.frame += 8 * _reg_count(operands)
            elif re.match(r"^(sub|subw|sub\.w|tsub|tsubs)$", mnemonic) and \
                    re.match(r"^\s*sp\s*,", operands):
                value = _imm(operands)
                if value is None:
                    current.dynamic = True
                else:
                    current.frame += abs(value)
            elif mnemonic in ("addi", "c.addi16sp", "c.addi") and \
                    re.match(r"^\s*sp\s*,\s*sp", operands):
                value = _imm(operands)
                if value is not None and value < 0:
                    current.frame += -value
            elif mnemonic in ("sub", "c.sub") and re.match(r"^\s*sp\s*,\s*sp\s*,", operands):
                current.dynamic = True
            elif mnemonic in ("add", "tadd") and re.match(r"^\s*sp\s*,", operands):
                value = _imm(operands)
                if value is None:
                    current.dynamic = True
                elif value < 0:
                    current.frame += -value
        target = TARGET_RE.search(operands)
        if target is None:
            if mnemonic in INDIRECT and not re.match(r"^\s*(lr|ra)\s*$", operands):
                current.indirect += 1
            continue
        symbol = target.group(2)
        address = int(target.group(1), 16) if target.group(1) else symbol
        if mnemonic in CALLS:
            current.calls.add(address)
        elif not re.search(r"\+0x[0-9a-f]+$", symbol) and address != cur_addr and \
                not NOT_BRANCH.match(mnemonic):
            current.tails.add(address)

    starts = sorted(funcs)
    by_name = {f.name: a for a, f in funcs.items()}

    def resolve(target) -> int | None:
        if isinstance(target, int):
            return target
        m = re.match(r"^(.*)\+0x([0-9a-f]+)$", target)
        base, offset = (m.group(1), int(m.group(2), 16)) if m else (target, 0)
        return by_name[base] + offset if base in by_name else None

    import bisect

    def owner(address: int) -> int | None:
        index = bisect.bisect_right(starts, address) - 1
        return starts[index] if index >= 0 else None

    for address, func in funcs.items():
        calls = (resolve(a) for a in func.calls)
        tails = (resolve(a) for a in func.tails)
        func.calls = {owner(a) for a in calls if a is not None} - {None}
        func.tails = {owner(a) for a in tails if a is not None} - {None, address}

    sys.setrecursionlimit(100000)
    depth: dict[int, int] = {}
    best_child: dict[int, int | None] = {}
    on_path: set[int] = set()
    recursive: set[int] = set()

    def visit(address: int) -> int:
        if address in depth:
            return depth[address]
        if address in on_path:
            recursive.add(address)
            return 0
        on_path.add(address)
        func = funcs[address]
        best, child = func.frame, None
        for callee in func.calls:
            if PANIC_RE.search(funcs[callee].name):
                continue
            value = func.frame + visit(callee)
            if value > best:
                best, child = value, callee
        for callee in func.tails:
            if PANIC_RE.search(funcs[callee].name):
                continue
            value = visit(callee)
            if value > best:
                best, child = value, callee
        on_path.discard(address)
        depth[address] = best
        best_child[address] = child
        return best

    for address in starts:
        visit(address)

    # Deepest chain that executes MAC code, measured from its outermost
    # caller: the stack actually needed while the MAC runs, independent of
    # where the inliner happened to cut the MAC/runtime boundary.
    through: dict[int, int] = {}
    through_child: dict[int, int | None] = {}

    def visit_through(address: int) -> int:
        if address in through:
            return through[address]
        through[address] = -1  # recursion guard
        func = funcs[address]
        if is_mac_fn(func.name):
            through[address], through_child[address] = depth[address], best_child[address]
            return depth[address]
        best, child = -1, None
        for callee in func.calls:
            if PANIC_RE.search(funcs[callee].name):
                continue
            inner = visit_through(callee)
            if inner >= 0 and func.frame + inner > best:
                best, child = func.frame + inner, callee
        for callee in func.tails:
            if PANIC_RE.search(funcs[callee].name):
                continue
            inner = visit_through(callee)
            if inner >= 0 and inner > best:
                best, child = inner, callee
        through[address], through_child[address] = best, child
        return best

    for address in starts:
        visit_through(address)

    def path(address: int) -> list[str]:
        names, seen = [], set()
        while address is not None and address not in seen and len(names) < 12:
            seen.add(address)
            func = funcs[address]
            names.append(f"{short(func.name)}[{func.frame}]")
            address = best_child.get(address)
        return names

    def chain(address: int) -> list[str]:
        names, seen = [], set()
        while address is not None and address not in seen and len(names) < 16:
            seen.add(address)
            func = funcs[address]
            names.append(f"{short(func.name)}[{func.frame}]")
            address = (best_child if is_mac_fn(func.name) else through_child).get(address)
        return names

    def worst(predicate) -> tuple[int, int | None]:
        best, root = 0, None
        for address in starts:
            if predicate(funcs[address].name) and depth[address] > best:
                best, root = depth[address], address
        return best, root

    is_mac = is_mac_fn
    total, root = worst(lambda _name: True)
    mac, mac_root = worst(is_mac)
    chain_root = max(starts, key=lambda a: through[a], default=None)
    mac_chain = through[chain_root] if chain_root is not None and through[chain_root] > 0 else 0
    mac_frames = sorted(((f.frame, f.name) for f in funcs.values() if is_mac(f.name)),
                        reverse=True)
    return {
        "stack": total,
        "stack_path": path(root) if root is not None else [],
        "mac_stack": mac,
        "mac_stack_path": path(mac_root) if mac_root is not None else [],
        "mac_chain": mac_chain,
        "mac_chain_path": chain(chain_root) if mac_chain else [],
        "mac_frame": mac_frames[0][0] if mac_frames else 0,
        "mac_frame_fn": short(mac_frames[0][1]) if mac_frames else "",
        "mac_text": sum(1 for f in funcs.values() if is_mac(f.name)),
        "indirect": sum(f.indirect for f in funcs.values()),
        "dyn": sorted(short(f.name) for f in funcs.values() if f.dynamic)[:8],
        "recursive": len(recursive),
    }


def is_mac_fn(name: str) -> bool:
    """True when the function itself (not a generic argument) is MAC code."""
    return name.startswith(("zigbee_mac::", "<zigbee_mac::"))


def short(name: str) -> str:
    name = re.sub(r"::h[0-9a-f]{16}$", "", name)
    return name if len(name) <= 110 else name[:60] + "…" + name[-49:]


def mac_text_bytes(elf: Path) -> int:
    total = 0
    for line in run("llvm-nm", elf, "-S", "-C").splitlines():
        parts = line.split(maxsplit=3)
        if len(parts) == 4 and parts[2] in "tT" and is_mac_fn(parts[3]):
            total += int(parts[1], 16)
    return total


def report_one(name: str, elf: Path) -> dict:
    result = {"image": name, **sections(elf)}
    found = pools(elf)
    result["pool"] = found[0][1] if found else 0
    result["pool_symbol"] = short(found[0][0]) if found else ""
    result["mac_code"] = mac_text_bytes(elf)
    result.update(analyse_stack(elf))
    return result


COLUMNS = ["flash", "text", "data", "bss", "pool", "mac_code", "stack",
           "mac_chain", "mac_stack", "mac_frame"]


def print_table(rows: list[dict]) -> None:
    print("| image | " + " | ".join(COLUMNS) + " |")
    print("|---|" + "---:|" * len(COLUMNS))
    for row in rows:
        print(f"| {row['image']} | " +
              " | ".join(f"{row[c]:,}" for c in COLUMNS) + " |")


def report(args) -> int:
    targets: list[tuple[str, Path]] = []
    if args.out:
        for elf in sorted(Path(args.out).glob("*.elf")):
            targets.append((elf.stem, elf))
    for item in args.elf:
        name, _, path = item.partition("=")
        targets.append((name, Path(path)))
    rows = [report_one(name, path) for name, path in targets]
    print_table(rows)
    if args.verbose:
        for row in rows:
            print(f"\n{row['image']}: pool={row['pool_symbol']!r} "
                  f"indirect={row['indirect']} recursive={row['recursive']} "
                  f"dyn={row['dyn']}")
            print("  stack path: " + " -> ".join(row["stack_path"]))
            print("  mac chain:  " + " -> ".join(row["mac_chain_path"]))
            print("  mac path:   " + " -> ".join(row["mac_stack_path"]))
            print(f"  mac frame:  {row['mac_frame']} {row['mac_frame_fn']}")
    if args.json:
        Path(args.json).write_text(json.dumps(rows, indent=1) + "\n")
    return 0


def compare(args) -> int:
    base = {r["image"]: r for r in json.loads(Path(args.base).read_text())}
    new = {r["image"]: r for r in json.loads(Path(args.new).read_text())}
    grew = False
    print("| image | " + " | ".join(COLUMNS) + " |")
    print("|---|" + "---:|" * len(COLUMNS))
    for name in sorted(set(base) | set(new)):
        if name not in base or name not in new:
            print(f"| {name} | " + "missing | " * len(COLUMNS))
            continue
        cells = []
        for column in COLUMNS:
            if column not in base[name] or column not in new[name]:
                cells.append("n/a")
                continue
            delta = new[name][column] - base[name][column]
            if delta > 0 and column not in ("text", "mac_code", "mac_frame", "mac_stack"):
                grew = True
            cells.append(f"{new[name][column]:,} ({delta:+,})")
        print(f"| {name} | " + " | ".join(cells) + " |")
    return 1 if grew else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    b = sub.add_parser("build", help="build images like CI and collect ELFs")
    b.add_argument("--out", required=True)
    b.add_argument("--keep-going", action="store_true")
    b.add_argument("images", nargs="*", help="image-name regexes (default: all)")
    r = sub.add_parser("report", help="measure ELF files")
    r.add_argument("--out", help="directory of <image>.elf files")
    r.add_argument("--json", help="write rows as JSON")
    r.add_argument("-v", "--verbose", action="store_true")
    r.add_argument("elf", nargs="*", help="NAME=PATH")
    c = sub.add_parser("compare", help="diff two report JSON files")
    c.add_argument("base")
    c.add_argument("new")
    sub.add_parser("list", help="list known images")
    args = parser.parse_args()
    if args.command == "build":
        return build(select_images(args.images), Path(args.out), args.keep_going)
    if args.command == "report":
        return report(args)
    if args.command == "compare":
        return compare(args)
    for image in IMAGES:
        print(f"{image.name}: {image.workdir} {' '.join(image.args)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
