#!/usr/bin/env python3
"""Static stack gate for the ESP32-H2/C6 ZRHP NCP images (RISC-V, esp-hal).

Reuses the prologue/call-graph analysis of ``tools/mac-footprint.py`` and
checks one linked ELF against a reviewed arguments file::

    tools/esp32-ncp-stack-gate.py ELF @tools/esp32-ncp-stack-gate.args

The esp-hal linker script gives the main stack all RAM above ``.bss`` and
``.noinit``: ``_stack_end_cpu0`` (low end) .. ``_stack_start_cpu0`` (top).
esp-hal's stack guard word sits ``--guard`` bytes above the low end and a
data watchpoint faults on a write to it, so the usable stack ends there.

Checks:

* layout: ``_stack_end_cpu0`` is at or above ``_bss_end`` (and
  ``_noinit_end`` when present) and below ``_stack_start_cpu0``;
* budget: the worst task path (``--entry``, normally ``main``: the
  composition steps and the executor's root future are direct calls) plus
  the worst indirectly reached task-context function (``--task-extra``, e.g.
  the ``log::Log`` sink every ``log!`` reaches through ``dyn Log``) plus the
  interrupt budget leaves at least ``--min-gap`` bytes of the usable stack;
* interrupts: esp-hal re-enables interrupts while a handler runs, so a
  higher-priority interrupt nests. Every ``--irq-handler`` match (each an
  interrupt source that occurs at most once on the stack) is summed, each
  with one trap frame (``--trap-frame``) and the dispatcher
  (``--irq-dispatch``), plus the worst ``--irq-callback`` (functions the
  handlers reach through pointers, e.g. radio completion callbacks);
* frames: no function frame exceeds ``--max-frame`` unless listed in
  ``--frame-allow SYMBOL=CAP`` (exact demangled name without hash), and no
  function has a stack adjustment the prologue analysis cannot bound unless
  listed in ``--dynamic-allow REGEX`` (the trap entry's overflow stack
  switch);
* chains: every ``--chain-budget NAME=REGEX=BYTES`` holds (worst depth of
  any function matching REGEX), so a regression of a reviewed path fails.

This is a heuristic direct-call bound, not a whole-program proof: indirect
calls other than the named extras are not followed. Hardware stack paint
stays the authoritative runtime evidence.
"""

from __future__ import annotations

import argparse
import importlib.util
import re
import sys
from dataclasses import dataclass
from pathlib import Path

TOOLS = Path(__file__).resolve().parent


def _load_footprint():
    spec = importlib.util.spec_from_file_location("mac_footprint", TOOLS / "mac-footprint.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["mac_footprint"] = module
    spec.loader.exec_module(module)
    return module


footprint = _load_footprint()

LAYOUT_SYMBOLS = ("_stack_end_cpu0", "_stack_start_cpu0", "_bss_end")
OPTIONAL_SYMBOLS = ("_noinit_end",)


def plain(name: str) -> str:
    """Demangled name without the trailing ``::h<hash>``."""
    return re.sub(r"::h[0-9a-f]{16}$", "", name)


@dataclass
class Report:
    failures: list[str]
    lines: list[str]


def _int(text: str) -> int:
    return int(text, 0)


def _pair(text: str) -> tuple[str, int]:
    name, _, value = text.rpartition("=")
    if not name:
        raise argparse.ArgumentTypeError(f"expected NAME=BYTES, got {text!r}")
    return name, int(value, 0)


def _chain(text: str) -> tuple[str, re.Pattern, int]:
    name, _, rest = text.partition("=")
    pattern, _, value = rest.rpartition("=")
    if not name or not pattern:
        raise argparse.ArgumentTypeError(f"expected NAME=REGEX=BYTES, got {text!r}")
    return name, re.compile(pattern), int(value, 0)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                fromfile_prefix_chars="@")
    p.convert_arg_line_to_args = lambda line: (
        [] if not line.strip() or line.lstrip().startswith("#") else [line.strip()])
    p.add_argument("elf", nargs="?", type=Path)
    p.add_argument("--disassembly", type=Path,
                   help="use this llvm-objdump -d -C listing instead of the ELF's")
    p.add_argument("--guard", type=_int, required=True,
                   help="bytes above the stack's low end that hold the guard word")
    p.add_argument("--min-gap", type=_int, required=True)
    p.add_argument("--max-frame", type=_int, required=True)
    p.add_argument("--entry", type=re.compile, required=True)
    p.add_argument("--task-extra", type=re.compile, action="append", default=[])
    p.add_argument("--trap-frame", type=_int, required=True)
    p.add_argument("--irq-dispatch", type=re.compile, required=True)
    p.add_argument("--irq-handler", type=re.compile, required=True)
    p.add_argument("--irq-callback", type=re.compile, action="append", default=[])
    p.add_argument("--frame-allow", type=_pair, action="append", default=[])
    p.add_argument("--dynamic-allow", type=re.compile, action="append", default=[])
    p.add_argument("--chain-budget", type=_chain, action="append", default=[])
    return p


def read_symbols(elf: Path) -> dict[str, int]:
    symbols = {}
    wanted = LAYOUT_SYMBOLS + OPTIONAL_SYMBOLS
    for line in footprint.run("llvm-nm", elf).splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] in wanted:
            symbols[parts[2]] = int(parts[0], 16)
    return symbols


def check_layout(args, symbols: dict[str, int], report: Report) -> int:
    """Usable stack bytes (0 when the layout is unusable)."""
    missing = [name for name in LAYOUT_SYMBOLS if name not in symbols]
    if missing:
        report.failures.append(f"layout: missing symbols {', '.join(missing)}")
        return 0
    low, top = symbols["_stack_end_cpu0"], symbols["_stack_start_cpu0"]
    data_end = max(symbols["_bss_end"], symbols.get("_noinit_end", 0))
    usable = top - low - args.guard
    report.lines.append(
        f"layout: stack 0x{low:08x}..0x{top:08x} ({top - low} B, {usable} B above the "
        f"{args.guard} B guard), RAM data end 0x{data_end:08x}")
    if data_end > low:
        report.failures.append("layout: .bss/.noinit overlap the stack")
    if usable <= 0:
        report.failures.append("layout: no usable stack")
        return 0
    return usable


def check_graph(args, disasm: str, stack_bytes: int, report: Report) -> None:
    funcs = footprint.build_call_graph(disasm)
    depth, best_child, _recursive = footprint.stack_depths(funcs)

    def matches(pattern: re.Pattern) -> list[int]:
        return [a for a, f in funcs.items() if pattern.search(plain(f.name))]

    def worst(pattern: re.Pattern, what: str, required: bool = True) -> tuple[int, int | None]:
        found = matches(pattern)
        if not found:
            if required:
                report.failures.append(f"graph: no function matches {what} {pattern.pattern!r}")
            return 0, None
        address = max(found, key=lambda a: depth[a])
        return depth[address], address

    def path(address: int | None) -> str:
        names, seen = [], set()
        while address is not None and address not in seen and len(names) < 16:
            seen.add(address)
            names.append(f"{footprint.short(plain(funcs[address].name))}[{funcs[address].frame}]")
            address = best_child.get(address)
        return " -> ".join(names)

    entry, entry_at = worst(args.entry, "--entry")
    extra = 0
    for pattern in args.task_extra:
        value, _ = worst(pattern, "--task-extra")
        extra = max(extra, value)

    dispatch, _ = worst(args.irq_dispatch, "--irq-dispatch")
    handlers = sorted(matches(args.irq_handler), key=lambda a: -depth[a])
    if not handlers:
        report.failures.append(f"graph: no function matches --irq-handler "
                               f"{args.irq_handler.pattern!r}")
    callback = 0
    for pattern in args.irq_callback:
        value, _ = worst(pattern, "--irq-callback", required=False)
        callback = max(callback, value)
    irq = sum(args.trap_frame + dispatch + depth[a] for a in handlers) + callback

    total = entry + extra + irq
    gap = stack_bytes - total
    report.lines.append(f"budget: task {entry} + task extra {extra} + irq {irq} "
                        f"= {total} B of {stack_bytes} B (gap {gap} B, min {args.min_gap} B)")
    report.lines.append(f"  task path: {path(entry_at)}")
    report.lines.append(
        f"  irq: {len(handlers)} nestable handler(s) x (trap {args.trap_frame} + dispatch "
        f"{dispatch}) + callback {callback}: " + ", ".join(
            f"{footprint.short(plain(funcs[a].name))}[{depth[a]}]" for a in handlers))
    if gap < args.min_gap:
        report.failures.append(f"budget: gap {gap} B below the {args.min_gap} B minimum")

    allowed = dict(args.frame_allow)
    seen_allowed = set()
    for func in funcs.values():
        name = plain(func.name)
        if func.dynamic and not any(p.search(name) for p in args.dynamic_allow):
            report.failures.append(f"frame: {footprint.short(name)} has a stack adjustment "
                                   f"the analysis cannot bound (counted {func.frame} B)")
        if name in allowed:
            seen_allowed.add(name)
            if func.frame > allowed[name]:
                report.failures.append(f"frame: {footprint.short(name)} is {func.frame} B, "
                                       f"reviewed cap {allowed[name]} B")
        elif func.frame > args.max_frame:
            report.failures.append(f"frame: {footprint.short(name)} is {func.frame} B "
                                   f"(> --max-frame {args.max_frame} B)")
    for name in sorted(set(allowed) - seen_allowed):
        report.lines.append(f"  note: --frame-allow {footprint.short(name)} is not in this image")

    for name, pattern, budget in args.chain_budget:
        value, address = worst(pattern, f"chain {name}")
        status = "ok" if value <= budget else "OVER"
        report.lines.append(f"chain {name}: {value} B (budget {budget} B) {status}")
        if value > budget:
            report.failures.append(f"chain {name}: {value} B over its {budget} B budget: "
                                   f"{path(address)}")


def run(args, symbols: dict[str, int], disasm: str) -> Report:
    report = Report([], [])
    stack_bytes = check_layout(args, symbols, report)
    check_graph(args, disasm, stack_bytes, report)
    return report


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if args.elf is None:
        raise SystemExit("an ELF is required")
    disasm = (args.disassembly.read_text() if args.disassembly
              else footprint.run("llvm-objdump", args.elf, "-d", "-C", "--no-show-raw-insn"))
    report = run(args, read_symbols(args.elf), disasm)
    print(f"{args.elf.name}:")
    for line in report.lines:
        print(f"  {line}")
    for failure in report.failures:
        print(f"FAIL {failure}", file=sys.stderr)
    return 1 if report.failures else 0


if __name__ == "__main__":
    sys.exit(main())
