#!/usr/bin/env python3
"""Static stack gate for the BL702 XT-ZB1 images (RISC-V, rv32imc).

Reuses the prologue/call-graph analysis of ``tools/mac-footprint.py`` and
checks one linked ELF against a reviewed arguments file::

    tools/bl702-stack-gate.py ELF @tools/bl702-router-stack-gate.args

Checks:

* layout: ``.stack`` spans exactly ``--declared`` bytes, ends at or below
  ``--stack-ceiling``, and the ``--guard``-byte stack guard
  (``_stack_guard_start.._stack_guard_end``) sits immediately below it,
  above ``.bss``/``.uninit``/heap;
* budget: the worst task path (``--entry`` + the deepest ``--task`` poll,
  which the executor reaches through a function pointer) plus the worst
  interrupt path (``--irq-entry`` + the deepest ``--irq-handler``, reached
  through the CLIC dispatch pointer; interrupts do not nest) leaves at least
  ``--min-gap`` bytes of the declared stack;
* frames: no function frame exceeds ``--max-frame`` unless listed in
  ``--frame-allow SYMBOL=CAP`` (exact demangled name without hash), and no
  function has a stack adjustment the prologue analysis cannot bound (it
  would otherwise count as 0 B and hide a large frame);
* chains: every ``--chain-budget NAME=REGEX=BYTES`` holds (worst depth of
  any function matching REGEX), so a regression of a reviewed path fails.

This is a heuristic direct-call bound, not a whole-program proof: indirect
calls (trait objects, function pointers) other than the two named roots are
not followed. Hardware stack paint stays the authoritative runtime evidence.
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

LAYOUT_SYMBOLS = ("__estack", "__sstack", "_stack_guard_start", "_stack_guard_end",
                  "__ebss", "__euninit", "__eheap")


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
    p.add_argument("--declared", type=_int, required=True, help="stack bytes")
    p.add_argument("--guard", type=_int, required=True, help="guard bytes")
    p.add_argument("--stack-ceiling", type=_int, required=True)
    p.add_argument("--min-gap", type=_int, required=True)
    p.add_argument("--max-frame", type=_int, required=True)
    p.add_argument("--entry", type=re.compile, required=True)
    p.add_argument("--task", type=re.compile, required=True)
    p.add_argument("--irq-entry", type=re.compile, required=True)
    p.add_argument("--irq-handler", type=re.compile, required=True)
    p.add_argument("--frame-allow", type=_pair, action="append", default=[])
    p.add_argument("--chain-budget", type=_chain, action="append", default=[])
    return p


def read_symbols(elf: Path) -> dict[str, int]:
    symbols = {}
    for line in footprint.run("llvm-nm", elf).splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] in LAYOUT_SYMBOLS:
            symbols[parts[2]] = int(parts[0], 16)
    return symbols


def check_layout(args, symbols: dict[str, int], report: Report) -> None:
    missing = [name for name in LAYOUT_SYMBOLS if name not in symbols]
    if missing:
        report.failures.append(f"layout: missing symbols {', '.join(missing)}")
        return
    low, top = symbols["__estack"], symbols["__sstack"]
    guard_start, guard_end = symbols["_stack_guard_start"], symbols["_stack_guard_end"]
    data_end = max(symbols["__ebss"], symbols["__euninit"], symbols["__eheap"])
    report.lines.append(
        f"layout: stack 0x{low:08x}..0x{top:08x} ({top - low} B), "
        f"guard 0x{guard_start:08x}..0x{guard_end:08x}, RAM data end 0x{data_end:08x} "
        f"({guard_start - data_end} B free below the guard)")
    if top - low != args.declared:
        report.failures.append(f"layout: stack is {top - low} B, declared {args.declared} B")
    if top > args.stack_ceiling:
        report.failures.append(f"layout: stack top 0x{top:08x} above 0x{args.stack_ceiling:08x}")
    if guard_end != low or guard_end - guard_start != args.guard:
        report.failures.append("layout: the guard is not immediately below the stack "
                               f"or is not {args.guard} B")
    if data_end > guard_start:
        report.failures.append("layout: .bss/.uninit/heap overlap the stack guard")


def check_graph(args, disasm: str, stack_bytes: int, report: Report) -> None:
    funcs = footprint.build_call_graph(disasm)
    depth, best_child, _recursive = footprint.stack_depths(funcs)

    def worst(pattern: re.Pattern, what: str) -> tuple[int, int | None]:
        matches = [a for a, f in funcs.items() if pattern.search(plain(f.name))]
        if not matches:
            report.failures.append(f"graph: no function matches {what} {pattern.pattern!r}")
            return 0, None
        address = max(matches, key=lambda a: depth[a])
        return depth[address], address

    def path(address: int | None) -> str:
        names, seen = [], set()
        while address is not None and address not in seen and len(names) < 14:
            seen.add(address)
            names.append(f"{footprint.short(plain(funcs[address].name))}[{funcs[address].frame}]")
            address = best_child.get(address)
        return " -> ".join(names)

    entry, entry_at = worst(args.entry, "--entry")
    task, task_at = worst(args.task, "--task")
    irq_entry, _ = worst(args.irq_entry, "--irq-entry")
    irq_handler, irq_at = worst(args.irq_handler, "--irq-handler")
    task_total = entry + task
    irq_total = irq_entry + irq_handler
    total = task_total + irq_total
    gap = stack_bytes - total
    report.lines.append(f"budget: entry {entry} + task {task} + irq {irq_entry}+{irq_handler} "
                        f"= {total} B of {stack_bytes} B (gap {gap} B, min {args.min_gap} B)")
    report.lines.append(f"  task path: {path(task_at)}")
    report.lines.append(f"  irq path:  {path(irq_at)}")
    if entry_at is not None and entry > 1024:
        report.lines.append(f"  entry path: {path(entry_at)}")
    if gap < args.min_gap:
        report.failures.append(f"budget: gap {gap} B below the {args.min_gap} B minimum")

    allowed = dict(args.frame_allow)
    seen_allowed = set()
    for func in funcs.values():
        name = plain(func.name)
        if func.dynamic:
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
    check_layout(args, symbols, report)
    # Budget against the linked stack (the declared size if the layout
    # symbols are missing; that already failed above).
    if "__estack" in symbols and "__sstack" in symbols:
        stack_bytes = min(args.declared, symbols["__sstack"] - symbols["__estack"])
    else:
        stack_bytes = args.declared
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
