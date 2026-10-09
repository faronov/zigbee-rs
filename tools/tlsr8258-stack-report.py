#!/usr/bin/env python3
"""Static stack report for a linked TLSR8258 (TC32) firmware ELF.

The report is derived only from the linked image:

* linker stack symbols (`_svc_stack_*`, `_irq_stack_*`, `_rf_dma_end_`);
* per-function frame sizes decoded from the TC32 prologue
  (`tpush`, `tsub sp, #imm`, `tadd sp, rX` with a negative literal-pool
  constant, and `sp` re-alignment);
* direct call edges (`tjl`/`tj` to another function symbol).

It is a heuristic, not a sound whole-program bound:

* indirect calls use the TC32 `__call_via_rN` idiom (`tjl` to a local
  `tjex rN` stub). A site is resolved to the function pointers held in the
  vtables/function-pointer literals loaded by the same function (this covers
  `dyn Future::poll`). Unresolved sites are listed; `--indirect global`
  resolves them to every address-taken function instead (coarser bound);
* `tmov pc, rX` that is not the TC32 jump-table idiom is reported;
* recursive edges are reported and cut, not unrolled;
* literal pools mis-decoded as instructions can create spurious edges, which
  only over-approximate;
* tail calls (`tj`) are counted as nested calls, which over-approximates;
* runtime reachability of an edge is not proven. A worst path is a static
  upper bound of the direct-call graph, not a measured high-water.

The HIL stack-paint high-water remains the authoritative runtime evidence.

Exit status with `--check`: 1 when any gate fails:

* the worst SVC or IRQ path exceeds the declared linker stack;
* `--declared-svc`/`--declared-irq` differ from the linked stack size;
* the physical gap between `_rf_dma_end_` and the stack guard (or the SVC
  bottom when no guard is linked) is below `--min-gap`;
* a frame exceeds `--max-frame` and is not covered by a capped
  `--frame-allow NAME=BYTES` exception. NAME is an exact demangled symbol
  name, not a pattern; a missing exception symbol also fails;
* a `--frame-allow` symbol, or any frame above `--max-frame`, is reachable
  from a `--runtime-root REGEX` function (the steady-state executor) or from
  the IRQ root: reviewed frame exceptions are bootstrap-only;
* the worst path through any function matching a `--chain-budget
  LABEL=REGEX=BYTES` exceeds its budget (or nothing matches);
* a function with unresolved indirect call sites is not matched by a
  `--allow-unresolved REGEX`. Allowlisting only records that the site was
  reviewed; it does NOT prove the unresolved edges are bounded.

Arguments may be read from a file with `@FILE` (one argument per line, `#`
comments and blank lines ignored).
"""

import argparse
import json
import os
import re
import struct
import subprocess
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_TOOLCHAIN = ROOT_DIR / ".toolchains" / "tc32-1.98.1-20261003-31a272"

DEFAULT_FOCUS = [
    ("app::run", r"telink_tlsr8258_router::app::run$"),
    ("block_on", r"tlsr8258_rt::block_on"),
    ("RouterCore future", r"router_app::app::RouterCore<.*\{closure#0\}"),
    ("ParentRouterApp::run", r"router_app::app::ParentRouterApp<.*>::run::\{closure"),
    ("ApsTableJournal::current/newest", r"ApsTableJournal<.*>::(current|newest)"),
    ("ApsTableJournal::store/write_record", r"ApsTableJournal<.*>.*(store|write_record)"),
    ("ApsTableJournal::load", r"ApsTableJournal<.*>.*::load"),
    ("PersistentApsTables::decode", r"PersistentApsTables>::decode|PersistentApsTables::decode"),
    ("ChildTableJournal", r"ChildTableJournal<"),
    ("SecurityStateJournal", r"SecurityStateJournal<[^>]*>[^{]*::(load|store|commit|current|newest|append|write)"),
    ("TrustCenter store", r"trust_center_store"),
    ("restore/clear APS tables", r"(restore_aps_tables|clear_persisted_aps_tables|persist_if_dirty|clear_stale_before_fresh)"),
    ("ZDO receive", r"zigbee_zdo::.*(handle|process|dispatch)"),
    ("ZCL dispatch", r"zcl_dispatch::.*dispatch"),
]


class Elf:
    def __init__(self, path):
        self.data = Path(path).read_bytes()
        if self.data[:4] != b"\x7fELF" or self.data[4] != 1 or self.data[5] != 1:
            raise SystemExit(f"{path}: not a 32-bit little-endian ELF")
        (shoff,) = struct.unpack_from("<I", self.data, 0x20)
        shentsize, shnum, shstrndx = struct.unpack_from("<HHH", self.data, 0x2E)
        self.sections = []
        for index in range(shnum):
            fields = struct.unpack_from("<IIIIIIIIII", self.data, shoff + index * shentsize)
            self.sections.append(fields)
        self.symbols = {}
        for fields in self.sections:
            if fields[1] != 2:  # SHT_SYMTAB
                continue
            strtab = self.sections[fields[6]]
            for offset in range(fields[4], fields[4] + fields[5], 16):
                name_off, value, _size, _info, _other, _shndx = struct.unpack_from(
                    "<IIIBBH", self.data, offset
                )
                start = strtab[4] + name_off
                end = self.data.index(b"\0", start)
                name = self.data[start:end].decode("utf-8", "replace")
                if name:
                    self.symbols.setdefault(name, value)

    def read_word(self, address):
        for _name, kind, _flags, addr, offset, size, *_ in self.sections:
            if kind == 1 and addr <= address and address + 4 <= addr + size:
                return struct.unpack_from("<i", self.data, offset + address - addr)[0]
        return None


HEADER = re.compile(r"^([0-9a-f]+) <(.+)>:$")
INSN = re.compile(r"^\s*([0-9a-f]+):\s+\t(\S+)(?:\t(.*))?$")
LITERAL = re.compile(r"\[pc, #0x[0-9a-f]+\]\s+@ 0x([0-9a-f]+)")
BRANCH = re.compile(r"^0x[0-9a-f]+ <([^>+]+)(\+0x[0-9a-f]+)?>")


def register_count(register_list):
    count = 0
    for item in register_list.split(","):
        item = item.strip()
        match = re.fullmatch(r"r(\d+)-r(\d+)", item)
        count += int(match.group(2)) - int(match.group(1)) + 1 if match else 1
    return count


COPY_ROUTINES = ("memcpy", "memmove", "memset", "__aeabi_memcpy", "__aeabi_memmove",
                 "__aeabi_memset", "__aeabi_memclr")


def track_constants(registers, mnemonic, operands, current):
    """Heuristic constant tracking so `bl memcpy` sites can report r2 (length).

    Only `tmov rX, #imm` and `tshftl rX, rY, #n` produce known values here;
    literal loads are recorded by the caller. Any other write to a low
    register forgets it, and calls clobber r0-r3. The result is an
    attribution aid, not a proof.
    """
    target = operands.split(",")[0].strip()
    immediate = re.fullmatch(r"(r[0-7]), #(0x[0-9a-f]+|\d+)", operands.strip())
    shift = re.fullmatch(r"(r[0-7]), (r[0-7]), #(0x[0-9a-f]+|\d+)", operands.strip())
    if mnemonic in ("tjl", "tj"):
        branch = BRANCH.match(operands)
        callee = branch.group(1) if branch else ""
        if mnemonic == "tjl" and callee in COPY_ROUTINES:
            length = registers.get("r2")
            if length is not None and 0 < length < 0x10000:
                current["copies"].append((length, callee))
        for register in ("r0", "r1", "r2", "r3"):
            registers.pop(register, None)
    elif mnemonic == "tmov" and immediate:
        registers[immediate.group(1)] = int(immediate.group(2), 0)
    elif mnemonic == "tshftl" and shift and shift.group(2) in registers:
        registers[shift.group(1)] = registers[shift.group(2)] << int(shift.group(3), 0)
    elif mnemonic == "tloadr" and LITERAL.search(operands):
        pass
    elif re.fullmatch(r"r[0-7]", target) and mnemonic not in ("tstorer", "tstorerb", "tstorerh",
                                                              "tcmp", "tcmpn", "tnand", "tpush"):
        registers.pop(target, None)


def parse_disassembly(lines, read_word):
    stubs = set()
    for line in lines:
        insn = INSN.match(line)
        if insn and insn.group(2) == "tjex" and re.fullmatch(r"r[0-7]", (insn.group(3) or "").strip()):
            stubs.add(int(insn.group(1), 16))
    functions = {}
    current = None
    registers = {}
    history = []
    for line in lines:
        header = HEADER.match(line)
        if header:
            current = {
                "address": int(header.group(1), 16),
                "push": 0,
                "alloc": 0,
                "align": 0,
                "adjustments": 0,
                "calls": {},
                "indirect": [],
                "indirect_calls": [],
                "literals": set(),
                "copies": [],
            }
            functions[header.group(2)] = current
            name = header.group(2)
            registers = {}
            history = []
            continue
        if current is None:
            continue
        insn = INSN.match(line)
        if not insn:
            continue
        address, mnemonic, operands = insn.group(1), insn.group(2), insn.group(3) or ""
        history = (history + [(mnemonic, operands)])[-8:]
        track_constants(registers, mnemonic, operands, current)
        if mnemonic == "tpush":
            current["push"] += 4 * register_count(operands.strip("{} "))
        elif mnemonic == "tsub" and operands.startswith("sp, #"):
            current["alloc"] += int(operands.split("#")[1], 16)
            current["adjustments"] += 1
        elif mnemonic == "tloadr":
            literal = LITERAL.search(operands)
            if literal:
                value = read_word(int(literal.group(1), 16))
                if value is not None:
                    registers[operands.split(",")[0]] = value
                    current["literals"].add(value & 0xFFFFFFFF)
        elif mnemonic == "tadd" and re.fullmatch(r"sp, r\d+", operands.strip()):
            value = registers.get(operands.split(",")[1].strip())
            if value is not None and value < 0:
                current["alloc"] += -value
                current["adjustments"] += 1
        elif mnemonic == "tmov" and re.fullmatch(r"sp, r\d+", operands.strip()):
            shifts = [m for m, _ in history[-4:] if m in ("tshftr", "tshftl")]
            if len(shifts) >= 2:
                current["align"] = max(current["align"], 12)
        elif mnemonic == "tmov" and operands.startswith("pc, r"):
            if not any(m == "tadd" and o.endswith(", pc") for m, o in history[:-1]):
                current["indirect"].append(address)
        elif mnemonic in ("tjl", "tj") and int(operands.split()[0], 16) in stubs:
            current["indirect_calls"].append(address)
        elif mnemonic in ("tjl", "tj"):
            branch = BRANCH.match(operands)
            if branch and branch.group(2) is None and branch.group(1) != name:
                kind = "call" if mnemonic == "tjl" else "tail"
                current["calls"].setdefault(branch.group(1), kind)
    for function in functions.values():
        function["frame"] = function["push"] + function["alloc"] + function["align"]
    return functions


def resolve_indirect(functions, read_word, mode):
    """Attach indirect-call targets; return {function: unresolved site count}."""
    by_address = {f["address"]: name for name, f in functions.items()}
    # The hardware reset vector at 0x0 is never a Rust function-pointer
    # target: a Thumb-tagged literal `1` is an ordinary integer constant, and
    # even a real jump to reset re-initializes SP rather than nesting stack.
    by_address.pop(0, None)

    def pointer(word):
        if word is not None and word & 1:
            return by_address.get((word & 0xFFFFFFFF) - 1)
        return None

    def block(value):
        direct = pointer(value)
        if direct:
            return [direct]
        targets, address = [], value
        for index in range(16):
            word = read_word(address)
            if word is None:
                break
            if index >= 3 and (word == 0 or pointer(word)):
                size, align = read_word(address + 4), read_word(address + 8)
                if size is not None and 0 <= size < 0x10000 and align in (1, 2, 4, 8, 16):
                    break  # the next Rust vtable header: drop, size, align
            target = pointer(word)
            if target:
                targets.append(target)
            elif not 0 <= word < 0x10000:
                break
            address += 4
        return targets

    taken = set()
    for value in {v for f in functions.values() for v in f["literals"]}:
        taken.update(block(value))
    unresolved = {}
    for name, function in functions.items():
        if not function["indirect_calls"]:
            continue
        targets = set()
        for value in function["literals"]:
            targets.update(block(value))
        targets.discard(name)
        if not targets:
            unresolved[name] = len(function["indirect_calls"])
            if mode == "global":
                targets = taken - {name}
        for target in targets:
            function["calls"].setdefault(target, "indirect")
    return unresolved, len(taken)


class Graph:
    def __init__(self, functions):
        self.functions = functions
        self.recursive_edges = set()
        self.subtree = {}
        self.next = {}

    def frame(self, name):
        return self.functions.get(name, {}).get("frame", 0)

    def address(self, name):
        return self.functions.get(name, {}).get("address", 0)

    def callees(self, name):
        return self.functions.get(name, {}).get("calls", {})

    def analyze(self, root):
        sys.setrecursionlimit(max(10000, 4 * len(self.functions)))
        state = {}
        order = []

        def visit(name):
            state[name] = 1
            best, best_child = 0, None
            for child in self.callees(name):
                if state.get(child) == 1:
                    self.recursive_edges.add((name, child))
                    continue
                if child not in state:
                    visit(child)
                if self.subtree[child] > best:
                    best, best_child = self.subtree[child], child
            self.subtree[name] = self.frame(name) + best
            self.next[name] = best_child
            state[name] = 2
            order.append(name)

        visit(root)
        prefix = {root: 0}
        parent = {root: None}
        for name in reversed(order):
            if name not in prefix:
                continue
            reach = prefix[name] + self.frame(name)
            for child in self.callees(name):
                if (name, child) in self.recursive_edges:
                    continue
                if reach > prefix.get(child, -1):
                    prefix[child] = reach
                    parent[child] = name
        return prefix, parent

    def worst_path(self, name):
        path = []
        while name is not None:
            path.append(name)
            name = self.next.get(name)
        return path


def compact_generics(name):
    """Collapse generic arguments nested two or more levels deep to `<..>`."""
    out, depth = [], 0
    for char in name:
        if char == "<":
            depth += 1
            if depth == 2:
                out.append("<..>")
            if depth >= 2:
                continue
        elif char == ">" and depth >= 2:
            depth -= 1
            continue
        elif char == ">":
            depth -= 1
        elif depth >= 2:
            continue
        out.append(char)
    return "".join(out)


def demangler(cxxfilt, compact=False):
    cache = {}

    def demangle_all(names):
        missing = [name for name in names if name not in cache]
        if missing and cxxfilt and Path(cxxfilt).exists():
            result = subprocess.run(
                [cxxfilt], input="\n".join(missing), capture_output=True, text=True, check=True
            )
            for name, pretty in zip(missing, result.stdout.splitlines()):
                pretty = re.sub(r"\[[0-9a-f]+\]|::h[0-9a-f]{16}", "", pretty)
                cache[name] = compact_generics(pretty) if compact else pretty
        for name in missing:
            cache.setdefault(name, name)

    def demangle(name):
        demangle_all([name])
        return cache[name]

    demangle.prime = demangle_all
    return demangle


def chain_to(parent, name):
    chain = []
    while name is not None:
        chain.append(name)
        name = parent.get(name)
    return list(reversed(chain))


def describe_path(graph, path, start, pretty, width):
    rows = []
    total = start
    for name in path:
        total += graph.frame(name)
        rows.append({"function": pretty(name)[:width], "address": graph.address(name),
                     "frame": graph.frame(name), "cumulative": total})
    return rows


def layout_failures(layout, declared_svc=None, declared_irq=None, min_gap=None):
    failures = []
    for label, required, actual in (
        ("SVC", declared_svc, layout["declared_svc_stack_bytes"]),
        ("IRQ", declared_irq, layout["declared_irq_stack_bytes"]),
    ):
        if required is not None and actual != required:
            failures.append(f"declared {label} stack {actual} B != required {required} B")
    gap = layout["physical_gap_bytes_rf_dma_end_to_guard"]
    if min_gap is not None and gap < min_gap:
        failures.append(f"rf_dma_end-to-guard gap {gap} B < {min_gap} B")
    return failures


def parse_frame_allow(items):
    """Exact-symbol frame exceptions: `NAME=BYTES` -> {NAME: BYTES}."""
    allow = {}
    for item in items:
        name, cap = item.rsplit("=", 1)
        if name in allow:
            raise SystemExit(f"duplicate --frame-allow for {name}")
        allow[name] = int(cap)
    return allow


def frame_limit(name, max_frame, frame_allow):
    """Effective frame limit: the reviewed cap for exactly `name`, else `max_frame`."""
    return frame_allow.get(name, max_frame)


def reachable_from(graph, roots):
    seen, stack = set(), list(roots)
    while stack:
        name = stack.pop()
        if name in seen:
            continue
        seen.add(name)
        stack.extend(graph.callees(name))
    return seen


def bootstrap_only_failures(graph, names, frame_allow, max_frame, runtime_roots, irq_root):
    """Frame exceptions must exist and stay off the runtime/IRQ call graphs;
    no frame above `max_frame` may be reachable from them either."""
    failures = []
    present = {pretty_name for pretty_name in names.values()}
    for name in sorted(frame_allow):
        if name not in present:
            failures.append(f"frame exception symbol {name} not found in the ELF")
    reach = reachable_from(graph, runtime_roots + [irq_root])
    for name in sorted(reach, key=lambda n: names.get(n, n)):
        pretty_name = names.get(name, name)
        if pretty_name in frame_allow:
            failures.append(f"frame exception {pretty_name} is reachable from a runtime/IRQ root")
        elif max_frame is not None and graph.frame(name) > max_frame:
            failures.append(f"runtime frame {graph.frame(name)} B > {max_frame} B in {pretty_name[:120]}")
    return failures


def chain_budget(graph, svc_prefix, names, item, svc_parent=None):
    label, rest = item.split("=", 1)
    pattern, budget = rest.rsplit("=", 1)
    regex = re.compile(pattern)
    matches = [name for name, pretty_name in names.items()
               if regex.search(pretty_name) and name in svc_prefix]
    worst = max((svc_prefix[name] + graph.subtree[name] for name in matches), default=None)
    result = {"label": label, "budget": int(budget), "worst_through": worst, "matches": len(matches)}
    if worst is not None:
        # Deterministic entry: the worst matching function closest to the root.
        entry = min((name for name in matches if svc_prefix[name] + graph.subtree[name] == worst),
                    key=lambda name: (svc_prefix[name], names[name]))
        path = (chain_to(svc_parent, entry) if svc_parent is not None else [entry])
        path += graph.worst_path(entry)[1:]
        largest = max(path, key=graph.frame)
        result.update({
            "entry": names[entry],
            "terminal": names.get(path[-1], path[-1]),
            "largest_frame": graph.frame(largest),
            "largest_frame_function": names.get(largest, largest),
            "path_bytes": sum(graph.frame(name) for name in path),
            "path": [names.get(name, name) for name in path],
        })
    if worst is None:
        return result, f"chain budget {label}: no reachable function matches {pattern}"
    if worst > int(budget):
        return result, f"chain budget {label}: worst through {worst} B > {budget} B"
    return result, None


def unreviewed(names, patterns):
    regexes = [re.compile(pattern) for pattern in patterns]
    return sorted(name for name in names if not any(regex.search(name) for regex in regexes))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0], fromfile_prefix_chars="@")
    parser.convert_arg_line_to_args = lambda line: (
        [] if not line.strip() or line.lstrip().startswith("#") else [line.strip()])
    parser.add_argument("elf")
    toolchain = Path(os.environ.get("TC32_TOOLCHAIN", DEFAULT_TOOLCHAIN))
    parser.add_argument("--objdump", default=str(toolchain / "llvm" / "bin" / "llvm-objdump"))
    parser.add_argument("--cxxfilt", default=str(toolchain / "llvm" / "bin" / "llvm-cxxfilt"))
    parser.add_argument("--disassembly", help="reuse an existing `llvm-objdump -d` listing")
    parser.add_argument("--svc-root", default="_rust_entry")
    parser.add_argument("--irq-root", default="__irq")
    parser.add_argument("--frame-threshold", type=int, default=1024)
    parser.add_argument("--max-frame", type=int, help="fail --check when a frame exceeds this")
    parser.add_argument("--focus", action="append", default=[], metavar="LABEL=REGEX")
    parser.add_argument("--width", type=int, default=160)
    parser.add_argument("--exclude", action="append", default=[], metavar="REGEX",
                        help="what-if: prune calls into functions whose demangled name matches")
    parser.add_argument("--compact", action="store_true",
                        help="collapse generic arguments nested >= 2 levels deep")
    parser.add_argument("--indirect", choices=("local", "global"), default="local",
                        help="resolution of indirect sites without a local vtable")
    parser.add_argument("--copies", type=int, default=0, metavar="N",
                        help="show the N largest constant memcpy/memset lengths per large frame")
    parser.add_argument("--json", help="write the machine-readable report here")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--declared-svc", type=int, help="required linked SVC stack size")
    parser.add_argument("--declared-irq", type=int, help="required linked IRQ stack size")
    parser.add_argument("--min-gap", type=int,
                        help="required bytes between _rf_dma_end_ and the guard/SVC bottom")
    parser.add_argument("--frame-allow", action="append", default=[], metavar="NAME=BYTES",
                        help="reviewed bootstrap-only exception to --max-frame for exactly the "
                             "demangled symbol NAME, capped at BYTES")
    parser.add_argument("--runtime-root", action="append", default=[], metavar="REGEX",
                        help="steady-state executor root(s); --frame-allow symbols and frames "
                             "above --max-frame must not be reachable from them")
    parser.add_argument("--chain-budget", action="append", default=[], metavar="LABEL=REGEX=BYTES",
                        help="worst SVC path through any matching function must stay <= BYTES")
    parser.add_argument("--allow-unresolved", action="append", default=[], metavar="REGEX",
                        help="reviewed function allowed to keep unresolved indirect call sites")
    args = parser.parse_args()

    elf = Elf(args.elf)
    if args.disassembly:
        lines = Path(args.disassembly).read_text().splitlines()
    else:
        lines = subprocess.run(
            [args.objdump, "-d", "--no-show-raw-insn", args.elf], capture_output=True, text=True, check=True
        ).stdout.splitlines()
    functions = parse_disassembly(lines, elf.read_word)
    unresolved, taken_count = resolve_indirect(functions, elf.read_word, args.indirect)
    pretty = demangler(args.cxxfilt, compact=args.compact)
    pretty.prime(list(functions))
    excluded = []
    if args.exclude:
        # What-if pruning: drop every edge into a matching function so the
        # report shows the worst path that does not traverse it.
        regexes = [re.compile(pattern) for pattern in args.exclude]
        excluded = sorted(name for name in functions
                          if any(regex.search(pretty(name)) for regex in regexes))
        for function in functions.values():
            for name in excluded:
                function["calls"].pop(name, None)
    graph = Graph(functions)

    symbols = elf.symbols
    for required in ("_svc_stack_bottom", "_svc_stack_top", "_irq_stack_bottom",
                     "_irq_stack_top", "_rf_dma_end_"):
        if required not in symbols:
            raise SystemExit(f"missing linker symbol {required}")
    layout = {
        "rf_dma_end": symbols["_rf_dma_end_"],
        "svc_stack_bottom": symbols["_svc_stack_bottom"],
        "svc_stack_top": symbols["_svc_stack_top"],
        "irq_stack_bottom": symbols["_irq_stack_bottom"],
        "irq_stack_top": symbols["_irq_stack_top"],
    }
    layout["declared_svc_stack_bytes"] = layout["svc_stack_top"] - layout["svc_stack_bottom"]
    layout["declared_irq_stack_bytes"] = layout["irq_stack_top"] - layout["irq_stack_bottom"]
    layout["physical_bytes_svc_top_to_rf_dma_end"] = layout["svc_stack_top"] - layout["rf_dma_end"]
    guard = symbols.get("_svc_stack_guard_start")
    if guard is not None:
        layout["svc_stack_guard_start"] = guard
        layout["svc_stack_guard_bytes"] = layout["svc_stack_bottom"] - guard
    layout["physical_gap_bytes_rf_dma_end_to_guard"] = (
        guard if guard is not None else layout["svc_stack_bottom"]) - layout["rf_dma_end"]

    report = {"layout": layout, "indirect_mode": args.indirect,
              "excluded_functions": [pretty(name)[:args.width] for name in excluded],
              "indirect_sites": sum(len(f["indirect_calls"]) for f in functions.values()),
              "address_taken_functions": taken_count,
              "unresolved_indirect": {pretty(n)[:args.width]: c for n, c in unresolved.items()}, "roots": {}, "large_frames": [], "focus": [], "recursion": [],
              "indirect_branches": []}
    failures = layout_failures(layout, args.declared_svc, args.declared_irq, args.min_gap)
    frame_allow = parse_frame_allow(args.frame_allow)
    for label, root, budget in (
        ("svc", args.svc_root, layout["declared_svc_stack_bytes"]),
        ("irq", args.irq_root, layout["declared_irq_stack_bytes"]),
    ):
        if root not in functions:
            raise SystemExit(f"root function {root} not found")
        prefix, parent = graph.analyze(root)
        worst = graph.subtree[root]
        report["roots"][label] = {
            "root": root,
            "worst_bytes": worst,
            "declared_budget": budget,
            "path": describe_path(graph, graph.worst_path(root), 0, pretty, args.width),
            "reachable_functions": len(prefix),
        }
        if worst > budget:
            failures.append(f"{label} worst static path {worst} B > declared {budget} B")
        if label == "svc":
            svc_prefix, svc_parent = prefix, parent

    for name, function in sorted(functions.items(), key=lambda item: -item[1]["frame"]):
        if function["frame"] < args.frame_threshold:
            break
        reachable = name in svc_prefix
        entry = {
            "function": pretty(name)[: args.width],
            "frame": function["frame"],
            "push": function["push"],
            "alloc": function["alloc"],
            "align": function["align"],
            "reachable_from_svc_root": reachable,
            "worst_prefix": svc_prefix.get(name),
            "worst_subtree": graph.subtree.get(name),
            "largest_constant_copies": [
                {"bytes": length, "routine": routine}
                for length, routine in sorted(function["copies"], reverse=True)[:args.copies]
            ],
        }
        if reachable:
            entry["worst_through"] = svc_prefix[name] + graph.subtree[name]
        report["large_frames"].append(entry)
        if args.max_frame is not None:
            limit = frame_limit(pretty(name), args.max_frame, frame_allow)
            if limit != args.max_frame:
                entry["frame_exception_cap"] = limit
            if function["frame"] > limit:
                failures.append(f"frame {function['frame']} B > {limit} B in {pretty(name)[:120]}")

    focus = [tuple(item.split("=", 1)) for item in args.focus] or DEFAULT_FOCUS
    for label, pattern in focus:
        regex = re.compile(pattern)
        matches = [name for name in functions if regex.search(pretty(name)) and name in svc_prefix]
        matches.sort(key=lambda name: -(svc_prefix[name] + graph.subtree[name]))
        for name in matches[:3]:
            chain = chain_to(svc_parent, name)
            report["focus"].append({
                "label": label,
                "function": pretty(name)[: args.width],
                "frame": graph.frame(name),
                "worst_prefix": svc_prefix[name],
                "worst_subtree": graph.subtree[name],
                "worst_through": svc_prefix[name] + graph.subtree[name],
                "caller_chain": describe_path(graph, chain, 0, pretty, args.width),
                "callee_worst_path": describe_path(
                    graph, graph.worst_path(name)[1:], svc_prefix[name] + graph.frame(name),
                    pretty, args.width),
            })
        if not matches:
            report["focus"].append({"label": label, "function": None})

    report["chain_budgets"] = []
    names = {name: pretty(name) for name in functions}
    for item in args.chain_budget:
        result, failure = chain_budget(graph, svc_prefix, names, item, svc_parent)
        report["chain_budgets"].append(result)
        if failure:
            failures.append(failure)

    runtime_roots = []
    for pattern in args.runtime_root:
        regex = re.compile(pattern)
        found = [name for name in functions if regex.search(pretty(name))]
        if not found:
            failures.append(f"runtime root {pattern} not found")
        runtime_roots.extend(found)
    report["runtime_roots"] = sorted(pretty(name) for name in runtime_roots)
    report["frame_exceptions"] = frame_allow
    if args.max_frame is not None and (frame_allow or runtime_roots):
        runtime_reach = reachable_from(graph, runtime_roots) if runtime_roots else set()
        report["max_runtime_frame"] = max(
            ((graph.frame(name), pretty(name)) for name in runtime_reach), default=(0, None))
        failures.extend(bootstrap_only_failures(
            graph, names, frame_allow, args.max_frame, runtime_roots, args.irq_root))

    report["unreviewed_unresolved_indirect"] = unreviewed(
        [pretty(name) for name in unresolved], args.allow_unresolved)
    for name in report["unreviewed_unresolved_indirect"]:
        failures.append(f"unreviewed unresolved indirect call in {name[:160]}")

    report["recursion"] = sorted(
        f"{pretty(a)[:100]} -> {pretty(b)[:100]}" for a, b in graph.recursive_edges)
    report["indirect_branches"] = sorted(
        f"{pretty(name)[:100]} @0x{address}" for name, f in functions.items() for address in f["indirect"])

    print("== linker layout")
    for key, value in layout.items():
        print(f"  {key:40s} {value:#x} ({value})" if "bytes" not in key else f"  {key:40s} {value}")
    for label, root in report["roots"].items():
        print(f"== worst static {label.upper()} path from {root['root']}: {root['worst_bytes']} B "
              f"(declared {root['declared_budget']} B, {root['reachable_functions']} reachable functions)")
        for row in root["path"]:
            print(f"  {row['frame']:6d} {row['cumulative']:7d}  {row['address']:#07x}  {row['function']}")
    print(f"== frames >= {args.frame_threshold} B (prefix = worst caller depth from SVC root)")
    for entry in report["large_frames"]:
        through = entry.get("worst_through")
        print(f"  {entry['frame']:6d} prefix={entry['worst_prefix']} subtree={entry['worst_subtree']} "
              f"through={through}  {entry['function']}")
        if entry["largest_constant_copies"]:
            copies = ", ".join(f"{c['routine']}({c['bytes']})" for c in entry["largest_constant_copies"])
            print(f"         copies: {copies}")
    print("== focus functions (worst caller chain from SVC root)")
    for entry in report["focus"]:
        if entry["function"] is None:
            print(f"  [{entry['label']}] not present in the linked SVC call graph")
            continue
        print(f"  [{entry['label']}] frame={entry['frame']} prefix={entry['worst_prefix']} "
              f"through={entry['worst_through']}  {entry['function']}")
        for row in entry["caller_chain"]:
            print(f"      {row['frame']:6d} {row['cumulative']:7d}  {row['function']}")
        for row in entry["callee_worst_path"]:
            print(f"    > {row['frame']:6d} {row['cumulative']:7d}  {row['function']}")
    if report["runtime_roots"]:
        frame, name = report.get("max_runtime_frame", (0, None))
        print(f"== runtime roots: {', '.join(report['runtime_roots'])}")
        print(f"  largest frame reachable from runtime/IRQ roots: {frame} B  {name}")
    for name, cap in sorted(frame_allow.items()):
        print(f"== frame exception (bootstrap-only): {name} <= {cap} B")
    if report["chain_budgets"]:
        print("== chain budgets (worst SVC path through any matching function)")
        print(f"  {'chain':22s} {'bytes':>6s} {'budget':>6s} {'max frame':>9s}  entry -> terminal")
        for item in report["chain_budgets"]:
            if item["worst_through"] is None:
                print(f"  {item['label']:22s} {'-':>6s} {item['budget']:6d}  no match")
                continue
            print(f"  {item['label']:22s} {item['worst_through']:6d} {item['budget']:6d} "
                  f"{item['largest_frame']:9d}  {item['entry'][:90]} -> {item['terminal'][:60]}")
            print(f"  {'':22s} max-frame fn: {item['largest_frame_function'][:110]}")
    print(f"== recursive edges cut: {len(report['recursion'])}")
    for edge in report["recursion"]:
        print(f"  {edge}")
    print(f"== indirect call sites: {report['indirect_sites']} "
          f"(mode={args.indirect}, address-taken functions={taken_count}); "
          f"functions without a local vtable/pointer: {len(unresolved)}")
    for name, count in sorted(report["unresolved_indirect"].items()):
        print(f"  {count} site(s) unresolved in {name}")
    print(f"== non-jump-table indirect branches: {len(report['indirect_branches'])}")
    for site in report["indirect_branches"]:
        print(f"  {site}")
    print("== caveat: heuristic direct-call bound; HIL stack paint is authoritative")

    if args.json:
        Path(args.json).write_text(json.dumps(report, indent=2) + "\n")
    if args.check:
        for failure in failures:
            print(f"stack-check FAIL: {failure}", file=sys.stderr)
        if failures:
            return 1
        print("stack-check OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
