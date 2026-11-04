"""Self-tests for tools/bl702-stack-gate.py (synthetic RISC-V listings)."""

import importlib.util
from pathlib import Path
import sys
import unittest

SCRIPT = Path(__file__).resolve().parent / "bl702-stack-gate.py"
SPEC = importlib.util.spec_from_file_location("bl702_stack_gate", SCRIPT)
GATE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = GATE
SPEC.loader.exec_module(GATE)

GOOD_SYMBOLS = {
    "__ebss": 0x4201E6C0,
    "__euninit": 0x4201E6C0,
    "__eheap": 0x4201E6C0,
    "_stack_guard_start": 0x42025FC0,
    "_stack_guard_end": 0x42026000,
    "__estack": 0x42026000,
    "__sstack": 0x4202C000,
}


def function(address: int, name: str, frame: int, *calls: tuple[int, str]) -> list[str]:
    lines = [f"{address:08x} <{name}>:"]
    pc = address
    if frame:
        lines.append(f"{pc:08x}:      \taddi\tsp, sp, -0x{frame:x}")
        pc += 2
    for target, callee in calls:
        lines.append(f"{pc:08x}:      \tjal\t0x{target:x} <{callee}>")
        pc += 4
    lines.append(f"{pc:08x}:      \tret")
    return lines


def listing(task_frame: int = 0x400, compose_frame: int = 0x200) -> str:
    """main -> init; executor reaches the poll and the trap reaches the
    dispatcher through pointers, so they are separate roots."""
    lines: list[str] = []
    lines += function(0x23000000, "main", 0x10, (0x23000100, "hal::init"))
    lines += function(0x23000100, "hal::init", 0x20)
    lines += function(0x23000200, "<embassy_executor::raw::TaskStorage<app::run>>::poll",
                      task_frame, (0x23000300, "app::compose"), (0x23000400, "app::scan"))
    lines += function(0x23000300, "app::compose", compose_frame)
    lines += function(0x23000400, "app::scan", 0x100, (0x23000500, "app::leaf"))
    lines += function(0x23000500, "app::leaf", 0x40)
    lines += function(0x23000600, "_bl702_clic_trap", 0x40)
    lines += function(0x23000700, "app::time_driver::dispatch", 0x10,
                      (0x23000800, "app::radio::service"))
    lines += function(0x23000800, "app::radio::service", 0x30)
    return "\n".join(lines)


def arguments(*extra: str) -> list[str]:
    return ["dummy.elf", "--declared", "24576", "--guard", "64",
            "--stack-ceiling", "0x4202C000", "--min-gap", "4096", "--max-frame", "0x800",
            "--entry", "^main$", "--task", r"TaskStorage<.*>>::poll$",
            "--irq-entry", "^_bl702_clic_trap$", "--irq-handler", "::time_driver::dispatch$",
            *extra]


def run(symbols=GOOD_SYMBOLS, disasm=None, *extra: str):
    args = GATE.parser().parse_args(arguments(*extra))
    return GATE.run(args, dict(symbols), disasm if disasm is not None else listing())


class Bl702StackGateTests(unittest.TestCase):
    def test_good_image_passes_and_sums_task_and_interrupt_roots(self):
        report = run()
        self.assertEqual(report.failures, [])
        # entry 0x10+0x20; task 0x400 + max(compose 0x200, scan 0x100+0x40);
        # trap 0x40; handler 0x10+0x30.
        total = 0x30 + 0x600 + 0x40 + 0x40
        self.assertIn(f"= {total} B of 24576 B", report.lines[1])

    def test_guard_must_sit_immediately_below_the_stack(self):
        symbols = dict(GOOD_SYMBOLS, _stack_guard_start=0x42025F00, _stack_guard_end=0x42025F40)
        self.assertTrue(any("guard" in f for f in run(symbols).failures))

    def test_bss_overlapping_the_guard_fails(self):
        symbols = dict(GOOD_SYMBOLS, __ebss=0x42025FC4)
        self.assertTrue(any(".bss" in f for f in run(symbols).failures))

    def test_wrong_stack_size_fails_and_budgets_the_linked_stack(self):
        symbols = dict(GOOD_SYMBOLS, __estack=0x42028400, _stack_guard_start=0x420283C0,
                       _stack_guard_end=0x42028400)
        report = run(symbols, listing(task_frame=0x3000))
        self.assertTrue(any("declared 24576" in f for f in report.failures))
        self.assertTrue(any(f.startswith("budget:") for f in report.failures))

    def test_missing_guard_symbols_fail(self):
        symbols = {k: v for k, v in GOOD_SYMBOLS.items() if "guard" not in k}
        self.assertTrue(any("missing symbols" in f for f in run(symbols).failures))

    def test_stack_above_the_ceiling_fails(self):
        symbols = dict(GOOD_SYMBOLS, __sstack=0x4202C400, __estack=0x42026400,
                       _stack_guard_start=0x420263C0, _stack_guard_end=0x42026400)
        self.assertTrue(any("above" in f for f in run(symbols).failures))

    def test_gap_below_minimum_fails(self):
        report = run(GOOD_SYMBOLS, listing(task_frame=0x4F00), "--max-frame", "0x8000")
        self.assertTrue(any(f.startswith("budget:") for f in report.failures))

    def test_frame_above_max_fails_unless_reviewed(self):
        disasm = listing(compose_frame=0x1000)
        self.assertTrue(any("app::compose" in f for f in run(GOOD_SYMBOLS, disasm).failures))
        report = run(GOOD_SYMBOLS, disasm, "--frame-allow", "app::compose=4096")
        self.assertEqual(report.failures, [])
        report = run(GOOD_SYMBOLS, disasm, "--frame-allow", "app::compose=4000")
        self.assertTrue(any("reviewed cap" in f for f in report.failures))

    def test_chain_budget_regression_fails(self):
        self.assertEqual(run(GOOD_SYMBOLS, None, "--chain-budget", "scan=app::scan$=320").failures,
                         [])
        report = run(GOOD_SYMBOLS, None, "--chain-budget", "scan=app::scan$=319")
        self.assertTrue(any(f.startswith("chain scan") for f in report.failures))

    def test_unmatched_root_or_chain_fails(self):
        report = run(GOOD_SYMBOLS, None, "--chain-budget", "gone=no_such_fn=1")
        self.assertTrue(any("no function matches chain gone" in f for f in report.failures))

    def test_args_file_comments_and_blank_lines_are_ignored(self):
        convert = GATE.parser().convert_arg_line_to_args
        self.assertEqual(convert("# comment"), [])
        self.assertEqual(convert("   "), [])
        self.assertEqual(convert("  --min-gap  "), ["--min-gap"])

    def test_checked_in_args_files_parse(self):
        for name in ("router", "ed", "sensor"):
            path = SCRIPT.parent / f"bl702-{name}-stack-gate.args"
            args = GATE.parser().parse_args(["dummy.elf", f"@{path}"])
            self.assertEqual(args.declared, 24576)
            self.assertEqual(args.guard, 64)
            self.assertGreaterEqual(args.min_gap, 3072)


if __name__ == "__main__":
    unittest.main()
