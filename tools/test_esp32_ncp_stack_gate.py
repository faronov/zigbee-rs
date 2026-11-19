"""Self-tests for tools/esp32-ncp-stack-gate.py (synthetic RISC-V listings)."""

import importlib.util
from pathlib import Path
import sys
import unittest

SCRIPT = Path(__file__).resolve().parent / "esp32-ncp-stack-gate.py"
SPEC = importlib.util.spec_from_file_location("esp32_ncp_stack_gate", SCRIPT)
GATE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = GATE
SPEC.loader.exec_module(GATE)

# 0x8000 B of stack above .bss.
GOOD_SYMBOLS = {
    "_bss_end": 0x40828000,
    "_stack_end_cpu0": 0x40828000,
    "_stack_start_cpu0": 0x40830000,
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


def listing(compose_frame: int = 0x200, handler_frame: int = 0x30) -> str:
    """main -> compose / serve -> executor; the logger and the handlers are
    reached through pointers, so they are separate roots."""
    lines: list[str] = []
    lines += function(0x42000000, "main", 0x10, (0x42000100, "app::compose"),
                      (0x42000200, "app::serve"))
    lines += function(0x42000100, "app::compose", compose_frame, (0x42000300, "nwk::write_into"))
    lines += function(0x42000200, "app::serve", 0x100, (0x42000300, "nwk::write_into"))
    lines += function(0x42000300, "nwk::write_into", 0x400)
    lines += function(0x42000400, "<app::RingLogger as log::Log>::log", 0x80)
    lines += function(0x40800600, "handle_interrupts", 0x50)
    lines += function(0x42000700, "radio::__esp_hal_internal_zb_mac_handler", handler_frame,
                      (0x42000800, "radio::service"))
    lines += function(0x42000800, "radio::service", 0x20)
    lines += function(0x42000900, "time::__esp_hal_internal_on_alarm", 0x10)
    lines += function(0x42000a00, "zigbee_mac::esp::radio_phy::on_tx_done", 0x40)
    return "\n".join(lines)


def arguments(*extra: str) -> list[str]:
    return ["dummy.elf", "--guard", "64", "--min-gap", "4096", "--max-frame", "0x800",
            "--entry", "^main$", "--task-extra", r"RingLogger as log::Log>::log$",
            "--trap-frame", "64", "--irq-dispatch", "^handle_interrupts$",
            "--irq-handler", "::__esp_hal_internal_",
            "--irq-callback", "^zigbee_mac::esp::radio_phy::on_", *extra]


def run(symbols=GOOD_SYMBOLS, disasm=None, *extra: str):
    args = GATE.parser().parse_args(arguments(*extra))
    return GATE.run(args, dict(symbols), disasm if disasm is not None else listing())


class Esp32NcpStackGateTests(unittest.TestCase):
    def test_good_image_passes_and_sums_nested_interrupts(self):
        report = run()
        self.assertEqual(report.failures, [])
        task = 0x10 + 0x200 + 0x400
        # Each handler nests once: trap 64 + dispatch 0x50 + its depth, plus
        # the worst pointer-reached callback.
        irq = (64 + 0x50 + 0x30 + 0x20) + (64 + 0x50 + 0x10) + 0x40
        self.assertIn(f"task {task} + task extra {0x80} + irq {irq} = "
                      f"{task + 0x80 + irq} B of {0x8000 - 64} B", report.lines[1])

    def test_bss_overlapping_the_stack_fails(self):
        symbols = dict(GOOD_SYMBOLS, _bss_end=0x40828010)
        self.assertTrue(any(".bss" in f for f in run(symbols).failures))

    def test_noinit_overlapping_the_stack_fails(self):
        symbols = dict(GOOD_SYMBOLS, _noinit_end=0x40828004)
        self.assertTrue(any(".noinit" in f for f in run(symbols).failures))

    def test_missing_layout_symbols_fail(self):
        symbols = {k: v for k, v in GOOD_SYMBOLS.items() if k != "_stack_end_cpu0"}
        self.assertTrue(any("missing symbols" in f for f in run(symbols).failures))

    def test_gap_below_minimum_fails(self):
        report = run(GOOD_SYMBOLS, listing(compose_frame=0x7000), "--max-frame", "0x8000")
        self.assertTrue(any(f.startswith("budget:") for f in report.failures))

    def test_a_deeper_handler_counts_against_the_budget(self):
        small = run().lines[1]
        deep = run(GOOD_SYMBOLS, listing(handler_frame=0x330)).lines[1]
        self.assertNotEqual(small, deep)
        self.assertIn(f"irq {(64 + 0x50 + 0x330 + 0x20) + (64 + 0x50 + 0x10) + 0x40}", deep)

    def test_frame_above_max_fails_unless_reviewed(self):
        disasm = listing(compose_frame=0x1000)
        self.assertTrue(any("app::compose" in f for f in run(GOOD_SYMBOLS, disasm).failures))
        report = run(GOOD_SYMBOLS, disasm, "--frame-allow", "app::compose=4096")
        self.assertEqual(report.failures, [])
        report = run(GOOD_SYMBOLS, disasm, "--frame-allow", "app::compose=4000")
        self.assertTrue(any("reviewed cap" in f for f in report.failures))

    def test_unbounded_frame_fails_unless_allowed(self):
        lines = listing().splitlines()
        at = lines.index("42000100 <app::compose>:")
        lines[at + 2:at + 2] = ["42000102:      \tsub\tsp, sp, a2"]
        disasm = "\n".join(lines)
        report = run(GOOD_SYMBOLS, disasm)
        self.assertTrue(any("cannot bound" in f and "app::compose" in f
                            for f in report.failures))
        report = run(GOOD_SYMBOLS, disasm, "--dynamic-allow", "^app::compose$")
        self.assertEqual(report.failures, [])

    def test_chain_budget_regression_fails(self):
        self.assertEqual(
            run(GOOD_SYMBOLS, None, "--chain-budget", "boot=^app::compose$=0x600").failures, [])
        report = run(GOOD_SYMBOLS, None, "--chain-budget", "boot=^app::compose$=0x5ff")
        self.assertTrue(any(f.startswith("chain boot") for f in report.failures))

    def test_unmatched_root_fails(self):
        report = run(GOOD_SYMBOLS, None, "--chain-budget", "gone=no_such_fn=1")
        self.assertTrue(any("no function matches chain gone" in f for f in report.failures))
        args = arguments()
        args[args.index("^main$")] = "^no_main$"
        report = GATE.run(GATE.parser().parse_args(args), dict(GOOD_SYMBOLS), listing())
        self.assertTrue(any("--entry" in f for f in report.failures))

    def test_checked_in_args_file_parses(self):
        path = SCRIPT.parent / "esp32-ncp-stack-gate.args"
        args = GATE.parser().parse_args(["dummy.elf", f"@{path}"])
        self.assertEqual(args.guard, 64)
        self.assertGreaterEqual(args.min_gap, 16384)
        self.assertTrue(args.chain_budget)


if __name__ == "__main__":
    unittest.main()
