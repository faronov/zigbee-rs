"""Self-tests for the prologue stack-frame analysis in mac-footprint.py."""

import importlib.util
from pathlib import Path
import sys
import unittest


SCRIPT = Path(__file__).resolve().parent / "mac-footprint.py"
SPEC = importlib.util.spec_from_file_location("mac_footprint", SCRIPT)
TOOL = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = TOOL  # dataclasses look up their module
SPEC.loader.exec_module(TOOL)


def frame(disassembly: str) -> tuple[int, bool]:
    """Return (stack, dynamic) for a one-function disassembly."""
    result = TOOL.analyse_disassembly(disassembly)
    return result["stack"], bool(result["dyn"])


def riscv(*body: str) -> str:
    lines = ["42000000 <big>:"]
    lines += [f"42{index:06x}:      \t{insn}" for index, insn in enumerate(body, 1)]
    lines.append(f"42{len(body) + 1:06x}:      \tret")
    return "\n".join(lines)


class RiscvLargeFrameTests(unittest.TestCase):
    def test_small_frame_uses_the_addi_immediate(self):
        self.assertEqual(frame(riscv("addi\tsp, sp, -0x30", "sw\tra, 0x2c(sp)")),
                         (0x30, False))

    def test_lui_addi_sub_frame_is_counted(self):
        # The esp32c6-sensor main prologue: 0x100 + (0x2000 - 0x780).
        self.assertEqual(frame(riscv("addi\tsp, sp, -0x100", "sw\tra, 0xfc(sp)",
                                     "lui\ta0, 0x2", "addi\ta0, a0, -0x780",
                                     "sub\tsp, sp, a0")),
                         (0x100 + 0x2000 - 0x780, False))

    def test_li_sub_frame_is_counted(self):
        self.assertEqual(frame(riscv("li\ta1, 0x7f0", "sub\tsp, sp, a1")), (0x7f0, False))

    def test_negative_constant_added_to_sp_is_counted(self):
        self.assertEqual(frame(riscv("lui\tt0, 0xffffe", "add\tsp, sp, t0")), (0x2000, False))

    def test_overwritten_register_is_dynamic(self):
        self.assertEqual(frame(riscv("lui\ta0, 0x2", "lw\ta0, 0x0(a1)", "sub\tsp, sp, a0")),
                         (0, True))

    def test_unknown_register_is_dynamic(self):
        self.assertEqual(frame(riscv("sub\tsp, sp, a2")), (0, True))

    def test_stack_switch_stays_dynamic(self):
        # esp-hal's stack-guard trap moves sp to its own stack.
        self.assertTrue(frame(riscv("auipc\tsp, 0x6e", "addi\tsp, sp, 0x484",
                                    "li\tt0, 0x4", "sub\tsp, sp, t0"))[1])

    def test_constant_does_not_leak_into_the_next_function(self):
        disassembly = riscv("lui\ta0, 0x2") + "\n" + "\n".join([
            "42000100 <next>:",
            "42000102:      \tsub\tsp, sp, a0",
            "42000104:      \tret",
        ])
        self.assertTrue(TOOL.analyse_disassembly(disassembly)["dyn"])


class ArmFrameTests(unittest.TestCase):
    def test_thumb2_immediate_frame_is_unchanged(self):
        disassembly = "\n".join([
            "00001000 <f>:",
            "    1000:      \tpush\t{r4, r5, r6, lr}",
            "    1002:      \tsub\tsp, #0x40",
            "    1004:      \tpop\t{r4, r5, r6, pc}",
        ])
        self.assertEqual(frame(disassembly), (16 + 0x40, False))

    def test_thumb1_literal_pool_frame_is_unchanged(self):
        disassembly = "\n".join([
            "00001000 <f>:",
            "    1000:      \tpush\t{r4, lr}",
            "    1002:      \tldr\tr3, [pc, #0x4]  @ 0x1008",
            "    1004:      \tadd\tsp, r3",
            "    1006:      \tpop\t{r4, pc}",
            "    1008: 44 d0 ff ff  \t.word\t0xffffd044",
        ])
        self.assertEqual(frame(disassembly), (8 + 0x2FBC, False))

    def test_thumb2_register_sub_stays_dynamic(self):
        disassembly = "\n".join([
            "00001000 <f>:",
            "    1000:      \tpush\t{r7, lr}",
            "    1002:      \tsub.w\tsp, sp, r0",
            "    1006:      \tpop\t{r7, pc}",
        ])
        self.assertEqual(frame(disassembly), (8, True))


if __name__ == "__main__":
    unittest.main()
