#!/usr/bin/env python3
"""Unit tests for the TLSR8258 static stack report and stack-paint HIL guard."""

import importlib.util
import struct
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


report = load("tlsr8258_stack_report", TOOLS / "tlsr8258-stack-report.py")
hil = load("tlsr8258_stack_hil", TOOLS / "telink-tlsr8258-lab" / "scripts" / "tlsr8258-stack-hil.py")


def insn(address, mnemonic, operands=""):
    return f"    {address:x}:      \t{mnemonic}" + (f"\t{operands}" if operands else "")


def listing(*functions):
    lines = []
    for address, name, body in functions:
        lines.append(f"{address:08x} <{name}>:")
        lines.extend(body)
        lines.append("")
    return lines


LITERALS = {0x3000: -0x2000, 0x3004: 0x0A10}


def read_word(address):
    return LITERALS.get(address)


class ParseDisassemblyTest(unittest.TestCase):
    def parse(self, *functions):
        return report.parse_disassembly(listing(*functions), read_word)

    def test_frame_counts_push_sub_and_negative_literal_add(self):
        functions = self.parse((0x1000, "root", [
            insn(0x1000, "tpush", "{r4-r7, lr}"),
            insn(0x1002, "tsub", "sp, #0x100"),
            insn(0x1004, "tloadr", "r4, [pc, #0x10]        @ 0x3000"),
            insn(0x1006, "tadd", "sp, r4"),
        ]))
        self.assertEqual(functions["root"]["push"], 20)
        self.assertEqual(functions["root"]["alloc"], 0x100 + 0x2000)
        self.assertEqual(functions["root"]["frame"], 20 + 0x100 + 0x2000)

    def test_direct_calls_and_constant_copy_lengths(self):
        functions = self.parse(
            (0x1000, "caller", [
                insn(0x1000, "tmov", "r2, #0x71"),
                insn(0x1002, "tjl", "0x2000 <memcpy>        @ imm = #0xffe"),
                insn(0x1004, "tmov", "r1, #0xa1"),
                insn(0x1006, "tshftl", "r2, r1, #0x4"),
                insn(0x1008, "tjl", "0x2000 <memcpy>        @ imm = #0xff8"),
                insn(0x100a, "tloadr", "r2, [pc, #0x10]        @ 0x3004"),
                insn(0x100c, "tjl", "0x2010 <memset>        @ imm = #0x1004"),
                insn(0x100e, "tjl", "0x2020 <callee>        @ imm = #0x1012"),
            ]),
            (0x2000, "memcpy", [insn(0x2000, "tjex", "lr")]),
            (0x2010, "memset", [insn(0x2010, "tjex", "lr")]),
            (0x2020, "callee", [insn(0x2020, "tjex", "lr")]),
        )
        caller = functions["caller"]
        self.assertEqual(caller["calls"], {"memcpy": "call", "memset": "call", "callee": "call"})
        self.assertEqual(sorted(caller["copies"]),
                         [(0x71, "memcpy"), (0xA10, "memcpy"), (0xA10, "memset")])

    def test_unknown_or_clobbered_length_is_not_reported(self):
        functions = self.parse((0x1000, "caller", [
            insn(0x1000, "tmov", "r2, #0x10"),
            insn(0x1002, "tloadr", "r2, [sp, #0x20]"),
            insn(0x1004, "tjl", "0x2000 <memcpy>        @ imm = #0xffa"),
            insn(0x1006, "tmov", "r2, #0x10"),
            insn(0x1008, "tjl", "0x2020 <other>        @ imm = #0x1016"),
            insn(0x100a, "tjl", "0x2000 <memcpy>        @ imm = #0xff4"),
        ]))
        self.assertEqual(functions["caller"]["copies"], [])

    def test_tjex_stub_call_is_an_indirect_site(self):
        functions = self.parse(
            (0x1000, "caller", [insn(0x1000, "tjl", "0x2000 <stub>        @ imm = #0xffe")]),
            (0x2000, "stub", [insn(0x2000, "tjex", "r3")]),
        )
        self.assertEqual(functions["caller"]["indirect_calls"], ["1000"])
        self.assertEqual(functions["caller"]["calls"], {})


def call_graph(frames, calls):
    return {name: {"frame": frame, "address": index, "calls": dict.fromkeys(calls.get(name, []), "call")}
            for index, (name, frame) in enumerate(frames.items())}


class GraphTest(unittest.TestCase):
    def functions(self, frames, calls):
        return call_graph(frames, calls)

    def test_worst_path_and_prefix(self):
        graph = report.Graph(self.functions(
            {"root": 100, "small": 10, "big": 1000, "leaf": 5},
            {"root": ["small", "big"], "small": ["leaf"], "big": ["leaf"]}))
        prefix, parent = graph.analyze("root")
        self.assertEqual(graph.subtree["root"], 1105)
        self.assertEqual(graph.worst_path("root"), ["root", "big", "leaf"])
        self.assertEqual(prefix["leaf"], 1100)
        self.assertEqual(parent["leaf"], "big")

    def test_recursion_is_cut_and_recorded(self):
        graph = report.Graph(self.functions({"a": 10, "b": 20}, {"a": ["b"], "b": ["a"]}))
        graph.analyze("a")
        self.assertEqual(graph.subtree["a"], 30)
        self.assertIn(("b", "a"), graph.recursive_edges)

    def test_compact_generics(self):
        self.assertEqual(report.compact_generics("<a::B<c::D<E>>>::f"), "<a::B<..>>::f")
        self.assertEqual(report.compact_generics("plain::name"), "plain::name")


class GateTest(unittest.TestCase):
    LAYOUT = {"declared_svc_stack_bytes": 20480, "declared_irq_stack_bytes": 1024,
              "physical_gap_bytes_rf_dma_end_to_guard": 7068}

    def test_layout_gates(self):
        self.assertEqual(report.layout_failures(self.LAYOUT, 20480, 1024, 4096), [])
        failures = report.layout_failures(self.LAYOUT, 16384, 1024, 8192)
        self.assertEqual(len(failures), 2)
        self.assertIn("SVC stack 20480 B != required 16384 B", failures[0])
        self.assertIn("gap 7068 B < 8192 B", failures[1])

    def test_frame_exception_is_exact_symbol_and_capped(self):
        allow = report.parse_frame_allow(["x::app::build_device=4672"])
        self.assertEqual(report.frame_limit("x::app::build_device", 4096, allow), 4672)
        # Exact name only: no suffix/prefix/regex matching.
        self.assertEqual(report.frame_limit("x::app::build_device::inner", 4096, allow), 4096)
        self.assertEqual(report.frame_limit("y::x::app::build_device", 4096, allow), 4096)
        self.assertEqual(report.frame_limit("x::app::build_devic.", 4096, allow), 4096)
        self.assertEqual(report.frame_limit("other", 4096, allow), 4096)
        with self.assertRaises(SystemExit):
            report.parse_frame_allow(["a=1", "a=2"])

    def bootstrap_graph(self, runtime_calls_boot=False, runtime_frame=100):
        calls = {"entry": ["boot", "rt"], "rt": ["step"], "boot": ["leaf"], "irq": []}
        if runtime_calls_boot:
            calls["step"] = ["boot"]
        frames = {"entry": 0, "boot": 4600, "leaf": 10, "rt": 768, "step": runtime_frame, "irq": 50}
        graph = report.Graph(call_graph(frames, calls))
        names = {name: name for name in frames}
        return graph, names

    def test_frame_exception_must_stay_bootstrap_only(self):
        allow = {"boot": 4672}
        graph, names = self.bootstrap_graph()
        self.assertEqual(report.bootstrap_only_failures(graph, names, allow, 4096, ["rt"], "irq"), [])
        graph, names = self.bootstrap_graph(runtime_calls_boot=True)
        failures = report.bootstrap_only_failures(graph, names, allow, 4096, ["rt"], "irq")
        self.assertEqual(failures, ["frame exception boot is reachable from a runtime/IRQ root"])
        graph, names = self.bootstrap_graph(runtime_frame=4100)
        failures = report.bootstrap_only_failures(graph, names, allow, 4096, ["rt"], "irq")
        self.assertEqual(failures, ["runtime frame 4100 B > 4096 B in step"])
        graph, names = self.bootstrap_graph()
        failures = report.bootstrap_only_failures(graph, names, {"gone": 4672}, 4096, ["rt"], "irq")
        self.assertEqual(failures, ["frame exception symbol gone not found in the ELF"])

    def test_chain_budget(self):
        graph = report.Graph(call_graph(
            {"root": 100, "store": 50, "leaf": 1000}, {"root": ["store"], "store": ["leaf"]}))
        prefix, parent = graph.analyze("root")
        names = {"root": "root", "store": "Journal<X>::store", "leaf": "leaf"}
        result, failure = report.chain_budget(graph, prefix, names, r"persist=Journal<.*>::store=1150", parent)
        self.assertIsNone(failure)
        self.assertEqual(result["worst_through"], 1150)
        self.assertEqual(result["path_bytes"], 1150)
        self.assertEqual(result["entry"], "Journal<X>::store")
        self.assertEqual(result["terminal"], "leaf")
        self.assertEqual(result["path"], ["root", "Journal<X>::store", "leaf"])
        self.assertEqual((result["largest_frame"], result["largest_frame_function"]), (1000, "leaf"))
        _, failure = report.chain_budget(graph, prefix, names, r"persist=Journal<.*>::store=1149")
        self.assertIn("worst through 1150 B > 1149 B", failure)
        _, failure = report.chain_budget(graph, prefix, names, r"gone=missing=1")
        self.assertIn("no reachable function", failure)

    def test_reset_vector_is_never_an_indirect_target(self):
        words = {0x4000: 0x1, 0x4004: 0x2001}
        functions = {
            "_reset_vector": {"address": 0x0, "literals": [], "indirect_calls": [], "calls": {}},
            "handler": {"address": 0x2000, "literals": [], "indirect_calls": [], "calls": {}},
            "caller": {"address": 0x1000, "literals": [0x1, 0x2001], "indirect_calls": ["1002"],
                       "calls": {}},
        }
        report.resolve_indirect(functions, words.get, "local")
        self.assertEqual(functions["caller"]["calls"], {"handler": "indirect"})

    def test_new_unresolved_indirect_site_is_reported(self):
        names = ["a::dispatch", "b::new_dyn_call"]
        self.assertEqual(report.unreviewed(names, [r"^a::dispatch$"]), ["b::new_dyn_call"])
        self.assertEqual(report.unreviewed(names, [r"^a::", r"^b::"]), [])


class HilGuardTest(unittest.TestCase):
    def test_spans_round_to_sectors(self):
        self.assertEqual(hil.we_span(0x70010, 0x10), (0x70000, 0x71000))
        self.assertEqual(hil.es_span(0x70000, 0x2001), (0x70000, 0x73000))

    def test_protected_ranges_are_refused(self):
        hil.check_span((0x70000, 0x76000))
        for span in ((0x75000, 0x77000), (0x76000, 0x77000), (0x7F000, 0x80000),
                     (0x0, 0x80000), (0x70000, 0x81000), (0x72000, 0x72000)):
            with self.assertRaises(hil.ProtectedRangeError, msg=f"{span}"):
                hil.check_span(span)

    def test_restore_beyond_product_nv_hits_guard(self):
        with self.assertRaises(hil.ProtectedRangeError):
            hil.check_span(hil.we_span(0x70000, 0x7000))


class HilAnalyzeTest(unittest.TestCase):
    LAYOUT = {"_rf_dma_start_": 0x846A64, "_rf_dma_end_": 0x846CA4,
              "_svc_stack_bottom": 0x84BC00, "_svc_stack_top": 0x84FC00,
              "_irq_stack_bottom": 0x84FC00, "_irq_stack_top": 0x850000,
              "_end_bss_": 0x846A64}
    SVC_BASE = 0x846A00
    IRQ_BASE = 0x84FC00

    def dumps(self):
        svc = bytearray(hil.PAINT * ((0x84FC00 - self.SVC_BASE) // 4))
        irq = bytearray(hil.PAINT * (0x400 // 4))
        return svc, irq

    def run_analyze(self, svc, irq):
        return hil.analyze(self.LAYOUT, bytes(svc), self.SVC_BASE, bytes(irq), self.IRQ_BASE)

    def test_stack_within_declared_region_is_safe(self):
        svc, irq = self.dumps()
        svc[0x84F000 - self.SVC_BASE:0x84FC00 - self.SVC_BASE] = bytes(0xC00)
        irq[0x300:0x400] = bytes(0x100)
        result = self.run_analyze(svc, irq)
        self.assertTrue(result["svc_stack_safe"])
        self.assertEqual(result["svc_high_water_bytes"], 0xC00)
        self.assertEqual(result["irq_high_water_bytes"], 0x100)
        self.assertTrue(result["irq_stack_safe"])
        self.assertFalse(result["rf_dma_reached_by_stack"])

    def test_crossing_declared_bottom_fails(self):
        svc, irq = self.dumps()
        svc[0x847000 - self.SVC_BASE:0x84FC00 - self.SVC_BASE] = bytes(0x84FC00 - 0x847000)
        result = self.run_analyze(svc, irq)
        self.assertFalse(result["svc_stack_safe"])
        self.assertEqual(result["svc_bytes_below_declared_bottom"], 0x84BC00 - 0x847000)

    def test_rf_dma_payload_is_not_stack_evidence(self):
        svc, irq = self.dumps()
        svc[0x846B00 - self.SVC_BASE:0x846B10 - self.SVC_BASE] = b"\x41\x88\x01\x00" * 4
        result = self.run_analyze(svc, irq)
        self.assertFalse(result["rf_dma_reached_by_stack"])
        self.assertTrue(result["svc_stack_safe"])

    def test_stack_pointer_word_inside_rf_dma_is_detected(self):
        svc, irq = self.dumps()
        svc[0x846CC4 - self.SVC_BASE:0x84FC00 - self.SVC_BASE] = bytes(0x84FC00 - 0x846CC4)
        struct.pack_into("<I", svc, 0x846ADC - self.SVC_BASE, 0x849E90)
        result = self.run_analyze(svc, irq)
        self.assertTrue(result["rf_dma_reached_by_stack"])
        self.assertTrue(result["svc_high_water_is_lower_bound"])
        self.assertEqual(result["svc_lowest_touched"], "0x846adc")
        self.assertEqual(result["svc_high_water_bytes"], 0x84FC00 - 0x846ADC)
        self.assertEqual(result["rf_dma_stack_pointer_words"], ["0x846adc=0x849e90"])
        self.assertFalse(result["svc_stack_safe"])

    def test_fully_dirty_irq_stack_is_unsafe(self):
        svc, irq = self.dumps()
        irq[:] = bytes(len(irq))
        result = self.run_analyze(svc, irq)
        self.assertFalse(result["irq_stack_safe"])


class HilGuardedLayoutTest(unittest.TestCase):
    LAYOUT = {"_rf_dma_start_": 0x848DE4, "_rf_dma_end_": 0x849024,
              "_svc_stack_guard_start": 0x84ABC0, "_svc_stack_guard_end": 0x84AC00,
              "_svc_stack_bottom": 0x84AC00, "_svc_stack_top": 0x84FC00,
              "_irq_stack_bottom": 0x84FC00, "_irq_stack_top": 0x850000,
              "_end_bss_": 0x848DE4}
    SVC_BASE = 0x848DE4

    def run_analyze(self, used, guard=None):
        svc = bytearray(hil.PAINT * ((0x84FC00 - self.SVC_BASE) // 4))
        for address in range(0x84ABC0, 0x84AC00, 4):
            struct.pack_into("<I", svc, address - self.SVC_BASE, hil.GUARD_WORD)
        svc[0x84FC00 - used - self.SVC_BASE:0x84FC00 - self.SVC_BASE] = bytes(used)
        for address, value in (guard or {}).items():
            struct.pack_into("<I", svc, address - self.SVC_BASE, value)
        irq = hil.PAINT * (0x400 // 4)
        return hil.analyze(self.LAYOUT, bytes(svc), self.SVC_BASE, irq, 0x84FC00)

    def test_armed_guard_is_not_stack_usage(self):
        result = self.run_analyze(0x2000)
        self.assertEqual(result["svc_high_water_bytes"], 0x2000)
        self.assertEqual(result["svc_headroom_bytes"], 20480 - 0x2000)
        self.assertTrue(result["guard_intact"])
        self.assertTrue(result["svc_runtime_headroom_ok"])
        self.assertTrue(result["svc_stack_safe"])

    def test_headroom_below_two_kib_is_flagged_but_inside_stack(self):
        result = self.run_analyze(20480 - 1024)
        self.assertFalse(result["svc_runtime_headroom_ok"])
        self.assertTrue(result["svc_stack_safe"])

    def test_damaged_guard_fails_and_counts_as_high_water(self):
        result = self.run_analyze(0x2000, {0x84ABF0: 0x0084AE00})
        self.assertFalse(result["guard_intact"])
        self.assertEqual(result["guard_damaged_words"], ["0x84abf0"])
        self.assertEqual(result["svc_lowest_touched"], "0x84abf0")
        self.assertTrue(result["svc_crossed_declared_bottom"])
        self.assertFalse(result["svc_stack_safe"])

    def test_write_below_guard_fails(self):
        result = self.run_analyze(0x2000, {0x84A000: 0})
        self.assertEqual(result["below_guard_words_written"], 1)
        self.assertFalse(result["svc_stack_safe"])

    def test_unarmed_guard_is_reported(self):
        result = self.run_analyze(0x2000, {a: hil.PAINT_WORD for a in range(0x84ABC0, 0x84AC00, 4)})
        self.assertFalse(result["guard_armed"])
        self.assertFalse(result["svc_stack_safe"])


if __name__ == "__main__":
    unittest.main()
