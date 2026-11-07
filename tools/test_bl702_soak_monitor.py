"""Host tests for tools/bl702_soak_monitor.py (standard library only)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bl702_soak_monitor as soak  # noqa: E402

LOADER = "bl702-boot: v2"
APP = "bl702-boot: app v0.2.2+0 @ 0x2300e100"
SOAK = [
    "soak: bl702-ota-soak 0.2.2 file 0x25000202 pad 212992 B (LAB ONLY)",
    "soak: crc loader=11111111 nv=22222222",
    "soak: confirm ok trial Some(Confirmed)",
    "soak: re-staging 237958 B",
    "soak: re-armed, resetting",
]


def run(lines: list[str]) -> tuple[soak.Monitor, list[str]]:
    monitor = soak.Monitor()
    verdicts = []
    for number, line in enumerate(lines):
        verdict = monitor.feed(float(number), line + "\r\n")
        if verdict:
            verdicts.append(verdict)
    return monitor, verdicts


def full_cycle() -> list[str]:
    return [LOADER, "bl702-boot: installing update .....", "bl702-boot: update installed",
            "bl702-boot: trial boot 1", APP, *SOAK]


class MonitorTest(unittest.TestCase):
    def test_complete_cycles_are_not_cuts(self):
        monitor, verdicts = run(full_cycle() + full_cycle() + [LOADER])
        self.assertEqual(verdicts, ["cycle", "cycle"])
        self.assertEqual(monitor.cuts(), 0)
        self.assertEqual(monitor.failures, [])

    def test_cut_classification(self):
        lines = [
            LOADER, "bl702-boot: installing update ...",          # cut-copy
            LOADER, "bl702-boot: installing update ......",
            "bl702-boot: update installed",                        # cut-install
            LOADER,                                                # cut-loader
            LOADER, APP, SOAK[0],                                  # cut-app
            LOADER, APP, *SOAK[:4],                                # cut-staging
            LOADER,
        ]
        monitor, verdicts = run(lines)
        self.assertEqual(
            verdicts, ["cut-copy", "cut-install", "cut-loader", "cut-app", "cut-staging"])
        self.assertEqual(monitor.cuts(), 5)
        self.assertEqual(monitor.failures, [])

    def test_partial_dots_line_closed_by_banner(self):
        # The banner's leading CR LF ends the unterminated progress line.
        monitor, verdicts = run(["bl702-boot: v2", "bl702-boot: installing update ..",
                                 "\x00\x00bl702-boot: v2"])
        self.assertEqual(verdicts, ["cut-copy"])

    def test_crc_change_fails(self):
        changed = [line.replace("nv=22222222", "nv=33333333") for line in full_cycle()]
        monitor, _ = run(full_cycle() + changed)
        self.assertTrue(any("CRC changed" in failure for failure in monitor.failures))

    def test_loader_errors_and_watchdog_fail(self):
        for line in ("bl702-boot: no bootable image: hash mismatch",
                     "bl702-boot: watchdog reset", "soak: FAIL verify staged image",
                     "bl702-boot: update rejected: downgrade"):
            monitor, _ = run([LOADER, line])
            self.assertTrue(monitor.failures, line)


if __name__ == "__main__":
    unittest.main()
