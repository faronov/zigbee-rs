"""Host tests for tools/bl702_soak_monitor.py (standard library only)."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bl702_soak_monitor as soak  # noqa: E402

LOADER = "bl702-boot: v2"
APP = "bl702-boot: app v0.2.2+0 @ 0x2300e100"
CRC = "soak: crc loader=11111111 nv=22222222"
SOAK = [
    "soak: bl702-ota-soak 0.2.2 file 0x25000202 pad 212992 B (LAB ONLY)",
    CRC,
    "soak: confirm ok trial Some(Confirmed)",
    "soak: re-staging 237958 B",
    "soak: re-armed, resetting",
]
LOST, BACK = soak.PORT_LOST, soak.PORT_BACK


def full_cycle() -> list[str]:
    return [LOADER, "bl702-boot: installing update .....", "bl702-boot: update installed",
            "bl702-boot: trial boot 1", APP, *SOAK]


def run(lines: list[str], monitor: soak.Monitor | None = None):
    monitor = monitor or soak.Monitor()
    verdicts = []
    for number, line in enumerate(lines):
        verdict = monitor.feed(float(number), line + "\r\n")
        if verdict:
            verdicts.append(verdict)
    return monitor, verdicts


class MonitorTest(unittest.TestCase):
    def test_complete_cycles_are_not_cuts(self):
        monitor, verdicts = run(full_cycle() + full_cycle() + [LOADER])
        self.assertEqual(verdicts, ["cycle", "cycle"])
        self.assertEqual(monitor.cuts(), 0)
        self.assertEqual(monitor.failures, [])

    def test_nothing_counts_before_the_soak_runs(self):
        # Sensor boots, OTA resets and unplugging before the soak are ignored.
        monitor, verdicts = run([LOADER, APP, "[ota] activated", LOST, BACK, LOADER, APP])
        self.assertEqual(verdicts, [])
        self.assertEqual(monitor.cuts(), 0)

    def test_port_loss_is_the_cut_classified_by_phase(self):
        lines = [
            *full_cycle(),
            LOST, BACK, LOADER, "bl702-boot: installing update ...",      # cut-copy
            LOST, BACK, "......",                                         # cut-copy
            LOST, BACK, "bl702-boot: update installed",                   # cut-install
            LOST, BACK, LOADER,                                           # cut-loader
            LOST, BACK,                                                   # cut-boot
            LOST, BACK, APP, SOAK[0], CRC,                                # cut-app
            LOST, BACK, SOAK[0], CRC, SOAK[3],                            # cut-staging
            LOST, BACK, SOAK[0], CRC, *SOAK[3:],                          # cut-rearmed
            LOST, BACK, LOADER,
        ]
        monitor, verdicts = run(lines)
        self.assertEqual(verdicts, ["cut-rearmed", "cut-copy", "cut-copy", "cut-install",
                                    "cut-loader", "cut-boot", "cut-app", "cut-staging",
                                    "cut-rearmed"])
        self.assertEqual(monitor.cuts(), 9)
        self.assertEqual(monitor.counts["cycle"], 0)
        self.assertEqual(monitor.failures, [])

    def test_reset_without_a_cut_is_reported_not_counted(self):
        # Counting starts at the soak's first CRC line.
        monitor, verdicts = run(full_cycle()[:7] + [LOADER])
        self.assertEqual(verdicts, ["reset-app"])
        self.assertEqual(monitor.cuts(), 0)
        self.assertEqual(monitor.failures, [])

    def test_no_application_after_power_up_fails(self):
        monitor, _ = run(full_cycle() + [LOST, BACK, LOADER],
                         soak.Monitor(recover=10.0))
        monitor.check(1000.0)
        self.assertTrue(any("no soak application" in f for f in monitor.failures))
        recovered, _ = run(full_cycle() + [LOST, BACK, LOADER, APP, SOAK[0], CRC],
                           soak.Monitor(recover=10.0))
        recovered.check(1000.0)
        self.assertEqual(recovered.failures, [])

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

    def test_replay_reads_timestamped_logs(self):
        with tempfile.NamedTemporaryFile("w", suffix=".log", delete=False) as log:
            for number, line in enumerate(full_cycle() + [LOST, BACK, LOADER]):
                log.write(f"{1000 + number}.000 {line}\n")
        monitor = soak.replay(log.name)
        Path(log.name).unlink()
        self.assertEqual(monitor.counts["cut-rearmed"], 1)
        self.assertEqual(monitor.copy_seconds, [1.0])


if __name__ == "__main__":
    unittest.main()
