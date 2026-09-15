import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parent / "firmware-size-report.sh"


class FirmwareSizeReportTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.binary = self.root / "firmware with spaces.bin"
        self.output = self.root / "size report.json"

    def run_report(self, *args, env=None):
        return subprocess.run(
            ["sh", str(SCRIPT), *map(str, args)],
            capture_output=True, text=True, check=False, env=env,
        )

    def test_measures_without_an_artificial_limit_or_validation_claim(self):
        for size in (0, 225280, 430081, 1048577):
            with self.subTest(size=size):
                with self.binary.open("wb") as binary:
                    binary.truncate(size)
                result = self.run_report("sensor-default", self.binary, self.output)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(
                    json.loads(self.output.read_text()),
                    {"name": "sensor-default", "bytes": size},
                )
                self.assertEqual(result.stdout, f"sensor-default: {size} bytes\n")

    def test_missing_binary_fails_without_a_report(self):
        result = self.run_report("sensor", self.binary, self.output)
        self.assertEqual(result.returncode, 2)
        self.assertIn("firmware binary not found", result.stderr)
        self.assertFalse(self.output.exists())

    def test_invalid_names_are_rejected(self):
        self.binary.write_bytes(b"image")
        for name in ("", 'bad"name', "bad\nname"):
            with self.subTest(name=name):
                result = self.run_report(name, self.binary, self.output)
                self.assertEqual(result.returncode, 2)
                self.assertIn("invalid firmware name", result.stderr)
                self.assertFalse(self.output.exists())

    def test_legacy_budget_argument_is_not_silently_accepted(self):
        result = self.run_report("sensor", self.binary, "1", self.output)
        self.assertEqual(result.returncode, 2)
        self.assertIn("<name> <binary> <output-json>", result.stderr)
        self.assertFalse(self.output.exists())

    def test_output_errors_propagate(self):
        self.binary.write_bytes(b"image")
        result = self.run_report("sensor", self.binary, self.root / "absent" / "report.json")
        self.assertNotEqual(result.returncode, 0)
        self.assertTrue(result.stderr)

    def test_measurement_errors_propagate_without_a_report(self):
        self.binary.write_bytes(b"image")
        failing_wc = self.root / "wc"
        failing_wc.write_text("#!/bin/sh\necho 'injected read failure' >&2\nexit 1\n")
        failing_wc.chmod(0o755)
        result = self.run_report(
            "sensor", self.binary, self.output,
            env={**os.environ, "PATH": str(self.root) + os.pathsep + os.environ["PATH"]},
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("injected read failure", result.stderr)
        self.assertFalse(self.output.exists())


if __name__ == "__main__":
    unittest.main()
