import importlib.util
import itertools
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parent.parent
SPEC = importlib.util.spec_from_file_location("prepare_pages", ROOT / "tools/prepare-pages.py")
PAGES = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PAGES)
REVISION = "a" * 40


class PagesTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.book = self.root / "book"
        self.book.mkdir()
        (self.book / "index.html").write_text("documentation")
        self.firmware = self.root / "firmware"
        self.firmware.mkdir()
        for name in ("esp32c6-sensor.bin", "esp32h2-sensor.bin"):
            (self.firmware / name).write_bytes(b"artifact")
        self.template = ROOT / "docs/flasher/index.html"

    def prepare(self, output, results):
        PAGES.prepare(
            self.book, self.template, self.firmware, output, REVISION, 123, results,
        )

    def test_all_result_combinations_fail_closed(self):
        for index, values in enumerate(itertools.product(PAGES.RESULTS, repeat=3)):
            results = dict(zip(("esp32c6", "esp32h2", "bl702"), values))
            with self.subTest(results=results):
                output = self.root / f"site-{index}"
                self.prepare(output, results)
                flasher = output / "flasher"
                available = all(value == "success" for value in values)
                manifest = json.loads((flasher / "manifest.json").read_text())
                metadata = json.loads((flasher / "status.json").read_text())
                html = (flasher / "index.html").read_text()
                self.assertEqual((output / "index.html").read_text(), "documentation")
                self.assertEqual(metadata["firmware_available"], available)
                self.assertEqual(metadata["firmware_jobs"], results)
                self.assertEqual(manifest["version"], REVISION)
                self.assertEqual(len(manifest["builds"]), 2 if available else 0)
                self.assertEqual((flasher / "firmware").exists(), available)
                self.assertEqual('<button slot="activate">' in html, available)
                self.assertEqual("<button disabled>" in html, not available)
                if available:
                    self.assertEqual(
                        {item["chipFamily"] for item in manifest["builds"]},
                        {"ESP32-C6", "ESP32-H2"},
                    )
                    for item in manifest["builds"]:
                        part = item["parts"][0]
                        self.assertEqual(part["offset"], 0)
                        self.assertEqual((flasher / part["path"]).read_bytes(), b"artifact")

    def test_qualified_publication_requires_both_artifacts(self):
        for contents in (None, b""):
            with self.subTest(contents=contents):
                missing = self.firmware / "esp32h2-sensor.bin"
                missing.unlink(missing_ok=True)
                if contents is not None:
                    missing.write_bytes(contents)
                output = self.root / "missing-site"
                with self.assertRaisesRegex(ValueError, "missing or empty"):
                    self.prepare(output, dict.fromkeys(("esp32c6", "esp32h2", "bl702"), "success"))
                self.assertFalse(output.exists())

    def test_existing_destination_is_never_reused(self):
        output = self.root / "old-site"
        output.mkdir()
        (output / "stale.bin").write_bytes(b"old")
        with self.assertRaisesRegex(ValueError, "refusing to reuse"):
            self.prepare(output, dict.fromkeys(("esp32c6", "esp32h2", "bl702"), "failure"))
        self.assertEqual((output / "stale.bin").read_bytes(), b"old")

    def test_missing_or_unknown_results_are_rejected(self):
        for results in ({}, {"esp32c6": "success", "esp32h2": "success", "bl702": "unknown"}):
            with self.subTest(results=results), self.assertRaises(ValueError):
                self.prepare(self.root / "invalid-site", results)

    def test_template_slots_are_required(self):
        with self.assertRaisesRegex(ValueError, "template slot"):
            PAGES.replace_slot("no marker", "PREBUILT_INSTALL", "button")


if __name__ == "__main__":
    unittest.main()
