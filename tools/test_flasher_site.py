import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parent.parent
SPEC = importlib.util.spec_from_file_location("flasher_site", ROOT / "tools/flasher_site.py")
SITE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SITE)

PARTS = {
    "bootloader": (0x0, b"\xe9boot"),
    "partition_table": (0x8000, b"\xaa\x50table"),
    "otadata": (0x9000, b"\xff" * 16),
    "app": (0x10000, b"\xe9application"),
}


def board_manifest(tag, version, files):
    return {
        "schema": 1, "product": "zrhp-ncp", "name": "zigbee-rs ZRHP NCP", "tag": tag,
        "version": version, "commit": "c" * 40, "build_id": "c" * 16,
        "boards": [{
            "id": "esp32h2-devkit-ncp", "title": "ESP32-H2 DevKit", "chip_family": "ESP32-H2",
            "flash_size": "4MB", "layout": "esp32h2-ncp-4mb", "app_version": version,
            "files": {role: {"name": f"{tag}-{role}.bin", "offset": off, "size": len(data),
                             "sha256": hashlib.sha256(data).hexdigest()}
                      for role, (off, data) in files.items()},
            "flash": list(files),
            "fw_update": None,
        }],
    }


class FakeGitHub:
    """Release listing and asset downloads of one repository."""

    def __init__(self, releases):
        self.releases = releases  # tag -> (manifest dict, {asset name: bytes}, prerelease, published)

    def get(self, url, *, api=False, attempts=4):
        if url.startswith(SITE.API):
            return json.dumps([
                {"tag_name": tag, "draft": False, "prerelease": pre, "published_at": published,
                 "html_url": f"https://github.com/o/r/releases/tag/{tag}",
                 "assets": [{"name": name, "browser_download_url": f"https://dl/{tag}/{name}"}
                            for name in assets]}
                for tag, (_, assets, pre, published) in self.releases.items()
            ]).encode()
        tag, name = url.removeprefix("https://dl/").split("/", 1)
        return self.releases[tag][1][name]


def release(tag, version, *, published, prerelease=False, corrupt=None, manifest=True):
    files = dict(PARTS)
    m = board_manifest(tag, version, files)
    assets = {f"{tag}-{role}.bin": data for role, (_, data) in files.items()}
    if corrupt:
        assets[f"{tag}-{corrupt}.bin"] = b"tampered"
    if manifest:
        assets["release-manifest.json"] = json.dumps(m).encode()
    return tag, (m, assets, prerelease, published)


class FlasherSiteTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def fetch(self, releases, keep=5):
        fake = FakeGitHub(dict(releases))
        out = self.root / f"out-{len(list(self.root.iterdir()))}"
        args = mock.Mock(repo=["o/r:ncp-v"], keep=keep, out=str(out))
        with mock.patch.object(SITE, "http_get", fake.get):
            SITE.cmd_fetch(args)
        return out, json.loads((out / "releases.json").read_text())

    def test_update_keeps_and_erase_erases(self):
        out, index = self.fetch([release("ncp-v0.1.0", "0.1.0", published="2026-11-21T10:00:00Z")])
        board = index["products"][0]["releases"][0]["boards"][0]
        update = json.loads((out / board["manifests"]["update"]).read_text())
        erase = json.loads((out / board["manifests"]["erase"]).read_text())
        # Update: ESP Web Tools asks before erasing (unticked); erase: a name no
        # running firmware reports, and no prompt, so it always erases.
        self.assertTrue(update["new_install_prompt_erase"])
        self.assertFalse(erase["new_install_prompt_erase"])
        self.assertNotEqual(update["name"], erase["name"])
        self.assertEqual(update["new_install_improv_wait_time"], 0)
        parts = update["builds"][0]["parts"]
        self.assertEqual([p["offset"] for p in parts], [0x0, 0x8000, 0x9000, 0x10000])
        self.assertEqual(update["builds"], erase["builds"])
        for part in parts:
            self.assertTrue((out / "releases/ncp-v0.1.0" / part["path"]).is_file())
        SITE.cmd_check(mock.Mock(dir=str(out)))

    def test_keeps_the_newest_releases_and_skips_releases_without_manifest(self):
        out, index = self.fetch([
            release("ncp-v0.1.0", "0.1.0", published="2026-11-01T00:00:00Z"),
            release("ncp-v0.2.0-rc.1", "0.2.0-rc.1", published="2026-11-03T00:00:00Z", prerelease=True),
            release("ncp-v0.1.1", "0.1.1", published="2026-11-02T00:00:00Z"),
            release("ncp-v0.0.9", "0.0.9", published="2026-10-01T00:00:00Z", manifest=False),
        ], keep=3)
        tags = [r["tag"] for r in index["products"][0]["releases"]]
        self.assertEqual(tags, ["ncp-v0.2.0-rc.1", "ncp-v0.1.1", "ncp-v0.1.0"])
        self.assertTrue(index["products"][0]["releases"][0]["prerelease"])
        self.assertFalse((out / "releases/ncp-v0.0.9").exists())

    def test_tampered_asset_is_refused(self):
        with self.assertRaisesRegex(SITE.SiteError, "does not match"):
            self.fetch([release("ncp-v0.1.0", "0.1.0", published="2026-11-21T10:00:00Z", corrupt="app")])

    def test_manifest_validation(self):
        good = board_manifest("ncp-v0.1.0", "0.1.0", dict(PARTS))
        SITE.validate_manifest(good, "ncp-v0.1.0")
        cases = {
            "is for": ("tag", "ncp-v9.9.9"),
            "bad file entry": ("name", "../../index.html"),
            "overlaps": ("overlap", None),
            "4 KiB aligned": ("offset", 0x8001),
            "chip family": ("chip", "ESP8266-X"),
        }
        for message, (what, value) in cases.items():
            with self.subTest(message=message):
                m = json.loads(json.dumps(good))
                board = m["boards"][0]
                if what == "tag":
                    m["tag"] = value
                elif what == "name":
                    board["files"]["app"]["name"] = value
                elif what == "overlap":
                    board["files"]["app"]["offset"] = 0x9000
                elif what == "offset":
                    board["files"]["partition_table"]["offset"] = value
                elif what == "chip":
                    board["chip_family"] = value
                with self.assertRaisesRegex(SITE.SiteError, message):
                    SITE.validate_manifest(m, "ncp-v0.1.0")

    def test_fingerprint_ignores_order_and_tracks_content(self):
        a = [("o/r", "ncp-v0.1.0", "1" * 64), ("o/h", "hub-v0.1.0", "2" * 64)]
        self.assertEqual(SITE.fingerprint(a), SITE.fingerprint(list(reversed(a))))
        self.assertNotEqual(SITE.fingerprint(a), SITE.fingerprint([a[0], ("o/h", "hub-v0.1.0", "3" * 64)]))


if __name__ == "__main__":
    unittest.main()
