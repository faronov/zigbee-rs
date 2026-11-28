"""Release tooling of the ZRHP NCP (tools/ncp-release/esp_release.py):
version sources, Gecko bootloader images and the release manifest."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest import mock
import zlib


ROOT = Path(__file__).resolve().parent.parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REL = load("esp_release", "tools/ncp-release/esp_release.py")
SITE = load("flasher_site", "tools/flasher_site.py")

APP_START = 0x0800_6000
NV_WINDOW = (0x0815_8000, 0x0818_0000)


def record(tag, body):
    return struct.pack("<II", tag, len(body)) + body


def gbl(*records, version=REL.GBL_VERSION, gbl_type=0, pad=True):
    """A plain GBL v3 file: header, `records`, END with its CRC, padded to a
    multiple of four bytes as tools/efr32-gbl and Commander do."""
    data = record(REL.GBL_TAG_HEADER, struct.pack("<II", version, gbl_type)) + b"".join(records)
    data += struct.pack("<II", REL.GBL_TAG_END, 4)
    data += struct.pack("<I", zlib.crc32(data))
    if pad:
        data += b"\xff" * (-len(data) % 4)
    return data


def application(version_word=0x0001_0300):
    # ApplicationData_t: type, version, capabilities, productId[16].
    return record(REL.GBL_TAG_APPLICATION, struct.pack("<III", 1 << 6, version_word, 0) + bytes(16))


def zrhp(fw="0.1.3", build_id=bytes(range(8))):
    text = fw.encode()
    return record(REL.GBL_TAG_METADATA, b"ZRHP" + bytes([1]) + build_id + bytes([len(text)]) + text)


def prog(start, data, tag=REL.GBL_TAG_ERASEPROG):
    return record(tag, struct.pack("<I", start) + data)


def app_gbl(fw="0.1.3", version_word=0x0001_0300, start=APP_START):
    return gbl(application(version_word), zrhp(fw), prog(start, b"\x00\x10\x00\x20" * 16))


def clear_gbl():
    return gbl(prog(NV_WINDOW[0], b"\xff" * (NV_WINDOW[1] - NV_WINDOW[0])))


def gecko_args(path, out, kind, expect="0.1.3", ident=None):
    return argparse.Namespace(
        gbl=str(path), id=ident or f"efr32mg24-{kind}", title=f"MG24 {kind}", kind=kind,
        chip_family="EFR32MG24", install="Gecko bootloader (XMODEM)", out=str(out),
        expect_version=expect)


class GblTest(unittest.TestCase):
    def test_application_gbl(self):
        data = app_gbl()
        unpadded = gbl(application(), zrhp(), prog(APP_START, b"\x00\x10\x00\x20" * 16), pad=False)
        # The ZRHP record's length makes the file need padding.
        self.assertNotEqual(len(unpadded) % 4, 0)
        self.assertEqual(data, unpadded + b"\xff" * (len(data) - len(unpadded)))
        self.assertEqual(len(data) % 4, 0)
        info = REL.read_gbl(data)
        self.assertEqual(info["application"]["version"], 0x0001_0300)
        self.assertEqual(info["zrhp"], {"build_id": "0001020304050607", "fw_version": "0.1.3"})
        self.assertEqual(info["programs"], [(APP_START, APP_START + 64)])
        self.assertEqual(REL.gecko_version(0x0001_0300), "0.1.3")

    def test_maintenance_gbl(self):
        info = REL.read_gbl(clear_gbl())
        self.assertIsNone(info["application"])
        self.assertIsNone(info["zrhp"])
        self.assertEqual(info["programs"], [NV_WINDOW])

    def test_damaged_or_foreign_files_are_refused(self):
        good = bytearray(app_gbl())
        cases = {
            "crc": bytes(good[:40]) + bytes([good[40] ^ 1]) + bytes(good[41:]),
            "truncated": bytes(good[:-20]),
            "trailing data": bytes(good) + b"\xff" * 4,
            "non-0xff padding": gbl(application(), zrhp(), pad=False) + b"\x00",
            "version 2 header": gbl(application(), version=0x0200_0000),
            "encrypted": gbl(application(), gbl_type=1),
            "signature record": gbl(application(), record(0xF70A0AF7, bytes(64))),
            "bootloader record": gbl(record(0xF50909F5, bytes(16))),
        }
        for name, data in cases.items():
            with self.subTest(name), self.assertRaises(REL.ReleaseError):
                REL.read_gbl(data)


class GeckoTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, name, data):
        path = self.dir / name
        path.write_bytes(data)
        return path

    def test_describes_the_ncp_and_the_nv_clear_image(self):
        out = self.dir / "gecko"
        ncp = self.write("efr32mg24-ncp-sonoff-dongle-m.gbl", app_gbl())
        clear = self.write("efr32mg24-nv-clear.gbl", clear_gbl())
        with mock.patch("builtins.print"):
            REL.cmd_gecko(gecko_args(ncp, out, "application", ident="sonoff-dongle-max-ncp"))
            REL.cmd_gecko(gecko_args(clear, out, "maintenance", ident="efr32mg24-nv-clear"))
            # Describing an image again replaces its entry.
            REL.cmd_gecko(gecko_args(clear, out, "maintenance", ident="efr32mg24-nv-clear"))
        entries = json.loads((out / "gecko.json").read_text())
        self.assertEqual([e["id"] for e in entries], ["sonoff-dongle-max-ncp", "efr32mg24-nv-clear"])
        app, nv = entries
        self.assertEqual(app["app_version"], "0.1.3")
        self.assertEqual(app["gecko_version"], "0.1.3")
        self.assertEqual(app["build_id"], "0001020304050607")
        self.assertEqual(app["program"], [{"start": "0x08006000", "end": "0x08006040"}])
        self.assertEqual(app["file"]["sha256"], hashlib.sha256(ncp.read_bytes()).hexdigest())
        self.assertNotIn("app_version", nv)
        self.assertEqual(nv["program"], [{"start": "0x08158000", "end": "0x08180000"}])
        self.assertEqual((out / clear.name).read_bytes(), clear.read_bytes())

    def test_refuses_what_the_release_must_not_carry(self):
        out = self.dir / "gecko"
        cases = [
            ("other version", app_gbl(fw="0.1.2", version_word=0x0001_0200), "application"),
            ("ZRHP and Gecko versions differ", app_gbl(fw="0.1.3", version_word=0x0001_0200), "application"),
            ("no ZRHP record", gbl(application(), prog(APP_START, bytes(8))), "application"),
            ("no application record", clear_gbl(), "application"),
            ("maintenance with an app", app_gbl(), "maintenance"),
            ("writes the bootloader", app_gbl(start=0x0800_0000), "application"),
            ("unknown kind", app_gbl(), "bootloader"),
        ]
        for name, data, kind in cases:
            with self.subTest(name), self.assertRaises(REL.ReleaseError):
                REL.cmd_gecko(gecko_args(self.write("image.gbl", data), out, kind))
        self.assertFalse((out / "gecko.json").exists())

    def test_pre_release(self):
        """The Gecko word has no room for `-rc.1` (gecko.rs drops it); the
        ZRHP record carries the full version."""
        out = self.dir / "gecko"
        rc = self.write("rc.gbl", app_gbl(fw="0.2.0-rc.1", version_word=0x0002_0000))
        with mock.patch("builtins.print"):
            REL.cmd_gecko(gecko_args(rc, out, "application", expect="0.2.0-rc.1"))
        entry = json.loads((out / "gecko.json").read_text())[0]
        self.assertEqual((entry["app_version"], entry["gecko_version"]), ("0.2.0-rc.1", "0.2.0"))
        cases = [
            ("final image for an rc tag", app_gbl(fw="0.2.0", version_word=0x0002_0000)),
            ("other rc", app_gbl(fw="0.2.0-rc.2", version_word=0x0002_0000)),
            ("Gecko word of another release", app_gbl(fw="0.2.0-rc.1", version_word=0x0001_0900)),
        ]
        for name, data in cases:
            with self.subTest(name), self.assertRaises(REL.ReleaseError):
                REL.cmd_gecko(gecko_args(self.write("image.gbl", data), out, "application",
                                         expect="0.2.0-rc.1"))
        # Without a release to compare with, the two records must still agree.
        with self.assertRaises(REL.ReleaseError):
            REL.cmd_gecko(gecko_args(self.write("image.gbl", app_gbl(fw="0.2.0-rc.1", version_word=0x0001_0900)),
                                     out, "application", expect=None))

    def test_core_version(self):
        self.assertEqual(REL.core_version("0.2.0-rc.1"), "0.2.0")
        self.assertEqual(REL.core_version("1.2.3+build.5"), "1.2.3")
        self.assertEqual(REL.core_version("0.1.3"), "0.1.3")
        for bad in ("0.2", "v0.2.0", "0.2.0rc1", ""):
            with self.subTest(bad), self.assertRaises(REL.ReleaseError):
                REL.core_version(bad)


class ManifestTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def esp_board(self, version):
        """A board directory as `assemble` leaves it (unsigned)."""
        directory = self.dir / "esp"
        directory.mkdir()
        parts = {"merged": 0, "bootloader": 0, "partition_table": 0x8000, "otadata": 0xD000, "app": 0x20000}
        files = {}
        for role, offset in parts.items():
            path = directory / f"zrhp-ncp-{version}-h2-{role}.bin"
            path.write_bytes(role.encode() * 16)
            files[role] = REL.file_entry(path, offset)
        entry = {"id": "esp32h2-devkit-ncp", "title": "ESP32-H2 DevKit", "chip_family": "ESP32-H2",
                 "flash_size": "4MB", "layout": "esp32h2-ncp-4mb", "app_version": version,
                 "files": files, "flash": ["bootloader", "partition_table", "otadata", "app"],
                 "fw_update": None}
        (directory / "board.json").write_text(json.dumps(entry))
        return directory

    def gecko_dir(self, fw="0.1.3", word=0x0001_0300):
        out = self.dir / "gecko"
        ncp = self.dir / "efr32mg24-ncp-sonoff-dongle-m.gbl"
        ncp.write_bytes(app_gbl(fw=fw, version_word=word))
        clear = self.dir / "efr32mg24-nv-clear.gbl"
        clear.write_bytes(clear_gbl())
        with mock.patch("builtins.print"):
            REL.cmd_gecko(gecko_args(ncp, out, "application", expect=None, ident="sonoff-dongle-max-ncp"))
            REL.cmd_gecko(gecko_args(clear, out, "maintenance", expect=None, ident="efr32mg24-nv-clear"))
        return out

    def manifest(self, dirs, gecko, version="0.1.3"):
        args = argparse.Namespace(
            dirs=[str(d) for d in dirs], gecko=[str(g) for g in gecko] if gecko else None,
            out=str(self.dir / "dist"), tag=f"ncp-v{version}", version=version, commit="c" * 40,
            repository="faronov/zigbee-rs", date="2026-11-30T00:00:00Z", toolchain=None)
        with mock.patch("builtins.print"):
            REL.cmd_manifest(args)
        return json.loads((self.dir / "dist/release-manifest.json").read_text())

    def test_gecko_images_are_assets_but_not_boards(self):
        manifest = self.manifest([self.esp_board("0.1.3")], [self.gecko_dir()])
        self.assertEqual([b["id"] for b in manifest["boards"]], ["esp32h2-devkit-ncp"])
        self.assertFalse(manifest["boards"][0]["fw_update"]["signed"])
        self.assertEqual([g["id"] for g in manifest["gecko_images"]],
                         ["sonoff-dongle-max-ncp", "efr32mg24-nv-clear"])
        # The web flasher accepts the manifest and only sees the ESP board.
        SITE.validate_manifest(manifest, "ncp-v0.1.3")
        dist = self.dir / "dist"
        sums = dict(reversed(line.split("  ")) for line in (dist / "SHA256SUMS").read_text().splitlines())
        for name in ("efr32mg24-ncp-sonoff-dongle-m.gbl", "efr32mg24-nv-clear.gbl", "release-manifest.json"):
            self.assertEqual(sums[name], hashlib.sha256((dist / name).read_bytes()).hexdigest())
        for g in manifest["gecko_images"]:
            self.assertEqual(sums[g["file"]["name"]], g["file"]["sha256"])

    def test_without_gecko_images_the_manifest_is_unchanged(self):
        manifest = self.manifest([self.esp_board("0.1.3")], None)
        self.assertNotIn("gecko_images", manifest)

    def test_gecko_image_of_another_version_is_refused(self):
        with self.assertRaises(REL.ReleaseError):
            self.manifest([self.esp_board("0.1.3")], [self.gecko_dir(fw="0.1.2", word=0x0001_0200)])

    def test_pre_release_manifest(self):
        manifest = self.manifest([self.esp_board("0.2.0-rc.1")],
                                 [self.gecko_dir(fw="0.2.0-rc.1", word=0x0002_0000)], version="0.2.0-rc.1")
        self.assertTrue(manifest["prerelease"])
        app = manifest["gecko_images"][0]
        self.assertEqual((app["app_version"], app["gecko_version"]), ("0.2.0-rc.1", "0.2.0"))
        SITE.validate_manifest(manifest, "ncp-v0.2.0-rc.1")

    def test_gecko_image_of_another_pre_release_is_refused(self):
        with self.assertRaises(REL.ReleaseError):
            self.manifest([self.esp_board("0.2.0-rc.2")],
                          [self.gecko_dir(fw="0.2.0-rc.1", word=0x0002_0000)], version="0.2.0-rc.2")

    def test_gecko_image_changed_after_description_is_refused(self):
        gecko = self.gecko_dir()
        (gecko / "efr32mg24-nv-clear.gbl").write_bytes(clear_gbl() + b"\xff\xff\xff\xff")
        with self.assertRaises(REL.ReleaseError):
            self.manifest([self.esp_board("0.1.3")], [gecko])


class VersionsTest(unittest.TestCase):
    def test_the_tree_has_one_ncp_version(self):
        versions = set(REL.ncp_versions().values())
        self.assertEqual(len(versions), 1, REL.ncp_versions())

    def tree(self, root, version, mismatch=None):
        for directory, package, locks in REL.NCP_VERSION_SOURCES:
            manifest = root / directory / "Cargo.toml"
            manifest.parent.mkdir(parents=True, exist_ok=True)
            v = "9.9.9" if mismatch == f"{directory}/Cargo.toml" else version
            manifest.write_text(f'[package]\nname = "{package}"\nversion = "{v}"\n\n[dependencies]\nversion = "1"\n')
        locks = {}
        for _, package, files in REL.NCP_VERSION_SOURCES:
            for lock in files:
                locks.setdefault(lock, []).append(package)
        for lock, packages in locks.items():
            path = root / lock
            path.parent.mkdir(parents=True, exist_ok=True)
            body = "".join(f'[[package]]\nname = "{p}"\nversion = "{"9.9.9" if mismatch == lock else version}"\n\n'
                           for p in packages)
            path.write_text("version = 4\n\n" + body)

    def run_versions(self, root, expect=None):
        with mock.patch.object(REL, "ROOT", root), mock.patch("builtins.print") as out:
            REL.cmd_versions(argparse.Namespace(expect=expect))
        return out.call_args.args[0]

    def test_versions_must_agree(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.tree(root, "0.1.3")
            self.assertEqual(self.run_versions(root), "0.1.3")
            self.assertEqual(self.run_versions(root, "0.1.3"), "0.1.3")
            with self.assertRaises(REL.ReleaseError):
                self.run_versions(root, "0.1.4")
        for source in ("products/efr32mg24/Cargo.toml", "examples/efr32mg24-router/Cargo.lock"):
            with self.subTest(source), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                self.tree(root, "0.1.3", mismatch=source)
                with self.assertRaises(REL.ReleaseError):
                    self.run_versions(root)


if __name__ == "__main__":
    unittest.main()
