"""Run with python3 -m unittest discover -s <this directory>.

Set ESP_OTA_C6_IMAGE / ESP_OTA_H2_IMAGE to real espflash application files to
include artifact compatibility. --write-corpus DIR writes these same vectors
for the opt-in Rust `python_preflight_corpus` differential regression. Add
--chip esp32c6 or --chip esp32h2 to require/generate only that chip's real-image
corpus. Without --chip, both real images are required and both corpora are written.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from esp_app_image import (
    APP_DESC_MAGIC, CHIP_IDS, ImageValidationError, validate_application,
)

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SLOT_SIZE = 0x1F0000
SLOTS = (0x10000, 0x200000)


def load_script(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


SIZE = load_script("esp_size_check", HERE / "check-app-image-size.py")
OTA = load_script("esp_ota_packager", ROOT / "tools/create-esp32-ota.py")


def rehash(image: bytes) -> bytes:
    return bytes(image[:-32]) + hashlib.sha256(image[:-32]).digest()


def finish(image: bytes) -> bytes:
    cursor, checksum = 24, 0xEF
    for _ in range(image[1]):
        length = struct.unpack_from("<I", image, cursor + 4)[0]
        cursor += 8
        for byte in image[cursor:cursor + length]:
            checksum ^= byte
        cursor += length
    trailer_end = (cursor + 16) & ~15
    result = bytes(image[:cursor]) + bytes(trailer_end - cursor - 1) + bytes([checksum])
    return result + hashlib.sha256(result).digest()


def application(chip: str, code_length: int = 16) -> bytes:
    # Same complete descriptor + mapped RISC-V JAL/NOP fixture as Rust tests.
    header = bytearray(24)
    header[0:2] = bytes([0xE9, 2])
    struct.pack_into("<I", header, 4, 0x42000128)
    struct.pack_into("<H", header, 12, CHIP_IDS[chip])
    struct.pack_into("<H", header, 17, 0xFFFF)
    header[23] = 1
    descriptor = bytearray(256)
    struct.pack_into("<I", descriptor, 0, APP_DESC_MAGIC)
    struct.pack_into("<H", descriptor, 178, 0xFFFF)
    descriptor[180] = 16
    code = struct.pack("<I", 0x6F) + struct.pack("<I", 0x13) * (code_length // 4 - 1)
    return finish(
        header + struct.pack("<II", 0x42000020, 256) + descriptor
        + struct.pack("<II", 0x42000128, len(code)) + code
    )


def append_segment(image: bytes, address: int, data: bytes) -> bytes:
    cursor = 24
    for _ in range(image[1]):
        cursor += 8 + struct.unpack_from("<I", image, cursor + 4)[0]
    result = bytearray(image[:cursor])
    result[1] += 1
    return finish(result + struct.pack("<II", address, len(data)) + data)


def changed(image: bytes, offset: int, value: bytes) -> bytes:
    result = bytearray(image)
    result[offset:offset + len(value)] = value
    return rehash(result)


def vectors(chip: str):
    image = application(chip)
    yield "complete", image, None
    for length in (4, 8, 12, 4096):
        yield f"code-{length}", application(chip, length), None
    padded = append_segment(image, 0, b"\xfe\0\0\0")
    loaded = append_segment(padded, 0x40800000, b"\x13\0\0\0")
    loaded = append_segment(loaded, 0x50000000, b"\x13\0\0\0")
    while loaded[1] < 16:
        loaded = append_segment(loaded, 0, b"")
    yield "sixteen-padding-loaded-empty", loaded, None
    repro = bytearray(56)
    repro[0:2] = bytes([0xE9, 17])
    struct.pack_into("<H", repro, 12, CHIP_IDS[chip])
    repro[23] = 1
    yield "original-56-byte-repro", rehash(repro), "TooSmall"
    for count, error in ((0, "NoSegments"), (17, "TooManySegments"), (255, "TooManySegments")):
        yield f"segment-count-{count}", changed(image, 1, bytes([count])), error
    yield "bad-magic", changed(image, 0, b"\x7f"), "BadMagic"
    foreign = CHIP_IDS["esp32h2" if chip == "esp32c6" else "esp32c6"]
    yield "foreign-chip", changed(image, 12, struct.pack("<H", foreign)), "ChipMismatch"
    for flag in (0, 2, 255):
        yield f"hash-flag-{flag}", changed(image, 23, bytes([flag])), "NoAppendedHash"
    for length in (3, 0xFFFFFFFC, 0xFFFFFFFF, 0x1000000):
        yield f"length-{length}", changed(image, 28, struct.pack("<I", length)), "SegmentLength"
    yield "segment-overrun", changed(image, 28, struct.pack("<I", 0x10000)), "SegmentBounds"
    yield "short-descriptor", changed(image, 28, struct.pack("<I", 4)), "AppDescriptor"
    yield "bad-descriptor", changed(image, 32, b"\0"), "AppDescriptor"
    yield "bad-page-size", changed(image, 212, b"\x20"), "UnsupportedMmuPageSize"
    yield "secure-version", changed(image, 36, b"\x01"), "UnsupportedSecureVersion"
    for address in (0x60000000, 0xFFFFFFFC, 0x4087FFFC):
        yield f"bad-load-{address}", changed(image, 288, struct.pack("<I", address)), "LoadAddress"
    yield "bad-entry", changed(image, 4, struct.pack("<I", 0x40000000)), "EntryAddress"
    yield "odd-entry", changed(image, 4, struct.pack("<I", 0x42000129)), "EntryAddress"
    yield "bad-mapping", changed(image, 24, struct.pack("<I", 0x42000024)), "MappingAlignment"
    overlap = append_segment(image, 0x40800000, b"\x13\0\0\0")
    overlap = append_segment(overlap, 0x40800000, b"\x13\0\0\0")
    yield "overlap", overlap, "OverlappingSegments"
    yield "bad-checksum-rehashed", changed(image, len(image) - 33, bytes([image[-33] ^ 1])), "Checksum"
    yield "bad-hash", image[:-1] + bytes([image[-1] ^ 1]), "Hash"
    yield "trailing-data-rehashed", rehash(image + bytes(32)), "ImageLength"
    yield "truncated-trailer", image[:-1], "ImageLength"


def real_image(chip: str, *, required: bool = False) -> bytes:
    key = "ESP_OTA_C6_IMAGE" if chip == "esp32c6" else "ESP_OTA_H2_IMAGE"
    path = os.environ.get(key)
    if not path:
        message = f"set {key} to an actual espflash-generated application"
        if required:
            raise SystemExit(message)
        raise unittest.SkipTest(message)
    return Path(path).read_bytes()


class PreflightTests(unittest.TestCase):
    def test_shared_vectors_and_both_entrypoints(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "app.bin"
            for chip in CHIP_IDS:
                config = OTA.CHIPS[chip]
                for name, image, expected in vectors(chip):
                    with self.subTest(chip=chip, vector=name):
                        path.write_bytes(image)
                        for offset in SLOTS:
                            args = (image, CHIP_IDS[chip], offset, SLOT_SIZE)
                            slot = SIZE.Partition("test", offset, SLOT_SIZE)
                            if expected is None:
                                validate_application(*args, chip_revision=0, efuse_block_revision=0)
                                self.assertEqual(SIZE.check_application_image(path, chip, slot), image)
                            else:
                                with self.assertRaises(ImageValidationError) as error:
                                    validate_application(*args, chip_revision=0, efuse_block_revision=0)
                                self.assertEqual(error.exception.code, expected)
                                with self.assertRaisesRegex(SystemExit, expected):
                                    SIZE.check_application_image(path, chip, slot)
                        if expected is None:
                            OTA.check_esp_image(image, config)
                            blob = OTA.build_container(image, 1, config)
                            OTA.verify_container(blob, image, 1, config)
                        else:
                            with self.assertRaisesRegex(SystemExit, expected):
                                OTA.check_esp_image(image, config)
                            with self.assertRaisesRegex(SystemExit, expected):
                                OTA.verify_container(OTA.build_container(image, 1, config), image, 1, config)

    def test_every_truncation_rejected_without_parser_exception(self):
        for chip in CHIP_IDS:
            image = application(chip)
            for size in range(len(image)):
                with self.subTest(chip=chip, size=size), self.assertRaises(ImageValidationError):
                    validate_application(image[:size], CHIP_IDS[chip], SLOTS[0], SLOT_SIZE)

    def test_explicit_revision_compatibility_and_unknown_offline_revision(self):
        for chip in CHIP_IDS:
            image = changed(application(chip), 15, struct.pack("<H", 1))
            # Offline preflight reports limits, never guesses a revision.
            info = validate_application(image, CHIP_IDS[chip], SLOTS[0], SLOT_SIZE)
            self.assertEqual(info.chip_revision_range, (1, 0xFFFF))
            with self.assertRaisesRegex(ImageValidationError, "ChipRevision"):
                validate_application(image, CHIP_IDS[chip], SLOTS[0], SLOT_SIZE, chip_revision=0)
            image = changed(application(chip), 208, struct.pack("<H", 1))
            with self.assertRaisesRegex(ImageValidationError, "EfuseBlockRevision"):
                validate_application(image, CHIP_IDS[chip], SLOTS[0], SLOT_SIZE, efuse_block_revision=0)
            image = changed(application(chip), 17, struct.pack("<H", 99))
            validate_application(image, CHIP_IDS[chip], SLOTS[0], SLOT_SIZE, chip_revision=99)
            with self.assertRaisesRegex(ImageValidationError, "ChipRevision"):
                validate_application(image, CHIP_IDS[chip], SLOTS[0], SLOT_SIZE, chip_revision=100)

    def test_physical_slot_size_and_mapping_boundaries(self):
        for chip in CHIP_IDS:
            image = application(chip)
            with self.assertRaisesRegex(ImageValidationError, "ImageLength"):
                validate_application(image, CHIP_IDS[chip], SLOTS[0], len(image) - 1)
            with self.assertRaisesRegex(ImageValidationError, "ImageLength"):
                validate_application(image, CHIP_IDS[chip], 0xFFFFFFF0, SLOT_SIZE)
            with self.assertRaisesRegex(ImageValidationError, "MappingAlignment"):
                validate_application(image, CHIP_IDS[chip], SLOTS[0] + 4, SLOT_SIZE)

    def test_index_rejects_preexisting_malformed_application(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            config = OTA.CHIPS["esp32c6"]
            image = application("esp32c6")
            valid = OTA.build_container(image, 1, config)
            (output / "valid.ota").write_bytes(valid)
            index = OTA.write_index(output)
            before = index.read_bytes()
            self.assertEqual(len(json.loads(before)["firmwares"]), 1)
            malformed = changed(image, 1, b"\x11")
            (output / "invalid.ota").write_bytes(OTA.build_container(malformed, 2, config))
            with self.assertRaisesRegex(SystemExit, "TooManySegments"):
                OTA.write_index(output)
            self.assertEqual(index.read_bytes(), before)

    def check_real(self, chip: str):
        image = real_image(chip)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "actual.app.bin"
            path.write_bytes(image)
            for offset in SLOTS:
                info = validate_application(image, CHIP_IDS[chip], offset, SLOT_SIZE)
                self.assertGreater(info.segment_count, 0)
                self.assertEqual(SIZE.check_application_image(
                    path, chip, SIZE.Partition("test", offset, SLOT_SIZE)), image)
            config = OTA.CHIPS[chip]
            blob = OTA.build_container(image, 1, config)
            OTA.verify_container(blob, image, 1, config)
            self.assertEqual(len(blob), len(image) + 66)

    def test_real_c6_image(self):
        self.check_real("esp32c6")

    def test_real_h2_image(self):
        self.check_real("esp32h2")

    def run_corpus(self, destination: Path, fixtures: dict[str, str], *options: str):
        env = dict(os.environ)
        for key in ("ESP_OTA_C6_IMAGE", "ESP_OTA_H2_IMAGE"):
            env.pop(key, None)
        env.update(fixtures)
        return subprocess.run(
            [sys.executable, str(Path(__file__).resolve()),
             "--write-corpus", str(destination), *options],
            env=env, capture_output=True, text=True, check=False,
        )

    def test_selected_chip_corpus_needs_only_its_own_real_image(self):
        for chip, key in (("esp32c6", "ESP_OTA_C6_IMAGE"), ("esp32h2", "ESP_OTA_H2_IMAGE")):
            image = real_image(chip)
            with self.subTest(chip=chip), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                fixture = root / "actual.app.bin"
                fixture.write_bytes(image)
                destination = root / "corpus"
                result = self.run_corpus(destination, {key: str(fixture)}, "--chip", chip)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual({path.name for path in destination.iterdir()}, {chip})
                self.assertEqual((destination / chip / "real-generated.bin").read_bytes(), image)
                manifest = (destination / chip / "manifest.tsv").read_text().splitlines()
                self.assertEqual(len(manifest), (len(list(vectors(chip))) + 1) * len(SLOTS))

    def test_default_corpus_still_requires_and_generates_both_chips(self):
        images = {chip: real_image(chip) for chip in CHIP_IDS}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fixtures = {}
            for chip, key in (("esp32c6", "ESP_OTA_C6_IMAGE"), ("esp32h2", "ESP_OTA_H2_IMAGE")):
                fixture = root / f"{chip}.app.bin"
                fixture.write_bytes(images[chip])
                fixtures[key] = str(fixture)
            destination = root / "both"
            result = self.run_corpus(destination, fixtures)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual({path.name for path in destination.iterdir()}, set(CHIP_IDS))
            for key, path in fixtures.items():
                result = self.run_corpus(root / "missing-other", {key: path})
                self.assertNotEqual(result.returncode, 0)
                missing = next(name for name in fixtures if name != key)
                self.assertIn(missing, result.stderr)
                self.assertFalse((root / "missing-other").exists())

    def test_selected_corpus_rejects_missing_or_invalid_real_image(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            invalid = root / "invalid.app.bin"
            invalid.write_bytes(bytes(56))
            for chip, key in (("esp32c6", "ESP_OTA_C6_IMAGE"), ("esp32h2", "ESP_OTA_H2_IMAGE")):
                for label, fixtures in (
                    ("unset", {}),
                    ("absent", {key: str(root / "absent.app.bin")}),
                    ("invalid", {key: str(invalid)}),
                ):
                    with self.subTest(chip=chip, fixture=label):
                        destination = root / f"{chip}-{label}"
                        result = self.run_corpus(destination, fixtures, "--chip", chip)
                        self.assertNotEqual(result.returncode, 0, result.stdout)
                        self.assertFalse((destination / chip / "manifest.tsv").exists())
                        if label == "unset":
                            self.assertIn(key, result.stderr)


class OtaBuildTests(unittest.TestCase):
    def test_build_preserves_version_toolchain_and_features(self):
        for chip in CHIP_IDS:
            for features in ("", "light-sleep"):
                with self.subTest(chip=chip, features=features), tempfile.TemporaryDirectory() as directory:
                    example = Path(directory) / f"{chip}-sensor"
                    elf = example / "target" / OTA.DEFAULT_TARGET / "release" / example.name
                    elf.parent.mkdir(parents=True)
                    elf.touch()
                    with mock.patch.object(OTA, "run") as run:
                        self.assertEqual(OTA.build_elf(7, example, "nightly-2026-08-01", features), elf)
                    command = [
                        "cargo", "+nightly-2026-08-01", "build", "--release",
                        "--locked", "-Z", "build-std=core,alloc",
                    ]
                    if features:
                        command += ["--features", features]
                    self.assertEqual(run.call_args.args[0], command)
                    self.assertEqual(run.call_args.kwargs["cwd"], example)
                    self.assertEqual(run.call_args.kwargs["env"]["ESP32_OTA_VERSION"], "7")

    def test_packager_forwards_feature_selection(self):
        for chip in CHIP_IDS:
            for features in ("", "light-sleep"):
                with self.subTest(chip=chip, features=features), tempfile.TemporaryDirectory() as directory:
                    args = ["create-ota.py", "7", directory, "--chip", chip,
                            "--toolchain", "nightly-2026-08-01"]
                    if features:
                        args += ["--features", features]
                    with (
                        mock.patch.object(sys, "argv", args),
                        mock.patch.object(OTA, "build_elf", return_value=Path("firmware.elf")) as build,
                        mock.patch.object(OTA, "save_image", return_value=application(chip)),
                        mock.patch("builtins.print"),
                    ):
                        self.assertEqual(OTA.main(), 0)
                    build.assert_called_once_with(
                        7, OTA.CHIPS[chip].example_dir, "nightly-2026-08-01", features,
                    )
                    container = Path(directory) / f"{chip}-sensor-v7.ota"
                    self.assertEqual(OTA.parse_container(container.read_bytes())["file_version"], 7)

    def test_wrappers_reject_features_with_prebuilt_elf(self):
        for chip in CHIP_IDS:
            with self.subTest(chip=chip):
                script = ROOT / "examples" / f"{chip}-sensor" / "tools/create-ota.py"
                result = subprocess.run(
                    [sys.executable, str(script), "7", "--features", "light-sleep",
                     "--elf", "absent.elf"],
                    capture_output=True, text=True, check=False,
                )
                self.assertEqual(result.returncode, 2)
                self.assertIn("not allowed with argument", result.stderr)


def write_corpus(destination: Path, chip: str | None = None) -> None:
    if chip is not None and chip not in CHIP_IDS:
        raise ValueError(f"unsupported chip: {chip}")
    chips = (chip,) if chip is not None else tuple(CHIP_IDS)
    images = {selected: real_image(selected, required=True) for selected in chips}
    for chip in chips:
        directory = destination / chip
        directory.mkdir(parents=True, exist_ok=True)
        manifest = []
        cases = list(vectors(chip)) + [("real-generated", images[chip], None)]
        for name, image, expected in cases:
            path = directory / f"{name}.bin"
            path.write_bytes(image)
            for offset in SLOTS:
                try:
                    validate_application(image, CHIP_IDS[chip], offset, SLOT_SIZE,
                                         chip_revision=0, efuse_block_revision=0)
                    accepted = True
                except ImageValidationError as error:
                    if error.code != expected:
                        raise AssertionError(f"{name}: {error.code} != {expected}") from error
                    accepted = False
                if accepted != (expected is None):
                    raise AssertionError(f"{name}: unexpected acceptance")
                manifest.append(f"{path.name}\t{int(accepted)}\t{offset}\n")
        (directory / "manifest.tsv").write_text("".join(manifest), encoding="ascii")
        print(f"{chip}: {len(manifest)} shared Rust/Python cases in {directory}")


if __name__ == "__main__":
    if any(arg == "--write-corpus" or arg.startswith("--write-corpus=") for arg in sys.argv[1:]):
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--write-corpus", type=Path, required=True, metavar="DIR")
        parser.add_argument("--chip", choices=sorted(CHIP_IDS),
                            help="generate only this chip; default: both chips")
        args = parser.parse_args()
        write_corpus(args.write_corpus, args.chip)
    else:
        unittest.main()
