"""Host tests for tools/bl702_ota.py (standard library only; imgtool is
exercised by the BL702 CI job that signs real images)."""

from __future__ import annotations

import hashlib
import json
import struct
import sys
import tempfile
import unittest
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bl702_ota as ota  # noqa: E402


def signed_image(body: bytes = b"\x13" * 1000, version=(0, 1, 1)) -> bytes:
    """A structurally valid MCUboot image: header, body, SHA-256 TLV and a
    placeholder ECDSA TLV (the signature value is checked by the Rust tests
    and the loader, not this tool)."""
    major, minor, rev = version
    header = struct.pack("<IIHHIIBBHI", ota.IMAGE_MAGIC, 0, ota.IMAGE_HEADER_SIZE, 0,
                         len(body), 0, major, minor, rev, 0)
    hashed = header.ljust(ota.IMAGE_HEADER_SIZE, b"\0") + body
    tlvs = struct.pack("<BxH", ota.TLV_SHA256, 32) + hashlib.sha256(hashed).digest()
    tlvs += struct.pack("<BxH", ota.TLV_ECDSA_SIG, 72) + b"\x30" * 72
    return hashed + struct.pack("<HH", ota.TLV_INFO_MAGIC, 4 + len(tlvs)) + tlvs


def boot_image(loader: bytes = b"\x97\x01\x00\x00" * 64) -> bytes:
    """A boot ROM image shaped like `bflb-mcu-tool --build` output."""
    image = bytearray(b"\xff" * ota.LOADER_START)
    image[:ota.BFNP_CRC + 4] = b"\0" * (ota.BFNP_CRC + 4)
    image[:4] = ota.BFNP_MAGIC
    image[ota.BFNP_CLOCK_OFFSET:ota.BFNP_CLOCK_OFFSET + 5] = ota.BFNP_CLOCK
    struct.pack_into("<II", image, ota.BFNP_IMG_LEN, len(loader), 0)
    struct.pack_into("<I", image, ota.BFNP_IMG_START, ota.LOADER_START)
    image[ota.BFNP_HASH:ota.BFNP_HASH + 32] = hashlib.sha256(loader).digest()
    struct.pack_into("<I", image, ota.BFNP_CRC, zlib.crc32(bytes(image[:ota.BFNP_CRC])))
    return bytes(image) + loader


def loader_info(key_id: bytes = bytes(range(8))) -> bytes:
    record = bytearray(ota.LOADER_INFO_LEN)
    record[:8] = ota.LOADER_INFO_MAGIC
    struct.pack_into("<H", record, 8, 1)
    record[16:24] = key_id
    struct.pack_into("<I", record, 28, zlib.crc32(bytes(record[:28])))
    return bytes(record)


class LayoutTest(unittest.TestCase):
    def test_application_runs_behind_the_image_header(self):
        self.assertEqual(ota.APP_XIP_START, 0x2300E100)
        self.assertEqual(ota.MAX_IMAGE_LEN, 0x76000)

    def test_regions_are_sector_aligned_and_ordered(self):
        for address in (ota.LOADER_START, ota.PRIMARY_SLOT, ota.SECONDARY_SLOT,
                        ota.NV_START, ota.SLOT_SIZE):
            self.assertEqual(address % ota.SECTOR_SIZE, 0)
        self.assertEqual(ota.PRIMARY_SLOT + ota.SLOT_SIZE, ota.SECONDARY_SLOT)
        self.assertEqual(ota.SECONDARY_SLOT + ota.SLOT_SIZE, ota.NV_START)
        self.assertLess(ota.LOADER_INFO_ADDR + ota.LOADER_INFO_LEN, ota.PRIMARY_SLOT + 1)


class VersionTest(unittest.TestCase):
    def test_file_version_round_trip(self):
        self.assertEqual(ota.file_version(0, 1, 1), 0x25000101)
        self.assertEqual(ota.split_file_version(0x25010203), (1, 2, 3))

    def test_foreign_prefix_is_rejected(self):
        with self.assertRaises(ota.ToolError):
            ota.split_file_version(0x01000100)


class OtaFileTest(unittest.TestCase):
    def test_round_trip_and_check(self):
        image = signed_image()
        data = ota.ota_file(image, ota.file_version(0, 1, 1), b"bl702-xt-zb1 0.1.1")
        header, element = ota.parse_ota_file(data)
        self.assertEqual(len(data), ota.OTA_HEADER_LEN + 6 + len(image))
        self.assertEqual(header.total_size, len(data))
        self.assertEqual((header.manufacturer, header.image_type), (0x1234, 0x0702))
        self.assertEqual(header.header_string, b"bl702-xt-zb1 0.1.1")
        self.assertEqual(element, image)
        self.assertIn("file version 0x25000101", ota.check(data))

    def test_version_mismatch_is_rejected(self):
        data = ota.ota_file(signed_image(version=(0, 1, 0)), ota.file_version(0, 1, 1))
        with self.assertRaisesRegex(ota.ToolError, "version"):
            ota.check(data)

    def test_truncated_file_is_rejected(self):
        data = ota.ota_file(signed_image(), ota.file_version(0, 1, 1))
        with self.assertRaises(ota.ToolError):
            ota.check(data[:-1])

    def test_corrupted_image_is_rejected(self):
        data = bytearray(ota.ota_file(signed_image(), ota.file_version(0, 1, 1)))
        data[ota.OTA_HEADER_LEN + 6 + ota.IMAGE_HEADER_SIZE] ^= 1
        with self.assertRaisesRegex(ota.ToolError, "SHA-256"):
            ota.check(bytes(data))

    def test_unsigned_image_is_rejected(self):
        image = signed_image()
        info = image.index(struct.pack("<H", ota.TLV_INFO_MAGIC), ota.IMAGE_HEADER_SIZE + 1000)
        unsigned = image[:info] + struct.pack("<HH", ota.TLV_INFO_MAGIC, 4 + 36) \
            + image[info + 4:info + 4 + 36]
        with self.assertRaisesRegex(ota.ToolError, "signature"):
            ota.check(ota.ota_file(unsigned, ota.file_version(0, 1, 1)))

    def test_oversized_image_is_rejected(self):
        image = signed_image(b"\0" * ota.MAX_IMAGE_LEN)
        with self.assertRaisesRegex(ota.ToolError, "slot"):
            ota.check(ota.ota_file(image, ota.file_version(0, 1, 1)))


class BootImageTest(unittest.TestCase):
    def test_valid_header(self):
        self.assertEqual(ota.check_boot_header(boot_image(b"\0" * 0x100)), (0x2000, 0x100))

    def test_header_crc_is_checked(self):
        image = bytearray(boot_image())
        image[0x10] ^= 1
        with self.assertRaisesRegex(ota.ToolError, "CRC"):
            ota.check_boot_header(bytes(image))

    def test_image_hash_is_checked(self):
        image = bytearray(boot_image())
        image[ota.LOADER_START] ^= 1
        with self.assertRaisesRegex(ota.ToolError, "hash"):
            ota.check_boot_header(bytes(image))

    def test_pll_clock_is_rejected(self):
        image = bytearray(boot_image())
        image[ota.BFNP_CLOCK_OFFSET + 1] = 4
        struct.pack_into("<I", image, ota.BFNP_CRC, zlib.crc32(bytes(image[:ota.BFNP_CRC])))
        with self.assertRaisesRegex(ota.ToolError, "clocks"):
            ota.check_boot_header(bytes(image))

    def test_hashed_range_must_end_before_loader_info(self):
        too_long = b"\0" * (ota.LOADER_INFO_ADDR - ota.LOADER_START + 0x10)
        with self.assertRaisesRegex(ota.ToolError, "past"):
            ota.check_boot_header(boot_image(too_long))


class LoaderInfoTest(unittest.TestCase):
    def test_key_id(self):
        self.assertEqual(ota.decode_loader_info(loader_info(b"ABCDEFGH")), b"ABCDEFGH")

    def test_crc_is_checked(self):
        record = bytearray(loader_info())
        record[16] ^= 1
        with self.assertRaisesRegex(ota.ToolError, "CRC"):
            ota.decode_loader_info(bytes(record))


class FullImageTest(unittest.TestCase):
    def test_layout(self):
        loader = b"\x97\x01\x00\x00" * 64
        boot = boot_image(loader)
        info = loader_info()
        app = signed_image()
        full = ota.full_flash_image(boot, info, app)
        self.assertEqual(len(full), ota.SECONDARY_SLOT)
        self.assertEqual(full[:len(boot)], boot)
        self.assertTrue(all(byte == 0xFF for byte in full[len(boot):ota.LOADER_INFO_ADDR]))
        self.assertEqual(full[ota.LOADER_INFO_ADDR:ota.PRIMARY_SLOT], info)
        self.assertEqual(full[ota.PRIMARY_SLOT:ota.PRIMARY_SLOT + len(app)], app)
        self.assertTrue(all(byte == 0xFF for byte in full[ota.PRIMARY_SLOT + len(app):]))


class IndexTest(unittest.TestCase):
    def test_zigpy_local_index(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "images").mkdir()
            data = ota.ota_file(signed_image(), ota.file_version(0, 1, 1))
            path = root / "images" / "1234-0702-25000101.zigbee"
            path.write_bytes(data)
            ota.write_index(root / "index.json", [path])
            index = json.loads((root / "index.json").read_text())
        entry, = index["firmwares"]
        self.assertEqual(entry["path"], "images/1234-0702-25000101.zigbee")
        self.assertEqual(entry["file_version"], 0x25000101)
        self.assertEqual(entry["file_size"], len(data))
        self.assertEqual((entry["manufacturer_id"], entry["image_type"]), (0x1234, 0x0702))
        self.assertEqual(entry["checksum"], "sha3-256:" + hashlib.sha3_256(data).hexdigest())


if __name__ == "__main__":
    unittest.main()
