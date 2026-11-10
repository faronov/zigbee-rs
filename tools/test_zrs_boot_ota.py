"""Host tests for tools/zrs_boot_ota.py (standard library only; imgtool
signing is exercised against a real loader-linked build, see
docs/book/src/advanced/bootloader.md)."""

from __future__ import annotations

import hashlib
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bl702_ota  # noqa: E402
import zrs_boot_ota as ota  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
MG22_LAYOUT = REPO / "products/efr32mg22-sensor/boot/devkit.layout"

LAYOUT = """\
# test layout
name        = test-board
flash       = 0x0..0x80000
sector      = 0x2000
loader      = 0x0..0x4000
primary     = 0x4000..0x40000   # comment
secondary   = 0x40000..0x7C000
journal     = 0x7C000..0x80000
header      = 0x200
env-prefix  = TESTB
key-file    = keys/own.xy.hex
"""

KEY_XY = bytes(range(64))
IDENTITY = ota.Identity(0x1234, 0x0022, 0x25010203)


def layout(text: str = LAYOUT) -> ota.Layout:
    return ota.Layout.parse(text, Path("/base"))


def signed_image(body: bytes = b"\x13" * 1000, version=(1, 2, 3), header=0x200) -> bytes:
    major, minor, rev = version
    head = struct.pack("<IIHHIIBBHI", bl702_ota.IMAGE_MAGIC, 0, header, 0, len(body), 0,
                       major, minor, rev, 0)
    hashed = head.ljust(header, b"\0") + body
    tlvs = struct.pack("<BxH", bl702_ota.TLV_SHA256, 32) + hashlib.sha256(hashed).digest()
    tlvs += struct.pack("<BxH", bl702_ota.TLV_ECDSA_SIG, 72) + b"\x30" * 72
    return hashed + struct.pack("<HH", bl702_ota.TLV_INFO_MAGIC, 4 + len(tlvs)) + tlvs


def elf(vector_addr: int, identity: ota.Identity | None) -> bytes:
    """A minimal little-endian ELF32 with `.vector_table`, `.rodata`
    holding the identity words, and a symbol table naming them."""
    rodata_addr = vector_addr + 0x100
    vectors = b"\0" * 8
    rodata = b"" if identity is None else struct.pack(
        "<II", identity.manufacturer | identity.image_type << 16, identity.file_version)
    strtab = b"\0" + ota.IDENTITY_SYMBOL.encode() + b"\0"
    symtab = b"\0" * 16
    if identity is not None:
        symtab += struct.pack("<IIIBBH", 1, rodata_addr, 8, 0x11, 0, 2)
    names = [b"", b".vector_table", b".rodata", b".symtab", b".strtab", b".shstrtab"]
    shstrtab = b"\0".join(names) + b"\0"
    name_offsets, offset = [], 0
    for name in names:
        name_offsets.append(offset)
        offset += len(name) + 1

    body = bytearray(b"\0" * 52)
    sections = []  # (name, type, addr, offset, size, link)
    for index, (data, kind, addr, link) in enumerate(
            [(vectors, 1, vector_addr, 0), (rodata, 1, rodata_addr, 0), (symtab, 2, 0, 4),
             (strtab, 3, 0, 0), (shstrtab, 3, 0, 0)], 1):
        sections.append((name_offsets[index], kind, addr, len(body), len(data), link))
        body += data
    shoff = len(body)
    body += b"\0" * 40
    for name, kind, addr, off, size, link in sections:
        body += struct.pack("<IIIIIIIIII", name, kind, 0, addr, off, size, link, 0, 4,
                            16 if kind == 2 else 0)
    header = b"\x7fELF\x01\x01\x01" + b"\0" * 9
    header += struct.pack("<HHIIIIIHHHHHH", 2, 40, 1, vector_addr, 0, shoff, 0, 52, 0, 0,
                          40, len(sections) + 1, len(sections))
    body[:52] = header
    return bytes(body)


class LayoutTest(unittest.TestCase):
    def test_parse(self):
        parsed = layout()
        self.assertEqual(parsed.name, "test-board")
        self.assertEqual(parsed.slot_size, 0x3C000)
        self.assertEqual(parsed.max_image_len, 0x3A000)
        self.assertEqual(parsed.app_start, 0x4200)
        self.assertEqual(parsed.tlv_reserve, 0x300)
        self.assertEqual(parsed.key_file, Path("/base/keys/own.xy.hex"))

    def test_mg22_product_layout(self):
        parsed = ota.Layout.load(MG22_LAYOUT)
        self.assertEqual(parsed.app_start, 0x4200)
        self.assertEqual(parsed.max_image_len, 0x3C000 - 0x2000)
        self.assertEqual(len(ota.trusted_keys(parsed, env={})[0]), 64)

    def test_unequal_slots_are_rejected(self):
        with self.assertRaisesRegex(ota.ToolError, "differ"):
            layout(LAYOUT.replace("0x40000..0x7C000", "0x40000..0x7A000"))

    def test_missing_and_duplicate_keys_are_rejected(self):
        with self.assertRaisesRegex(ota.ToolError, "missing `secondary`"):
            layout(LAYOUT.replace("secondary", "#"))
        with self.assertRaisesRegex(ota.ToolError, "given twice"):
            layout(LAYOUT + "sector = 0x1000\n")
        with self.assertRaisesRegex(ota.ToolError, "not a `start..end`"):
            layout(LAYOUT.replace("0x4000..0x40000", "0x4000"))


class KeyTest(unittest.TestCase):
    def test_key_file_and_environment(self):
        with tempfile.TemporaryDirectory() as scratch:
            (Path(scratch) / "keys").mkdir()
            (Path(scratch) / "keys/own.xy.hex").write_text(KEY_XY.hex() + "\n")
            parsed = ota.Layout.parse(LAYOUT, Path(scratch))
            self.assertEqual(ota.trusted_keys(parsed, env={}), [KEY_XY])
            other = bytes(64 * [7])
            env = {"TESTB_OTA_KEY_XY": other.hex(), "TESTB_OTA_KEY3_XY": KEY_XY.hex()}
            self.assertEqual(ota.trusted_keys(parsed, env=env), [other, KEY_XY])

    def test_malformed_key_is_rejected(self):
        with self.assertRaisesRegex(ota.ToolError, "64 hex-encoded bytes"):
            ota.parse_xy("key", "abcd")


class IdentityTest(unittest.TestCase):
    def test_identity_from_elf(self):
        app = bl702_ota.Elf(elf(0x4200, IDENTITY))
        self.assertEqual(ota.elf_identity(app), IDENTITY)
        self.assertEqual(IDENTITY.version_text, "1.2.3")
        ota.check_app_elf(app, layout())

    def test_elf_without_identity_is_rejected(self):
        with self.assertRaisesRegex(ota.ToolError, "no ZRS_OTA_IDENTITY"):
            ota.elf_identity(bl702_ota.Elf(elf(0x4200, None)))

    def test_direct_map_elf_is_rejected(self):
        with self.assertRaisesRegex(ota.ToolError, "not linked for this layout"):
            ota.check_app_elf(bl702_ota.Elf(elf(0x0, IDENTITY)), layout())


class OtaFileTest(unittest.TestCase):
    def test_round_trip_and_check(self):
        image = signed_image()
        data = ota.ota_file(image, IDENTITY, b"test-board 1.2.3")
        header, parsed = bl702_ota.parse_ota_file(data)
        self.assertEqual(parsed, image)
        self.assertEqual((header.manufacturer, header.image_type, header.file_version),
                         (0x1234, 0x0022, 0x25010203))
        self.assertIn("image 1.2.3", ota.check(data, layout(), IDENTITY))

    def test_identity_mismatch_is_rejected(self):
        data = ota.ota_file(signed_image(), IDENTITY)
        with self.assertRaisesRegex(ota.ToolError, "not the application's"):
            ota.check(data, layout(), ota.Identity(0x1234, 0x0702, IDENTITY.file_version))
        with self.assertRaisesRegex(ota.ToolError, "file version"):
            ota.check(data, layout(), ota.Identity(0x1234, 0x0022, 0x25010204))

    def test_version_mismatch_is_rejected(self):
        data = ota.ota_file(signed_image(version=(1, 2, 4)), IDENTITY)
        with self.assertRaisesRegex(ota.ToolError, "version"):
            ota.check(data, layout())

    def test_header_size_must_match_the_layout(self):
        data = ota.ota_file(signed_image(header=0x100), IDENTITY)
        with self.assertRaisesRegex(ota.ToolError, "header size"):
            ota.check(data, layout())

    def test_padded_image_is_rejected(self):
        data = ota.ota_file(signed_image() + b"\xff" * 16, IDENTITY)
        with self.assertRaisesRegex(ota.ToolError, "unpadded"):
            ota.check(data, layout())

    def test_oversized_image_is_rejected(self):
        body = b"\x13" * (0x3A000 - 0x200)
        with self.assertRaisesRegex(ota.ToolError, "slot holds"):
            ota.check(ota.ota_file(signed_image(body), IDENTITY), layout())

    def test_corrupted_or_unsigned_image_is_rejected(self):
        image = bytearray(signed_image())
        image[0x300] ^= 1
        with self.assertRaisesRegex(ota.ToolError, "SHA-256"):
            ota.check(ota.ota_file(bytes(image), IDENTITY), layout())


class IndexTest(unittest.TestCase):
    def test_zigpy_local_index(self):
        with tempfile.TemporaryDirectory() as scratch:
            path = Path(scratch) / "a.zigbee"
            path.write_bytes(ota.ota_file(signed_image(), IDENTITY))
            out = Path(scratch) / "index.json"
            ota.main(["index", "--manufacturer-name", "Zigbee-RS", "--model", "M", str(out),
                      str(path)])
            entry = json.loads(out.read_text())["firmwares"][0]
            self.assertEqual(entry["path"], "a.zigbee")
            self.assertEqual(entry["file_version"], 0x25010203)
            self.assertEqual(entry["image_type"], 0x0022)
            self.assertEqual(entry["model_names"], ["M"])


if __name__ == "__main__":
    unittest.main()
