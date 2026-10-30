#!/usr/bin/env python3
"""Release tooling for the BL702 XT-ZB1 sensor: MCUboot signing, Zigbee OTA
files and the full-flash image for the first USB/UART ISP programming.

Subcommands
-----------
key-xy    Print the raw P-256 public key (X || Y, 128 hex digits) of a
          signing key, for `BL702_OTA_KEY_XY` when building the loader and
          the application.
release   Application ELF + loader ELF + loader boot-ROM image ->
          signed application, Zigbee OTA file and full-flash image.
check     Validate .zigbee files (OTA header, element, MCUboot header,
          SHA-256 TLV, signature presence, identity, version).
index     Write a zigpy local OTA index (`zigpy_local` provider) for
          .zigbee files.

Flash layout (products/bl702-xt-zb1/layout/src/lib.rs, 4 KiB sectors)::

    0x00000  boot ROM header (BFNP) of the loader, img_start 0x2000
    0x02000  bl702-boot loader, runs at XIP 0x23000000
    0x0FFE0  LoaderInfo record (outside the ROM-hashed loader image)
    0x10000  primary slot: MCUboot header 0x100, application at 0x2300E100
    0x87000  secondary slot: OTA download
    0xFE000  security journal (never written by this tool)

Signing key: `--key`, else `$BL702_OTA_KEY`, else the flower project's own
key `~/.flower-keys/flower-own-ec-p256.pem`. The private key is never part
of the repository; the loader and the application embed only its public
X || Y (`BL702_OTA_KEY_XY`).

Needs the Python standard library, `imgtool` (MCUboot, pinned 2.4.0) and
`llvm-objcopy` (rustup component llvm-tools).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import zlib
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

# products/bl702-xt-zb1/layout/src/lib.rs; the layout crate's tests check
# that these lines match it.
LOADER_START = 0x2000
LOADER_INFO_ADDR = 0xFFE0
PRIMARY_SLOT = 0x10000
SECONDARY_SLOT = 0x87000
SLOT_SIZE = 0x77000
SECTOR_SIZE = 0x1000
NV_START = 0xFE000
IMAGE_HEADER_SIZE = 0x100
XIP_BASE = 0x23000000

LOADER_INFO_LEN = 32
MAX_IMAGE_LEN = SLOT_SIZE - SECTOR_SIZE  # the last sector holds the trailer
APP_XIP_START = XIP_BASE + PRIMARY_SLOT + IMAGE_HEADER_SIZE - LOADER_START

# products/bl702-xt-zb1/src/ota.rs
MANUFACTURER = 0x1234
IMAGE_TYPE = 0x0702
FILE_VERSION_PREFIX = 0x25
VERSION_SYMBOL = "BL702_OTA_FILE_VERSION"
PRODUCT = "bl702-xt-zb1"
MODEL = "XT-ZB1 Sensor"
MANUFACTURER_NAME = "Zigbee-RS"

DEFAULT_KEY = Path.home() / ".flower-keys" / "flower-own-ec-p256.pem"

# Zigbee OTA Upgrade file (ZCL OTA Upgrade cluster specification, 11.4).
OTA_MAGIC = 0x0BEEF11E
OTA_HEADER_VERSION = 0x0100
OTA_HEADER_LEN = 56
OTA_HEADER_FORMAT = "<IHHHHHIH32sI"
OTA_STACK_ZIGBEE_PRO = 0x0002
OTA_TAG_UPGRADE_IMAGE = 0x0000

# MCUboot image format (bootutil/image.h, bootutil_priv.h).
IMAGE_MAGIC = 0x96F3B83D
TLV_INFO_MAGIC = 0x6907
TLV_PROT_INFO_MAGIC = 0x6908
TLV_SHA256 = 0x10
TLV_ECDSA_SIG = 0x22
BOOT_MAGIC = bytes.fromhex("77c295f360d2ef7f3552500f2cb67980")
LOADER_INFO_MAGIC = b"ZRSBOOT1"

# BL702 boot ROM header (bl702_boot_header.h, bflb-mcu-tool 1.10.0).
BFNP_MAGIC = b"BFNP"
BFNP_CLOCK_OFFSET = 0x68
# xtal 32 MHz, PLL off (XTAL root), hclk_div 0, bclk_div 0, flash clock XCLK.
BFNP_CLOCK = bytes((1, 1, 0, 0, 1))
BFNP_IMG_LEN = 0x78
BFNP_IMG_START = 0x80
BFNP_HASH = 0x84
BFNP_CRC = 0xAC


class ToolError(Exception):
    pass


# ── Versions ────────────────────────────────────────────────────────────


def file_version(major: int, minor: int, patch: int) -> int:
    """`0x25MMmmpp`, as on the CC2340 flower product."""
    return FILE_VERSION_PREFIX << 24 | major << 16 | minor << 8 | patch


def split_file_version(version: int) -> tuple[int, int, int]:
    if version >> 24 != FILE_VERSION_PREFIX:
        raise ToolError(f"file version 0x{version:08X} lacks the 0x{FILE_VERSION_PREFIX:02X} prefix")
    return version >> 16 & 0xFF, version >> 8 & 0xFF, version & 0xFF


# ── ELF ─────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Section:
    name: str
    kind: int
    addr: int
    offset: int
    size: int


class Elf:
    """Just enough of a little-endian ELF32 reader for the checks here."""

    SHT_SYMTAB = 2
    SHT_NOBITS = 8

    def __init__(self, data: bytes):
        if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
            raise ToolError("not a little-endian ELF32 file")
        self.data = data
        (self.entry,) = struct.unpack_from("<I", data, 24)
        shoff, = struct.unpack_from("<I", data, 32)
        shentsize, shnum, shstrndx = struct.unpack_from("<HHH", data, 46)
        raw = [struct.unpack_from("<IIIIIIIIII", data, shoff + i * shentsize)
               for i in range(shnum)]
        names = raw[shstrndx]
        self.sections = []
        self.links = []
        for name, kind, _flags, addr, offset, size, link, _info, _align, _entsize in raw:
            self.sections.append(Section(self._string(names[4], name), kind, addr, offset, size))
            self.links.append(link)

    def _string(self, table_offset: int, index: int) -> str:
        start = table_offset + index
        return self.data[start:self.data.index(b"\0", start)].decode()

    def section(self, name: str) -> Section:
        for section in self.sections:
            if section.name == name:
                return section
        raise ToolError(f"ELF has no {name} section")

    def section_bytes(self, name: str) -> bytes:
        section = self.section(name)
        return self.data[section.offset:section.offset + section.size]

    def symbols(self) -> dict[str, int]:
        found = {}
        for index, section in enumerate(self.sections):
            if section.kind != self.SHT_SYMTAB:
                continue
            strings = self.sections[self.links[index]].offset
            for offset in range(section.offset, section.offset + section.size, 16):
                name, value = struct.unpack_from("<II", self.data, offset)
                if name:
                    found[self._string(strings, name)] = value
        return found

    def read_u32(self, address: int) -> int:
        for section in self.sections:
            if (section.kind != self.SHT_NOBITS and section.addr
                    and section.addr <= address < section.addr + section.size):
                return struct.unpack_from("<I", self.data,
                                          section.offset + address - section.addr)[0]
        raise ToolError(f"0x{address:08X} is in no loaded ELF section")


def app_file_version(app: Elf) -> int:
    symbols = app.symbols()
    if VERSION_SYMBOL not in symbols:
        raise ToolError(f"application ELF has no {VERSION_SYMBOL} symbol")
    version = app.read_u32(symbols[VERSION_SYMBOL])
    split_file_version(version)
    return version


# ── LoaderInfo and boot ROM header ──────────────────────────────────────


def decode_loader_info(record: bytes) -> bytes:
    """Validate a LoaderInfo record; returns its 8-byte key id."""
    if len(record) != LOADER_INFO_LEN or record[:8] != LOADER_INFO_MAGIC:
        raise ToolError("no LoaderInfo record")
    if struct.unpack_from("<H", record, 8)[0] != 1:
        raise ToolError("unsupported LoaderInfo format")
    if struct.unpack_from("<I", record, 28)[0] != zlib.crc32(record[:28]):
        raise ToolError("LoaderInfo CRC mismatch")
    return record[16:24]


def check_boot_header(image: bytes) -> tuple[int, int]:
    """Validate the loader's BFNP boot image from `bflb-mcu-tool --build`;
    returns (img_start, img_len)."""
    if image[:4] != BFNP_MAGIC:
        raise ToolError("loader boot image lacks the BFNP magic")
    clock = image[BFNP_CLOCK_OFFSET:BFNP_CLOCK_OFFSET + len(BFNP_CLOCK)]
    if clock != BFNP_CLOCK:
        raise ToolError(f"loader boot clocks {clock.hex(' ')} != {BFNP_CLOCK.hex(' ')}")
    if struct.unpack_from("<I", image, BFNP_CRC)[0] != zlib.crc32(image[:BFNP_CRC]):
        raise ToolError("boot header CRC mismatch")
    img_len, = struct.unpack_from("<I", image, BFNP_IMG_LEN)
    img_start, = struct.unpack_from("<I", image, BFNP_IMG_START)
    if img_start != LOADER_START:
        raise ToolError(f"boot header img_start 0x{img_start:X} != 0x{LOADER_START:X}")
    end = img_start + img_len
    # The ROM hashes img_len bytes from img_start on every boot: they must
    # end before the LoaderInfo record and the primary slot.
    if end > LOADER_INFO_ADDR:
        raise ToolError(f"ROM-hashed loader image ends at 0x{end:X}, past 0x{LOADER_INFO_ADDR:X}")
    if len(image) < end:
        raise ToolError("loader boot image is truncated")
    if hashlib.sha256(image[img_start:end]).digest() != image[BFNP_HASH:BFNP_HASH + 32]:
        raise ToolError("boot header image hash mismatch")
    if any(byte != 0xFF for byte in image[0x1000:LOADER_START]):
        raise ToolError("unexpected data between the boot header and the loader")
    return img_start, img_len


# ── Zigbee OTA file ─────────────────────────────────────────────────────


@dataclass(frozen=True)
class OtaHeader:
    manufacturer: int
    image_type: int
    file_version: int
    total_size: int
    header_string: bytes = b""
    stack_version: int = OTA_STACK_ZIGBEE_PRO
    field_control: int = 0

    def encode(self) -> bytes:
        if len(self.header_string) > 32:
            raise ToolError("OTA header string longer than 32 bytes")
        return struct.pack(
            OTA_HEADER_FORMAT, OTA_MAGIC, OTA_HEADER_VERSION, OTA_HEADER_LEN,
            self.field_control, self.manufacturer, self.image_type, self.file_version,
            self.stack_version, self.header_string.ljust(32, b"\0"), self.total_size,
        )

    @classmethod
    def decode(cls, data: bytes) -> "OtaHeader":
        if len(data) < OTA_HEADER_LEN:
            raise ToolError("file shorter than an OTA header")
        (magic, version, length, control, manufacturer, image_type, fversion, stack,
         string, total) = struct.unpack_from(OTA_HEADER_FORMAT, data)
        if magic != OTA_MAGIC:
            raise ToolError(f"bad OTA magic 0x{magic:08X}")
        if version != OTA_HEADER_VERSION:
            raise ToolError(f"unsupported OTA header version 0x{version:04X}")
        if control != 0 or length != OTA_HEADER_LEN:
            raise ToolError("optional OTA header fields are not produced by this tool")
        return cls(manufacturer, image_type, fversion, total, string.rstrip(b"\0"), stack,
                   control)


def ota_file(image: bytes, version: int, header_string: bytes = b"") -> bytes:
    """A file with a single Upgrade Image (tag 0) element holding `image`."""
    element = struct.pack("<HI", OTA_TAG_UPGRADE_IMAGE, len(image)) + image
    header = OtaHeader(MANUFACTURER, IMAGE_TYPE, version, OTA_HEADER_LEN + len(element),
                       header_string)
    return header.encode() + element


def parse_ota_file(data: bytes) -> tuple[OtaHeader, bytes]:
    header = OtaHeader.decode(data)
    if header.total_size != len(data):
        raise ToolError(f"total size {header.total_size} != file size {len(data)}")
    offset = OTA_HEADER_LEN
    image = None
    while offset < len(data):
        if offset + 6 > len(data):
            raise ToolError("truncated sub-element header")
        tag, length = struct.unpack_from("<HI", data, offset)
        offset += 6
        if offset + length > len(data):
            raise ToolError(f"sub-element 0x{tag:04X} overruns the file")
        if tag == OTA_TAG_UPGRADE_IMAGE:
            if image is not None:
                raise ToolError("more than one Upgrade Image element")
            image = data[offset:offset + length]
        offset += length
    if image is None:
        raise ToolError("no Upgrade Image element")
    return header, image


# ── MCUboot image ───────────────────────────────────────────────────────


@dataclass(frozen=True)
class McubootImage:
    hdr_size: int
    img_size: int
    version: tuple[int, int, int, int]
    sha256_ok: bool
    has_signature: bool
    end: int
    padded: bool


def parse_mcuboot(image: bytes) -> McubootImage:
    """Structure and SHA-256 check. The signature itself is verified by the
    Rust tests, the application before activation and the loader."""
    if len(image) < 32:
        raise ToolError("image shorter than an MCUboot header")
    magic, _load, hdr_size, protect, img_size, _flags, major, minor, rev, build = \
        struct.unpack_from("<IIHHIIBBHI", image)
    if magic != IMAGE_MAGIC:
        raise ToolError(f"bad MCUboot magic 0x{magic:08X}")
    offset = hdr_size + img_size
    if offset + 4 > len(image):
        raise ToolError("image body overruns the payload")
    if protect:
        info_magic, info_len = struct.unpack_from("<HH", image, offset)
        if info_magic != TLV_PROT_INFO_MAGIC or info_len != protect:
            raise ToolError("bad protected TLV area")
        offset += protect
    hashed = image[:offset]
    info_magic, info_len = struct.unpack_from("<HH", image, offset)
    if info_magic != TLV_INFO_MAGIC:
        raise ToolError(f"bad TLV info magic 0x{info_magic:04X}")
    end = offset + info_len
    if end > len(image):
        raise ToolError("TLV area overruns the payload")
    cursor = offset + 4
    digest = None
    signature = False
    while cursor + 4 <= end:
        tlv_type, tlv_len = struct.unpack_from("<BxH", image, cursor)
        value = image[cursor + 4:cursor + 4 + tlv_len]
        if tlv_type == TLV_SHA256:
            digest = value
        elif tlv_type == TLV_ECDSA_SIG:
            signature = True
        cursor += 4 + tlv_len
    if digest is None:
        raise ToolError("no SHA-256 TLV")
    padded = len(image) == SLOT_SIZE and image[-len(BOOT_MAGIC):] == BOOT_MAGIC
    return McubootImage(hdr_size, img_size, (major, minor, rev, build),
                        hashlib.sha256(hashed).digest() == digest, signature, end, padded)


def check_signed(image: bytes) -> McubootImage:
    mcuboot = parse_mcuboot(image)
    if not mcuboot.sha256_ok:
        raise ToolError("SHA-256 TLV does not match the image")
    if not mcuboot.has_signature:
        raise ToolError("image carries no ECDSA P-256 signature")
    if mcuboot.hdr_size != IMAGE_HEADER_SIZE:
        raise ToolError(f"header size 0x{mcuboot.hdr_size:X} != 0x{IMAGE_HEADER_SIZE:X}")
    if mcuboot.padded or mcuboot.end > MAX_IMAGE_LEN:
        raise ToolError("image must be unpadded and fit the slot below its trailer sector")
    return mcuboot


def check(data: bytes) -> str:
    """Validate a .zigbee file; returns a one-line summary."""
    header, image = parse_ota_file(data)
    mcuboot = check_signed(image)
    if (header.manufacturer, header.image_type) != (MANUFACTURER, IMAGE_TYPE):
        raise ToolError(
            f"identity 0x{header.manufacturer:04X}/0x{header.image_type:04X} is not "
            f"the XT-ZB1's 0x{MANUFACTURER:04X}/0x{IMAGE_TYPE:04X}"
        )
    major, minor, rev, _build = mcuboot.version
    if rev > 0xFF or file_version(major, minor, rev) != header.file_version:
        raise ToolError("OTA file version does not match the MCUboot image version")
    return (
        f"manufacturer 0x{header.manufacturer:04X} image type 0x{header.image_type:04X} "
        f"file version 0x{header.file_version:08X} image {major}.{minor}.{rev} "
        f"{len(image)} B (slot margin {MAX_IMAGE_LEN - mcuboot.end} B)"
    )


# ── Full-flash image ────────────────────────────────────────────────────


def full_flash_image(boot_image: bytes, loader_info: bytes, signed_app: bytes) -> bytes:
    """Physical 0x0..SECONDARY_SLOT: the loader's boot ROM image, its
    LoaderInfo record and the signed application in the primary slot,
    0xFF elsewhere. Programming it erases neither the secondary slot nor
    the security journal."""
    img_start, img_len = check_boot_header(boot_image)
    decode_loader_info(loader_info)
    mcuboot = check_signed(signed_app)
    out = bytearray(b"\xff" * SECONDARY_SLOT)
    out[:img_start + img_len] = boot_image[:img_start + img_len]
    out[LOADER_INFO_ADDR:LOADER_INFO_ADDR + LOADER_INFO_LEN] = loader_info
    out[PRIMARY_SLOT:PRIMARY_SLOT + mcuboot.end] = signed_app[:mcuboot.end]
    return bytes(out)


# ── zigpy index ─────────────────────────────────────────────────────────


def index_entry(path: Path, root: Path) -> dict:
    data = path.read_bytes()
    header, _image = parse_ota_file(data)
    return {
        "path": os.path.relpath(path, root),
        "file_version": header.file_version,
        "file_size": len(data),
        "image_type": header.image_type,
        "manufacturer_id": header.manufacturer,
        "manufacturer_names": [MANUFACTURER_NAME],
        "model_names": [MODEL],
        "checksum": "sha3-256:" + hashlib.sha3_256(data).hexdigest(),
    }


def write_index(output: Path, files: list[Path]) -> dict:
    root = output.resolve().parent
    index = {"firmwares": [index_entry(path.resolve(), root) for path in files]}
    output.write_text(json.dumps(index, indent=2) + "\n")
    return index


# ── External tools ──────────────────────────────────────────────────────


def find_imgtool(override: str | None) -> str:
    candidates = [override, shutil.which("imgtool"),
                  str(Path.home() / "Library/Python/3.11/bin/imgtool")]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return candidate
    raise ToolError("imgtool not found: pip install imgtool==2.4.0 (or pass --imgtool)")


def find_objcopy(override: str | None) -> str:
    if override:
        return override
    found = shutil.which("llvm-objcopy") or shutil.which("rust-objcopy")
    if found:
        return found
    sysroot = subprocess.run(["rustc", "--print", "sysroot"], check=True,
                             capture_output=True, text=True).stdout.strip()
    for candidate in Path(sysroot).glob("lib/rustlib/*/bin/llvm-objcopy"):
        return str(candidate)
    raise ToolError("llvm-objcopy not found: rustup component add llvm-tools")


def resolve_key(override: str | None) -> Path:
    for candidate in (override, os.environ.get("BL702_OTA_KEY")):
        if candidate:
            if not Path(candidate).is_file():
                raise ToolError(f"signing key {candidate} does not exist")
            return Path(candidate)
    if DEFAULT_KEY.is_file():
        return DEFAULT_KEY
    raise ToolError(f"no signing key: pass --key, set BL702_OTA_KEY or create {DEFAULT_KEY}")


def elf_to_bin(objcopy: str, elf: Path, out: Path, remove: tuple[str, ...] = ()) -> bytes:
    command = [objcopy, "--gap-fill", "0xff", "-O", "binary"]
    for section in remove:
        command += ["--remove-section", section]
    subprocess.run(command + [str(elf), str(out)], check=True)
    return out.read_bytes()


def imgtool_sign(imgtool: str, key: Path, version: str, payload: Path, out: Path) -> bytes:
    command = [
        imgtool, "sign",
        "--header-size", hex(IMAGE_HEADER_SIZE), "--pad-header",
        "--align", "4",
        "--slot-size", hex(SLOT_SIZE),
        "--version", version,
        "--key", str(key),
        str(payload), str(out),
    ]
    subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
    return out.read_bytes()


def imgtool_public_xy(imgtool: str, key: Path) -> bytes:
    """Raw X || Y of `key`. `imgtool getpub --encoding raw` prints the DER
    SubjectPublicKeyInfo, which ends with the uncompressed point."""
    result = subprocess.run([imgtool, "getpub", "--key", str(key), "--encoding", "raw"],
                            check=True, capture_output=True)
    der = result.stdout
    if len(der) != 91 or der[-65] != 0x04:
        raise ToolError(f"{key} is not a P-256 key ({len(der)}-byte public key)")
    return der[-64:]


# ── Commands ────────────────────────────────────────────────────────────


def cmd_key_xy(args: argparse.Namespace) -> None:
    print(imgtool_public_xy(find_imgtool(args.imgtool), resolve_key(args.key)).hex())


def cmd_release(args: argparse.Namespace) -> None:
    imgtool = find_imgtool(args.imgtool)
    objcopy = find_objcopy(args.objcopy)
    key = resolve_key(args.key)
    xy = imgtool_public_xy(imgtool, key)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    app_elf = Path(args.app_elf)
    app = Elf(app_elf.read_bytes())
    if app.entry != APP_XIP_START:
        raise ToolError(f"application entry 0x{app.entry:08X} != 0x{APP_XIP_START:08X}: "
                        "not linked for the primary slot")
    fversion = app_file_version(app)
    major, minor, patch = split_file_version(fversion)
    version = f"{major}.{minor}.{patch}"

    loader = Elf(Path(args.loader_elf).read_bytes())
    if loader.entry != XIP_BASE:
        raise ToolError(f"loader entry 0x{loader.entry:08X} != 0x{XIP_BASE:08X}")
    loader_info = loader.section_bytes(".loader_info")
    if decode_loader_info(loader_info) != xy[:8]:
        raise ToolError(f"the loader trusts another key than {key}")
    boot_image = Path(args.loader_image).read_bytes()
    check_boot_header(boot_image)

    stem = f"{PRODUCT}-{args.label}-{version}" if args.label else f"{PRODUCT}-{version}"
    with tempfile.TemporaryDirectory() as scratch:
        payload = Path(scratch) / "app.bin"
        unsigned = elf_to_bin(objcopy, app_elf, payload)
        if xy not in unsigned:
            raise ToolError(f"the application does not embed the public key of {key}: "
                            "build it with BL702_OTA_KEY_XY set to that key")
        if IMAGE_HEADER_SIZE + len(unsigned) > MAX_IMAGE_LEN:
            raise ToolError("application binary does not fit the slot")
        signed = imgtool_sign(imgtool, key, version, payload, out / f"{stem}-signed.bin")

    ota_name = f"{MANUFACTURER:04X}-{IMAGE_TYPE:04X}-{fversion:08X}-{stem}.zigbee"
    data = ota_file(signed, fversion, f"{PRODUCT} {version}".encode())
    (out / ota_name).write_bytes(data)
    print(f"{ota_name}: {check(data)}")

    full_name = f"{stem}-full.bin"
    (out / full_name).write_bytes(full_flash_image(boot_image, loader_info, signed))
    print(f"{full_name}: program at 0x0 (0x0..0x{SECONDARY_SLOT:X}); "
          f"loader 0x{LOADER_START:X}, application 0x{PRIMARY_SLOT:X}")


def cmd_check(args: argparse.Namespace) -> None:
    failed = False
    for name in args.files:
        try:
            print(f"{name}: {check(Path(name).read_bytes())}")
        except ToolError as error:
            print(f"{name}: INVALID: {error}", file=sys.stderr)
            failed = True
    if failed:
        sys.exit(1)


def cmd_index(args: argparse.Namespace) -> None:
    files = [Path(name) for name in args.files]
    for path in files:
        check(path.read_bytes())
    write_index(Path(args.output), files)
    print(f"{args.output}: {len(files)} image(s)")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Sign, package and check BL702 XT-ZB1 firmware images.")
    sub = parser.add_subparsers(dest="command", required=True)

    key_xy = sub.add_parser("key-xy", help="print the public X || Y of the signing key")
    key_xy.add_argument("--key")
    key_xy.add_argument("--imgtool")
    key_xy.set_defaults(func=cmd_key_xy)

    release = sub.add_parser("release", help="sign and package one build")
    release.add_argument("--app-elf", required=True)
    release.add_argument("--loader-elf", required=True)
    release.add_argument("--loader-image", required=True,
                         help="the loader's BFNP image from bflb-mcu-tool --build")
    release.add_argument("--label", default="",
                         help="file-name label, e.g. production or diagnostic")
    release.add_argument("--key", help="signing key (default: $BL702_OTA_KEY, then "
                         f"{DEFAULT_KEY})")
    release.add_argument("--imgtool")
    release.add_argument("--objcopy")
    release.add_argument("--out", default="dist")
    release.set_defaults(func=cmd_release)

    checker = sub.add_parser("check", help="validate .zigbee files")
    checker.add_argument("files", nargs="+")
    checker.set_defaults(func=cmd_check)

    index = sub.add_parser("index", help="write a zigpy_local OTA index")
    index.add_argument("--output", required=True)
    index.add_argument("files", nargs="+")
    index.set_defaults(func=cmd_index)

    args = parser.parse_args(argv)
    try:
        args.func(args)
    except (ToolError, subprocess.CalledProcessError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
