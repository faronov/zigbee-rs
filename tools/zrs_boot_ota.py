#!/usr/bin/env python3
"""Signed images and Zigbee OTA files for zigbee-rs loader products.

Everything product-specific comes from the product's loader layout file
(the `.layout` read by `zrs-boot-layout`) and from the application ELF:

- slots, sector, image header and TLV reserve from the layout;
- trusted keys from `<env-prefix>_OTA_KEY_XY` (or `key-file`) and the
  optional `<env-prefix>_OTA_KEY2_XY` / `_KEY3_XY`, as in `zrs-boot-layout`;
- manufacturer code, image type and Zigbee file version from the
  `ZRS_OTA_IDENTITY` symbol (`[u32; 2]`: `manufacturer | image_type << 16`,
  then the file version `0xPPMMmmpp`, signed as `MM.mm.pp+0`).

Commands:

    release --layout L --elf E --out DIR [--key K] [--label X]
        Sign the application unpadded (`imgtool --pad-header`), verify the
        signature and write `<name>-<version>.signed.bin` (for SWD into the
        primary slot) and `<name>-<version>.zigbee` (Zigbee OTA file).
    check --layout L [--elf E] FILE.zigbee
        Validate an OTA file against the layout (and the ELF's identity).
    key-xy --key K
        Print the key's P-256 X || Y (for `<env-prefix>_OTA_KEY_XY`).
    index --manufacturer-name N --model M OUT FILE.zigbee...
        Write a zigpy local OTA index for the files.

BL702 keeps its own `bl702_ota.py` (boot ROM header and loader records);
the generic ELF, OTA-file and MCUboot helpers are shared with it.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from bl702_ota import (  # noqa: E402
    OTA_HEADER_LEN,
    OTA_TAG_UPGRADE_IMAGE,
    Elf,
    OtaHeader,
    ToolError,
    elf_to_bin,
    find_imgtool,
    find_objcopy,
    imgtool_public_xy,
    parse_mcuboot,
    parse_ota_file,
)

IDENTITY_SYMBOL = "ZRS_OTA_IDENTITY"
MAX_TRUSTED_KEYS = 3


# ── Layout ──────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Region:
    start: int
    end: int

    def __len__(self) -> int:
        return self.end - self.start


def parse_number(text: str) -> int:
    text = text.strip().replace("_", "")
    try:
        return int(text, 16) if text.lower().startswith("0x") else int(text)
    except ValueError:
        raise ToolError(f"`{text}` is not a number") from None


def parse_region(text: str) -> Region:
    start, sep, end = text.partition("..")
    if not sep:
        raise ToolError(f"`{text}` is not a `start..end` range")
    region = Region(parse_number(start), parse_number(end))
    if region.end <= region.start:
        raise ToolError(f"range `{text}` is empty")
    return region


@dataclass(frozen=True)
class Layout:
    """The subset of a `zrs-boot-layout` file this tool needs. Defaults
    match `zrs-boot-layout`; unknown keys are ignored (the Rust parser
    validates the full file at build time)."""

    name: str
    sector: int
    primary: Region
    secondary: Region
    header: int
    tlv_reserve: int
    xip_base: int
    env_prefix: str
    key_file: Path | None

    @classmethod
    def parse(cls, text: str, base: Path) -> "Layout":
        values: dict[str, str] = {}
        for number, line in enumerate(text.splitlines(), 1):
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            key, sep, value = line.partition("=")
            if not sep:
                raise ToolError(f"line {number}: `{line}` is not `key = value`")
            key = key.strip()
            if key in values and key != "reserved":
                raise ToolError(f"line {number}: `{key}` given twice")
            values[key] = value.strip()

        def need(key: str) -> str:
            if key not in values:
                raise ToolError(f"layout: missing `{key}`")
            return values[key]

        layout = cls(
            name=need("name"),
            sector=parse_number(need("sector")),
            primary=parse_region(need("primary")),
            secondary=parse_region(need("secondary")),
            header=parse_number(values.get("header", "0x100")),
            tlv_reserve=parse_number(values.get("tlv-reserve", "0x300")),
            xip_base=parse_number(values.get("xip-base", "0")),
            env_prefix=need("env-prefix"),
            key_file=base / values["key-file"] if "key-file" in values else None,
        )
        if len(layout.primary) != len(layout.secondary):
            raise ToolError("layout: the slots differ in size")
        if layout.max_image_len <= layout.header:
            raise ToolError("layout: no room for an image in the slot")
        return layout

    @classmethod
    def load(cls, path: Path) -> "Layout":
        return cls.parse(path.read_text(), path.resolve().parent)

    @property
    def slot_size(self) -> int:
        return len(self.primary)

    @property
    def max_image_len(self) -> int:
        """Header through TLVs: the slot's last sector is the trailer."""
        return self.slot_size - self.sector

    @property
    def app_start(self) -> int:
        """Mapped address of the application's vector table."""
        return self.xip_base + self.primary.start + self.header


def parse_xy(origin: str, text: str) -> bytes:
    digits = "".join(text.split())
    try:
        xy = bytes.fromhex(digits)
    except ValueError:
        raise ToolError(f"{origin} is not hex") from None
    if len(xy) != 64:
        raise ToolError(f"{origin} must hold 64 hex-encoded bytes (P-256 X || Y)")
    return xy


def trusted_keys(layout: Layout, env: dict[str, str] | None = None) -> list[bytes]:
    """The layout's trusted P-256 keys (X || Y), resolved as the loader
    and application builds resolve them."""
    env = os.environ if env is None else env

    def load(var: str) -> bytes | None:
        value = env.get(var)
        if value is None:
            return None
        digits = "".join(value.split())
        if len(digits) == 128 and all(c in "0123456789abcdefABCDEF" for c in digits):
            return parse_xy(var, digits)
        return parse_xy(var, Path(value.strip()).read_text())

    prefix = layout.env_prefix
    first = load(f"{prefix}_OTA_KEY_XY")
    if first is None:
        if layout.key_file is None:
            raise ToolError(f"no trusted key: set {prefix}_OTA_KEY_XY or `key-file`")
        first = parse_xy(str(layout.key_file), layout.key_file.read_text())
    keys = [first]
    for index in range(2, MAX_TRUSTED_KEYS + 1):
        extra = load(f"{prefix}_OTA_KEY{index}_XY")
        if extra is not None:
            keys.append(extra)
    return keys


# ── Identity ────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Identity:
    manufacturer: int
    image_type: int
    file_version: int

    @property
    def version(self) -> tuple[int, int, int]:
        v = self.file_version
        return v >> 16 & 0xFF, v >> 8 & 0xFF, v & 0xFF

    @property
    def version_text(self) -> str:
        return "{}.{}.{}".format(*self.version)


def elf_identity(app: Elf) -> Identity:
    symbols = app.symbols()
    if IDENTITY_SYMBOL not in symbols:
        raise ToolError(f"application ELF has no {IDENTITY_SYMBOL} symbol "
                        "(not an OTA-capable zigbee-rs loader build)")
    address = symbols[IDENTITY_SYMBOL]
    word = app.read_u32(address)
    return Identity(word & 0xFFFF, word >> 16, app.read_u32(address + 4))


def check_app_elf(app: Elf, layout: Layout) -> None:
    """The ELF must be linked for the layout's primary slot."""
    vectors = app.section(".vector_table").addr
    if vectors != layout.app_start:
        raise ToolError(f"vector table at 0x{vectors:08X}, not 0x{layout.app_start:08X}: "
                        "not linked for this layout's primary slot")


# ── Signed image and OTA file ───────────────────────────────────────────


def check_image(image: bytes, layout: Layout, identity: Identity | None = None) -> str:
    """Structure, SHA-256, size and version of an unpadded signed image.
    The signature is verified by imgtool on release, by the application
    before activation and by the loader."""
    mcuboot = parse_mcuboot(image)
    if not mcuboot.sha256_ok:
        raise ToolError("SHA-256 TLV does not match the image")
    if not mcuboot.has_signature:
        raise ToolError("image carries no ECDSA P-256 signature")
    if mcuboot.hdr_size != layout.header:
        raise ToolError(f"header size 0x{mcuboot.hdr_size:X} != layout 0x{layout.header:X}")
    if len(image) != mcuboot.end:
        raise ToolError("image must be unpadded (no data after the TLVs)")
    if mcuboot.end > layout.max_image_len:
        raise ToolError(f"image is {mcuboot.end} B; the slot holds {layout.max_image_len} B "
                        "below its trailer sector")
    major, minor, rev, _build = mcuboot.version
    if identity is not None and (major, minor, rev) != identity.version:
        raise ToolError(f"image version {major}.{minor}.{rev} != identity "
                        f"{identity.version_text}")
    return f"image {major}.{minor}.{rev} {len(image)} B " \
           f"(slot margin {layout.max_image_len - mcuboot.end} B)"


def ota_file(image: bytes, identity: Identity, header_string: bytes = b"") -> bytes:
    """A file with a single Upgrade Image (tag 0) element holding `image`."""
    element = OTA_TAG_UPGRADE_IMAGE.to_bytes(2, "little") + len(image).to_bytes(4, "little")
    element += image
    header = OtaHeader(identity.manufacturer, identity.image_type, identity.file_version,
                       OTA_HEADER_LEN + len(element), header_string)
    return header.encode() + element


def check(data: bytes, layout: Layout, identity: Identity | None = None) -> str:
    """Validate a .zigbee file; returns a one-line summary."""
    header, image = parse_ota_file(data)
    if identity is not None and (header.manufacturer, header.image_type) != (
            identity.manufacturer, identity.image_type):
        raise ToolError(
            f"identity 0x{header.manufacturer:04X}/0x{header.image_type:04X} is not the "
            f"application's 0x{identity.manufacturer:04X}/0x{identity.image_type:04X}")
    if identity is not None and header.file_version != identity.file_version:
        raise ToolError(f"file version 0x{header.file_version:08X} != the application's "
                        f"0x{identity.file_version:08X}")
    file_identity = Identity(header.manufacturer, header.image_type, header.file_version)
    summary = check_image(image, layout, file_identity)
    return (f"manufacturer 0x{header.manufacturer:04X} image type 0x{header.image_type:04X} "
            f"file version 0x{header.file_version:08X} {summary}")


# ── zigpy index ─────────────────────────────────────────────────────────


def index_entry(path: Path, root: Path, manufacturer_name: str, model: str) -> dict:
    data = path.read_bytes()
    header, _image = parse_ota_file(data)
    return {
        "path": os.path.relpath(path, root),
        "file_version": header.file_version,
        "file_size": len(data),
        "image_type": header.image_type,
        "manufacturer_id": header.manufacturer,
        "manufacturer_names": [manufacturer_name],
        "model_names": [model],
        "checksum": "sha3-256:" + hashlib.sha3_256(data).hexdigest(),
    }


# ── Commands ────────────────────────────────────────────────────────────


def imgtool_sign(imgtool: str, key: Path, layout: Layout, version: str, payload: Path,
                 out: Path) -> bytes:
    subprocess.run([
        imgtool, "sign",
        "--header-size", hex(layout.header), "--pad-header",
        "--align", "4",
        "--slot-size", hex(layout.slot_size),
        "--version", version,
        "--key", str(key),
        str(payload), str(out),
    ], check=True, stdout=subprocess.DEVNULL)
    return out.read_bytes()


def imgtool_verify(imgtool: str, key: Path, image: Path) -> None:
    result = subprocess.run([imgtool, "verify", "--key", str(key), str(image)],
                            capture_output=True, text=True)
    if result.returncode != 0:
        raise ToolError(f"imgtool verify failed: {result.stdout}{result.stderr}".strip())


def cmd_key_xy(args: argparse.Namespace) -> None:
    print(imgtool_public_xy(find_imgtool(args.imgtool), Path(args.key)).hex())


def cmd_release(args: argparse.Namespace) -> None:
    layout = Layout.load(Path(args.layout))
    imgtool = find_imgtool(args.imgtool)
    objcopy = find_objcopy(args.objcopy)
    key = Path(args.key)
    if not key.is_file():
        raise ToolError(f"signing key {key} does not exist")
    if imgtool_public_xy(imgtool, key) not in trusted_keys(layout):
        raise ToolError(f"{key} is not a trusted key of layout {layout.name}")

    app_elf = Path(args.elf)
    app = Elf(app_elf.read_bytes())
    check_app_elf(app, layout)
    identity = elf_identity(app)
    version = identity.version_text

    name = args.name or layout.name
    stem = f"{name}-{args.label}-{version}" if args.label else f"{name}-{version}"
    header_string = f"{name} {version}".encode()[:32]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as scratch:
        payload = Path(scratch) / "app.bin"
        elf_to_bin(objcopy, app_elf, payload)
        signed_path = out / f"{stem}.signed.bin"
        signed = imgtool_sign(imgtool, key, layout, f"{version}+0", payload, signed_path)
    imgtool_verify(imgtool, key, signed_path)
    check_image(signed, layout, identity)
    zigbee = ota_file(signed, identity, header_string)
    zigbee_path = out / f"{stem}.zigbee"
    zigbee_path.write_bytes(zigbee)
    print(f"{signed_path}\n{zigbee_path}: {check(zigbee, layout, identity)}")


def cmd_check(args: argparse.Namespace) -> None:
    layout = Layout.load(Path(args.layout))
    identity = elf_identity(Elf(Path(args.elf).read_bytes())) if args.elf else None
    for path in args.files:
        print(f"{path}: {check(Path(path).read_bytes(), layout, identity)}")


def cmd_index(args: argparse.Namespace) -> None:
    output = Path(args.output)
    root = output.resolve().parent
    index = {"firmwares": [index_entry(Path(path).resolve(), root, args.manufacturer_name,
                                       args.model) for path in args.files]}
    output.write_text(json.dumps(index, indent=2) + "\n")
    print(f"{output}: {len(index['firmwares'])} image(s)")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)

    release = sub.add_parser("release", help="sign an application and build its OTA file")
    release.add_argument("--layout", required=True, help="the product's loader .layout file")
    release.add_argument("--elf", required=True, help="application ELF (loader-linked)")
    release.add_argument("--out", required=True, help="output directory")
    release.add_argument("--key", required=True, help="P-256 signing key (PEM)")
    release.add_argument("--name", help="file name stem (default: the layout name)")
    release.add_argument("--label", help="extra file name tag, e.g. test")
    release.add_argument("--imgtool")
    release.add_argument("--objcopy")
    release.set_defaults(func=cmd_release)

    check_cmd = sub.add_parser("check", help="validate OTA files")
    check_cmd.add_argument("--layout", required=True)
    check_cmd.add_argument("--elf", help="compare with this application's identity")
    check_cmd.add_argument("files", nargs="+")
    check_cmd.set_defaults(func=cmd_check)

    key_xy = sub.add_parser("key-xy", help="print a key's X || Y for <PREFIX>_OTA_KEY_XY")
    key_xy.add_argument("--key", required=True)
    key_xy.add_argument("--imgtool")
    key_xy.set_defaults(func=cmd_key_xy)

    index = sub.add_parser("index", help="write a zigpy local OTA index")
    index.add_argument("--manufacturer-name", required=True)
    index.add_argument("--model", required=True)
    index.add_argument("output")
    index.add_argument("files", nargs="+")
    index.set_defaults(func=cmd_index)

    args = parser.parse_args(argv)
    try:
        args.func(args)
    except (ToolError, OSError, subprocess.CalledProcessError) as error:
        sys.exit(f"error: {error}")


if __name__ == "__main__":
    main()
