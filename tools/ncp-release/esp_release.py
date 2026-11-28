#!/usr/bin/env python3
"""ZRHP NCP release images: assemble, sign and describe them.

Used by `.github/workflows/ncp-release.yml` (tagged GitHub releases) and
`ncp-release-artifacts.yml` (ad-hoc CI artifacts). Board definitions live in
`boards.json` next to this file. The release manifest format is the contract
with the web flasher (`tools/flasher_site.py`); it is documented in the book
(`docs/book/src/getting-started/web-flasher.md`).

    esp_release.py assemble --board h2-usj --elf ELF --stem STEM --out DIR [--expect-version V]
    esp_release.py sign --dir DIR --key PEM --version V
    esp_release.py gecko --gbl GBL --id ID --title T --kind application|maintenance --out DIR [--expect-version V]
    esp_release.py manifest --dirs DIR... [--gecko DIR...] --out DIR --tag TAG --version V --commit SHA ...

Gecko bootloader images (EFR32 GBL files, for boards such as the SONOFF
Dongle Max whose MG24 is only reachable through its Gecko bootloader) are
release assets listed under `gecko_images`, never under `boards`: the web
flasher installs ESP images only and ignores them.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import zlib

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
BOARDS = json.loads((HERE / "boards.json").read_text())
PARTITIONS = ROOT / "products/esp32-zigbee-ncp/partitions/esp32h2-ncp-4mb.csv"
LAYOUT = "esp32h2-ncp-4mb"
FLASH_SIZE = "4MB"
PARTITION_TABLE_OFFSET = 0x8000
SCHEMA = 1

ESP_IMAGE_MAGIC = 0xE9
APP_DESC_MAGIC = 0xABCD5432


class ReleaseError(Exception):
    pass


def board(name: str) -> dict:
    for entry in BOARDS:
        if entry["name"] == name:
            return entry
    raise ReleaseError(f"unknown board {name!r}")


def read_partitions(csv: Path) -> dict[str, tuple[int, int]]:
    parts = {}
    for line in csv.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = [f.strip() for f in line.split(",")]
        parts[fields[0]] = (int(fields[3], 0), int(fields[4], 0))
    return parts


def esp_image_len(buf: bytes, off: int = 0) -> int:
    """Length of the ESP image at `off`: header, segments, checksum padded
    to 16 bytes, and the appended SHA-256 when the header asks for it."""
    if buf[off] != ESP_IMAGE_MAGIC:
        raise ReleaseError(f"no ESP image at 0x{off:x}")
    p = off + 24
    for _ in range(buf[off + 1]):
        p += 8 + int.from_bytes(buf[p + 4:p + 8], "little")
    p += 1
    p = (p - off + 15) // 16 * 16 + off
    if buf[off + 23] == 1:
        p += 32
    return p - off


def app_version(image: bytes) -> str:
    """`esp_app_desc_t.version` of an ESP application image (the descriptor
    opens the first segment)."""
    if image[0] != ESP_IMAGE_MAGIC:
        raise ReleaseError("not an ESP image")
    desc = 24 + 8
    if int.from_bytes(image[desc:desc + 4], "little") != APP_DESC_MAGIC:
        raise ReleaseError("no esp_app_desc_t at the start of the first segment")
    return image[desc + 16:desc + 48].split(b"\0", 1)[0].decode()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(*cmd: str) -> None:
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)


def file_entry(path: Path, offset: int | None) -> dict:
    entry = {"name": path.name, "size": path.stat().st_size, "sha256": sha256(path)}
    if offset is not None:
        entry["offset"] = offset
    return entry


# GBL v3 records (Silicon Labs UG266; tools/efr32-gbl).
GBL_TAG_HEADER = 0x03A617EB
GBL_TAG_APPLICATION = 0xF40A0AF4
GBL_TAG_METADATA = 0xF60808F6
GBL_TAG_PROG = 0xFE0101FE
GBL_TAG_ERASEPROG = 0xFD0303FD
GBL_TAG_END = 0xFC0404FC
GBL_VERSION = 0x03000000
GECKO_KINDS = ("application", "maintenance")

# Every version a ZRHP NCP image reports or carries, with the lock files
# that pin it: the release tag must match all of them.
NCP_VERSION_SOURCES = [
    # ESP32-H2/C6: app descriptor, SYS_GET_INFO.fw_version, FW_*.
    ("examples/esp32-ncp", "esp32-ncp", ["examples/esp32-ncp/Cargo.lock"]),
    # CC26x2: SYS_GET_INFO.fw_version.
    ("examples/cc26x2-ncp", "cc26x2-ncp", ["examples/cc26x2-ncp/Cargo.lock"]),
    # EFR32MG24: SYS_GET_INFO.fw_version and the GBL's ZRHP record.
    ("examples/efr32mg24-ncp", "efr32mg24-ncp", ["examples/efr32mg24-ncp/Cargo.lock"]),
    # EFR32MG24: the Gecko ApplicationProperties_t version (also the Router's).
    ("products/efr32mg24", "efr32mg24-product",
     ["examples/efr32mg24-ncp/Cargo.lock", "examples/efr32mg24-router/Cargo.lock",
      "products/efr32mg24/Cargo.lock"]),
]


def package_version(manifest: Path) -> str:
    """`version` of the `[package]` table of a Cargo.toml."""
    in_package = False
    for line in manifest.read_text().splitlines():
        line = line.strip()
        if line.startswith("["):
            in_package = line == "[package]"
        elif in_package:
            m = re.fullmatch(r'version\s*=\s*"([^"]+)"', line)
            if m:
                return m.group(1)
    raise ReleaseError(f"{manifest}: no [package] version")


def locked_version(lock: Path, package: str) -> str:
    m = re.search(r'\[\[package\]\]\nname = "%s"\nversion = "([^"]+)"' % re.escape(package), lock.read_text())
    if m is None:
        raise ReleaseError(f"{lock}: no entry for {package}")
    return m.group(1)


def ncp_versions(root: Path | None = None) -> dict[str, str]:
    """Every NCP version source and its value."""
    root = root or ROOT
    found = {}
    for directory, package, locks in NCP_VERSION_SOURCES:
        found[f"{directory}/Cargo.toml"] = package_version(root / directory / "Cargo.toml")
        for lock in locks:
            found[f"{lock} ({package})"] = locked_version(root / lock, package)
    return found


def cmd_versions(args: argparse.Namespace) -> None:
    """Print the NCP version; fail unless every source agrees (and equals
    `--expect` when given)."""
    found = ncp_versions()
    values = set(found.values())
    expect = args.expect or (values.pop() if len(values) == 1 else None)
    wrong = {k: v for k, v in found.items() if v != expect}
    if expect is None or wrong:
        detail = ", ".join(f"{k} = {v}" for k, v in sorted(found.items()))
        raise ReleaseError(f"NCP versions disagree{' with ' + args.expect if args.expect else ''}: {detail}")
    print(expect)


def read_gbl(data: bytes) -> dict:
    """Records of a plain (unsigned, unencrypted) GBL v3 file, CRC checked:
    application data, program ranges and the ZRHP build metadata."""
    records = []
    offset = 0
    while offset + 8 <= len(data):
        tag, length = struct.unpack_from("<II", data, offset)
        body = data[offset + 8:offset + 8 + length]
        if len(body) != length:
            raise ReleaseError("GBL record runs past the end of the file")
        records.append((tag, offset, body))
        offset += 8 + length
        if tag == GBL_TAG_END:
            break
    # Simplicity Commander (and tools/efr32-gbl) pad the file, not a
    # record, to a multiple of four bytes.
    tail = data[offset:]
    if not records or records[-1][0] != GBL_TAG_END or len(tail) > 3 or tail.strip(b"\xff"):
        raise ReleaseError("GBL does not end with its END record")
    if records[0][0] != GBL_TAG_HEADER or struct.unpack_from("<II", records[0][2])[0] != GBL_VERSION:
        raise ReleaseError("GBL does not start with a version 3 header")
    gbl_type = struct.unpack_from("<II", records[0][2])[1]
    if gbl_type != 0:
        raise ReleaseError(f"GBL type {gbl_type:#x}: only plain GBLs are released")
    end_offset = records[-1][1]
    crc = struct.unpack_from("<I", records[-1][2])[0]
    if zlib.crc32(data[:end_offset + 8]) != crc:
        raise ReleaseError("GBL CRC mismatch")
    info = {"application": None, "programs": [], "zrhp": None}
    for tag, _, body in records[1:-1]:
        if tag == GBL_TAG_APPLICATION:
            app_type, version, capabilities = struct.unpack_from("<III", body)
            info["application"] = {"type": app_type, "version": version, "capabilities": capabilities}
        elif tag in (GBL_TAG_PROG, GBL_TAG_ERASEPROG):
            start = struct.unpack_from("<I", body)[0]
            info["programs"].append((start, start + len(body) - 4))
        elif tag == GBL_TAG_METADATA and body[:4] == b"ZRHP" and len(body) >= 14 and body[4] == 1:
            n = body[13]
            if len(body) == 14 + n:
                info["zrhp"] = {"build_id": body[5:13].hex(), "fw_version": body[14:].decode()}
        else:
            raise ReleaseError(f"GBL record {tag:#010x}: not released (signature, encryption, bootloader?)")
    return info


def gecko_version(word: int) -> str:
    """`ApplicationData_t.version` (`0xMMmmpp00`, EmberZNet style) as text."""
    return f"{word >> 24}.{(word >> 16) & 0xFF}.{(word >> 8) & 0xFF}"


def cmd_gecko(args: argparse.Namespace) -> None:
    """Describe one Gecko bootloader image (`gecko.json` in `--out`, a list
    the manifest step collects) and copy it next to that description."""
    if args.kind not in GECKO_KINDS:
        raise ReleaseError(f"kind {args.kind!r} is not one of {GECKO_KINDS}")
    src = Path(args.gbl)
    info = read_gbl(src.read_bytes())
    app = info["application"]
    if args.kind == "application":
        if app is None or info["zrhp"] is None:
            raise ReleaseError(f"{src.name}: an application GBL needs its application and ZRHP records")
        version = gecko_version(app["version"])
        fw = info["zrhp"]["fw_version"]
        if args.expect_version and (version != args.expect_version or fw != args.expect_version):
            raise ReleaseError(f"{src.name}: Gecko version {version}, ZRHP firmware {fw}, "
                               f"release {args.expect_version}")
    elif app is not None:
        raise ReleaseError(f"{src.name}: a maintenance GBL carries no application record")
    if any(start < 0x0800_6000 for start, _ in info["programs"]):
        raise ReleaseError(f"{src.name}: writes into the Gecko bootloader")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src, out / src.name)
    entry = {
        "id": args.id,
        "title": args.title,
        "chip_family": args.chip_family,
        "kind": args.kind,
        "install": args.install,
        "program": [{"start": f"0x{a:08x}", "end": f"0x{b:08x}"} for a, b in info["programs"]],
        "file": file_entry(out / src.name, None),
    }
    if args.kind == "application":
        entry["app_version"] = gecko_version(app["version"])
        entry["build_id"] = info["zrhp"]["build_id"]
    listing = out / "gecko.json"
    entries = json.loads(listing.read_text()) if listing.exists() else []
    entries = [e for e in entries if e["id"] != args.id] + [entry]
    listing.write_text(json.dumps(entries, indent=2) + "\n")
    print(json.dumps(entry))


def cmd_assemble(args: argparse.Namespace) -> None:
    spec = board(args.board)
    chip = spec["chip"]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    stem = args.stem
    parts = read_partitions(PARTITIONS)
    otadata_off, otadata_len = parts["otadata"]
    app_off, app_slot = parts["ota_0"]
    common = ["--skip-update-check", "--chip", chip, "--flash-size", FLASH_SIZE.lower(),
              "--partition-table", str(PARTITIONS), "--target-app-partition", "ota_0", args.elf]
    app = out / f"{stem}-app.bin"
    merged = out / f"{stem}-merged.bin"
    table = out / f"{stem}-partition-table.bin"
    boot = out / f"{stem}-bootloader.bin"
    otadata = out / f"{stem}-otadata-blank.bin"
    run("espflash", "save-image", *common, str(app))
    run("espflash", "save-image", "--merge", *common, str(merged))
    run("espflash", "partition-table", "--to-binary", "--output", str(table), str(PARTITIONS))

    image = merged.read_bytes()
    boot.write_bytes(image[:esp_image_len(image, 0)])
    app_bytes = app.read_bytes()
    if image[app_off:app_off + len(app_bytes)] != app_bytes:
        raise ReleaseError(f"the app at 0x{app_off:x} differs from the merged image")
    if len(app_bytes) > app_slot:
        raise ReleaseError(f"app {len(app_bytes)} B exceeds the {app_slot} B slot")
    table_bytes = table.read_bytes()
    if image[PARTITION_TABLE_OFFSET:PARTITION_TABLE_OFFSET + len(table_bytes)] != table_bytes:
        raise ReleaseError("the partition table differs from the merged image")
    otadata.write_bytes(b"\xff" * otadata_len)

    version = app_version(app_bytes)
    if args.expect_version is not None and version != args.expect_version:
        raise ReleaseError(f"firmware reports version {version!r}, the tag says {args.expect_version!r}")

    entry = {
        "id": spec["id"],
        "title": spec["title"],
        "product": "zrhp-ncp",
        "firmware": "esp32-ncp",
        "chip": chip,
        "chip_family": spec["chip_family"],
        "flash_size": FLASH_SIZE,
        "layout": LAYOUT,
        "transport": spec["transport"],
        "connect": spec["connect"],
        "features": spec["features"],
        "app_version": version,
        "files": {
            "merged": file_entry(merged, 0),
            "bootloader": file_entry(boot, 0),
            "partition_table": file_entry(table, PARTITION_TABLE_OFFSET),
            "otadata": file_entry(otadata, otadata_off),
            "app": file_entry(app, app_off),
        },
        # Written by the web flasher, in this order. Everything else is kept
        # unless the flasher erases the chip first.
        "flash": ["bootloader", "partition_table", "otadata", "app"],
        "preserved": [{"name": "zbnv", "offset": parts["zbnv"][0], "size": parts["zbnv"][1],
                       "what": "Zigbee network, Trust Center keys, NCP identity"}],
        "fw_update": None,
    }
    (out / "board.json").write_text(json.dumps(entry, indent=2) + "\n")
    print(f"{stem}: app {version}, bootloader {boot.stat().st_size} B, "
          f"app {len(app_bytes)} B, merged {len(image)} B")


def cmd_sign(args: argparse.Namespace) -> None:
    """Wrap the app in the `FW_*` envelope (imgtool, P-256), refusing a key
    the firmware does not trust."""
    sys.path.insert(0, str(ROOT / "tools"))
    from bl702_ota import imgtool_public_xy  # noqa: E402
    from zrs_boot_ota import parse_xy  # noqa: E402

    directory = Path(args.dir)
    entry = json.loads((directory / "board.json").read_text())
    app = directory / entry["files"]["app"]["name"]
    signed = app.with_name(app.name.replace("-app.bin", "-fw-signed.bin"))
    imgtool = shutil.which("imgtool") or "imgtool"
    trusted = parse_xy("bootloaders/keys/own-ec-p256.xy.hex",
                       (ROOT / "bootloaders/keys/own-ec-p256.xy.hex").read_text())
    if imgtool_public_xy(imgtool, Path(args.key)) != trusted:
        raise ReleaseError("the signing key is not the key the NCP firmware trusts "
                           "(bootloaders/keys/own-ec-p256.xy.hex)")
    m = re.fullmatch(r"(\d+)\.(\d+)\.(\d+)(?:[-+].*)?", args.version)
    if m is None:
        raise ReleaseError(f"version {args.version!r} is not major.minor.patch")
    major, minor, patch = (int(x) for x in m.groups())
    slot = read_partitions(PARTITIONS)["ota_0"][1]
    run(imgtool, "sign", "--header-size", "0x20", "--pad-header", "--align", "4",
        "--slot-size", hex(slot), "--version", f"{major}.{minor}.{patch}+0",
        "--key", args.key, str(app), str(signed))
    run(imgtool, "verify", "--key", args.key, str(signed))
    envelope = signed.read_bytes()
    if envelope[0x20:0x20 + app.stat().st_size] != app.read_bytes():
        raise ReleaseError("the envelope does not carry the app image unchanged")
    entry["fw_update"] = {
        "signed": True,
        "file": file_entry(signed, None),
        "version_word": f"0x{major << 24 | minor << 16 | patch:08x}",
    }
    (directory / "board.json").write_text(json.dumps(entry, indent=2) + "\n")


def unsigned_update(entry: dict) -> dict:
    """Without a signing key the update file is the plain app image, which
    only development firmware (`dev-unsigned-fw`) accepts over `FW_*`."""
    major, minor, patch = (int(x) for x in re.match(r"(\d+)\.(\d+)\.(\d+)", entry["app_version"]).groups())
    return {
        "signed": False,
        "file": entry["files"]["app"],
        "version_word": f"0x{major << 24 | minor << 16 | patch:08x}",
    }


def zrhp_version() -> str:
    text = (ROOT / "zigbee-zrhp/src/lib.rs").read_text()
    major = re.search(r"PROTOCOL_MAJOR: u8 = (\d+);", text).group(1)
    minor = re.search(r"PROTOCOL_MINOR: u8 = (\d+);", text).group(1)
    return f"{major}.{minor}"


def cmd_manifest(args: argparse.Namespace) -> None:
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    boards = []
    for directory in map(Path, args.dirs):
        entry = json.loads((directory / "board.json").read_text())
        if entry["fw_update"] is None:
            entry["fw_update"] = unsigned_update(entry)
        if entry["app_version"] != args.version:
            raise ReleaseError(f"{entry['id']}: firmware {entry['app_version']} != release {args.version}")
        for item in [*entry["files"].values(), entry["fw_update"]["file"]]:
            src = directory / item["name"]
            if sha256(src) != item["sha256"]:
                raise ReleaseError(f"{item['name']}: SHA-256 changed since assembly")
            shutil.copyfile(src, out / item["name"])
        boards.append(entry)
    gecko = []
    for directory in map(Path, args.gecko or []):
        for entry in json.loads((directory / "gecko.json").read_text()):
            src = directory / entry["file"]["name"]
            if sha256(src) != entry["file"]["sha256"]:
                raise ReleaseError(f"{src.name}: SHA-256 changed since it was described")
            if entry["kind"] == "application" and entry["app_version"] != args.version:
                raise ReleaseError(f"{entry['id']}: firmware {entry['app_version']} != release {args.version}")
            shutil.copyfile(src, out / src.name)
            gecko.append(entry)
    manifest = {
        "schema": SCHEMA,
        "product": "zrhp-ncp",
        "name": "zigbee-rs ZRHP NCP",
        "repository": args.repository,
        "tag": args.tag,
        "version": args.version,
        "prerelease": "-" in args.version,
        "commit": args.commit,
        "build_id": args.commit[:16],
        "date": args.date or datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "zrhp": {"protocol": zrhp_version()},
        "toolchain": json.loads(args.toolchain) if args.toolchain else {},
        "boards": boards,
    }
    if gecko:
        # Not for the web flasher (it reads `boards` only): images for the
        # Gecko bootloader, e.g. through the hub's NCP update.
        manifest["gecko_images"] = gecko
    (out / "release-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    names = sorted(p.name for p in out.iterdir() if p.is_file() and p.name != "SHA256SUMS")
    (out / "SHA256SUMS").write_text("".join(f"{sha256(out / n)}  {n}\n" for n in names))
    print(json.dumps({"tag": args.tag, "boards": [b["id"] for b in boards],
                      "signed": [b["fw_update"]["signed"] for b in boards],
                      "gecko_images": [g["id"] for g in gecko]}))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("assemble")
    p.add_argument("--board", required=True)
    p.add_argument("--elf", required=True)
    p.add_argument("--stem", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--expect-version")
    p.set_defaults(func=cmd_assemble)
    p = sub.add_parser("sign")
    p.add_argument("--dir", required=True)
    p.add_argument("--key", required=True)
    p.add_argument("--version", required=True)
    p.set_defaults(func=cmd_sign)
    p = sub.add_parser("versions", help="check that every NCP version source agrees; print it")
    p.add_argument("--expect")
    p.set_defaults(func=cmd_versions)
    p = sub.add_parser("gecko", help="describe a Gecko bootloader image (GBL) for the release")
    p.add_argument("--gbl", required=True)
    p.add_argument("--id", required=True)
    p.add_argument("--title", required=True)
    p.add_argument("--kind", required=True)
    p.add_argument("--chip-family", default="EFR32MG24")
    p.add_argument("--install", required=True, help="how the image is installed (text)")
    p.add_argument("--out", required=True)
    p.add_argument("--expect-version")
    p.set_defaults(func=cmd_gecko)
    p = sub.add_parser("manifest")
    p.add_argument("--dirs", nargs="+", required=True)
    p.add_argument("--gecko", nargs="+", help="directories with a gecko.json from `gecko`")
    p.add_argument("--out", required=True)
    p.add_argument("--tag", required=True)
    p.add_argument("--version", required=True)
    p.add_argument("--commit", required=True)
    p.add_argument("--repository", default=os.environ.get("GITHUB_REPOSITORY", "faronov/zigbee-rs"))
    p.add_argument("--date")
    p.add_argument("--toolchain", help="JSON object with the pinned tool versions")
    p.set_defaults(func=cmd_manifest)
    p = sub.add_parser("boards", help="print the board matrix as JSON (workflow matrix)")
    p.set_defaults(func=lambda _: print(json.dumps(BOARDS)))
    args = parser.parse_args()
    try:
        args.func(args)
    except (ReleaseError, OSError, subprocess.CalledProcessError) as error:
        sys.exit(f"esp_release: {error}")


if __name__ == "__main__":
    main()
