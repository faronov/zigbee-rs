#!/usr/bin/env python3
"""ZRHP NCP release images: assemble, sign and describe them.

Used by `.github/workflows/ncp-release.yml` (tagged GitHub releases) and
`ncp-release-artifacts.yml` (ad-hoc CI artifacts). Board definitions live in
`boards.json` next to this file. The release manifest format is the contract
with the web flasher (`tools/flasher_site.py`); it is documented in the book
(`docs/book/src/getting-started/web-flasher.md`).

    esp_release.py assemble --board h2-usj --elf ELF --stem STEM --out DIR [--expect-version V]
    esp_release.py sign --dir DIR --key PEM --version V
    esp_release.py manifest --dirs DIR... --out DIR --tag TAG --version V --commit SHA ...
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
import subprocess
import sys

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
    (out / "release-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    names = sorted(p.name for p in out.iterdir() if p.is_file() and p.name != "SHA256SUMS")
    (out / "SHA256SUMS").write_text("".join(f"{sha256(out / n)}  {n}\n" for n in names))
    print(json.dumps({"tag": args.tag, "boards": [b["id"] for b in boards],
                      "signed": [b["fw_update"]["signed"] for b in boards]}))


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
    p = sub.add_parser("manifest")
    p.add_argument("--dirs", nargs="+", required=True)
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
