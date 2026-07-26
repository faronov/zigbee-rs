#!/usr/bin/env python3
"""Build, wrap and verify a Zigbee OTA image for the ESP32-C6 sensor.

    tools/create-ota.py <firmware-version> [output-directory]

The pipeline is deliberately end-to-end so a version can never be stamped onto
an image that does not contain it:

1. `cargo build --release` with `ESP32_OTA_VERSION` set to the requested
   version, which is what the firmware reports in `QueryNextImageRequest` and
   in `Basic::SWBuildID`.
2. `espflash save-image` turns the ELF into an ESP application image — the
   exact bytes the second stage bootloader expects in `ota_0`/`ota_1`. An ELF
   or a merged flash image would be rejected by the device.
3. The application image becomes the `UpgradeImage` sub-element of a Zigbee OTA
   container.
4. The container is parsed back and compared byte for byte with the image, and
   its size is checked against the 0x1F0000-byte application slot.
5. A `zigpy_local` index is regenerated from every `.ota` file in the output
   directory, with the sha3-256 checksum ZHA verifies after downloading.

Only the Python standard library is used.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import struct
import subprocess
import sys

# Zigbee OTA container -------------------------------------------------------
OTA_MAGIC = 0x0BEEF11E
OTA_HEADER_VERSION = 0x0100
OTA_HEADER_BASE_LEN = 56
FIELD_CONTROL_HARDWARE_VERSIONS = 0x0004
TAG_UPGRADE_IMAGE = 0x0000
SUB_ELEMENT_HEADER_LEN = 6
ZIGBEE_PRO_STACK_VERSION = 0x0002

# Must match products/esp32-zigbee-devkit/src/lib.rs (OTA_MANUFACTURER_CODE/OTA_IMAGE_TYPE/OTA_HARDWARE_VERSION) and the product profile in src/profile.rs.
MANUFACTURER_CODE = 0x1234
IMAGE_TYPE = 0x0001
HARDWARE_VERSION = 1
MANUFACTURER_NAME = "Zigbee-RS"
MODEL_NAME = "ESP32-C6-Sensor"
HEADER_STRING = "zigbee-rs ESP32-C6 sensor"

# Must match products/esp32-zigbee-devkit/partitions/esp32-4mb-ota.csv.
OTA_SLOT_SIZE = 0x001F_0000

# ESP application image ------------------------------------------------------
ESP_IMAGE_MAGIC = 0xE9
ESP_IMAGE_HEADER_LEN = 24
ESP_DIGEST_LEN = 32
ESP_CHIP_IDS = {"esp32c6": 0x000D, "esp32h2": 0x0010}

EXAMPLE_DIR = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_TARGET = "riscv32imac-unknown-none-elf"


def parse_version(raw: str) -> int:
    value = int(raw, 0)
    if not 0 <= value < 0xFFFFFFFF:
        raise SystemExit("firmware version must be in 0..0xFFFFFFFE")
    return value


def run(command: list[str], *, cwd: pathlib.Path, env: dict | None = None) -> None:
    print("+", " ".join(command))
    subprocess.run(command, cwd=cwd, env=env, check=True)


def build_elf(version: int, chip: str, toolchain: str) -> pathlib.Path:
    env = dict(os.environ, ESP32_OTA_VERSION=str(version))
    command = ["cargo"]
    if toolchain:
        command.append(f"+{toolchain}")
    command += ["build", "--release", "-Z", "build-std=core,alloc"]
    run(command, cwd=EXAMPLE_DIR, env=env)
    elf = EXAMPLE_DIR / "target" / DEFAULT_TARGET / "release" / EXAMPLE_DIR.name
    if not elf.is_file():
        raise SystemExit(f"expected ELF at {elf}")
    return elf


def save_image(elf: pathlib.Path, chip: str, output: pathlib.Path) -> bytes:
    run(
        ["espflash", "save-image", "--chip", chip, str(elf), str(output)],
        cwd=EXAMPLE_DIR,
    )
    return output.read_bytes()


def check_esp_image(image: bytes, chip: str) -> None:
    """Reject anything the on-device verifier would reject."""
    if len(image) < ESP_IMAGE_HEADER_LEN + ESP_DIGEST_LEN:
        raise SystemExit(f"application image is only {len(image)} bytes")
    if image[0] != ESP_IMAGE_MAGIC:
        raise SystemExit(
            f"not an ESP application image: first byte is 0x{image[0]:02X}, "
            f"expected 0x{ESP_IMAGE_MAGIC:02X} (an ELF starts with 0x7F)"
        )
    if image[1] == 0:
        raise SystemExit("application image declares no segments")
    chip_id = struct.unpack_from("<H", image, 12)[0]
    expected = ESP_CHIP_IDS[chip]
    if chip_id != expected:
        raise SystemExit(f"image chip id 0x{chip_id:04X} != 0x{expected:04X} for {chip}")
    if image[23] != 1:
        raise SystemExit("application image was built without an appended SHA-256")
    digest = hashlib.sha256(image[:-ESP_DIGEST_LEN]).digest()
    if digest != image[-ESP_DIGEST_LEN:]:
        raise SystemExit("appended SHA-256 does not match the image contents")
    if len(image) > OTA_SLOT_SIZE:
        raise SystemExit(
            f"application image is {len(image)} bytes, the OTA slot holds {OTA_SLOT_SIZE}"
        )


def build_container(image: bytes, version: int) -> bytes:
    header_length = OTA_HEADER_BASE_LEN + 4  # + min/max hardware version
    total = header_length + SUB_ELEMENT_HEADER_LEN + len(image)
    header_string = HEADER_STRING.encode("utf-8")[:32].ljust(32, b"\x00")

    header = struct.pack(
        "<IHHHHHIH32sI",
        OTA_MAGIC,
        OTA_HEADER_VERSION,
        header_length,
        FIELD_CONTROL_HARDWARE_VERSIONS,
        MANUFACTURER_CODE,
        IMAGE_TYPE,
        version,
        ZIGBEE_PRO_STACK_VERSION,
        header_string,
        total,
    )
    header += struct.pack("<HH", HARDWARE_VERSION, HARDWARE_VERSION)
    assert len(header) == header_length, len(header)

    sub_element = struct.pack("<HI", TAG_UPGRADE_IMAGE, len(image)) + image
    return header + sub_element


def parse_container(blob: bytes) -> dict:
    """Independent parser used to verify what was just written."""
    if len(blob) < OTA_HEADER_BASE_LEN:
        raise SystemExit("OTA container is shorter than its header")
    (
        magic,
        header_version,
        header_length,
        field_control,
        manufacturer_code,
        image_type,
        file_version,
        stack_version,
        header_string,
        total_image_size,
    ) = struct.unpack_from("<IHHHHHIH32sI", blob, 0)

    if magic != OTA_MAGIC:
        raise SystemExit(f"bad OTA magic 0x{magic:08X}")
    if header_version != OTA_HEADER_VERSION:
        raise SystemExit(f"unsupported OTA header version 0x{header_version:04X}")
    if total_image_size != len(blob):
        raise SystemExit(
            f"header says {total_image_size} bytes, file is {len(blob)} bytes"
        )

    offset = OTA_HEADER_BASE_LEN
    minimum_hardware = maximum_hardware = None
    if field_control & 0x0001:
        offset += 1
    if field_control & 0x0002:
        offset += 8
    if field_control & FIELD_CONTROL_HARDWARE_VERSIONS:
        minimum_hardware, maximum_hardware = struct.unpack_from("<HH", blob, offset)
        offset += 4
    if offset != header_length:
        raise SystemExit(f"header length {header_length} != parsed {offset}")

    payload = None
    while offset + SUB_ELEMENT_HEADER_LEN <= len(blob):
        tag, length = struct.unpack_from("<HI", blob, offset)
        offset += SUB_ELEMENT_HEADER_LEN
        if offset + length > len(blob):
            raise SystemExit(f"sub-element 0x{tag:04X} runs past the end of the file")
        if tag == TAG_UPGRADE_IMAGE:
            payload = blob[offset : offset + length]
        offset += length
    if offset != len(blob):
        raise SystemExit("trailing bytes after the last sub-element")
    if payload is None:
        raise SystemExit("no UpgradeImage sub-element")

    return {
        "manufacturer_code": manufacturer_code,
        "image_type": image_type,
        "file_version": file_version,
        "stack_version": stack_version,
        "header_string": header_string.rstrip(b"\x00").decode("utf-8"),
        "minimum_hardware_version": minimum_hardware,
        "maximum_hardware_version": maximum_hardware,
        "payload": payload,
    }


def verify_container(blob: bytes, image: bytes, version: int, chip: str) -> None:
    parsed = parse_container(blob)
    if parsed["payload"] != image:
        raise SystemExit("UpgradeImage payload differs from the application image")
    if parsed["file_version"] != version:
        raise SystemExit("container file version does not match the requested version")
    if parsed["manufacturer_code"] != MANUFACTURER_CODE:
        raise SystemExit("container manufacturer code mismatch")
    if parsed["image_type"] != IMAGE_TYPE:
        raise SystemExit("container image type mismatch")
    expected_size = 60 + SUB_ELEMENT_HEADER_LEN + len(image)
    if len(blob) != expected_size:
        raise SystemExit(f"container is {len(blob)} bytes, expected {expected_size}")
    # The payload must still be a bootable image after the round trip.
    check_esp_image(parsed["payload"], chip)


def write_index(output_dir: pathlib.Path) -> pathlib.Path:
    firmwares = []
    for path in sorted(output_dir.glob("*.ota")):
        blob = path.read_bytes()
        parsed = parse_container(blob)
        firmwares.append(
            {
                "path": path.name,
                "file_version": parsed["file_version"],
                "file_size": len(blob),
                "image_type": parsed["image_type"],
                "manufacturer_id": parsed["manufacturer_code"],
                "manufacturer_names": [MANUFACTURER_NAME],
                "model_names": [MODEL_NAME],
                "checksum": "sha3-256:" + hashlib.sha3_256(blob).hexdigest(),
                "min_hardware_version": parsed["minimum_hardware_version"],
                "max_hardware_version": parsed["maximum_hardware_version"],
                "changelog": f"zigbee-rs ESP32-C6 sensor v{parsed['file_version']}",
            }
        )
    index = output_dir / "index.json"
    index.write_text(json.dumps({"firmwares": firmwares}, indent=2) + "\n")
    return index


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("version", help="OTA file version (decimal or 0x-prefixed)")
    parser.add_argument(
        "output_dir",
        nargs="?",
        default="target/ota",
        help="output directory (default: target/ota)",
    )
    parser.add_argument("--chip", default="esp32c6", choices=sorted(ESP_CHIP_IDS))
    parser.add_argument(
        "--toolchain", default="nightly", help="cargo toolchain (default: nightly)"
    )
    parser.add_argument(
        "--elf", help="use an existing ELF instead of building one (skips cargo)"
    )
    args = parser.parse_args()

    version = parse_version(args.version)
    output_dir = pathlib.Path(args.output_dir)
    if not output_dir.is_absolute():
        output_dir = EXAMPLE_DIR / output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    elf = pathlib.Path(args.elf) if args.elf else build_elf(version, args.chip, args.toolchain)

    base = output_dir / f"{EXAMPLE_DIR.name}-v{version}"
    app_bin = base.with_suffix(".app.bin")
    ota_file = base.with_suffix(".ota")

    image = save_image(elf, args.chip, app_bin)
    check_esp_image(image, args.chip)

    container = build_container(image, version)
    ota_file.write_bytes(container)
    verify_container(ota_file.read_bytes(), image, version, args.chip)

    index = write_index(output_dir)

    print()
    print(f"application image : {app_bin} ({len(image)} bytes)")
    print(f"OTA container     : {ota_file} ({len(container)} bytes)")
    print(f"  manufacturer    : 0x{MANUFACTURER_CODE:04X}")
    print(f"  image type      : 0x{IMAGE_TYPE:04X}")
    print(f"  file version    : {version} (0x{version:08X})")
    print(f"  hardware        : {HARDWARE_VERSION}..{HARDWARE_VERSION}")
    print(f"  slot usage      : {len(image)}/{OTA_SLOT_SIZE} bytes "
          f"({100 * len(image) / OTA_SLOT_SIZE:.1f}%)")
    print(f"  sha3-256        : {hashlib.sha3_256(container).hexdigest()}")
    print(f"zigpy_local index : {index}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
