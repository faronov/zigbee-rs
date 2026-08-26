#!/usr/bin/env python3
"""Verify an ESP application image and its merged 4 MiB flash image."""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import struct
import sys
from dataclasses import dataclass

FLASH_SIZE = 0x0040_0000
PARTITION_TABLE_OFFSET = 0x0000_8000
PARTITION_ENTRY_SIZE = 32
ESP_IMAGE_MAGIC = 0xE9
ESP_DIGEST_BYTES = 32
CHIP_IDS = {"esp32c6": 0x000D, "esp32h2": 0x0010}

EXPECTED_LAYOUT = {
    "otadata": (0x0000_9000, 0x0000_2000),
    "ota_0": (0x0001_0000, 0x001F_0000),
    "ota_1": (0x0020_0000, 0x001F_0000),
    "zbnv": (0x003F_0000, 0x0001_0000),
}

PARTITION_TYPES = {
    "otadata": (0x01, 0x00),
    "ota_0": (0x00, 0x10),
    "ota_1": (0x00, 0x11),
    "zbnv": (0x01, 0x06),
}


@dataclass(frozen=True)
class Partition:
    name: str
    offset: int
    size: int

    @property
    def end(self) -> int:
        return self.offset + self.size


def load_partitions() -> dict[str, Partition]:
    csv = pathlib.Path(__file__).with_name("esp32-4mb-ota.csv")
    partitions: dict[str, Partition] = {}
    for raw_line in csv.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        cells = [cell.strip() for cell in line.split(",")]
        name = cells[0]
        partitions[name] = Partition(name, int(cells[3], 0), int(cells[4], 0))

    if set(partitions) != set(EXPECTED_LAYOUT):
        raise SystemExit(
            f"partition names {sorted(partitions)} != {sorted(EXPECTED_LAYOUT)}"
        )
    for name, expected in EXPECTED_LAYOUT.items():
        partition = partitions[name]
        actual = (partition.offset, partition.size)
        if actual != expected:
            raise SystemExit(f"{name} is {actual!r}, expected {expected!r}")

    ordered = sorted(partitions.values(), key=lambda partition: partition.offset)
    previous_end = PARTITION_TABLE_OFFSET + 0x0C00
    for partition in ordered:
        if partition.offset < previous_end:
            raise SystemExit(f"{partition.name} overlaps the preceding flash region")
        if partition.end > FLASH_SIZE:
            raise SystemExit(f"{partition.name} ends beyond the 4 MiB flash")
        previous_end = partition.end
    if partitions["ota_0"].end != partitions["ota_1"].offset:
        raise SystemExit("ota_0 does not end at the ota_1 physical boundary")
    if partitions["ota_1"].end != partitions["zbnv"].offset:
        raise SystemExit("ota_1 does not end at the zbnv physical boundary")
    if partitions["zbnv"].end != FLASH_SIZE:
        raise SystemExit("zbnv does not end at the physical flash boundary")
    return partitions


def check_application_image(path: pathlib.Path, chip: str, slot: Partition) -> bytes:
    image = path.read_bytes()
    if len(image) < 24 + ESP_DIGEST_BYTES or image[0] != ESP_IMAGE_MAGIC:
        raise SystemExit(f"{path}: not an ESP application image")
    if image[1] == 0:
        raise SystemExit(f"{path}: image declares no loadable segments")
    chip_id = struct.unpack_from("<H", image, 12)[0]
    if chip_id != CHIP_IDS[chip]:
        raise SystemExit(
            f"{path}: chip id 0x{chip_id:04X}, expected 0x{CHIP_IDS[chip]:04X}"
        )
    if image[23] != 1:
        raise SystemExit(f"{path}: image does not carry an appended SHA-256")
    if hashlib.sha256(image[:-ESP_DIGEST_BYTES]).digest() != image[-ESP_DIGEST_BYTES:]:
        raise SystemExit(f"{path}: appended SHA-256 does not match")
    if slot.offset + len(image) > slot.end:
        raise SystemExit(
            f"{path}: physical range 0x{slot.offset:06X}.."
            f"0x{slot.offset + len(image):06X} exceeds {slot.name} end "
            f"0x{slot.end:06X}"
        )
    return image


def check_partition_entries(merged: bytes) -> None:
    for index, name in enumerate(EXPECTED_LAYOUT):
        start = PARTITION_TABLE_OFFSET + index * PARTITION_ENTRY_SIZE
        entry = merged[start : start + PARTITION_ENTRY_SIZE]
        if len(entry) != PARTITION_ENTRY_SIZE:
            raise SystemExit("merged image truncates the partition table")
        magic, kind, subtype, offset, size, raw_name, flags = struct.unpack(
            "<HBBII16sI", entry
        )
        label = raw_name.split(b"\0", 1)[0].decode("ascii")
        expected_kind, expected_subtype = PARTITION_TYPES[name]
        expected_offset, expected_size = EXPECTED_LAYOUT[name]
        actual = (magic, kind, subtype, offset, size, label, flags)
        expected = (
            0x50AA,
            expected_kind,
            expected_subtype,
            expected_offset,
            expected_size,
            name,
            0,
        )
        if actual != expected:
            raise SystemExit(
                f"merged partition entry {index} is {actual!r}, expected {expected!r}"
            )


def check_merged_image(
    path: pathlib.Path, chip: str, slot: Partition, application: bytes
) -> None:
    merged = path.read_bytes()
    expected_bytes = slot.offset + len(application)
    if len(merged) != expected_bytes:
        raise SystemExit(
            f"{path}: merged image is {len(merged)} bytes, expected exactly "
            f"0x{slot.offset:06X} + {len(application)} = {expected_bytes}"
        )
    if merged[0] != ESP_IMAGE_MAGIC:
        raise SystemExit(f"{path}: merged image is missing the ESP bootloader")
    bootloader_chip_id = struct.unpack_from("<H", merged, 12)[0]
    if bootloader_chip_id != CHIP_IDS[chip]:
        raise SystemExit(
            f"{path}: bootloader chip id 0x{bootloader_chip_id:04X}, "
            f"expected 0x{CHIP_IDS[chip]:04X}"
        )
    check_partition_entries(merged)
    if merged[slot.offset:] != application:
        raise SystemExit(
            f"{path}: bytes at physical {slot.name} offset "
            f"0x{slot.offset:06X} differ from the application image"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chip", required=True, choices=sorted(CHIP_IDS))
    parser.add_argument("--slot", default="ota_0", choices=("ota_0", "ota_1"))
    parser.add_argument("--merged", required=True, type=pathlib.Path)
    parser.add_argument("application", type=pathlib.Path)
    args = parser.parse_args()

    partitions = load_partitions()
    slot = partitions[args.slot]
    image = check_application_image(args.application, args.chip, slot)
    check_merged_image(args.merged, args.chip, slot, image)
    print(
        f"{args.application}: {len(image)}/{slot.size} bytes; physical "
        f"0x{slot.offset:06X}..0x{slot.offset + len(image):06X} "
        f"within {args.slot} 0x{slot.offset:06X}..0x{slot.end:06X}"
    )
    print(
        f"{args.merged}: bootloader + checked partition table + {args.slot} "
        f"({args.merged.stat().st_size} bytes at flash offset 0)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
