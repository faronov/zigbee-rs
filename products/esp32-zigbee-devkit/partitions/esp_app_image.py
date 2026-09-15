"""Shared preflight for unsigned, plaintext ESP32-C6/H2 application images.

Mirrors products/esp32-zigbee-devkit/src/esp_image.rs, based on ESP-IDF
v5.5.1 esp_image_format.c, bootloader_common_loader.c and C6/H2 soc.h.
Both artifact sizing and Zigbee OTA packaging must use this validator.

This is structural/integrity validation, NOT signature verification,
anti-rollback enforcement or proof of bootability. Offline tools do not know
the receiving device's silicon/eFuse revisions. Declared limits are returned;
actual compatibility remains the device's responsibility unless revisions are
explicitly supplied here. No revision is guessed from the image itself.
"""

from __future__ import annotations

import hashlib
import struct
from dataclasses import dataclass
from functools import reduce
from operator import xor

CHIP_IDS = {"esp32c6": 0x000D, "esp32h2": 0x0010}
HEADER_SIZE = 24
DIGEST_SIZE = 32
IMAGE_MAGIC = 0xE9
SEGMENT_HEADER_SIZE = 8
APP_DESC_SIZE = 256
APP_DESC_MAGIC = 0xABCD5432
MAX_SEGMENTS = 16
MIN_IMAGE_SIZE = (HEADER_SIZE + SEGMENT_HEADER_SIZE + APP_DESC_SIZE + 16 + DIGEST_SIZE) & ~15


class ImageValidationError(ValueError):
    def __init__(self, code: str, detail: str):
        self.code = code
        super().__init__(f"{code}: {detail}")


@dataclass(frozen=True)
class ImageInfo:
    segment_count: int
    mmu_page_size: int
    chip_revision_range: tuple[int, int]
    efuse_block_revision_range: tuple[int, int]
    digest: bytes


def _revision_range(
    minimum: int, maximum: int, actual: int | None, code: str
) -> None:
    if maximum not in (0, 0xFFFF) and minimum > maximum:
        raise ImageValidationError(code, f"empty revision range {minimum}..{maximum}")
    if actual is not None:
        if not 0 <= actual <= 0xFFFF:
            raise ValueError("actual revision must be in 0..65535 (major * 100 + minor)")
        if actual < minimum or (maximum not in (0, 0xFFFF) and actual > maximum):
            raise ImageValidationError(
                code, f"revision {actual} is outside {minimum}..{maximum}"
            )


def validate_application(
    image: bytes,
    chip_id: int,
    partition_offset: int,
    slot_size: int,
    *,
    chip_revision: int | None = None,
    efuse_block_revision: int | None = None,
) -> ImageInfo:
    """Check complete image bytes against the physical destination partition.

    An unset maximum (0 or 0xffff) means no upper bound. The descriptor's
    0xffff eFuse minimum is also unset; the image header's chip minimum is not.
    Explicit revisions use IDF's major * 100 + minor encoding and, like the
    Rust validator, conservatively enforce maxima even if an eFuse waiver
    could disable the bootloader's maximum-revision check.
    """
    if not 0 <= partition_offset <= 0xFFFFFFFF or not 0 < slot_size <= 0xFFFFFFFF:
        raise ValueError("invalid physical partition offset/size")
    size = len(image)
    if size > slot_size or partition_offset + size > 0xFFFFFFFF:
        raise ImageValidationError(
            "ImageLength", f"{size} bytes exceed the physical slot at 0x{partition_offset:X}"
        )
    if size < MIN_IMAGE_SIZE:
        raise ImageValidationError("TooSmall", f"application image is only {size} bytes")
    if image[0] != IMAGE_MAGIC:
        raise ImageValidationError("BadMagic", "not an ESP application image")
    count = image[1]
    if count == 0:
        raise ImageValidationError("NoSegments", "image declares no segments")
    if count > MAX_SEGMENTS:
        raise ImageValidationError("TooManySegments", f"{count} exceeds {MAX_SEGMENTS}")
    found_chip = struct.unpack_from("<H", image, 12)[0]
    if found_chip != chip_id or chip_id not in CHIP_IDS.values():
        raise ImageValidationError(
            "ChipMismatch", f"chip ID 0x{found_chip:04X}, expected 0x{chip_id:04X}"
        )
    if image[23] != 1:
        raise ImageValidationError("NoAppendedHash", "appended SHA-256 is required")
    ram_end, rtc_end = (
        (0x40880000, 0x50004000) if chip_id == CHIP_IDS["esp32c6"]
        else (0x40850000, 0x50001000)
    )
    entry = struct.unpack_from("<I", image, 4)[0]
    chip_range = struct.unpack_from("<HH", image, 15)
    _revision_range(*chip_range, chip_revision, "ChipRevision")
    hash_end = size - DIGEST_SIZE
    cursor = HEADER_SIZE
    checksum = 0xEF
    page_size = 0
    block_range = (0, 0)
    segments: list[tuple[int, int, int, bool]] = []
    entry_found = False
    data = memoryview(image)

    for index in range(count):
        data_offset = cursor + SEGMENT_HEADER_SIZE
        if data_offset > hash_end:
            raise ImageValidationError("SegmentBounds", f"missing segment {index} header")
        load, length = struct.unpack_from("<II", image, cursor)
        if length % 4 or length >= 0x1000000:
            raise ImageValidationError("SegmentLength", f"segment {index} length 0x{length:X}")
        end = data_offset + length
        if end > hash_end:
            raise ImageValidationError("SegmentBounds", f"segment {index} exceeds image data")
        load_end = load + length
        if load_end > 0xFFFFFFFF:
            raise ImageValidationError("LoadAddress", f"segment {index} load range overflows")
        if index == 0:
            if length < APP_DESC_SIZE:
                raise ImageValidationError("AppDescriptor", "first segment is too short")
            magic, secure_version = struct.unpack_from("<II", image, data_offset)
            if magic != APP_DESC_MAGIC:
                raise ImageValidationError("AppDescriptor", "missing application descriptor")
            if secure_version != 0:
                raise ImageValidationError(
                    "UnsupportedSecureVersion", "secure-version policy is not supported"
                )
            minimum, maximum = struct.unpack_from("<HH", image, data_offset + 176)
            block_range = (0 if minimum == 0xFFFF else minimum, maximum)
            _revision_range(*block_range, efuse_block_revision, "EfuseBlockRevision")
            page_log2 = image[data_offset + 180]
            if page_log2 not in (0, 13, 14, 15, 16):
                raise ImageValidationError(
                    "UnsupportedMmuPageSize", f"descriptor page-size exponent {page_log2}"
                )
            page_size = 1 << (page_log2 or 16)

        mapped = 0x42000000 <= load < 0x42000000 + page_size * 256
        if index == 0 and not mapped:
            raise ImageValidationError("AppDescriptor", "first segment must be flash-mapped")
        if load != 0 and length != 0:
            valid = (
                load_end <= 0x42000000 + page_size * 256 if mapped else
                (0x40800000 <= load and load_end <= ram_end)
                or (0x50000000 <= load and load_end <= rtc_end)
            )
            if not valid or load % 4:
                raise ImageValidationError("LoadAddress", f"invalid segment {index} load range")
            if mapped and (
                load % page_size != (partition_offset + data_offset) % page_size
                or partition_offset + end > page_size * 256
            ):
                raise ImageValidationError("MappingAlignment", f"invalid segment {index} mapping")
            for previous, previous_end, previous_offset, previous_mapped in segments:
                if previous == 0 or previous == previous_end:
                    continue
                if load < previous_end and previous < load_end:
                    raise ImageValidationError("OverlappingSegments", f"segment {index} overlaps")
                if (
                    mapped and previous_mapped
                    and load // page_size <= (previous_end - 1) // page_size
                    and previous // page_size <= (load_end - 1) // page_size
                    and load - data_offset != previous - previous_offset
                ):
                    raise ImageValidationError(
                        "MappingAlignment", f"segment {index} conflicts on a shared MMU page"
                    )
            executable_start = load + (APP_DESC_SIZE if index == 0 else 0)
            entry_found |= executable_start <= entry < load_end and entry % 2 == 0
        segments.append((load, load_end, data_offset, mapped))
        checksum = reduce(xor, data[data_offset:end], checksum)
        cursor = end

    if not entry_found:
        raise ImageValidationError("EntryAddress", "entry is outside loaded/mapped program data")
    digest_offset = (cursor + 16) & ~15
    if digest_offset != hash_end:
        raise ImageValidationError("ImageLength", "checksum/hash trailer or image length is invalid")
    if image[digest_offset - 1] != checksum:
        raise ImageValidationError("Checksum", "segment XOR checksum does not match")
    digest = hashlib.sha256(data[:digest_offset]).digest()
    if digest != image[digest_offset:]:
        raise ImageValidationError("Hash", "appended SHA-256 does not match")
    return ImageInfo(count, page_size, chip_range, block_range, digest)
