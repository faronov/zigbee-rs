#!/usr/bin/env python3
"""Verify the BRD4181A application, persistence, RAM, and stack layout."""

import argparse
import struct
import sys
from pathlib import Path

FLASH_START = 0x00004000
FLASH_END = 0x0007C000
PERSISTENCE_END = 0x00080000
RAM_START = 0x20000000
RAM_END = 0x20010000
MIN_STACK_BYTES = 16 * 1024
EXPECTED_VECTOR_BYTES = (16 + 51) * 4

PT_LOAD = 1
SHT_SYMTAB = 2
SHT_NOBITS = 8
SHF_ALLOC = 1 << 1


def fail(message: str) -> None:
    raise ValueError(message)


def checked_slice(data: bytes, offset: int, size: int, description: str) -> bytes:
    end = offset + size
    if offset < 0 or end < offset or end > len(data):
        fail(f"{description} is outside the ELF")
    return data[offset:end]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("elf", type=Path)
    args = parser.parse_args()
    data = args.elf.read_bytes()

    if data[:6] != b"\x7fELF\x01\x01":
        fail("expected a 32-bit little-endian ELF")

    header = struct.unpack_from("<16sHHIIIIIHHHHHH", data, 0)
    entry = header[4]
    phoff, shoff = header[5], header[6]
    phentsize, phnum = header[9], header[10]
    shentsize, shnum, shstrndx = header[11], header[12], header[13]
    if phentsize < 32 or shentsize < 40 or shstrndx >= shnum:
        fail("invalid ELF table geometry")

    sections = []
    for index in range(shnum):
        fields = struct.unpack_from("<IIIIIIIIII", data, shoff + index * shentsize)
        sections.append(
            {
                "name_offset": fields[0],
                "type": fields[1],
                "flags": fields[2],
                "address": fields[3],
                "offset": fields[4],
                "size": fields[5],
                "link": fields[6],
                "entry_size": fields[9],
            }
        )

    shstr = sections[shstrndx]
    shstr_data = checked_slice(data, shstr["offset"], shstr["size"], "section names")

    def string_at(table: bytes, offset: int) -> str:
        if offset >= len(table):
            fail("string-table offset is out of range")
        end = table.find(b"\0", offset)
        if end < 0:
            fail("unterminated ELF string")
        return table[offset:end].decode("ascii")

    for section in sections:
        section["name"] = string_at(shstr_data, section["name_offset"])

    segments = []
    for index in range(phnum):
        fields = struct.unpack_from("<IIIIIIII", data, phoff + index * phentsize)
        segments.append(
            {
                "type": fields[0],
                "offset": fields[1],
                "vaddr": fields[2],
                "paddr": fields[3],
                "file_size": fields[4],
                "mem_size": fields[5],
            }
        )

    symbols = {}
    for section in sections:
        if section["type"] != SHT_SYMTAB:
            continue
        if section["link"] >= len(sections):
            fail("symbol string-table index is invalid")
        strings = sections[section["link"]]
        string_data = checked_slice(
            data, strings["offset"], strings["size"], "symbol strings"
        )
        entry_size = section["entry_size"] or 16
        for offset in range(
            section["offset"], section["offset"] + section["size"], entry_size
        ):
            name_offset, value, _, _, _, _ = struct.unpack_from("<IIIBBH", data, offset)
            name = string_at(string_data, name_offset)
            if name:
                symbols[name] = value

    def symbol(name: str) -> int:
        if name not in symbols:
            fail(f"required symbol {name} is missing")
        return symbols[name]

    vector = next(
        (section for section in sections if section["name"] == ".vector_table"), None
    )
    if (
        vector is None
        or vector["address"] != FLASH_START
        or vector["size"] < EXPECTED_VECTOR_BYTES
    ):
        fail("51-IRQ vector table is not present at 0x00004000")
    vector_data = checked_slice(data, vector["offset"], vector["size"], "vector table")
    initial_sp, reset_vector = struct.unpack_from("<II", vector_data)

    def vector_word(index: int) -> int:
        return struct.unpack_from("<I", vector_data, index * 4)[0]

    def require_handler(index: int, name: str) -> None:
        handler = vector_word(index)
        expected = symbol(name)
        if handler & 1 == 0 or (handler & ~1) != (expected & ~1):
            fail(
                f"vector {index} does not point to {name}: "
                f"0x{handler:08X} != 0x{expected:08X}"
            )

    stack_start = symbol("_stack_start")
    stack_end = symbol("_stack_end")
    reset = symbol("Reset")
    if initial_sp != stack_start:
        fail(
            f"initial SP 0x{initial_sp:08X} does not equal "
            f"_stack_start 0x{stack_start:08X}"
        )
    if stack_start != RAM_END or stack_start % 8:
        fail(f"_stack_start 0x{stack_start:08X} is not the aligned RAM end")
    if not (RAM_START <= stack_end <= stack_start) or stack_end % 4:
        fail(f"invalid _stack_end 0x{stack_end:08X}")
    stack_bytes = stack_start - stack_end
    if stack_bytes < MIN_STACK_BYTES:
        fail(
            f"linked stack is {stack_bytes} bytes; "
            f"minimum is {MIN_STACK_BYTES} bytes"
        )
    if entry != reset or reset_vector != reset:
        fail(
            f"entry/vector Reset mismatch: entry=0x{entry:08X} "
            f"vector=0x{reset_vector:08X} symbol=0x{reset:08X}"
        )
    if reset & 1 == 0 or not (FLASH_START <= (reset & ~1) < FLASH_END):
        fail(f"invalid Thumb reset vector 0x{reset:08X}")
    require_handler(3, "HardFault")
    require_handler(16 + 3, "GPIO_EVEN")
    require_handler(16 + 36, "FRC_PRI")

    load_ranges = []
    bss_bytes = 0
    ram_static_end = RAM_START
    for section in sections:
        size = section["size"]
        if not section["flags"] & SHF_ALLOC or size == 0:
            continue
        start = section["address"]
        end = start + size
        if FLASH_START <= start < FLASH_END:
            if end > FLASH_END:
                fail(f"{section['name']} enters persistence at 0x{end:08X}")
        elif RAM_START <= start < RAM_END:
            if end > RAM_END:
                fail(f"{section['name']} exceeds RAM at 0x{end:08X}")
            ram_static_end = max(ram_static_end, end)
            if section["type"] == SHT_NOBITS:
                bss_bytes += size
        else:
            fail(
                f"allocatable section {section['name']} has unsupported VMA "
                f"0x{start:08X}..0x{end:08X}"
            )

        if section["type"] == SHT_NOBITS:
            continue

        containing = [
            segment
            for segment in segments
            if segment["type"] == PT_LOAD
            and segment["vaddr"] <= start
            and end <= segment["vaddr"] + segment["mem_size"]
            and segment["offset"] <= section["offset"]
            and section["offset"] + size
            <= segment["offset"] + segment["file_size"]
        ]
        if len(containing) != 1:
            fail(f"cannot determine one load address for {section['name']}")
        segment = containing[0]
        load_start = segment["paddr"] + start - segment["vaddr"]
        load_end = load_start + size
        if load_start < FLASH_START or load_end > FLASH_END:
            fail(
                f"{section['name']} load 0x{load_start:08X}..0x{load_end:08X} "
                "is outside application flash"
            )
        load_ranges.append((load_start, load_end))

    for segment in segments:
        if segment["type"] != PT_LOAD or segment["file_size"] == 0:
            continue
        physical_start = segment["paddr"]
        physical_end = physical_start + segment["file_size"]
        if FLASH_START <= physical_start and physical_end <= FLASH_END:
            continue

        backs_allocated_data = any(
            section["flags"] & SHF_ALLOC
            and section["type"] != SHT_NOBITS
            and section["size"] != 0
            and segment["offset"] <= section["offset"]
            and section["offset"] + section["size"]
            <= segment["offset"] + segment["file_size"]
            for section in sections
        )
        # Some ELF layouts map the ELF/program headers at physical address
        # zero even though no allocatable section belongs to that segment.
        # Treat that metadata as metadata, not as bytes destined for flash.
        header_only = (
            physical_start == 0
            and segment["vaddr"] == 0
            and segment["offset"] == 0
            and not backs_allocated_data
        )
        if not header_only:
            fail(
                f"file-backed PT_LOAD 0x{physical_start:08X}.."
                f"0x{physical_end:08X} is outside application flash"
            )

    if not load_ranges:
        fail("ELF has no allocatable file-backed firmware")
    first_load = min(start for start, _ in load_ranges)
    highest_load = max(end for _, end in load_ranges)
    loaded_bytes = sum(end - start for start, end in load_ranges)
    if first_load != FLASH_START:
        fail(f"first firmware load is 0x{first_load:08X}, expected 0x00004000")
    if ram_static_end > stack_end:
        fail(
            f"RAM allocations end at 0x{ram_static_end:08X}, "
            f"above _stack_end 0x{stack_end:08X}"
        )

    print(f"PASS: {args.elf}")
    print(
        f"  load=0x{first_load:08X}..0x{highest_load:08X} "
        f"(span={highest_load - first_load} bytes, file-backed={loaded_bytes} bytes)"
    )
    print(
        f"  persistence=0x{FLASH_END:08X}..0x{PERSISTENCE_END:08X} "
        "(2 x 8192-byte sectors)"
    )
    print(
        f"  BSS={bss_bytes} bytes RAM-static-end=0x{ram_static_end:08X} "
        f"stack={stack_bytes} bytes"
    )
    print(
        f"  SP=0x{initial_sp:08X} Reset=0x{reset_vector:08X} "
        f"vector-bytes={vector['size']}"
    )


if __name__ == "__main__":
    try:
        main()
    except (OSError, UnicodeDecodeError, ValueError, struct.error) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
