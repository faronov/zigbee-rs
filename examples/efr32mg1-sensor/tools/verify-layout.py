#!/usr/bin/env python3
"""Verify the bootloader-safe EFR32MG1 application ELF layout."""

import argparse
import struct
import sys
from pathlib import Path

FLASH_START = 0x00004000
FLASH_END = 0x0003A000
RAM_START = 0x20000000
RAM_END = 0x20007C00
APP_PROPERTIES_MAGIC = bytes.fromhex(
    "13 b7 79 fa c9 25 dd b7 ad f3 cf e0 f1 b6 14 b8"
)


def fail(message: str) -> None:
    raise ValueError(message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("elf", type=Path)
    args = parser.parse_args()
    data = args.elf.read_bytes()

    if data[:4] != b"\x7fELF" or data[4:6] != b"\x01\x01":
        fail("expected a 32-bit little-endian ELF")

    header = struct.unpack_from("<16sHHIIIIIHHHHHH", data, 0)
    entry = header[4]
    phoff, shoff = header[5], header[6]
    phentsize, phnum = header[9], header[10]
    shentsize, shnum, shstrndx = header[11], header[12], header[13]

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
    shstr_data = data[shstr["offset"] : shstr["offset"] + shstr["size"]]

    def string_at(table: bytes, offset: int) -> str:
        end = table.find(b"\0", offset)
        return table[offset:end].decode("ascii")

    for section in sections:
        section["name"] = string_at(shstr_data, section["name_offset"])

    symbols = {}
    for section in sections:
        if section["type"] != 2:  # SHT_SYMTAB
            continue
        strings = sections[section["link"]]
        string_data = data[strings["offset"] : strings["offset"] + strings["size"]]
        entry_size = section["entry_size"] or 16
        for offset in range(section["offset"], section["offset"] + section["size"], entry_size):
            name_offset, value, size, info, other, section_index = struct.unpack_from(
                "<IIIBBH", data, offset
            )
            del size, info, other, section_index
            name = string_at(string_data, name_offset)
            if name:
                symbols[name] = value

    vector = next((section for section in sections if section["name"] == ".vector_table"), None)
    if vector is None or vector["address"] != FLASH_START or vector["size"] < 0x40:
        fail("vector table is not present at 0x4000")
    vector_data = data[vector["offset"] : vector["offset"] + vector["size"]]
    initial_sp, reset_vector = struct.unpack_from("<II", vector_data)
    properties_pointer = struct.unpack_from("<I", vector_data, 0x34)[0]

    if not (RAM_START <= initial_sp <= RAM_END) or initial_sp % 8:
        fail(f"invalid initial SP 0x{initial_sp:08X}")
    if not (FLASH_START <= (reset_vector & ~1) < FLASH_END) or reset_vector & 1 == 0:
        fail(f"invalid reset vector 0x{reset_vector:08X}")
    if properties_pointer != symbols.get("APP_PROPERTIES"):
        fail(
            "vector word 13 does not equal APP_PROPERTIES "
            f"(0x{properties_pointer:08X} != 0x{symbols.get('APP_PROPERTIES', 0):08X})"
        )

    def bytes_at_vaddr(address: int, size: int) -> bytes:
        for section in sections:
            start = section["address"]
            if section["type"] != 8 and start <= address and address + size <= start + section["size"]:
                offset = section["offset"] + address - start
                return data[offset : offset + size]
        fail(f"address 0x{address:08X} is not backed by file data")

    if bytes_at_vaddr(properties_pointer, 16) != APP_PROPERTIES_MAGIC:
        fail("APP_PROPERTIES magic does not match Silicon Labs format")

    reset = symbols.get("Reset")
    if reset is None or entry != reset:
        fail(f"ELF entry 0x{entry:08X} does not equal Reset")
    if reset_vector != reset:
        fail(f"vector Reset 0x{reset_vector:08X} does not equal the Reset symbol")
    reset_code = reset & ~1
    first_instructions = bytes_at_vaddr(reset_code, 6)
    load_vtor, load_vector, store = struct.unpack("<HHH", first_instructions)
    if load_vtor & 0xFF00 != 0x4800 or load_vector & 0xFF00 != 0x4900 or store != 0x6001:
        fail(
            "Reset does not begin with the expected VTOR write sequence: "
            f"{first_instructions.hex()}"
        )

    def literal_value(instruction_address: int, instruction: int) -> int:
        literal_address = ((instruction_address + 4) & ~3) + (instruction & 0xFF) * 4
        return struct.unpack("<I", bytes_at_vaddr(literal_address, 4))[0]

    if literal_value(reset_code, load_vtor) != 0xE000ED08:
        fail("Reset VTOR destination is not SCB->VTOR")
    if literal_value(reset_code + 2, load_vector) != FLASH_START:
        fail("Reset VTOR value is not the vector table at 0x4000")

    loads = []
    for index in range(phnum):
        fields = struct.unpack_from("<IIIIIIII", data, phoff + index * phentsize)
        p_type, _, _, physical, file_size, _, _, _ = fields
        if p_type == 1 and file_size:
            loads.append((physical, physical + file_size))
    if not loads:
        fail("ELF has no file-backed PT_LOAD segments")
    first_load = min(start for start, _ in loads)
    highest_load = max(end for _, end in loads)
    if first_load != FLASH_START:
        fail(f"first file-backed PT_LOAD is 0x{first_load:08X}, expected 0x00004000")
    if highest_load > FLASH_END:
        fail(f"highest file-backed load 0x{highest_load:08X} enters legacy NVM3")
    for start, end in loads:
        if start < FLASH_START or end > FLASH_END:
            fail(f"file-backed PT_LOAD 0x{start:08X}..0x{end:08X} is unsafe")

    print(f"PASS: {args.elf}")
    print(
        f"  load=0x{first_load:08X}..0x{highest_load:08X} "
        f"SP=0x{initial_sp:08X} Reset=0x{reset_vector:08X}"
    )
    print(
        f"  vector[13]=APP_PROPERTIES=0x{properties_pointer:08X}; "
        "Reset sets VTOR=0x00004000"
    )
    print("  no file-backed bootloader/legacy-NVM3 records")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, struct.error) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
