#!/bin/sh
set -eu

ELF=${1:?usage: check-layout.sh <firmware.elf>}
SYSROOT=$(rustc --print sysroot)
OBJDUMP=${OBJDUMP:-$(find "$SYSROOT" -name llvm-objdump | head -1)}
NM=${NM:-$(find "$SYSROOT" -name llvm-nm | head -1)}

section_vma() {
    "$OBJDUMP" -h "$ELF" | awk -v name="$1" '$2 == name { print $4 }'
}

require_vma() {
    actual=$(section_vma "$1")
    if [ "$actual" != "$2" ]; then
        echo "$1 VMA is 0x$actual, expected 0x$2" >&2
        exit 1
    fi
}

require_ram_symbol() {
    address=$("$NM" -n -C "$ELF" | awk -v name="$1" 'index($0, name) { print $1; exit }')
    case "$address" in
        1fff*) ;;
        *)
            echo "$1 is not linked into SRAM (address: ${address:-missing})" >&2
            exit 1
            ;;
    esac
}

require_vma .jump_table 1fff0000
require_vma .vector_table 1fff1838
require_vma .text 11010100
require_ram_symbol "phy6222_hal::flash::write"
require_ram_symbol "phy6222_hal::flash::write_inner"
require_ram_symbol "phy6222_hal::flash::erase_sector"
require_ram_symbol "phy6222_hal::flash::erase_sector_inner"
require_ram_symbol "phy6222_hal::flash::write_enable"
require_ram_symbol "phy6222_hal::flash::spif_wait_idle"
require_ram_symbol "phy6222_hal::flash::spif_wait_not_busy"
require_ram_symbol "phy6222_hal::flash::enter_cache_bypass"
require_ram_symbol "phy6222_hal::flash::exit_cache_bypass"
require_ram_symbol "phy6222_hal::flash::cache_flush"

if "$NM" -n -C "$ELF" | awk '$1 ~ /^1fff/ && $0 ~ /__Thumbv6MABSLongThunk/ { found = 1 } END { exit !found }'
then
    echo "SRAM code contains an absolute thunk that may jump back into XIP" >&2
    exit 1
fi

echo "PHY62x2 layout: ROM jump table, run descriptor, XIP, and flash RAM code are valid"
