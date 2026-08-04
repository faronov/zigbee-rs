#!/bin/sh
set -eu

ELF=${1:?usage: check-layout.sh <firmware.elf>}
SYSROOT=$(rustc --print sysroot)
NM=${NM:-$(find "$SYSROOT" -name llvm-nm | head -1)}

check_bufc_ring() {
    name=$1
    expected_size=$2
    expected_alignment=$3
    record=$("$NM" -S -n -C "$ELF" | awk -v name="$name" 'index($0, name) { print $1, $2; exit }')

    if [ -z "$record" ]; then
        echo "$name is missing from $ELF" >&2
        exit 1
    fi

    set -- $record
    address=$((0x$1))
    size=$((0x$2))

    if [ "$size" -ne "$expected_size" ]; then
        echo "$name is $size bytes, expected $expected_size" >&2
        exit 1
    fi

    if [ $((address % expected_alignment)) -ne 0 ]; then
        printf '%s is linked at 0x%x, expected %d-byte alignment\n' \
            "$name" "$address" "$expected_alignment" >&2
        exit 1
    fi
}

check_bufc_ring "zigbee_mac::efr32::driver::BUFC_RX_META_RAM" 64 4
check_bufc_ring "zigbee_mac::efr32::driver::BUFC_TX_RAM" 128 4
check_bufc_ring "zigbee_mac::efr32::driver::BUFC_RX_RAM" 512 4

symbol_address() {
    name=$1
    address=$("$NM" -n -C "$ELF" | awk -v name="$name" '$NF == name { print $1; exit }')
    if [ -z "$address" ]; then
        echo "$name is missing from $ELF" >&2
        exit 1
    fi
    printf '%d\n' "$((0x$address))"
}

stack_start=$(symbol_address "_stack_start")
stack_end=$(symbol_address "_stack_end")
stack_bytes=$((stack_start - stack_end))
min_stack_bytes=16384
if [ "$stack_bytes" -lt "$min_stack_bytes" ]; then
    echo "EFR32MG1 stack budget is $stack_bytes bytes, expected at least $min_stack_bytes" >&2
    exit 1
fi

echo "EFR32MG1 BUFC geometry and stack budget are valid ($stack_bytes bytes)"
