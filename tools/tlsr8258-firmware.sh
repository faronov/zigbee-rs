#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_TOOLCHAIN="${ROOT_DIR}/.toolchains/tc32-stage2-tc32-45"
TC32_TOOLCHAIN="${TC32_TOOLCHAIN:-$DEFAULT_TOOLCHAIN}"
CARGO_BIN="${CARGO_BIN:-$TC32_TOOLCHAIN/bin/cargo}"
LLVM_NM="${LLVM_NM:-$TC32_TOOLCHAIN/llvm/bin/llvm-nm}"
LLVM_OBJCOPY="${LLVM_OBJCOPY:-$TC32_TOOLCHAIN/llvm/bin/llvm-objcopy}"
TLSRPGM="${TLSRPGM:-$HOME/TLSRPGM/TlsrPgm.py}"
TELINK_PORT="${TELINK_PORT:-/dev/cu.usbserial-1410}"

usage() {
    echo "usage: $0 <check|build|flash> <crate-directory> <binary-name>" >&2
    exit 2
}

require_file() {
    if [[ ! -e "$1" ]]; then
        echo "missing $2: $1" >&2
        exit 1
    fi
}

verify_layout() {
    local elf="$1"
    local bin="$2"
    local ramcode_start=0 ramcode_end=0 ramcode_aligned=0
    local ictag_start=0 ictag_end=0 icache_data_end=0
    local sdata=0 ebss=0 svc_bottom=0
    local value name

    while read -r value _ name; do
        case "$name" in
            _ramcode_start_) ramcode_start=$((16#$value)) ;;
            _ramcode_end_) ramcode_end=$((16#$value)) ;;
            _ramcode_size_align_256_) ramcode_aligned=$((16#$value)) ;;
            _ictag_start_) ictag_start=$((16#$value)) ;;
            _ictag_end_) ictag_end=$((16#$value)) ;;
            _icache_data_end_) icache_data_end=$((16#$value)) ;;
            _sdata) sdata=$((16#$value)) ;;
            _ebss) ebss=$((16#$value)) ;;
            _svc_stack_bottom) svc_bottom=$((16#$value)) ;;
        esac
    done < <("$LLVM_NM" "$elf")

    if (( ramcode_end > 0x8000 )); then
        printf 'layout-check FAIL: .ram_code ends at 0x%X, after .text base 0x8000\n' \
            "$ramcode_end" >&2
        exit 1
    fi
    if (( ebss > svc_bottom )); then
        printf 'layout-check FAIL: .bss ends at 0x%X, stack starts at 0x%X\n' \
            "$ebss" "$svc_bottom" >&2
        exit 1
    fi

    local expected_tag_start=$((0x840000 + ramcode_aligned))
    local expected_data_start=$((expected_tag_start + 0x900))
    if (( ictag_start != expected_tag_start || ictag_end != ictag_start + 0x100 )); then
        echo "layout-check FAIL: invalid instruction-cache tag reservation" >&2
        exit 1
    fi
    if (( icache_data_end != expected_data_start || sdata < icache_data_end )); then
        echo "layout-check FAIL: writable data overlaps the instruction cache" >&2
        exit 1
    fi
    if (( ramcode_end - ramcode_start < 0x100 )); then
        echo "layout-check FAIL: flash routines are not retained in RAM code" >&2
        exit 1
    fi
    if ! "$LLVM_NM" -C "$elf" | awk '
        /zigbee_mac::telink::imp::TelinkMac/ { found = 1 }
        END { exit(found ? 0 : 1) }
    '; then
        echo "layout-check FAIL: firmware does not link the reusable Telink MAC" >&2
        exit 1
    fi
    if "$LLVM_NM" -C "$elf" | awk '
        /Tlsr8258Mac/ { found = 1 }
        END { exit(found ? 0 : 1) }
    '; then
        echo "layout-check FAIL: production firmware links legacy lab radio code" >&2
        exit 1
    fi

    local size
    size=$(wc -c < "$bin" | tr -d ' ')
    if (( size > 0x40000 )); then
        printf 'layout-check FAIL: image is %d bytes, above the 256 KiB slot\n' "$size" >&2
        exit 1
    fi
    printf 'layout-check OK: image=%d B ram_code=%d B data=0x%X bss_end=0x%X\n' \
        "$size" "$((ramcode_end - ramcode_start))" "$sdata" "$ebss"
}

[[ $# -eq 3 ]] || usage
command="$1"
crate_dir="$2"
binary_name="$3"

if [[ "$crate_dir" != /* ]]; then
    crate_dir="${ROOT_DIR}/${crate_dir}"
fi
require_file "$crate_dir/Cargo.toml" "Cargo manifest"
require_file "$CARGO_BIN" "tc32 cargo"

target_dir="${crate_dir}/target/tc32-unknown-none-elf/release"
elf="${target_dir}/${binary_name}"
bin="${elf}.bin"

case "$command" in
    check)
        (cd "$crate_dir" && "$CARGO_BIN" check --release --bin "$binary_name")
        ;;
    build|flash)
        (
            cd "$crate_dir"
            "$CARGO_BIN" rustc --release --bin "$binary_name" -- \
                -C lto=fat -C opt-level=s -C codegen-units=1
        )
        require_file "$LLVM_OBJCOPY" "llvm-objcopy"
        require_file "$LLVM_NM" "llvm-nm"
        "$LLVM_OBJCOPY" -O binary "$elf" "$bin"
        verify_layout "$elf" "$bin"
        if [[ "$command" == "flash" ]]; then
            require_file "$TLSRPGM" "TlsrPgm.py"
            python3 "$TLSRPGM" -p "$TELINK_PORT" -t 500 -a 200 -m we 0 "$bin"
        fi
        echo "$bin"
        ;;
    *)
        usage
        ;;
esac
