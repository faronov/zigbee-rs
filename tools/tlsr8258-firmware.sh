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
    local rf_dma_start=0 rf_dma_end=0
    local rf_rx_buf=0 rf_tx_buf=0 rf_ack_tx_buf=0
    local security_nv_start=0
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
            _rf_dma_start_) rf_dma_start=$((16#$value)) ;;
            _rf_dma_end_) rf_dma_end=$((16#$value)) ;;
            *RF_RX_BUF) rf_rx_buf=$((16#$value)) ;;
            *RF_TX_BUF) rf_tx_buf=$((16#$value)) ;;
            *RF_ACK_TX_BUF) rf_ack_tx_buf=$((16#$value)) ;;
            _security_nv_start_) security_nv_start=$((16#$value)) ;;
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
    if (( rf_dma_start < icache_data_end || rf_dma_end > svc_bottom )); then
        printf 'layout-check FAIL: .rf_dma [0x%X..0x%X) is outside DMA-safe RAM [0x%X..0x%X)\n' \
            "$rf_dma_start" "$rf_dma_end" "$icache_data_end" "$svc_bottom" >&2
        exit 1
    fi

    local dma_starts=("$rf_rx_buf" "$rf_tx_buf" "$rf_ack_tx_buf")
    local dma_sizes=$((2 * 144))
    local dma_lengths=("$dma_sizes" 144 144)
    local dma_names=("RF_RX_BUF" "RF_TX_BUF" "RF_ACK_TX_BUF")
    local i j start end other_start other_end
    for i in 0 1 2; do
        start="${dma_starts[$i]}"
        end=$((start + dma_lengths[i]))
        if (( start == 0 || start % 4 != 0 || start < rf_dma_start || end > rf_dma_end )); then
            printf 'layout-check FAIL: %s [0x%X..0x%X) is missing, misaligned, or outside .rf_dma\n' \
                "${dma_names[$i]}" "$start" "$end" >&2
            exit 1
        fi
        for ((j = i + 1; j < 3; j++)); do
            other_start="${dma_starts[$j]}"
            other_end=$((other_start + dma_lengths[j]))
            if (( start < other_end && other_start < end )); then
                printf 'layout-check FAIL: %s overlaps %s\n' \
                    "${dma_names[$i]}" "${dma_names[$j]}" >&2
                exit 1
            fi
        done
    done

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
    if (( security_nv_start == 0 || size > security_nv_start )); then
        printf 'layout-check FAIL: image is %d bytes, security journal starts at 0x%X\n' \
            "$size" "$security_nv_start" >&2
        exit 1
    fi
    printf 'layout-check OK: image=%d B flash_limit=0x%X ram_code=%d B data=0x%X bss_end=0x%X rf_dma=[0x%X..0x%X) (rx=0x%X tx=0x%X ack=0x%X)\n' \
        "$size" "$security_nv_start" "$((ramcode_end - ramcode_start))" "$sdata" "$ebss" \
        "$rf_dma_start" "$rf_dma_end" "$rf_rx_buf" "$rf_tx_buf" "$rf_ack_tx_buf"
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
        (
            cd "$crate_dir"
            "$CARGO_BIN" check --release --bin "$binary_name"
        )
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
