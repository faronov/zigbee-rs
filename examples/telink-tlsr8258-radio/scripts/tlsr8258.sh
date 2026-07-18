#!/usr/bin/env bash
# Build/dump helper for the standalone TLSR8258 raw-radio bring-up crate
# (examples/telink-tlsr8258-radio). Scoped down from
# examples/telink-tlsr8258-sensor/scripts/tlsr8258.sh: single binary, no
# feature-mode selection, and a verify_layout that covers this crate's own
# `.rf_dma`/`.diag` sections in addition to the shared I-cache-reservation
# invariants. Never uses `--noinhibit-exec`: a link that violates the
# memory.x ASSERT()s must fail the build, not silently produce an image.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
EXAMPLE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REPO_DIR="$(cd -- "${EXAMPLE_DIR}/../.." && pwd)"

DEFAULT_TC32_TOOLCHAIN="${REPO_DIR}/.toolchains/tc32-stage2-tc32-45"
TC32_TOOLCHAIN="${TC32_TOOLCHAIN:-$DEFAULT_TC32_TOOLCHAIN}"
CARGO_BIN="${CARGO_BIN:-$TC32_TOOLCHAIN/bin/cargo}"
LLVM_NM="${LLVM_NM:-$TC32_TOOLCHAIN/llvm/bin/llvm-nm}"
LLVM_OBJCOPY="${LLVM_OBJCOPY:-$TC32_TOOLCHAIN/llvm/bin/llvm-objcopy}"
LLVM_SIZE="${LLVM_SIZE:-$TC32_TOOLCHAIN/llvm/bin/llvm-size}"

DEFAULT_CARGO_HOME="${HOME}/.cargo"
CARGO_HOME="${CARGO_HOME:-$DEFAULT_CARGO_HOME}"

TLSRPGM="${TLSRPGM:-$HOME/TLSRPGM/TlsrPgm.py}"
TLSR_DEBUG="${TLSR_DEBUG:-$HOME/zboss_opensource/tlsr_debug.py}"
TELINK_PORT="${TELINK_PORT:-/dev/cu.usbserial-1410}"

TARGET_DIR="${EXAMPLE_DIR}/target/tc32-unknown-none-elf/release"
ELF_PATH="${TARGET_DIR}/telink-tlsr8258-radio"
BIN_PATH="${TARGET_DIR}/telink-tlsr8258-radio.bin"

# CPU and SWire aliases of the fixed diagnostic record. SWire clears the
# 0x800000 memory-space selector, so CPU SRAM 0x84FE00 is read as 0x04FE00.
DIAG_CPU_BASE="0x0084FE00"
DIAG_SWIRE_BASE="0x0004FE00"

usage() {
    cat <<'EOF'
Usage:
  scripts/tlsr8258.sh check                 # cargo check (tc32 target, no link)
  scripts/tlsr8258.sh build [band|control|scan|association|association-fresh|association-stress|mac-association|runtime-join]
  scripts/tlsr8258.sh test                   # host-side `cargo test` (pure logic only)
  scripts/tlsr8258.sh flash [band|control|scan|association|association-fresh|association-stress|mac-association|runtime-join]
  scripts/tlsr8258.sh dump-diag [word-count] # dump the diagnostic record (default 64 words)
  scripts/tlsr8258.sh dump <address> [word-count]
  scripts/tlsr8258.sh dump-activate <address> [word-count]

Environment overrides:
  TC32_TOOLCHAIN  Path to tc32-stage2 toolchain root (default: tc32-45)
  CARGO_BIN       Cargo binary to use instead of $TC32_TOOLCHAIN/bin/cargo
  LLVM_NM         llvm-nm binary used by verify_layout
  LLVM_OBJCOPY    llvm-objcopy binary to emit the flashable .bin
  LLVM_SIZE       llvm-size binary used to report text/data/bss
  CARGO_HOME      Cargo home for registry/cache
  TLSRPGM         Path to TlsrPgm.py
  TLSR_DEBUG      Path to tlsr_debug.py
  TELINK_PORT     Serial device used by both flasher and debugger

Examples:
  TC32_TOOLCHAIN=$REPO/.toolchains/tc32-stage2-tc32-43 scripts/tlsr8258.sh build
EOF
}

require_file() {
    local path="$1"
    local name="$2"
    if [[ ! -e "$path" ]]; then
        echo "Missing ${name}: ${path}" >&2
        exit 1
    fi
}

run_cargo() {
    local cargo_subcommand="$1"
    shift
    require_file "$CARGO_BIN" "tc32 cargo"
    (
        cd "$EXAMPLE_DIR"
        env CARGO_HOME="$CARGO_HOME" "$CARGO_BIN" "$cargo_subcommand" --release \
            --target tc32-unknown-none-elf \
            -Z build-std=core -Z build-std-features=compiler-builtins-mem \
            "$@"
    )
}

emit_bin() {
    require_file "$LLVM_OBJCOPY" "llvm-objcopy"
    require_file "$ELF_PATH" "ELF image"
    # Deliberately no --noinhibit-exec anywhere in this script: an ELF that
    # failed to satisfy memory.x's ASSERT()s must not be turned into a .bin.
    "$LLVM_OBJCOPY" -O binary "$ELF_PATH" "$BIN_PATH"
}

# Post-link safety check: some lld configurations silently swallow
# in-script ASSERT() failures, so this independently re-derives every layout
# invariant memory.x documents from the linked ELF's own symbol table and
# section headers. Covers, in order:
#   1. I-cache tag/data reservation vs. `.ram_code` size
#   2. `.data` (and its first word, the cache canary) starting at/after
#      `_icache_data_end_`
#   3. `.rf_dma` (RF_RX_BUF/RF_TX_BUF) placement outside the cache
#      reservation, below the IRQ stack, and 4-byte aligned
#   4. `.bss` staying below the IRQ stack
#   5. the diagnostic record's fixed address and reserved size
#   6. `.ram_code` fitting under the absolute `.text` base (0x8000)
#   7. image-size warnings/failures against the 256 KiB OTA slot and the
#      0x74000 security journal
verify_layout() {
    require_file "$ELF_PATH" "ELF image"
    require_file "$LLVM_NM" "llvm-nm"

    local ramcode_start=0 ramcode_end=0 ramcode_aligned=0
    local ictag_start=0 ictag_end=0 icache_data_end=0
    local sdata=0 edata=0 sbss=0 ebss=0
    local rf_dma_start=0 rf_dma_end=0 rf_rx_buf=0 rf_tx_buf=0
    local svc_top=0 svc_bot=0 irq_top=0 irq_bot=0
    local diag_start=0 diag_end=0
    local line value name
    while read -r line; do
        value=$(echo "$line" | awk '{print $1}')
        name=$(echo "$line" | awk '{print $NF}')
        case "$name" in
            _ramcode_start_)   ramcode_start=$((16#$value)) ;;
            _ramcode_end_)     ramcode_end=$((16#$value)) ;;
            _ramcode_size_align_256_) ramcode_aligned=$((16#$value)) ;;
            _ictag_start_)     ictag_start=$((16#$value)) ;;
            _ictag_end_)       ictag_end=$((16#$value)) ;;
            _icache_data_end_) icache_data_end=$((16#$value)) ;;
            _sdata)            sdata=$((16#$value)) ;;
            _edata)            edata=$((16#$value)) ;;
            _sbss)             sbss=$((16#$value)) ;;
            _ebss)             ebss=$((16#$value)) ;;
            _rf_dma_start_)    rf_dma_start=$((16#$value)) ;;
            _rf_dma_end_)      rf_dma_end=$((16#$value)) ;;
            *RF_RX_BUF)        rf_rx_buf=$((16#$value)) ;;
            *RF_TX_BUF)        rf_tx_buf=$((16#$value)) ;;
            _svc_stack_top)    svc_top=$((16#$value)) ;;
            _svc_stack_bottom) svc_bot=$((16#$value)) ;;
            _irq_stack_top)    irq_top=$((16#$value)) ;;
            _irq_stack_bottom) irq_bot=$((16#$value)) ;;
            _diag_start_)      diag_start=$((16#$value)) ;;
            _diag_end_)        diag_end=$((16#$value)) ;;
        esac
    done < <("$LLVM_NM" "$ELF_PATH")

    local fail=0
    fail_check() {
        printf 'layout-check FAIL: %s\n' "$1" >&2
        fail=1
    }

    # 1 & 2: cache layout, .data placement.
    local expected_tag_start=$((0x840000 + ramcode_aligned))
    if (( ictag_start != expected_tag_start )); then
        fail_check "I-cache tags start at 0x$(printf %X "$ictag_start"), expected 0x$(printf %X "$expected_tag_start") from aligned RAM-code size"
    fi
    if (( ictag_end != ictag_start + 0x100 )); then
        fail_check "I-cache tag range [0x$(printf %X "$ictag_start")..0x$(printf %X "$ictag_end")) is not 0x100 bytes"
    fi
    if (( icache_data_end != ictag_end + 0x800 )); then
        fail_check "I-cache data region does not end 0x800 bytes after tag end (icache_data_end=0x$(printf %X "$icache_data_end"), expected 0x$(printf %X $((ictag_end + 0x800))))"
    fi
    if (( sdata < icache_data_end )); then
        fail_check ".data (_sdata=0x$(printf %X "$sdata")) starts before _icache_data_end_=0x$(printf %X "$icache_data_end")"
    fi
    if (( sdata != icache_data_end )); then
        fail_check "cache canary is not the first word of .data: _sdata=0x$(printf %X "$sdata") != _icache_data_end_=0x$(printf %X "$icache_data_end")"
    fi

    # 3: .rf_dma placement + alignment.
    if (( rf_rx_buf == 0 || rf_tx_buf == 0 )); then
        fail_check "RF_RX_BUF/RF_TX_BUF symbols were not found in the linked ELF"
    fi
    if (( rf_dma_start < icache_data_end )); then
        fail_check ".rf_dma (0x$(printf %X "$rf_dma_start")) overlaps the I-cache reservation ending at 0x$(printf %X "$icache_data_end")"
    fi
    if (( rf_dma_end > irq_bot )); then
        fail_check ".rf_dma end (0x$(printf %X "$rf_dma_end")) extends into the IRQ stack region starting at 0x$(printf %X "$irq_bot")"
    fi
    if (( rf_rx_buf % 4 != 0 )); then
        fail_check "RF_RX_BUF (0x$(printf %X "$rf_rx_buf")) is not 4-byte aligned"
    fi
    if (( rf_tx_buf % 4 != 0 )); then
        fail_check "RF_TX_BUF (0x$(printf %X "$rf_tx_buf")) is not 4-byte aligned"
    fi
    if (( rf_rx_buf != 0 && rf_rx_buf < icache_data_end )); then
        fail_check "RF_RX_BUF (0x$(printf %X "$rf_rx_buf")) overlaps the I-cache reservation"
    fi
    if (( rf_tx_buf != 0 && rf_tx_buf < icache_data_end )); then
        fail_check "RF_TX_BUF (0x$(printf %X "$rf_tx_buf")) overlaps the I-cache reservation"
    fi

    # 4: .bss vs stacks/diag.
    if (( ebss > irq_bot )); then
        fail_check ".bss (_ebss=0x$(printf %X "$ebss")) extends into the IRQ stack region starting at 0x$(printf %X "$irq_bot")"
    fi
    if (( irq_top != svc_bot )); then
        fail_check "IRQ stack top (0x$(printf %X "$irq_top")) does not abut SVC stack bottom (0x$(printf %X "$svc_bot"))"
    fi
    if (( svc_top > diag_start )); then
        fail_check "SVC stack top (0x$(printf %X "$svc_top")) overlaps the diagnostic record starting at 0x$(printf %X "$diag_start")"
    fi

    # 5: diagnostic record address/size.
    local expected_diag_start
    expected_diag_start=$((16#${DIAG_CPU_BASE#0x}))
    if (( diag_start != expected_diag_start )); then
        fail_check "diagnostic record starts at 0x$(printf %X "$diag_start"), expected ${DIAG_CPU_BASE}"
    fi
    local diag_size=$(( diag_end - diag_start ))
    if (( diag_size < 64 )); then
        fail_check "diagnostic record region is only ${diag_size} bytes (DiagRecord will not fit)"
    fi
    if (( diag_size != 512 )); then
        printf 'layout-check WARN: diagnostic region is %d bytes, expected 512 (documented reservation)\n' "$diag_size" >&2
    fi

    # 6: .ram_code fits below the absolute .text base.
    if (( ramcode_end > 0x8000 )); then
        fail_check ".ram_code end (0x$(printf %X "$ramcode_end")) overflows the absolute .text base 0x8000"
    fi
    local ramcode_len=$(( ramcode_end - ramcode_start ))

    if (( fail != 0 )); then
        echo "layout-check: one or more invariants FAILED (see above)" >&2
        exit 1
    fi

    # 7: image-size warnings. Fail hard at the security journal; warn past
    # the 256 KiB production/OTA slot boundary.
    if [[ -f "$BIN_PATH" ]]; then
        local bin_size
        bin_size=$(wc -c < "$BIN_PATH" | tr -d ' ')
        if (( bin_size > 0x74000 )); then
            fail_check ".bin size=${bin_size} (0x$(printf %X "$bin_size")) reaches security journal at 0x74000"
            echo "layout-check: one or more invariants FAILED (see above)" >&2
            exit 1
        fi
        if (( bin_size > 0x40000 )); then
            printf 'layout-check WARN: .bin size=%d (0x%X) exceeds the 256 KiB production/OTA slot\n' \
                "$bin_size" "$bin_size" >&2
        fi
    fi

    printf 'layout-check OK: ram_code=%dB cache=[0x%X..0x%X) data=[0x%X..0x%X) rf_dma=[0x%X..0x%X) (rx=0x%X tx=0x%X) bss=[0x%X..0x%X) irq=[0x%X..0x%X) svc=[0x%X..0x%X) diag=[0x%X..0x%X)\n' \
        "$ramcode_len" "$ictag_start" "$icache_data_end" "$sdata" "$edata" \
        "$rf_dma_start" "$rf_dma_end" "$rf_rx_buf" "$rf_tx_buf" \
        "$sbss" "$ebss" "$irq_bot" "$irq_top" "$svc_bot" "$svc_top" "$diag_start" "$diag_end"
}

cmd_check() {
    run_cargo check
}

cmd_build() {
    local variant="${1:-band}"
    local feature_args=()
    case "$variant" in
        band) ;;
        control) feature_args=(--features control-channel-15) ;;
        scan) feature_args=(--features active-scan) ;;
        association) feature_args=(--features association) ;;
        association-fresh) feature_args=(--features association-fresh) ;;
        association-stress) feature_args=(--features association-stress) ;;
        mac-association) feature_args=(--features mac-driver) ;;
        runtime-join) feature_args=(--features runtime-join) ;;
        *)
            echo "Unsupported radio variant: ${variant} (expected band, control, scan, association, association-fresh, association-stress, mac-association, or runtime-join)" >&2
            exit 1
            ;;
    esac
    if ((${#feature_args[@]})); then
        run_cargo rustc "${feature_args[@]}" -- -C lto=no -C opt-level=1
    else
        run_cargo rustc -- -C lto=no -C opt-level=1
    fi
    emit_bin
    verify_layout
    if [[ -x "$LLVM_SIZE" ]]; then
        "$LLVM_SIZE" "$ELF_PATH"
    fi
    echo "ELF: ${ELF_PATH}"
    echo "BIN: ${BIN_PATH}"
}

cmd_test() {
    # Host-side unit/golden-vector tests use the ambient host cargo/rustc;
    # there is no host `std` for the tc32 target to run tests against.
    (
        cd "$EXAMPLE_DIR"
        local host_target
        host_target=$(rustc -vV | awk '/^host:/ {print $2}')
        cargo test --target "$host_target" "$@"
        cargo test --manifest-path "$REPO_DIR/tlsr8258-hal/Cargo.toml" \
            --target "$host_target" "$@"
    )
}

cmd_flash() {
    cmd_build "${1:-band}"
    require_file "$TLSRPGM" "TlsrPgm.py"
    python3 "$TLSRPGM" -p "$TELINK_PORT" -t 500 -a 200 -m we 0 "$BIN_PATH"
}

cmd_dump() {
    local address="${1:?dump requires an address}"
    local words="${2:-16}"
    local bytes=$((words * 4))
    require_file "$TLSR_DEBUG" "tlsr_debug.py"
    python3 "$TLSR_DEBUG" -p "$TELINK_PORT" read "$address" "$bytes"
}

cmd_dump_activate() {
    local address="${1:?dump-activate requires an address}"
    local words="${2:-16}"
    local bytes=$((words * 4))
    require_file "$TLSR_DEBUG" "tlsr_debug.py"
    python3 "$TLSR_DEBUG" -p "$TELINK_PORT" --activate read "$address" "$bytes"
}

cmd_dump_diag() {
    cmd_dump "$DIAG_SWIRE_BASE" "${1:-64}"
}

main() {
    local command="${1:-}"
    shift || true

    case "$command" in
        check) cmd_check "$@" ;;
        build) cmd_build "$@" ;;
        test) cmd_test "$@" ;;
        flash) cmd_flash "$@" ;;
        dump) cmd_dump "$@" ;;
        dump-activate) cmd_dump_activate "$@" ;;
        dump-diag) cmd_dump_diag "$@" ;;
        -h|--help|help|"") usage ;;
        *)
            echo "Unsupported command: ${command}" >&2
            usage >&2
            exit 1
            ;;
    esac
}

main "$@"
