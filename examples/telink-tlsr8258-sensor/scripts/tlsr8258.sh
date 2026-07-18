#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
EXAMPLE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REPO_DIR="$(cd -- "${EXAMPLE_DIR}/../.." && pwd)"

DEFAULT_TC32_TOOLCHAIN="${REPO_DIR}/.toolchains/tc32-stage2-tc32-45"
TC32_TOOLCHAIN="${TC32_TOOLCHAIN:-$DEFAULT_TC32_TOOLCHAIN}"
CARGO_BIN="${CARGO_BIN:-$TC32_TOOLCHAIN/bin/cargo}"
LLVM_OBJCOPY="${LLVM_OBJCOPY:-$TC32_TOOLCHAIN/llvm/bin/llvm-objcopy}"

DEFAULT_CARGO_HOME="${HOME}/.cargo"
CARGO_HOME="${CARGO_HOME:-$DEFAULT_CARGO_HOME}"

TLSRPGM="${TLSRPGM:-$HOME/TLSRPGM/TlsrPgm.py}"
TLSR_DEBUG="${TLSR_DEBUG:-$HOME/zboss_opensource/tlsr_debug.py}"
TELINK_PORT="${TELINK_PORT:-/dev/cu.usbserial-1410}"
PROBE_RS="${PROBE_RS:-/tmp/probe-rs-tc32-25521051175/probe-rs}"
PROBE_RS_PROBE="${PROBE_RS_PROBE:-sws:$TELINK_PORT}"
PROBE_RS_CHIP="${PROBE_RS_CHIP:-TLSR8258}"
PROBE_RS_PROTOCOL="${PROBE_RS_PROTOCOL:-swd}"
PROBE_RS_SCAN_REGION="${PROBE_RS_SCAN_REGION:-ram}"

TARGET_DIR="${EXAMPLE_DIR}/target/tc32-unknown-none-elf/release"
BIN_NAME="telink-tlsr8258-lab"
ELF_PATH="${TARGET_DIR}/${BIN_NAME}"
BIN_PATH="${TARGET_DIR}/${BIN_NAME}.bin"

DBG_BOOT_BASE="0x0084F000"
DBG_MODE_BASE="0x0084F100"

MODE_NAME="sensor"
declare -a CARGO_FEATURE_ARGS=()

usage() {
    cat <<'EOF'
Usage:
  scripts/tlsr8258.sh check [sensor|runtime-sensor|runtime-router|diag-assoc|diag-beacon|diag-smoke|diag-pm]
  scripts/tlsr8258.sh build [sensor|runtime-sensor|runtime-router|diag-assoc|diag-beacon|diag-smoke|diag-pm]
  scripts/tlsr8258.sh flash [sensor|runtime-sensor|runtime-router|diag-assoc|diag-beacon|diag-smoke|diag-pm]
  scripts/tlsr8258.sh dump-boot [word-count]
  scripts/tlsr8258.sh dump-mode [word-count]
  scripts/tlsr8258.sh dump <address> [word-count]
  scripts/tlsr8258.sh dump-activate <address> [word-count]
  scripts/tlsr8258.sh pgm-info
  scripts/tlsr8258.sh pgm-dump <address> [byte-count]
  scripts/tlsr8258.sh pgm-break <address>
  scripts/tlsr8258.sh pgm-step [count]
  scripts/tlsr8258.sh pgm-go
  scripts/tlsr8258.sh probe-list
  scripts/tlsr8258.sh probe-info
  scripts/tlsr8258.sh probe-attach [sensor|runtime-sensor|runtime-router|diag-assoc|diag-beacon|diag-smoke|diag-pm]
  scripts/tlsr8258.sh probe-list-rtt [sensor|runtime-sensor|runtime-router|diag-assoc|diag-beacon|diag-smoke|diag-pm]
  scripts/tlsr8258.sh probe-debug [sensor|runtime-sensor|runtime-router|diag-assoc|diag-beacon|diag-smoke|diag-pm]
  scripts/tlsr8258.sh probe-gdb [sensor|runtime-sensor|runtime-router|diag-assoc|diag-beacon|diag-smoke|diag-pm]

Environment overrides:
  TC32_TOOLCHAIN  Path to tc32-stage2 toolchain root
  CARGO_BIN       Cargo binary to use instead of $TC32_TOOLCHAIN/bin/cargo
  LLVM_OBJCOPY    llvm-objcopy binary to emit the flashable .bin
  CARGO_HOME      Cargo home for registry/cache
  TLSRPGM         Path to TlsrPgm.py
  TLSR_DEBUG      Path to tlsr_debug.py
  TELINK_PORT     Serial device used by both flasher and debugger
  PROBE_RS        Path to tc32-enabled probe-rs
  PROBE_RS_PROBE  probe-rs selector, e.g. sws:/dev/cu.usbserial-1410
  PROBE_RS_CHIP   probe-rs chip name
  PROBE_RS_PROTOCOL probe-rs wire protocol
  PROBE_RS_SCAN_REGION RTT scan region for probe-rs attach
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

resolve_mode() {
    local mode="${1:-sensor}"
    case "$mode" in
        sensor)
            MODE_NAME="sensor"
            BIN_NAME="telink-tlsr8258-lab"
            CARGO_FEATURE_ARGS=(--no-default-features --features sensor)
            ;;
        runtime-sensor)
            MODE_NAME="runtime-sensor"
            BIN_NAME="telink-tlsr8258-runtime"
            CARGO_FEATURE_ARGS=(--no-default-features --features runtime-sensor)
            ;;
        runtime-router)
            MODE_NAME="runtime-router"
            BIN_NAME="telink-tlsr8258-router"
            CARGO_FEATURE_ARGS=(--no-default-features --features runtime-router)
            ;;
        diag-assoc)
            MODE_NAME="diag-assoc"
            BIN_NAME="telink-tlsr8258-lab"
            CARGO_FEATURE_ARGS=(--no-default-features --features diag-assoc)
            ;;
        diag-beacon)
            MODE_NAME="diag-beacon"
            BIN_NAME="telink-tlsr8258-lab"
            CARGO_FEATURE_ARGS=(--no-default-features --features diag-beacon)
            ;;
        diag-smoke)
            MODE_NAME="diag-smoke"
            BIN_NAME="telink-tlsr8258-lab"
            CARGO_FEATURE_ARGS=(--no-default-features --features diag-smoke)
            ;;
        diag-pm)
            MODE_NAME="diag-pm"
            BIN_NAME="telink-tlsr8258-lab"
            CARGO_FEATURE_ARGS=(--no-default-features --features diag-pm)
            ;;
        *)
            echo "Unsupported mode: ${mode}" >&2
            usage >&2
            exit 1
            ;;
    esac
    ELF_PATH="${TARGET_DIR}/${BIN_NAME}"
    BIN_PATH="${TARGET_DIR}/${BIN_NAME}.bin"
}

run_cargo() {
    local cargo_subcommand="$1"
    shift
    require_file "$CARGO_BIN" "tc32 cargo"
    (
        local -a cargo_args=("$cargo_subcommand" --release)
        if (( ${#CARGO_FEATURE_ARGS[@]} > 0 )); then
            cargo_args+=("${CARGO_FEATURE_ARGS[@]}")
        fi
        cargo_args+=(--bin "$BIN_NAME")
        cargo_args+=("$@")
        cd "$EXAMPLE_DIR"
        env CARGO_HOME="$CARGO_HOME" "$CARGO_BIN" "${cargo_args[@]}"
    )
}

emit_bin() {
    require_file "$LLVM_OBJCOPY" "llvm-objcopy"
    require_file "$ELF_PATH" "ELF image"
    "$LLVM_OBJCOPY" -O binary "$ELF_PATH" "$BIN_PATH"
}

# Post-link safety check: ld.lld silently swallows ASSERT() in our scripts, so
# the in-file asserts in memory.x are documentation only. This re-checks the
# same invariants by reading the linked ELF's symbol table.
verify_layout() {
    require_file "$ELF_PATH" "ELF image"
    local nm="${TC32_TOOLCHAIN}/llvm/bin/llvm-nm"
    require_file "$nm" "llvm-nm"
    local ramcode_end=0 ramcode_start=0 ramcode_aligned=0
    local ictag_start=0 ictag_end=0 icache_data_end=0 sdata=0
    local ebss=0 svc_bot=0 svc_top=0 irq_top=0
    local rf_dma_start=0 rf_dma_end=0
    local rf_rx_buf=0 rf_tx_buf=0
    local line value name
    while read -r line; do
        # llvm-nm output is "<hex> <type> <name>" or "<hex> <type> <name> <size>"
        value=$(echo "$line" | awk '{print $1}')
        name=$(echo "$line" | awk '{print $NF}')
        case "$name" in
            _ramcode_start_)    ramcode_start=$((16#$value)) ;;
            _ramcode_end_)      ramcode_end=$((16#$value)) ;;
            _ramcode_size_align_256_) ramcode_aligned=$((16#$value)) ;;
            _ictag_start_)      ictag_start=$((16#$value)) ;;
            _ictag_end_)        ictag_end=$((16#$value)) ;;
            _icache_data_end_)  icache_data_end=$((16#$value)) ;;
            _sdata)             sdata=$((16#$value)) ;;
            _ebss)              ebss=$((16#$value)) ;;
            _svc_stack_bottom)  svc_bot=$((16#$value)) ;;
            _svc_stack_top)     svc_top=$((16#$value)) ;;
            _irq_stack_top)     irq_top=$((16#$value)) ;;
            _rf_dma_start_)     rf_dma_start=$((16#$value)) ;;
            _rf_dma_end_)       rf_dma_end=$((16#$value)) ;;
            *RF_RX_BUF)         rf_rx_buf=$((16#$value)) ;;
            *RF_TX_BUF)         rf_tx_buf=$((16#$value)) ;;
        esac
    done < <("$nm" "$ELF_PATH")
    if (( ramcode_end > 0x8000 )); then
        printf 'layout-check FAIL: _ramcode_end_=0x%X overflows .text base 0x8000\n' \
            "$ramcode_end" >&2
        exit 1
    fi
    if (( ebss > svc_bot )); then
        printf 'layout-check FAIL: _ebss=0x%X extends past _svc_stack_bottom=0x%X (shrink statics or lower _svc_stack_bottom)\n' \
            "$ebss" "$svc_bot" >&2
        exit 1
    fi
    local expected_tag_start=$((0x840000 + ramcode_aligned))
    local expected_data_start=$((expected_tag_start + 0x900))
    if (( ictag_start != expected_tag_start || ictag_end != ictag_start + 0x100 )); then
        printf 'layout-check FAIL: I-cache tag range [0x%X..0x%X) does not match expected [0x%X..0x%X)\n' \
            "$ictag_start" "$ictag_end" "$expected_tag_start" "$((expected_tag_start + 0x100))" >&2
        exit 1
    fi
    if (( icache_data_end != expected_data_start || sdata < icache_data_end )); then
        printf 'layout-check FAIL: cache data ends at 0x%X, but _sdata=0x%X (expected >=0x%X)\n' \
            "$icache_data_end" "$sdata" "$expected_data_start" >&2
        exit 1
    fi
    if (( rf_rx_buf != 0 && rf_rx_buf < icache_data_end )); then
        printf 'layout-check FAIL: RF_RX_BUF=0x%X overlaps cache reservation ending at 0x%X\n' \
            "$rf_rx_buf" "$icache_data_end" >&2
        exit 1
    fi
    if (( rf_tx_buf != 0 && rf_tx_buf < icache_data_end )); then
        printf 'layout-check FAIL: RF_TX_BUF=0x%X overlaps cache reservation ending at 0x%X\n' \
            "$rf_tx_buf" "$icache_data_end" >&2
        exit 1
    fi
    if (( rf_dma_start != 0 && rf_dma_start < icache_data_end )); then
        printf 'layout-check FAIL: .rf_dma starts at 0x%X before cache reservation end 0x%X\n' \
            "$rf_dma_start" "$icache_data_end" >&2
        exit 1
    fi
    if (( rf_dma_end > svc_bot )); then
        printf 'layout-check FAIL: .rf_dma ends at 0x%X past _svc_stack_bottom=0x%X\n' \
            "$rf_dma_end" "$svc_bot" >&2
        exit 1
    fi
    if [[ "$MODE_NAME" == "diag-pm" ]] \
        && (( svc_top > 0x848000 || irq_top > 0x848000 )); then
        printf 'layout-check FAIL: LOW32K stacks exceed retention: SVC top=0x%X IRQ top=0x%X\n' \
            "$svc_top" "$irq_top" >&2
        exit 1
    fi
    # The flash erase/program path must remain in RAM because executing flash
    # erase from XIP flash hangs the bus. Require at least 256 bytes of
    # ram_code body.
    local ramcode_len=$(( ramcode_end - ramcode_start ))
    if (( ramcode_len < 0x100 )); then
        printf 'layout-check FAIL: .ram_code body too small (%d bytes); flash routines likely inlined into XIP flash\n' \
            "$ramcode_len" >&2
        exit 1
    fi
    if [[ "$MODE_NAME" == "runtime-sensor" || "$MODE_NAME" == "runtime-router" ]]; then
        if ! "$nm" -C "$ELF_PATH" | awk '
            /zigbee_mac::telink::imp::TelinkMac/ { found = 1 }
            END { exit(found ? 0 : 1) }
        '; then
            echo "layout-check FAIL: ${MODE_NAME} does not link zigbee_mac::telink::TelinkMac" >&2
            exit 1
        fi
        if "$nm" -C "$ELF_PATH" | awk '
            /telink_tlsr8258_sensor::Tlsr8258Mac/ ||
            /telink_tlsr8258_sensor::radio::/ { found = 1 }
            END { exit(found ? 0 : 1) }
        '; then
            echo "layout-check FAIL: ${MODE_NAME} still links the legacy local MAC/radio" >&2
            exit 1
        fi
    fi
    # The security journal starts at 0x74000 and factory data at 0x76000.
    # Production/OTA images should stay below 0x40000, but the unoptimized
    # runtime build may exceed that boundary while remaining below the
    # journal. Keep this visible as a warning rather than blocking it.
    if [[ -f "$BIN_PATH" ]]; then
        local bin_size
        bin_size=$(wc -c < "$BIN_PATH" | tr -d ' ')
        if (( bin_size > 0x74000 )); then
            printf 'layout-check FAIL: .bin size=%d (0x%X) reaches security journal at 0x74000\n' \
                "$bin_size" "$bin_size" >&2
            exit 1
        fi
        if (( bin_size > 0x40000 )); then
            if [[ "$MODE_NAME" == "runtime-sensor" || "$MODE_NAME" == "runtime-router" ]]; then
                printf 'layout-check FAIL: %s .bin size=%d (0x%X) exceeds 256 KiB production/OTA slot\n' \
                    "$MODE_NAME" "$bin_size" "$bin_size" >&2
                exit 1
            fi
            printf 'layout-check WARN: .bin size=%d (0x%X) exceeds 256 KiB production/OTA slot\n' \
                "$bin_size" "$bin_size" >&2
        fi
    fi
    printf 'layout-check OK: ram_code=%d B cache=[0x%X..0x%X) _sdata=0x%X _ebss=0x%X _svc=[0x%X..0x%X] _irq_top=0x%X\n' \
        "$ramcode_len" "$ictag_start" "$icache_data_end" "$sdata" "$ebss" "$svc_bot" "$svc_top" "$irq_top"
}

cmd_check() {
    resolve_mode "${1:-sensor}"
    run_cargo check
}

cmd_build() {
    resolve_mode "${1:-sensor}"
    run_cargo rustc -- -C lto=fat -C opt-level=s -C codegen-units=1
    emit_bin
    verify_layout
    echo "Mode: ${MODE_NAME}"
    echo "ELF:  ${ELF_PATH}"
    echo "BIN:  ${BIN_PATH}"
}

cmd_flash() {
    resolve_mode "${1:-sensor}"
    cmd_build "$MODE_NAME"
    require_file "$TLSRPGM" "TlsrPgm.py"
    python3 "$TLSRPGM" -p "$TELINK_PORT" -t 500 -a 200 -m we 0 "$BIN_PATH"
}

cmd_dump() {
    local address="${1:?dump requires an address}"
    local words="${2:-16}"
    require_file "$TLSR_DEBUG" "tlsr_debug.py"
    python3 "$TLSR_DEBUG" -p "$TELINK_PORT" read "$address" "$words"
}

cmd_dump_activate() {
    local address="${1:?dump-activate requires an address}"
    local words="${2:-16}"
    require_file "$TLSR_DEBUG" "tlsr_debug.py"
    python3 "$TLSR_DEBUG" -p "$TELINK_PORT" --activate read "$address" "$words"
}

cmd_dump_boot() {
    cmd_dump "$DBG_BOOT_BASE" "${1:-16}"
}

cmd_dump_mode() {
    cmd_dump "$DBG_MODE_BASE" "${1:-20}"
}

cmd_pgm_info() {
    require_file "$TLSRPGM" "TlsrPgm.py"
    python3 "$TLSRPGM" -p "$TELINK_PORT" -t 500 -a 200 i
}

cmd_pgm_dump() {
    local address="${1:?pgm-dump requires an address}"
    local bytes="${2:-128}"
    require_file "$TLSRPGM" "TlsrPgm.py"
    python3 "$TLSRPGM" -p "$TELINK_PORT" -t 500 -a 200 -s df "$address" "$bytes"
}

cmd_pgm_break() {
    local address="${1:?pgm-break requires an address}"
    require_file "$TLSRPGM" "TlsrPgm.py"
    python3 "$TLSRPGM" -p "$TELINK_PORT" -t 500 -a 200 bkp "$address"
}

cmd_pgm_step() {
    local count="${1:-1}"
    if (( count < 1 )); then
        echo "pgm-step count must be >= 1" >&2
        exit 1
    fi
    require_file "$TLSRPGM" "TlsrPgm.py"
    python3 "$TLSRPGM" -p "$TELINK_PORT" -a 200 stp "$count"
}

cmd_pgm_go() {
    require_file "$TLSRPGM" "TlsrPgm.py"
    python3 "$TLSRPGM" -p "$TELINK_PORT" -a 200 -g i
}

cmd_probe_list() {
    require_file "$PROBE_RS" "probe-rs"
    "$PROBE_RS" list
}

cmd_probe_info() {
    require_file "$PROBE_RS" "probe-rs"
    "$PROBE_RS" info --probe "$PROBE_RS_PROBE" --chip "$PROBE_RS_CHIP" --protocol "$PROBE_RS_PROTOCOL"
}

cmd_probe_attach() {
    resolve_mode "${1:-sensor}"
    cmd_build "$MODE_NAME"
    require_file "$PROBE_RS" "probe-rs"
    "$PROBE_RS" attach \
        --probe "$PROBE_RS_PROBE" \
        --chip "$PROBE_RS_CHIP" \
        --protocol "$PROBE_RS_PROTOCOL" \
        --scan-region "$PROBE_RS_SCAN_REGION" \
        "$ELF_PATH"
}

cmd_probe_list_rtt() {
    resolve_mode "${1:-sensor}"
    cmd_build "$MODE_NAME"
    require_file "$PROBE_RS" "probe-rs"
    "$PROBE_RS" attach \
        --probe "$PROBE_RS_PROBE" \
        --chip "$PROBE_RS_CHIP" \
        --protocol "$PROBE_RS_PROTOCOL" \
        --scan-region "$PROBE_RS_SCAN_REGION" \
        --list-rtt \
        "$ELF_PATH"
}

cmd_probe_debug() {
    resolve_mode "${1:-sensor}"
    cmd_build "$MODE_NAME"
    require_file "$PROBE_RS" "probe-rs"
    "$PROBE_RS" debug --probe "$PROBE_RS_PROBE" --chip "$PROBE_RS_CHIP" --protocol "$PROBE_RS_PROTOCOL" "$ELF_PATH"
}

cmd_probe_gdb() {
    resolve_mode "${1:-sensor}"
    cmd_build "$MODE_NAME"
    require_file "$PROBE_RS" "probe-rs"
    "$PROBE_RS" gdb --probe "$PROBE_RS_PROBE" --chip "$PROBE_RS_CHIP" --protocol "$PROBE_RS_PROTOCOL" "$ELF_PATH"
}

main() {
    local command="${1:-}"
    shift || true

    case "$command" in
        check) cmd_check "$@" ;;
        build) cmd_build "$@" ;;
        flash) cmd_flash "$@" ;;
        dump) cmd_dump "$@" ;;
        dump-activate) cmd_dump_activate "$@" ;;
        dump-boot) cmd_dump_boot "$@" ;;
        dump-mode) cmd_dump_mode "$@" ;;
        pgm-info) cmd_pgm_info "$@" ;;
        pgm-dump) cmd_pgm_dump "$@" ;;
        pgm-break) cmd_pgm_break "$@" ;;
        pgm-step) cmd_pgm_step "$@" ;;
        pgm-go) cmd_pgm_go "$@" ;;
        probe-list) cmd_probe_list "$@" ;;
        probe-info) cmd_probe_info "$@" ;;
        probe-attach) cmd_probe_attach "$@" ;;
        probe-list-rtt) cmd_probe_list_rtt "$@" ;;
        probe-debug) cmd_probe_debug "$@" ;;
        probe-gdb) cmd_probe_gdb "$@" ;;
        -h|--help|help|"") usage ;;
        *)
            echo "Unsupported command: ${command}" >&2
            usage >&2
            exit 1
            ;;
    esac
}

main "$@"
