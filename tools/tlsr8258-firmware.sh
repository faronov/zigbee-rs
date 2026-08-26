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
    echo "usage: $0 <check|build|flash> <crate-directory> <binary-name> [cargo-feature]" >&2
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
    local image_feature="${3:-}"
    local ramcode_start=0 ramcode_end=0 ramcode_aligned=0
    local ictag_start=0 ictag_end=0 icache_data_end=0
    local sdata=0 ebss=0 svc_bottom=0 svc_top=0 irq_bottom=0 irq_top=0
    local rf_dma_start=0 rf_dma_end=0
    local retained_start=0 retained_end=0 guard_start=0 guard_end=0 retention_limit=0
    local rf_rx_buf=0 rf_tx_buf=0 rf_ack_tx_buf=0
    local security_nv_start=0 security_nv_end=0
    local child_nv_start=0 child_nv_end=0
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
            _svc_stack_top) svc_top=$((16#$value)) ;;
            _irq_stack_bottom) irq_bottom=$((16#$value)) ;;
            _irq_stack_top) irq_top=$((16#$value)) ;;
            _retained_start_) retained_start=$((16#$value)) ;;
            _retained_end_) retained_end=$((16#$value)) ;;
            _retention_stack_guard_start_) guard_start=$((16#$value)) ;;
            _retention_stack_guard_end_) guard_end=$((16#$value)) ;;
            _retention_limit_) retention_limit=$((16#$value)) ;;
            _rf_dma_start_) rf_dma_start=$((16#$value)) ;;
            _rf_dma_end_) rf_dma_end=$((16#$value)) ;;
            *RF_RX_BUF) rf_rx_buf=$((16#$value)) ;;
            *RF_TX_BUF) rf_tx_buf=$((16#$value)) ;;
            *RF_ACK_TX_BUF) rf_ack_tx_buf=$((16#$value)) ;;
            _security_nv_start_) security_nv_start=$((16#$value)) ;;
            _security_nv_end_) security_nv_end=$((16#$value)) ;;
            _child_nv_start_) child_nv_start=$((16#$value)) ;;
            _child_nv_end_) child_nv_end=$((16#$value)) ;;
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
        /aes::soft|SoftwareAes128/ { found = 1 }
        END { exit(found ? 0 : 1) }
    '; then
        echo "layout-check FAIL: firmware links software AES" >&2
        exit 1
    fi
    for required in HardwareAes128 install_aes_engine; do
        if ! "$LLVM_NM" -C "$elf" | awk -v required="$required" '
            index($0, required) { found = 1 }
            END { exit(found ? 0 : 1) }
        '; then
            printf 'layout-check FAIL: missing hardware AES symbol %s\n' "$required" >&2
            exit 1
        fi
    done
    if [[ "$binary_name" == "telink-tlsr8258-sensor" ]]; then
        if "$LLVM_NM" -C "$elf" | awk '
            /parent_beacon_response|handle_parent_command|handle_child_rejoin_request|queue_pending_child_update|process_pending_routing|send_link_status|ParentRouterApp/ {
                found = 1
            }
            END { exit(found ? 0 : 1) }
        '; then
            echo "layout-check FAIL: end-device sensor links parent/router lifecycle" >&2
            exit 1
        fi
        for required in \
            resume_end_device_timeout \
            service_end_device_timeout \
            apply_end_device_timeout_change \
            send_ed_timeout_request_tracked
        do
            if ! "$LLVM_NM" -C "$elf" | awk -v required="$required" '
                index($0, required) { found = 1 }
                END { exit(found ? 0 : 1) }
            '; then
                printf 'layout-check FAIL: missing End Device lifecycle symbol %s\n' \
                    "$required" >&2
                exit 1
            fi
        done
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
    local regression_budget=0
    case "$binary_name:$image_feature" in
        telink-tlsr8258-sensor:) regression_budget=294912 ;;
        telink-tlsr8258-sensor:retention-proof*) regression_budget=299008 ;;
        telink-tlsr8258-router:) regression_budget=356352 ;;
    esac
    if (( regression_budget != 0 && size > regression_budget )); then
        printf 'layout-check FAIL: %s image=%d exceeds %d-byte regression gate\n' \
            "$binary_name" "$size" "$regression_budget" >&2
        exit 1
    fi
    if (( child_nv_start == 0 || size > child_nv_start )); then
        printf 'layout-check FAIL: image is %d bytes, child-table journal starts at 0x%X\n' \
            "$size" "$child_nv_start" >&2
        exit 1
    fi
    # The journals must be ordered below Telink's factory EUI/config sectors.
    # Reaching 0x76000 would erase the device identity; reaching 0x77000 would
    # also erase factory configuration and ADC calibration.
    if (( child_nv_end == 0 || security_nv_start == 0 || security_nv_end == 0 )); then
        echo "layout-check FAIL: NV journal symbols are missing" >&2
        exit 1
    fi
    if (( child_nv_start >= child_nv_end ||
          security_nv_start >= security_nv_end ||
          child_nv_end > security_nv_start ||
          security_nv_end > 0x76000 )); then
        printf 'layout-check FAIL: child NV [0x%X..0x%X), security NV [0x%X..0x%X), factory EUI starts at 0x76000\n' \
            "$child_nv_start" "$child_nv_end" "$security_nv_start" "$security_nv_end" >&2
        exit 1
    fi

    if [[ "$image_feature" == retention-proof* ]]; then
        if (( retention_limit != 0x848000 ||
              sdata >= retention_limit || ebss > retained_start ||
              retained_end > rf_dma_start || rf_dma_end > guard_start ||
              guard_end - guard_start < 0x100 || guard_end > svc_bottom ||
              svc_top > irq_bottom || irq_top > retention_limit ||
              retention_limit - irq_top < 0x400 ||
              svc_top - svc_bottom < 0x2000 )); then
            printf 'retention-layout FAIL: data=0x%X bss=0x%X retained=[0x%X..0x%X) dma=[0x%X..0x%X) guard=[0x%X..0x%X) svc=[0x%X..0x%X) irq=[0x%X..0x%X) limit=0x%X\n' \
                "$sdata" "$ebss" "$retained_start" "$retained_end" \
                "$rf_dma_start" "$rf_dma_end" "$guard_start" "$guard_end" \
                "$svc_bottom" "$svc_top" "$irq_bottom" "$irq_top" "$retention_limit" >&2
            exit 1
        fi

        local required
        for required in \
            "TELINK_RETENTION_IMAGE" \
            "TELINK_RETAINED_DEVICE_STORAGE" \
            "TELINK_RETAINED_PROFILE_STORAGE" \
            "TELINK_RETAINED_SECURITY_STORAGE" \
            "TELINK_RETAINED_APP_STORAGE" \
            "_rust_cold_entry" \
            "_rust_retention_entry" \
            "_rust_retention_fault_entry" \
            "__tlsr8258_retention_probe" \
            "TELINK_RETENTION_FRESH_ROOT" \
            "cpu_sleep_timer_rc_retention_transaction" \
            "begin_low32k_resume" \
            "complete_low32k_resume" \
            "resume_after_retention"
        do
            if ! "$LLVM_NM" -C "$elf" | awk -v required="$required" '
                index($0, required) { found = 1 }
                END { exit(found ? 0 : 1) }
            '; then
                printf 'retention-symbol FAIL: missing %s\n' "$required" >&2
                exit 1
            fi
        done
        if "$LLVM_NM" -C "$elf" | awk '
            /parent_beacon_response|handle_parent_command|handle_child_rejoin_request|queue_pending_child_update|process_pending_routing|send_link_status|ParentRouterApp/ {
                found = 1
            }
            END { exit(found ? 0 : 1) }
        '; then
            echo "retention-role FAIL: sensor links parent/router lifecycle" >&2
            exit 1
        fi
        local block_on_count
        block_on_count=$("$LLVM_NM" -C "$elf" | awk '
            / [tT] tlsr8258_rt::block_on::</ { count++ }
            END { print count + 0 }
        ')
        if (( block_on_count != 1 )); then
            printf 'retention-root FAIL: expected one block_on monomorph, found %d\n' \
                "$block_on_count" >&2
            exit 1
        fi
        echo "retention-layout OK: all writable/DMA/stacks below LOW32K; fresh-root and restore symbols present"
    elif [[ "$binary_name" == "telink-tlsr8258-sensor" ]]; then
        if (( svc_bottom != 0x84BC00 || svc_top != 0x84FC00 ||
              irq_bottom != 0x84FC00 || irq_top != 0x850000 )); then
            echo "layout-check FAIL: default sensor no longer uses the validated full-SRAM stack layout" >&2
            exit 1
        fi
        if "$LLVM_NM" -C "$elf" | awk '
            /TELINK_RETENTION_IMAGE|cpu_sleep_timer_rc_retention_transaction|_rust_retention_entry/ {
                found = 1
            }
            END { exit(found ? 0 : 1) }
        '; then
            echo "layout-check FAIL: default sensor links the feature-gated LOW32K path" >&2
            exit 1
        fi
        for required in cpu_suspend_timer_rc_transaction resume_after_sleep rebase_after_suspend; do
            if ! "$LLVM_NM" -C "$elf" | awk -v required="$required" '
                index($0, required) { found = 1 }
                END { exit(found ? 0 : 1) }
            '; then
                printf 'layout-check FAIL: default Idle path is missing %s\n' "$required" >&2
                exit 1
            fi
        done
        echo "default-idle OK: LOW32K absent; full-SRAM atomic SUSPEND retained"
    fi

    printf 'layout-check OK: image=%d B flash_limit=0x%X ram_code=%d B data=0x%X bss_end=0x%X rf_dma=[0x%X..0x%X) (rx=0x%X tx=0x%X ack=0x%X) sec_nv=[0x%X..0x%X) child_nv=[0x%X..0x%X)\n' \
        "$size" "$child_nv_start" "$((ramcode_end - ramcode_start))" "$sdata" "$ebss" \
        "$rf_dma_start" "$rf_dma_end" "$rf_rx_buf" "$rf_tx_buf" "$rf_ack_tx_buf" \
        "$security_nv_start" "$security_nv_end" "$child_nv_start" "$child_nv_end"
}

[[ $# -eq 3 || $# -eq 4 ]] || usage
command="$1"
crate_dir="$2"
binary_name="$3"
image_feature="${4:-}"

if [[ "$crate_dir" != /* ]]; then
    crate_dir="${ROOT_DIR}/${crate_dir}"
fi
require_file "$crate_dir/Cargo.toml" "Cargo manifest"
require_file "$CARGO_BIN" "tc32 cargo"

target_dir="${crate_dir}/target/tc32-unknown-none-elf/release"
elf="${target_dir}/${binary_name}"
case "$image_feature" in
    retention-proof) bin="${elf}.retention-250.bin" ;;
    retention-proof-10s) bin="${elf}.retention-10s.bin" ;;
    *) bin="${elf}.bin" ;;
esac

case "$command" in
    check)
        (
            cd "$crate_dir"
            if [[ -n "$image_feature" ]]; then
                "$CARGO_BIN" check --release --locked --no-default-features \
                    --features "$image_feature" --bin "$binary_name"
            else
                "$CARGO_BIN" check --release --locked --bin "$binary_name"
            fi
        )
        ;;
    build|flash)
        (
            cd "$crate_dir"
            if [[ -n "$image_feature" ]]; then
                "$CARGO_BIN" rustc --release --locked --no-default-features \
                    --features "$image_feature" --bin "$binary_name" -- \
                    -C lto=fat -C opt-level=s -C codegen-units=1
            else
                "$CARGO_BIN" rustc --release --locked --bin "$binary_name" -- \
                    -C lto=fat -C opt-level=s -C codegen-units=1
            fi
        )
        require_file "$LLVM_OBJCOPY" "llvm-objcopy"
        require_file "$LLVM_NM" "llvm-nm"
        "$LLVM_OBJCOPY" -O binary "$elf" "$bin"
        verify_layout "$elf" "$bin" "$image_feature"
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
