//! Fixed-address SRAM diagnostic record for the TLSR8258 raw-radio bring-up
//! firmware.
//!
//! ## Placement
//!
//! The record lives in the `.diag` linker section defined in `memory.x`, at
//! the fixed, documented address **`0x0084FE00`** (512 bytes, ending at the
//! top of the 64 KiB SRAM window, `0x00850000`). It is `NOLOAD` (no
//! initializer copied from flash) so the assembly startup's `.data`-copy /
//! `.bss`-zero loops — which only touch `[_sdata, _edata)` and
//! `[_sbss, _ebss)` — never touch it. Both stacks are placed *below* this
//! address and grow downward (away from it), so a stack overflow cannot
//! silently corrupt the diagnostics before it corrupts something else first.
//!
//! ## Survival across resets
//!
//! Because the section is `NOLOAD`, its bytes are whatever SRAM already
//! contained at reset — which is the previous run's diagnostic record for
//! any reset that does not power-cycle SRAM (watchdog reset, SWire reset,
//! debugger reset). [`init`] validates `magic`/`version`/`checksum` and only
//! resets the record deterministically (zero + fresh header) when that
//! validation fails, e.g. on a genuine cold power-on where SRAM content is
//! undefined, or after a firmware/layout version bump.
//!
//! ## Cache-boundary canary
//!
//! [`CACHE_CANARY`] is placed in its own `.data.canary_first` input section,
//! which `memory.x` places ahead of the generic `.data`/`.data.*` wildcard.
//! That guarantees it is the first word of `.data`, i.e. it sits at exactly
//! `_icache_data_end_`. A TLSR8258 I-cache tag/data write overrun — the
//! historical bug class this bring-up is guarding against — corrupts this
//! word before any other static, making the overrun externally visible via
//! [`verify_cache_canary`] instead of silently corrupting program state.

/// Fixed SRAM address of the diagnostic record — see `memory.x` `.diag`.
/// Kept as a plain documented constant (not just a linker symbol) so this
/// address can be typed directly into `scripts/tlsr8258.sh dump`.
pub const DIAG_ADDR: u32 = 0x0084FE00;
pub const PANIC_MAGIC_ADDR: u32 = DIAG_ADDR + 0x1F8;
pub const PANIC_LR_ADDR: u32 = DIAG_ADDR + 0x1FC;

/// "TDIA" (Telink DIAgnostics), little-endian bytes 'T','D','I','A'.
pub const DIAG_MAGIC: u32 = 0x4149_4454;

/// Bump when [`DiagRecord`]'s layout or semantics change. A boot with a
/// stale version in SRAM is treated the same as a corrupt/cold-boot record.
pub const DIAG_VERSION: u16 = 26;

/// Cache-boundary canary value. Chosen to be recognizable in a hex dump and
/// asymmetric under byte-swap (so a byte-order mixup is also visible).
pub const CANARY_VALUE: u32 = 0xC4C3_C2C1;

/// First word of `.data`, forced to `_icache_data_end_` by the linker script.
/// `#[used]` prevents the optimizer from discarding it as dead (nothing else
/// reads this particular static directly — it is read back by address from
/// `_icache_data_end_`/`_sdata`, which are the same address by construction).
///
/// Declared `static mut` (accessed only through `addr_of_mut!` +
/// `read_volatile`, never through a `&`/`&mut` reference) rather than a
/// plain immutable `static`: LLVM treats a never-mutated, no-interior-
/// mutability `static` as a true constant regardless of `#[link_section]`,
/// which showed up as the ELF `.data` output section losing its `SHF_WRITE`
/// flag (verified via `llvm-readelf -S`, section flagged `AR` instead of
/// `WA`) even though the *address* placement was still correct. `static mut`
/// forces LLVM to treat it as a genuine mutable global, giving `.data` the
/// write flag a copy-on-boot section should have.
///
/// The custom `.data.canary_first` link section is an ELF/`memory.x` layout
/// detail specific to the real target; host builds (`cargo test`, any
/// `target_arch != "tc32"`) use the ordinary `.data`/Mach-O/PE section
/// instead, since there is no linker-script-driven placement to prove there.
#[cfg_attr(target_arch = "tc32", unsafe(link_section = ".data.canary_first"))]
#[used]
pub static mut CACHE_CANARY: u32 = CANARY_VALUE;

/// One-shot boolean packed as `u8` for a `#[repr(C)]`-stable, no-bitfield
/// diagnostic layout (bitfields have no fixed cross-compiler layout; plain
/// bytes do).
pub type DiagBool = u8;

pub const DIAG_TRUE: DiagBool = 1;
pub const DIAG_FALSE: DiagBool = 0;

/// mac_test state machine states, mirrored 1:1 with `mac_test::State`.
/// Kept as plain `u8` constants (rather than importing the enum) so this
/// module has no dependency on `mac_test`, keeping the module graph acyclic
/// and this file independently host-testable.
pub mod state {
    pub const BOOT: u8 = 0;
    pub const SET_CHANNEL: u8 = 1;
    pub const TX_BEACON_REQUEST: u8 = 2;
    pub const RX_WINDOW: u8 = 3;
    pub const NEXT_CHANNEL: u8 = 4;
    pub const ACTIVE_SCAN: u8 = 5;
    pub const ASSOCIATION_SCAN: u8 = 6;
    pub const ASSOCIATION_REQUEST: u8 = 7;
    pub const ASSOCIATION_DIRECT_WAIT: u8 = 8;
    pub const ASSOCIATION_POLL: u8 = 9;
    pub const ASSOCIATED: u8 = 10;
    pub const POST_ASSOCIATION_POLL: u8 = 11;
    pub const UNICAST_DATA_TX: u8 = 12;
    pub const STRESS: u8 = 13;
    pub const ASSOCIATION_FAILED: u8 = 14;
}

/// Fixed-layout diagnostic record. Every field has exactly one writer in the
/// firmware (documented at the call site), so no locking is needed beyond
/// "polling main loop, IRQs stay disabled" (see `platform::vectors`).
///
/// `repr(C)` with explicit reserved/padding fields keeps the layout stable
/// across compiler versions and matches how `scripts/tlsr8258.sh dump`/
/// `tlsr_debug.py` read it back as a flat word dump.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DiagRecord {
    /// [`DIAG_MAGIC`] when valid.
    pub magic: u32,
    /// [`DIAG_VERSION`] when valid.
    pub version: u16,
    pub _reserved0: u16,
    /// Additive checksum over every other field (see [`checksum_of`]).
    /// Computed last on every update so a torn write (e.g. power loss
    /// mid-update) is also detectable, not just cross-boot corruption.
    pub checksum: u32,

    /// Incremented by [`init`] every time the record is judged valid at
    /// boot; reset to 1 on a fresh/invalid record.
    pub boot_count: u32,
    /// Timer0 free-running tick snapshot, updated every state-machine step
    /// (see `platform::timer::now_ticks`). Used to externally confirm the
    /// firmware is alive and the timer is progressing, without needing RTT
    /// or UART.
    pub uptime_ticks: u32,

    /// Current `mac_test::State` (see the [`state`] module for values).
    pub state: u8,
    /// Current channel (11, 18, or 26).
    pub channel: u8,
    /// Index into the channel cycle, 0..=2.
    pub channel_index: u8,
    /// Last MAC sequence number used for a Beacon Request.
    pub last_seq: u8,

    pub tx_success_count: u32,
    pub tx_timeout_count: u32,

    pub beacons_ch11: u32,
    pub beacons_ch18: u32,
    pub beacons_ch26: u32,
    pub beacons_control: u32,

    pub invalid_length_count: u32,
    pub invalid_crc_count: u32,

    /// Total MAC PSDU length of the last frame examined (valid or not).
    pub last_frame_len: u8,
    /// LQI derived from the last frame's RSSI (see `radio::rssi_to_lqi`).
    pub last_frame_lqi: u8,
    /// RSSI in dBm of the last frame examined.
    pub last_rssi: i8,
    /// [`DIAG_TRUE`] if the last frame examined was accepted as a
    /// length-valid, CRC-valid Beacon frame.
    pub last_valid_beacon: DiagBool,

    pub last_pan_id: u16,
    pub last_coord_short: u16,
    /// Extended (IEEE) coordinator address, if the beacon's source
    /// addressing mode was extended; all-zero otherwise.
    pub last_coord_ext: [u8; 8],

    /// Result of the most recent [`verify_cache_canary`] call.
    pub cache_canary_ok: DiagBool,
    /// Result of the most recent `.data` initialization spot-check
    /// (verified once at boot in [`init`]).
    pub data_init_ok: DiagBool,
    /// Result of the most recent `.bss` zero-fill spot-check (verified once
    /// at boot in [`init`]).
    pub bss_zero_ok: DiagBool,
    pub _reserved1: u8,

    /// Number of complete 11 -> 18 -> 26 channel cycles finished.
    pub cycles_completed: u32,

    pub last_scan_ticks: u32,
    pub scan_descriptors_found: u32,
    pub last_extended_pan_id: [u8; 8],
    pub scan_duration_exponent: u8,
    pub scan_dwell_ok: DiagBool,
    pub last_protocol_id: u8,
    pub last_stack_profile: u8,
    pub last_protocol_version: u8,
    pub last_device_depth: u8,
    pub last_router_capacity: DiagBool,
    pub last_end_device_capacity: DiagBool,
    pub last_update_id: u8,
    pub last_association_permit: DiagBool,
    pub ieee_source: u8,
    pub _reserved2: u8,

    pub factory_ieee: [u8; 8],
    pub factory_ieee_valid: DiagBool,
    pub association_status: u8,
    pub association_channel: u8,
    pub association_parent_lqi: u8,
    pub assigned_short_address: u16,
    pub _reserved3: u16,
    pub association_attempt_count: u32,
    pub association_ack_count: u32,
    pub association_data_request_count: u32,
    pub association_response_count: u32,
    pub association_frame_pending_count: u32,
    pub last_ack_latency_ticks: u32,
    pub min_ack_latency_ticks: u32,
    pub max_ack_latency_ticks: u32,
    pub empty_poll_attempt_count: u32,
    pub empty_poll_ack_count: u32,
    pub empty_poll_no_pending_count: u32,
    pub unicast_data_attempt_count: u32,
    pub unicast_data_ack_count: u32,
    pub software_ack_tx_count: u32,
    pub software_ack_timeout_count: u32,
    pub stress_cycles_completed: u32,
    pub stress_poll_ack_count: u32,
    pub stress_empty_poll_count: u32,
    pub stress_unicast_ack_count: u32,
    pub stress_failure_count: u32,
    pub tx_invalid_frame_count: u32,
    pub cca_attempt_count: u32,
    pub cca_busy_count: u32,
    pub channel_access_failure_count: u32,
    pub frame_retry_count: u32,
    /// Last stress failure: bit 31 selects unicast (clear = poll), bits
    /// 8..=30 contain the cycle, and bits 0..=7 contain the MAC sequence.
    pub last_stress_failure: u32,
}

impl DiagRecord {
    pub const SIZE: usize = core::mem::size_of::<DiagRecord>();

    /// A fresh record: valid header, all counters zeroed, `boot_count = 1`.
    pub fn fresh() -> Self {
        let mut r = DiagRecord {
            magic: DIAG_MAGIC,
            version: DIAG_VERSION,
            _reserved0: 0,
            checksum: 0,
            boot_count: 1,
            uptime_ticks: 0,
            state: state::BOOT,
            channel: 0,
            channel_index: 0,
            last_seq: 0,
            tx_success_count: 0,
            tx_timeout_count: 0,
            beacons_ch11: 0,
            beacons_ch18: 0,
            beacons_ch26: 0,
            beacons_control: 0,
            invalid_length_count: 0,
            invalid_crc_count: 0,
            last_frame_len: 0,
            last_frame_lqi: 0,
            last_rssi: 0,
            last_valid_beacon: DIAG_FALSE,
            last_pan_id: 0,
            last_coord_short: 0,
            last_coord_ext: [0; 8],
            cache_canary_ok: DIAG_FALSE,
            data_init_ok: DIAG_FALSE,
            bss_zero_ok: DIAG_FALSE,
            _reserved1: 0,
            cycles_completed: 0,
            last_scan_ticks: 0,
            scan_descriptors_found: 0,
            last_extended_pan_id: [0; 8],
            scan_duration_exponent: 0,
            scan_dwell_ok: DIAG_FALSE,
            last_protocol_id: 0,
            last_stack_profile: 0,
            last_protocol_version: 0,
            last_device_depth: 0,
            last_router_capacity: DIAG_FALSE,
            last_end_device_capacity: DIAG_FALSE,
            last_update_id: 0,
            last_association_permit: DIAG_FALSE,
            ieee_source: 0,
            _reserved2: 0,
            factory_ieee: [0; 8],
            factory_ieee_valid: DIAG_FALSE,
            association_status: 0xFF,
            association_channel: 0,
            association_parent_lqi: 0,
            assigned_short_address: 0xFFFF,
            _reserved3: 0,
            association_attempt_count: 0,
            association_ack_count: 0,
            association_data_request_count: 0,
            association_response_count: 0,
            association_frame_pending_count: 0,
            last_ack_latency_ticks: 0,
            min_ack_latency_ticks: u32::MAX,
            max_ack_latency_ticks: 0,
            empty_poll_attempt_count: 0,
            empty_poll_ack_count: 0,
            empty_poll_no_pending_count: 0,
            unicast_data_attempt_count: 0,
            unicast_data_ack_count: 0,
            software_ack_tx_count: 0,
            software_ack_timeout_count: 0,
            stress_cycles_completed: 0,
            stress_poll_ack_count: 0,
            stress_empty_poll_count: 0,
            stress_unicast_ack_count: 0,
            stress_failure_count: 0,
            tx_invalid_frame_count: 0,
            cca_attempt_count: 0,
            cca_busy_count: 0,
            channel_access_failure_count: 0,
            frame_retry_count: 0,
            last_stress_failure: 0,
        };
        r.checksum = checksum_of(&r);
        r
    }

    /// `true` if `magic`/`version`/`checksum` are all self-consistent.
    pub fn is_valid(&self) -> bool {
        self.magic == DIAG_MAGIC
            && self.version == DIAG_VERSION
            && self.checksum == checksum_of(self)
    }

    /// Recompute and store the checksum. Call after mutating any field.
    pub fn seal(&mut self) {
        self.checksum = checksum_of(self);
    }
}

/// Wrapping 32-bit additive checksum over every field of `r` except
/// `checksum` itself. Deliberately simple (not CRC32) — this only needs to
/// catch torn writes and cross-boot corruption, not adversarial inputs.
pub fn checksum_of(r: &DiagRecord) -> u32 {
    let mut sum = 0u32;
    sum = sum.wrapping_add(r.magic);
    sum = sum.wrapping_add(r.version as u32);
    sum = sum.wrapping_add(r.boot_count);
    sum = sum.wrapping_add(r.uptime_ticks);
    sum = sum.wrapping_add(r.state as u32);
    sum = sum.wrapping_add(r.channel as u32);
    sum = sum.wrapping_add(r.channel_index as u32);
    sum = sum.wrapping_add(r.last_seq as u32);
    sum = sum.wrapping_add(r.tx_success_count);
    sum = sum.wrapping_add(r.tx_timeout_count);
    sum = sum.wrapping_add(r.beacons_ch11);
    sum = sum.wrapping_add(r.beacons_ch18);
    sum = sum.wrapping_add(r.beacons_ch26);
    sum = sum.wrapping_add(r.beacons_control);
    sum = sum.wrapping_add(r.invalid_length_count);
    sum = sum.wrapping_add(r.invalid_crc_count);
    sum = sum.wrapping_add(r.last_frame_len as u32);
    sum = sum.wrapping_add(r.last_frame_lqi as u32);
    sum = sum.wrapping_add(r.last_rssi as u8 as u32);
    sum = sum.wrapping_add(r.last_valid_beacon as u32);
    sum = sum.wrapping_add(r.last_pan_id as u32);
    sum = sum.wrapping_add(r.last_coord_short as u32);
    for b in r.last_coord_ext {
        sum = sum.wrapping_add(b as u32);
    }
    sum = sum.wrapping_add(r.cache_canary_ok as u32);
    sum = sum.wrapping_add(r.data_init_ok as u32);
    sum = sum.wrapping_add(r.bss_zero_ok as u32);
    sum = sum.wrapping_add(r.cycles_completed);
    sum = sum.wrapping_add(r.last_scan_ticks);
    sum = sum.wrapping_add(r.scan_descriptors_found);
    for b in r.last_extended_pan_id {
        sum = sum.wrapping_add(b as u32);
    }
    sum = sum.wrapping_add(r.scan_duration_exponent as u32);
    sum = sum.wrapping_add(r.scan_dwell_ok as u32);
    sum = sum.wrapping_add(r.last_protocol_id as u32);
    sum = sum.wrapping_add(r.last_stack_profile as u32);
    sum = sum.wrapping_add(r.last_protocol_version as u32);
    sum = sum.wrapping_add(r.last_device_depth as u32);
    sum = sum.wrapping_add(r.last_router_capacity as u32);
    sum = sum.wrapping_add(r.last_end_device_capacity as u32);
    sum = sum.wrapping_add(r.last_update_id as u32);
    sum = sum.wrapping_add(r.last_association_permit as u32);
    sum = sum.wrapping_add(r.ieee_source as u32);
    for b in r.factory_ieee {
        sum = sum.wrapping_add(b as u32);
    }
    sum = sum.wrapping_add(r.factory_ieee_valid as u32);
    sum = sum.wrapping_add(r.association_status as u32);
    sum = sum.wrapping_add(r.association_channel as u32);
    sum = sum.wrapping_add(r.association_parent_lqi as u32);
    sum = sum.wrapping_add(r.assigned_short_address as u32);
    sum = sum.wrapping_add(r.association_attempt_count);
    sum = sum.wrapping_add(r.association_ack_count);
    sum = sum.wrapping_add(r.association_data_request_count);
    sum = sum.wrapping_add(r.association_response_count);
    sum = sum.wrapping_add(r.association_frame_pending_count);
    sum = sum.wrapping_add(r.last_ack_latency_ticks);
    sum = sum.wrapping_add(r.min_ack_latency_ticks);
    sum = sum.wrapping_add(r.max_ack_latency_ticks);
    sum = sum.wrapping_add(r.empty_poll_attempt_count);
    sum = sum.wrapping_add(r.empty_poll_ack_count);
    sum = sum.wrapping_add(r.empty_poll_no_pending_count);
    sum = sum.wrapping_add(r.unicast_data_attempt_count);
    sum = sum.wrapping_add(r.unicast_data_ack_count);
    sum = sum.wrapping_add(r.software_ack_tx_count);
    sum = sum.wrapping_add(r.software_ack_timeout_count);
    sum = sum.wrapping_add(r.stress_cycles_completed);
    sum = sum.wrapping_add(r.stress_poll_ack_count);
    sum = sum.wrapping_add(r.stress_empty_poll_count);
    sum = sum.wrapping_add(r.stress_unicast_ack_count);
    sum = sum.wrapping_add(r.stress_failure_count);
    sum = sum.wrapping_add(r.tx_invalid_frame_count);
    sum = sum.wrapping_add(r.cca_attempt_count);
    sum = sum.wrapping_add(r.cca_busy_count);
    sum = sum.wrapping_add(r.channel_access_failure_count);
    sum = sum.wrapping_add(r.frame_retry_count);
    sum = sum.wrapping_add(r.last_stress_failure);
    sum
}

/// Pure check: does `canary` equal the expected [`CANARY_VALUE`]? Split out
/// from the MMIO read in `verify_cache_canary` so the comparison logic is
/// host-testable without a linked memory image.
pub fn canary_matches(canary: u32) -> bool {
    canary == CANARY_VALUE
}

#[cfg(target_arch = "tc32")]
mod hw {
    use super::*;
    use core::ptr::{read_volatile, write_volatile};

    #[inline(always)]
    fn record_ptr() -> *mut DiagRecord {
        DIAG_ADDR as *mut DiagRecord
    }

    /// Validate the record found at [`DIAG_ADDR`] and either bump
    /// `boot_count` (valid) or reset it deterministically (invalid/cold
    /// boot). Must run after `.data`/`.bss` init but does not itself depend
    /// on either, since `.diag` is a separate NOINIT section.
    pub fn init() {
        unsafe {
            write_volatile(PANIC_MAGIC_ADDR as *mut u32, 0);
            write_volatile(PANIC_LR_ADDR as *mut u32, 0);
            let p = record_ptr();
            let existing = read_volatile(p);
            let mut r = if existing.is_valid() {
                let mut r = existing;
                r.boot_count = r.boot_count.wrapping_add(1);
                r
            } else {
                DiagRecord::fresh()
            };
            r.data_init_ok = if crate::platform::linker::data_init_ok() {
                DIAG_TRUE
            } else {
                DIAG_FALSE
            };
            r.bss_zero_ok = if crate::platform::linker::bss_zero_ok() {
                DIAG_TRUE
            } else {
                DIAG_FALSE
            };
            r.seal();
            write_volatile(p, r);
        }
    }

    /// Read-modify-write helper: every diagnostic update goes through this
    /// so `seal()` (checksum recompute) can never be forgotten.
    pub fn update(f: impl FnOnce(&mut DiagRecord)) {
        unsafe {
            let p = record_ptr();
            let mut r = read_volatile(p);
            f(&mut r);
            r.seal();
            write_volatile(p, r);
        }
    }

    pub fn snapshot() -> DiagRecord {
        unsafe { read_volatile(record_ptr()) }
    }

    /// Read the live cache-boundary canary word and record the result in
    /// the diagnostic record. Called periodically from `mac_test::run`.
    pub fn verify_cache_canary() {
        let canary = unsafe { read_volatile(core::ptr::addr_of_mut!(CACHE_CANARY)) };
        let ok = super::canary_matches(canary);
        update(|r| r.cache_canary_ok = if ok { DIAG_TRUE } else { DIAG_FALSE });
    }
}

#[cfg(target_arch = "tc32")]
pub use hw::{init, snapshot, update, verify_cache_canary};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fresh_record_is_valid() {
        let r = DiagRecord::fresh();
        assert!(r.is_valid());
        assert_eq!(r.boot_count, 1);
        assert_eq!(r.magic, DIAG_MAGIC);
    }

    #[test]
    fn mutation_without_reseal_is_detected() {
        let mut r = DiagRecord::fresh();
        r.tx_success_count = 42;
        // Checksum now stale: is_valid() must fail until seal() runs again.
        assert!(!r.is_valid());
        r.seal();
        assert!(r.is_valid());
    }

    #[test]
    fn wrong_magic_is_invalid() {
        let mut r = DiagRecord::fresh();
        r.magic = 0xDEAD_BEEF;
        assert!(!r.is_valid());
    }

    #[test]
    fn wrong_version_is_invalid() {
        let mut r = DiagRecord::fresh();
        r.version += 1;
        assert!(!r.is_valid());
    }

    #[test]
    fn canary_value_matches_itself() {
        assert!(canary_matches(CANARY_VALUE));
        assert!(!canary_matches(CANARY_VALUE ^ 0xFFFF_FFFF));
        assert!(!canary_matches(0));
    }

    #[test]
    fn record_fits_reserved_diag_section() {
        // memory.x reserves 512 bytes for `.diag`; leave headroom for future
        // fields instead of using every last byte.
        assert!(
            DiagRecord::SIZE <= 256,
            "DiagRecord grew past the documented budget"
        );
    }

    #[test]
    fn checksum_changes_when_any_counted_field_changes() {
        let base = DiagRecord::fresh();
        let base_sum = checksum_of(&base);

        let mut a = base;
        a.invalid_crc_count = 1;
        assert_ne!(checksum_of(&a), base_sum);

        let mut b = base;
        b.last_coord_ext[7] = 0xFF;
        assert_ne!(checksum_of(&b), base_sum);
    }
}
