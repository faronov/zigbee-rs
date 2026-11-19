//! RF core command and data structures (SDK 8.33.00.16
//! `driverlib/rf_common_cmd.h`, `rf_ieee_cmd.h`, `rf_mailbox.h`,
//! `rf_data_entry.h`, `rf_ieee_mailbox.h`).
//!
//! Pointers are stored as `u32` so the layout is identical on the host,
//! where `tests` pin every field offset to the GCC layout of the TI headers
//! (`tools/check_rf_layout.c` re-derives them with `offsetof`). Bit-field
//! bytes are built with the helper constants below.

/// Command IDs.
pub mod id {
    pub const CMD_RADIO_SETUP: u16 = 0x0802;
    pub const CMD_FS_POWERDOWN: u16 = 0x080D;
    pub const CMD_SYNC_STOP_RAT: u16 = 0x0809;
    pub const CMD_SYNC_START_RAT: u16 = 0x080A;
    pub const CMD_ABORT: u16 = 0x0401;
    pub const CMD_PING: u16 = 0x0406;
    pub const CMD_CLEAR_RX: u16 = 0x0008;
    pub const CMD_SET_TX_POWER: u16 = 0x0010;
    pub const CMD_SET_TX20_POWER: u16 = 0x0014;
    pub const CMD_CHANGE_PA: u16 = 0x0015;
    pub const CMD_IEEE_RX: u16 = 0x2801;
    pub const CMD_IEEE_ED_SCAN: u16 = 0x2802;
    pub const CMD_IEEE_TX: u16 = 0x2C01;
    pub const CMD_IEEE_CSMA: u16 = 0x2C02;
    pub const CMD_IEEE_RX_ACK: u16 = 0x2C03;
    pub const CMD_IEEE_MOD_FILT: u16 = 0x2002;
    pub const CMD_IEEE_MOD_SRC_MATCH: u16 = 0x2003;
    pub const CMD_IEEE_ABORT_FG: u16 = 0x2401;
    /// RF core RAM/clock control (`RF_CMD0` of the TI RF driver).
    pub const CMD0: u16 = 0x0607;
}

/// Radio operation status words.
pub mod status {
    pub const IDLE: u16 = 0x0000;
    pub const PENDING: u16 = 0x0001;
    pub const ACTIVE: u16 = 0x0002;
    pub const SKIPPED: u16 = 0x0003;
    pub const DONE_OK: u16 = 0x0400;
    pub const IEEE_SUSPENDED: u16 = 0x2001;
    pub const IEEE_DONE_OK: u16 = 0x2400;
    pub const IEEE_DONE_BUSY: u16 = 0x2401;
    pub const IEEE_DONE_STOPPED: u16 = 0x2402;
    pub const IEEE_DONE_ACK: u16 = 0x2403;
    pub const IEEE_DONE_ACKPEND: u16 = 0x2404;
    pub const IEEE_DONE_TIMEOUT: u16 = 0x2405;
    pub const IEEE_DONE_BGEND: u16 = 0x2406;
    pub const IEEE_DONE_ABORT: u16 = 0x2407;

    /// The operation has ended (done, error or skipped).
    pub const fn is_terminal(s: u16) -> bool {
        s == SKIPPED || s & 0x0C00 != 0
    }
}

/// `CMDSTA` result of a doorbell write.
pub const CMDSTA_DONE: u8 = 0x01;

/// Trigger bytes.
pub const TRIG_NOW: u8 = 0;
pub const TRIG_NEVER: u8 = 1;
pub const TRIG_REL_START: u8 = 4;
pub const TRIG_PAST: u8 = 0x80;
/// Condition bytes.
pub const COND_ALWAYS: u8 = 0;
pub const COND_NEVER: u8 = 1;
pub const COND_STOP_ON_FALSE: u8 = 2;

/// Data entry status (`rf_mailbox.h`).
pub const DATA_ENTRY_PENDING: u8 = 0;
pub const DATA_ENTRY_FINISHED: u8 = 3;
pub const DATA_ENTRY_UNFINISHED: u8 = 4;
/// General entry, one-byte length prefix (`config.type = 0, lenSz = 1`).
pub const DATA_ENTRY_CONFIG_GEN_LEN1: u8 = 1 << 2;

/// End of an override list.
pub const END_OVERRIDE: u32 = 0xFFFF_FFFF;

/// Define a radio operation: the common header (`commandNo`, `status`,
/// `pNextOp`, `startTime`, `startTrigger`, `condition`; 14 bytes) followed
/// by the command fields, flattened as in the TI headers.
macro_rules! radio_op {
    ($(#[$m:meta])* $name:ident { $($(#[$fm:meta])* $f:ident : $t:ty),* $(,)? }) => {
        $(#[$m])*
        #[repr(C, align(4))]
        #[derive(Clone, Copy, Debug, Default)]
        pub struct $name {
            pub command_no: u16,
            pub status: u16,
            pub next_op: u32,
            pub start_time: u32,
            pub start_trigger: u8,
            pub condition: u8,
            $($(#[$fm])* pub $f: $t),*
        }

        impl $name {
            /// Header with `TRIG_NOW` and `condition`; other fields zero.
            pub const fn with_header(command_no: u16, condition: u8) -> Self {
                // SAFETY: every field is an integer, so all-zero is valid.
                let mut op: Self = unsafe { core::mem::zeroed() };
                op.command_no = command_no;
                op.condition = condition;
                op
            }
        }
    };
}

radio_op! {
    /// `rfc_CMD_RADIO_SETUP_PA_t` (also used as `CMD_RADIO_SETUP` on non-P
    /// chips, which ignore the two PA override pointers).
    CmdRadioSetupPa {
        mode: u8,
        lo_divider: u8,
        config: u16,
        tx_power: u16,
        reg_override: u32,
        reg_override_tx_std: u32,
        reg_override_tx20: u32,
    }
}

radio_op! {
    /// `rfc_CMD_SYNC_START_RAT_t` / `rfc_CMD_SYNC_STOP_RAT_t`.
    CmdSyncRat {
        _dummy0: u16,
        rat0: u32,
    }
}

radio_op! {
    /// `rfc_CMD_FS_POWERDOWN_t`.
    CmdFsPowerdown {}
}

radio_op! {
    /// `rfc_CMD_IEEE_RX_t`.
    CmdIeeeRx {
        channel: u8,
        rx_config: u8,
        rx_queue: u32,
        output: u32,
        frame_filt_opt: u16,
        frame_types: u8,
        cca_opt: u8,
        cca_rssi_thr: i8,
        _dummy0: u8,
        num_ext_entries: u8,
        num_short_entries: u8,
        ext_entry_list: u32,
        short_entry_list: u32,
        /// EUI-64 in over-the-air byte order, as two little-endian words.
        local_ext_addr: [u32; 2],
        local_short_addr: u16,
        local_pan_id: u16,
        _dummy1: u16,
        _dummy2: u8,
        end_trigger: u8,
        end_time: u32,
    }
}

/// `rxConfig` bits of [`CmdIeeeRx`].
pub mod rx_config {
    pub const AUTO_FLUSH_CRC: u8 = 1 << 0;
    pub const AUTO_FLUSH_IGN: u8 = 1 << 1;
    pub const APPEND_RSSI: u8 = 1 << 4;
    pub const APPEND_CORR_CRC: u8 = 1 << 5;
}

/// `frameFiltOpt` bits of [`CmdIeeeRx`] / [`CmdIeeeModFilt`].
pub mod filt {
    pub const FRAME_FILT_EN: u16 = 1 << 0;
    pub const FRAME_FILT_STOP: u16 = 1 << 1;
    pub const AUTO_ACK_EN: u16 = 1 << 2;
    pub const AUTO_PEND_EN: u16 = 1 << 4;
    pub const PEND_DATA_REQ_ONLY: u16 = 1 << 6;
    pub const PAN_COORD: u16 = 1 << 7;
    pub const MAX_FRAME_VERSION_3: u16 = 3 << 8;
    pub const STRICT_LEN_FILTER: u16 = 1 << 15;
}

/// `frameTypes` bits.
pub mod ft {
    pub const BEACON: u8 = 1 << 0;
    pub const DATA: u8 = 1 << 1;
    pub const ACK: u8 = 1 << 2;
    pub const MAC_CMD: u8 = 1 << 3;
}

/// `ccaOpt`: energy + correlator (both must be busy) + sync found,
/// correlation threshold 3 (OpenThread CC2652 platform values).
pub const CCA_OPT: u8 = 0b0110_1111;

radio_op! {
    /// `rfc_CMD_IEEE_CSMA_t`.
    CmdIeeeCsma {
        random_state: u16,
        mac_max_be: u8,
        mac_max_csma_backoffs: u8,
        csma_config: u8,
        nb: u8,
        be: u8,
        remaining_periods: u8,
        last_rssi: i8,
        end_trigger: u8,
        last_time_stamp: u32,
        end_time: u32,
    }
}

/// `csmaConfig`: initCW = 1, unslotted, RX kept on during backoffs.
pub const CSMA_CONFIG: u8 = 1;

radio_op! {
    /// `rfc_CMD_IEEE_TX_t`.
    CmdIeeeTx {
        tx_opt: u8,
        payload_len: u8,
        payload: u32,
        time_stamp: u32,
    }
}

radio_op! {
    /// `rfc_CMD_IEEE_RX_ACK_t`.
    CmdIeeeRxAck {
        seq_no: u8,
        end_trigger: u8,
        end_time: u32,
    }
}

radio_op! {
    /// `rfc_CMD_IEEE_ED_SCAN_t`.
    CmdIeeeEdScan {
        channel: u8,
        cca_opt: u8,
        cca_rssi_thr: i8,
        _dummy0: u8,
        max_rssi: i8,
        end_trigger: u8,
        end_time: u32,
    }
}

/// `rfc_CMD_IEEE_MOD_SRC_MATCH_t` (immediate).
#[repr(C, align(4))]
#[derive(Clone, Copy, Debug, Default)]
pub struct CmdIeeeModSrcMatch {
    pub command_no: u16,
    /// bit0 bEnable, bit1 srcPend, bit2 entryType (1 = short).
    pub options: u8,
    pub entry_no: u8,
}

/// `rfc_CMD_IEEE_MOD_FILT_t` (immediate).
#[repr(C, align(4))]
#[derive(Clone, Copy, Debug, Default)]
pub struct CmdIeeeModFilt {
    pub command_no: u16,
    pub new_frame_filt_opt: u16,
    pub new_frame_types: u8,
}

/// `rfc_CMD_SET_TX_POWER_t` (immediate).
#[repr(C, align(4))]
#[derive(Clone, Copy, Debug, Default)]
pub struct CmdSetTxPower {
    pub command_no: u16,
    pub tx_power: u16,
}

/// `rfc_CMD_CHANGE_PA_t` (immediate).
#[repr(C, align(4))]
#[derive(Clone, Copy, Debug, Default)]
pub struct CmdChangePa {
    pub command_no: u16,
    pub _dummy0: u16,
    pub reg_override: u32,
}

/// `rfc_CMD_CLEAR_RX_t` (immediate).
#[repr(C, align(4))]
#[derive(Clone, Copy, Debug, Default)]
pub struct CmdClearRx {
    pub command_no: u16,
    pub _dummy0: u16,
    pub queue: u32,
}

/// `rfc_ieeeRxOutput_t`.
#[repr(C, align(4))]
#[derive(Clone, Copy, Debug, Default)]
pub struct IeeeRxOutput {
    pub n_tx_ack: u8,
    pub n_rx_beacon: u8,
    pub n_rx_data: u8,
    pub n_rx_ack: u8,
    pub n_rx_mac_cmd: u8,
    pub n_rx_reserved: u8,
    pub n_rx_nok: u8,
    pub n_rx_ignored: u8,
    pub n_rx_buf_full: u8,
    pub last_rssi: i8,
    pub max_rssi: i8,
    pub _dummy0: u8,
    pub beacon_time_stamp: u32,
}

/// `dataQueue_t`.
#[repr(C, align(4))]
#[derive(Clone, Copy, Debug, Default)]
pub struct DataQueue {
    pub curr_entry: u32,
    pub last_entry: u32,
}

/// `NEW_OVERRIDE_SEGMENT(address)` for an SRAM (`0x20xx_xxxx`) list.
pub const fn new_override_segment(addr: u32) -> u32 {
    ((addr & 0x03FF_FFFC) << 6) | 0x000F | (0x01 << 4)
}

/// `HW_REG_OVERRIDE(addr, val)`.
pub const fn hw_reg_override(addr: u32, val: u16) -> u32 {
    (addr & 0xFFFC) | ((val as u32) << 16)
}

/// `TX_STD_POWER_OVERRIDE(txPower)`.
pub const fn tx_std_power_override(raw: u16) -> u32 {
    0x022B | ((raw as u32) << 10)
}

/// `TX20_POWER_OVERRIDE(tx20Power)`.
pub const fn tx20_power_override(raw: u32) -> u32 {
    0x002B | (raw << 10)
}

/// Direct command word (`CMDR_DIR_CMD`).
pub const fn dir_cmd(id: u16) -> u32 {
    ((id as u32) << 16) | 1
}

/// Direct command with a 16-bit parameter (`CMDR_DIR_CMD_2BYTE`).
pub const fn dir_cmd_2byte(id: u16, par: u16) -> u32 {
    ((id as u32) << 16) | ((par as u32) & 0xFFFC) | 1
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::{offset_of, size_of};

    /// Offsets of the TI structures as GCC lays them out for ARM
    /// (`__RFC_STRUCT_ATTR = aligned(4)`, natural member alignment).
    #[test]
    fn layouts_match_the_ti_headers() {
        assert_eq!(offset_of!(CmdFsPowerdown, condition), 13);
        assert_eq!(size_of::<CmdFsPowerdown>(), 16);

        assert_eq!(offset_of!(CmdRadioSetupPa, mode), 14);
        assert_eq!(offset_of!(CmdRadioSetupPa, config), 16);
        assert_eq!(offset_of!(CmdRadioSetupPa, tx_power), 18);
        assert_eq!(offset_of!(CmdRadioSetupPa, reg_override), 20);
        assert_eq!(offset_of!(CmdRadioSetupPa, reg_override_tx20), 28);
        assert_eq!(size_of::<CmdRadioSetupPa>(), 32);

        assert_eq!(offset_of!(CmdSyncRat, rat0), 16);
        assert_eq!(size_of::<CmdSyncRat>(), 20);

        assert_eq!(offset_of!(CmdIeeeRx, channel), 14);
        assert_eq!(offset_of!(CmdIeeeRx, rx_config), 15);
        assert_eq!(offset_of!(CmdIeeeRx, rx_queue), 16);
        assert_eq!(offset_of!(CmdIeeeRx, output), 20);
        assert_eq!(offset_of!(CmdIeeeRx, frame_filt_opt), 24);
        assert_eq!(offset_of!(CmdIeeeRx, frame_types), 26);
        assert_eq!(offset_of!(CmdIeeeRx, cca_opt), 27);
        assert_eq!(offset_of!(CmdIeeeRx, cca_rssi_thr), 28);
        assert_eq!(offset_of!(CmdIeeeRx, num_ext_entries), 30);
        assert_eq!(offset_of!(CmdIeeeRx, num_short_entries), 31);
        assert_eq!(offset_of!(CmdIeeeRx, ext_entry_list), 32);
        assert_eq!(offset_of!(CmdIeeeRx, short_entry_list), 36);
        assert_eq!(offset_of!(CmdIeeeRx, local_ext_addr), 40);
        assert_eq!(offset_of!(CmdIeeeRx, local_short_addr), 48);
        assert_eq!(offset_of!(CmdIeeeRx, local_pan_id), 50);
        assert_eq!(offset_of!(CmdIeeeRx, end_trigger), 55);
        assert_eq!(offset_of!(CmdIeeeRx, end_time), 56);
        assert_eq!(size_of::<CmdIeeeRx>(), 60);

        assert_eq!(offset_of!(CmdIeeeCsma, random_state), 14);
        assert_eq!(offset_of!(CmdIeeeCsma, mac_max_be), 16);
        assert_eq!(offset_of!(CmdIeeeCsma, csma_config), 18);
        assert_eq!(offset_of!(CmdIeeeCsma, be), 20);
        assert_eq!(offset_of!(CmdIeeeCsma, last_rssi), 22);
        assert_eq!(offset_of!(CmdIeeeCsma, end_trigger), 23);
        assert_eq!(offset_of!(CmdIeeeCsma, last_time_stamp), 24);
        assert_eq!(offset_of!(CmdIeeeCsma, end_time), 28);
        assert_eq!(size_of::<CmdIeeeCsma>(), 32);

        assert_eq!(offset_of!(CmdIeeeTx, tx_opt), 14);
        assert_eq!(offset_of!(CmdIeeeTx, payload_len), 15);
        assert_eq!(offset_of!(CmdIeeeTx, payload), 16);
        assert_eq!(size_of::<CmdIeeeTx>(), 24);

        assert_eq!(offset_of!(CmdIeeeRxAck, seq_no), 14);
        assert_eq!(offset_of!(CmdIeeeRxAck, end_trigger), 15);
        assert_eq!(offset_of!(CmdIeeeRxAck, end_time), 16);

        assert_eq!(offset_of!(CmdIeeeEdScan, channel), 14);
        assert_eq!(offset_of!(CmdIeeeEdScan, cca_rssi_thr), 16);
        assert_eq!(offset_of!(CmdIeeeEdScan, max_rssi), 18);
        assert_eq!(offset_of!(CmdIeeeEdScan, end_trigger), 19);
        assert_eq!(offset_of!(CmdIeeeEdScan, end_time), 20);

        assert_eq!(size_of::<CmdIeeeModSrcMatch>(), 4);
        assert_eq!(offset_of!(CmdChangePa, reg_override), 4);
        assert_eq!(offset_of!(IeeeRxOutput, beacon_time_stamp), 12);
    }

    #[test]
    fn override_macros_match_driverlib() {
        assert_eq!(hw_reg_override(0x60A8, 0x0401), 0x0401_60A8);
        assert_eq!(hw_reg_override(0x60A8, 0x0001), 0x0001_60A8);
        assert_eq!(tx20_power_override(0x3F_75F5), 0x002B | (0x3F_75F5 << 10));
        assert_eq!(tx_std_power_override(0x7217), 0x022B | (0x7217 << 10));
        assert_eq!(dir_cmd(id::CMD_PING), 0x0406_0001);
        assert_eq!(dir_cmd_2byte(id::CMD0, 0x50), 0x0607_0051);
        assert_eq!(new_override_segment(0x2000_1234), (0x0000_1234 << 6) | 0x1F);
        assert!(status::is_terminal(status::IEEE_DONE_ACK));
        assert!(status::is_terminal(status::SKIPPED));
        assert!(!status::is_terminal(status::IEEE_SUSPENDED));
        assert!(!status::is_terminal(status::ACTIVE));
    }
}
