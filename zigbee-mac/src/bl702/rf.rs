//! BL702 analog RF and digital PHY initialization.
//!
//! The register sequences in this module are a clean-room reconstruction of
//! Bouffalo's distributed BL702 radio objects. They intentionally use raw
//! volatile accesses because the public BL702 PAC does not describe this
//! proprietary register block.

mod calibration;

use core::{cell::UnsafeCell, hint::spin_loop};

use super::registers::{read32, write32};

const SAMPLE_RAM: u32 = 0x4202_4000;
const SAMPLE_WORDS: usize = 256;
const POLL_SPINS: usize = 1_000_000;

const CALIBRATION_STATE_REGISTERS: [u32; 16] = [
    0x4000_0050,
    0x4000_1004,
    0x4000_1008,
    0x4000_1100,
    0x4000_1120,
    0x4000_1528,
    0x4000_152c,
    0x4000_1510,
    0x4000_1224,
    0x4000_1010,
    0x4000_1514,
    0x4000_1258,
    0x4000_127c,
    0x4000_150c,
    0x4000_126c,
    0x4000_1280,
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum RfError {
    Timeout,
    InvalidMeasurement,
}

#[repr(C)]
struct CalibrationData {
    rx_offset_data: [u32; 16],
    rccal_data: u32,
    bt_lo_config_data: [u16; 40],
    zb_lo_config_data: [u16; 8],
    lo_acal_data: u32,
    tsen_value_init: i16,
    reserved: u16,
    tsen_value_status: u32,
}

impl CalibrationData {
    const ZERO: Self = Self {
        rx_offset_data: [0; 16],
        rccal_data: 0,
        bt_lo_config_data: [0; 40],
        zb_lo_config_data: [0; 8],
        lo_acal_data: 0,
        tsen_value_init: 0,
        reserved: 0,
        tsen_value_status: 0,
    };
}

struct Global<T>(UnsafeCell<T>);

unsafe impl<T> Sync for Global<T> {}

// This cache is populated during every cold boot. A future sleep implementation
// can place it in a board-specific retention section and restore it on wake.
static CALIBRATION_DATA: Global<CalibrationData> = Global(UnsafeCell::new(CalibrationData::ZERO));

#[derive(Clone, Copy)]
struct SavedRfState {
    values: [u32; CALIBRATION_STATE_REGISTERS.len()],
}

#[derive(Clone, Copy)]
struct TxPowerEntry {
    pa_seri_cs: u8,
    pa_para_cs: u8,
    pa_inbuf_unit: u8,
    lodist_75dc_sel: u8,
    pa_hp_en: u8,
    pa_lp_en: u8,
    pa_dac: u8,
}

const TX_POWER: [TxPowerEntry; 15] = [
    TxPowerEntry::new(6, 0, 3, 1, 0, 1, 19),
    TxPowerEntry::new(6, 0, 3, 1, 0, 1, 22),
    TxPowerEntry::new(15, 0, 7, 0, 0, 0, 10),
    TxPowerEntry::new(15, 0, 7, 0, 0, 0, 11),
    TxPowerEntry::new(15, 0, 7, 0, 0, 0, 12),
    TxPowerEntry::new(15, 0, 7, 0, 0, 0, 13),
    TxPowerEntry::new(15, 0, 7, 0, 0, 0, 15),
    TxPowerEntry::new(15, 0, 7, 0, 0, 0, 17),
    TxPowerEntry::new(15, 0, 7, 0, 0, 0, 19),
    TxPowerEntry::new(15, 0, 7, 0, 0, 0, 21),
    TxPowerEntry::new(15, 3, 7, 0, 1, 0, 15),
    TxPowerEntry::new(15, 3, 7, 0, 1, 0, 17),
    TxPowerEntry::new(15, 3, 7, 0, 1, 0, 20),
    TxPowerEntry::new(15, 3, 7, 0, 1, 0, 23),
    TxPowerEntry::new(15, 3, 7, 0, 1, 0, 26),
];

impl TxPowerEntry {
    const fn new(
        pa_seri_cs: u8,
        pa_para_cs: u8,
        pa_inbuf_unit: u8,
        lodist_75dc_sel: u8,
        pa_hp_en: u8,
        pa_lp_en: u8,
        pa_dac: u8,
    ) -> Self {
        Self {
            pa_seri_cs,
            pa_para_cs,
            pa_inbuf_unit,
            lodist_75dc_sel,
            pa_hp_en,
            pa_lp_en,
            pa_dac,
        }
    }
}

pub(super) fn initialize<D>(delay_us: &mut D) -> Result<(), RfError>
where
    D: FnMut(u32),
{
    configure_digital_phy();
    apply_rf_defaults();
    calibration::run_full_calibration(delay_us)?;
    set_tx_power(14);
    Ok(())
}

pub(super) fn set_tx_power(dbm: u8) {
    let entry = TX_POWER[usize::from(dbm)];

    let mut value = read32(0x4000_1204);
    value = (value & !0x0000_000f) | u32::from(entry.pa_seri_cs);
    value = (value & !(1 << 29)) | (u32::from(entry.pa_hp_en) << 29);
    value = (value & !(1 << 28)) | (u32::from(entry.pa_lp_en) << 28);
    value = (value & !0x0000_f000) | (u32::from(entry.pa_para_cs) << 12);
    write32(0x4000_1204, value);

    rmw(
        0x4000_1120,
        0xff8f_ffff,
        u32::from(entry.pa_inbuf_unit) << 20,
    );
    rmw(
        0x4000_123c,
        0xfffe_ffff,
        u32::from(entry.lodist_75dc_sel) << 16,
    );
    rmw(0x4000_1110, 0xfeff_fffe, 0x0100_0001);
    rmw(0x4000_1808, 0xffff_c1ff, u32::from(entry.pa_dac) << 9);
}

fn configure_digital_phy() {
    rmw(0x4000_1800, u32::MAX, 0x0000_0100);
    rmw(0x4000_1834, 0xff7f_ffff, 0x0080_0000);
    rmw(0x4000_1854, 0x00ff_ffff, 0);

    rmw(0x4000_1810, 0xffff_fffe, 0);
    rmw(0x4000_1810, u32::MAX, 0x2);
    rmw(0x4000_1824, 0xffff_e00f, 0x0000_02c0);
    rmw(0x4000_182c, 0xfffc_00ff, 0x0000_2c00);
    rmw(0x4000_182c, 0xffff_ff00, 0x10);

    rmw(0x4000_1cac, 0xffff_ffe0, 0x4);
    rmw(0x4000_1c78, 0xf03f_ffff, 0x0600_0000);
    rmw(0x4000_1c78, 0xffc0_ffff, 0x000c_0000);
    rmw(0x4000_1c78, 0xffff_03ff, 0x0000_1800);
    rmw(0x4000_1c78, 0xffff_fc0f, 0x30);
    rmw(0x4000_1c78, 0xffff_fff0, 0x0d);

    rmw(0x4000_1808, 0xff7f_ffff, 0);
    rmw(0x4000_180c, 0x00ff_ffff, 0x0800_0000);
    rmw(0x4000_180c, 0xff7f_ffff, 0x0080_0000);
    rmw(0x4000_180c, 0x00ff_ffff, 0x0400_0000);
    rmw(0x4000_180c, 0xffff_ff00, 0);

    rmw(0x4000_1854, 0xffff_03ff, 0x0000_0a00);
    rmw(0x4000_1844, 0xffff_03ff, 0x0000_0700);
    rmw(0x4000_1854, 0xff00_ffff, 0x00a0_0000);
    rmw(0x4000_1844, 0xff00_ffff, 0x00a0_0000);
    rmw(0x4000_1854, 0xffff_ff00, 0x0a);
    rmw(0x4000_1844, 0xffff_ff00, 0x0a);

    rmw(0x4000_1810, 0xffff_fffe, 0);
    rmw(0x4000_1810, u32::MAX, 0x2);
    rmw(0x4000_1c7c, 0xffff_fff0, 0x0b);
}

fn apply_rf_defaults() {
    rmw(0x4000_1200, 0xffff_fffc, 0x1);
    rmw(0x4000_1268, u32::MAX, 0x1000);
    rmw(0x4000_1254, 0xffff_ff8f, 0);
    rmw(0x4000_1240, 0xdfff_ffff, 0);
    rmw(0x4000_1280, 0xfffb_dfff, 0x0008_4000);
    rmw(0x4000_1294, 0xffff_fff8, 0);
    rmw(0x4000_126c, 0xf8fe_3fff, 0x0200_cc00);
    rmw(0x4000_1270, 0xfffe_3fff, 0x0000_cc00);
    rmw(0x4000_1274, 0xfffe_33ff, 0x0000_8400);
    rmw(0x4000_1284, 0xffff_3cff, 0x010c_8200);
    rmw(0x4000_127c, 0xfffe_ffff, 0x0030_0300);
    rmw(0x4000_1264, 0xffff_fcff, 0x0000_0200);
    rmw(0x4000_123c, 0xffff_ffcc, 0x22);
    rmw(0x4000_1208, 0xf0ff_ffff, 0x0700_0000);
    rmw(0x4000_1538, 0xfff8_0200, 0x0031_c00f);
    rmw(0x4000_110c, u32::MAX, 0x2);
    rmw(0x4000_1114, u32::MAX, 0x2);
    rmw(0x4000_1228, u32::MAX, 0x1000);
}

pub(super) fn with_saved_state<T>(
    operation: impl FnOnce() -> Result<T, RfError>,
) -> Result<T, RfError> {
    let saved = save_state();
    let result = operation();
    restore_state(saved);
    result
}

fn save_state() -> SavedRfState {
    let mut values = [0u32; CALIBRATION_STATE_REGISTERS.len()];
    for (value, address) in values.iter_mut().zip(CALIBRATION_STATE_REGISTERS) {
        *value = read32(address);
    }
    SavedRfState { values }
}

fn restore_state(saved: SavedRfState) {
    for (value, address) in saved.values.into_iter().zip(CALIBRATION_STATE_REGISTERS) {
        write32(address, value);
    }
}

pub(super) fn switch_channel<D>(delay_us: &mut D, frequency_mhz: f64)
where
    D: FnMut(u32),
{
    rmw(0x4000_1008, 0xefef_effe, 0);
    rmw(0x4000_126c, 0xefff_ffff, 0);
    rmw(0x4000_1280, 0xfffb_ffff, 0);
    rmw(0x4000_1010, 0xffff_ffef, 0);
    rmw(0x4000_1100, 0xffff_fdff, 0x3dfc);

    let frequency_control_word = (frequency_mhz * 0.03125 * 131_072.0) as i32 as u32;
    rmw(0x4000_1258, 0xfe00_0000, frequency_control_word);
    delay_us(1_000);

    rmw(0x4000_1100, u32::MAX, 0x0a00_0000);
    delay_us(1_000);
    rmw(0x4000_1100, 0xf5ff_ffff, 0);
    delay_us(1_000);
}

pub(super) fn configure_pucr(mode: u8) {
    rmw(0x4000_1008, 0xffff_fffe, 0);
    let bits = match mode {
        3 => 0x3c10,
        4 => 0x3c3c,
        _ => 0,
    };
    if bits != 0 {
        rmw(0x4000_1100, u32::MAX, bits);
    }
}

pub(super) fn wait_for_set(address: u32, bits: u32) -> Result<u32, RfError> {
    for _ in 0..POLL_SPINS {
        let value = read32(address);
        if value & bits == bits {
            return Ok(value);
        }
        spin_loop();
    }
    Err(RfError::Timeout)
}

pub(super) fn capture_samples() -> Result<(), RfError> {
    rmw(0x4000_1528, 0xffff_fffd, 0);
    rmw(0x4000_1528, u32::MAX, 0x2);
    wait_for_set(0x4000_1528, 0x1)?;
    rmw(0x4000_1528, 0xffff_fffd, 0);
    Ok(())
}

pub(super) fn sample(index: usize) -> u32 {
    read32(SAMPLE_RAM + (index as u32 * 4))
}

pub(super) fn save_sample_ram() -> [u32; SAMPLE_WORDS] {
    let mut saved = [0u32; SAMPLE_WORDS];
    for (index, value) in saved.iter_mut().enumerate() {
        *value = sample(index);
    }
    saved
}

pub(super) fn restore_sample_ram(saved: &[u32; SAMPLE_WORDS]) {
    for (index, value) in saved.iter().copied().enumerate() {
        write32(SAMPLE_RAM + (index as u32 * 4), value);
    }
}

fn calibration_ptr() -> *mut CalibrationData {
    CALIBRATION_DATA.0.get()
}

pub(super) fn read_rx_offset(index: usize) -> u32 {
    unsafe {
        core::ptr::read_volatile(core::ptr::addr_of!(
            (*calibration_ptr()).rx_offset_data[index]
        ))
    }
}

pub(super) fn write_rx_offset(index: usize, value: u32) {
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!((*calibration_ptr()).rx_offset_data[index]),
            value,
        );
    }
}

pub(super) fn read_rccal() -> u32 {
    unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*calibration_ptr()).rccal_data)) }
}

pub(super) fn write_rccal(value: u32) {
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!((*calibration_ptr()).rccal_data),
            value,
        );
    }
}

pub(super) fn read_bt_lo(index: usize) -> u16 {
    unsafe {
        core::ptr::read_volatile(core::ptr::addr_of!(
            (*calibration_ptr()).bt_lo_config_data[index]
        ))
    }
}

pub(super) fn write_bt_lo(index: usize, value: u16) {
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!((*calibration_ptr()).bt_lo_config_data[index]),
            value,
        );
    }
}

pub(super) fn read_zb_lo(index: usize) -> u16 {
    unsafe {
        core::ptr::read_volatile(core::ptr::addr_of!(
            (*calibration_ptr()).zb_lo_config_data[index]
        ))
    }
}

pub(super) fn write_zb_lo(index: usize, value: u16) {
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!((*calibration_ptr()).zb_lo_config_data[index]),
            value,
        );
    }
}

pub(super) fn read_lo_acal() -> u32 {
    unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*calibration_ptr()).lo_acal_data)) }
}

pub(super) fn write_lo_acal(value: u32) {
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!((*calibration_ptr()).lo_acal_data),
            value,
        );
    }
}

pub(super) fn rmw(address: u32, preserve_mask: u32, set_bits: u32) {
    write32(address, (read32(address) & preserve_mask) | set_bits);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calibration_cache_matches_vendor_layout() {
        assert_eq!(core::mem::size_of::<CalibrationData>(), 176);
        assert_eq!(core::mem::offset_of!(CalibrationData, rccal_data), 64);
        assert_eq!(
            core::mem::offset_of!(CalibrationData, bt_lo_config_data),
            68
        );
        assert_eq!(
            core::mem::offset_of!(CalibrationData, zb_lo_config_data),
            148
        );
        assert_eq!(core::mem::offset_of!(CalibrationData, lo_acal_data), 164);
        assert_eq!(
            core::mem::offset_of!(CalibrationData, tsen_value_status),
            172
        );
    }

    #[test]
    fn tx_power_table_matches_integer_vendor_points() {
        assert_eq!(TX_POWER[0].pa_dac, 19);
        assert_eq!(TX_POWER[9].pa_dac, 21);
        assert_eq!(TX_POWER[14].pa_dac, 26);
        assert_eq!(TX_POWER[0].lodist_75dc_sel, 1);
        assert_eq!(TX_POWER[0].pa_lp_en, 1);
    }
}
