//! ESP32 MAC backend (ESP32-C6 / ESP32-H2).
//!
//! The ESP32-C6/H2 IEEE 802.15.4 radio (`esp-radio::ieee802154`) is exposed
//! through the portable [`crate::RadioPhy`] contract ([`EspRadioPhy`]) and
//! driven by the shared [`crate::SoftMacCore`] end-device MAC
//! ([`EspSoftMac`]): scan, association, polling, CSMA-CA backoff and frame
//! retries are the same code every other software-MAC backend runs.
//!
//! # Supported chips
//! - ESP32-C6 (RISC-V, 2.4 GHz WiFi + BLE + 802.15.4)
//! - ESP32-H2 (RISC-V, BLE + 802.15.4)
//!
//! # Features
//! - Hardware auto-ACK and address filtering, software ACK-wait window
//! - EUI-64 address from eFuse factory MAC
//! - Bounded, fail-closed hardware AES-128 for Zigbee security
//! - Hardware RNG service while the RF subsystem is active
//! - Retained light sleep through [`crate::RadioPhy::sleep`] / `wake`

mod radio_phy;

use crate::esp_aes::{AesDriver, AesRegisters};

pub use crate::esp_aes::AesEngineError;
pub use esp_radio::ieee802154::SuspendError;
pub use radio_phy::{DEFAULT_TX_POWER_DBM, ESP_PENDING_RX, EspRadioPhy, EspSoftMac};

const AES_BASE: usize = 0x6008_8000;
const AES_KEY_OFFSET: usize = 0x00;
const AES_TEXT_IN_OFFSET: usize = 0x20;
const AES_TEXT_OUT_OFFSET: usize = 0x30;
const AES_MODE_OFFSET: usize = 0x40;
const AES_ENDIAN_OFFSET: usize = 0x44;
const AES_TRIGGER_OFFSET: usize = 0x48;
const AES_STATE_OFFSET: usize = 0x4c;
const AES_WAIT_LIMIT: u32 = 100_000;

struct EspAesRegisters;

fn release_aes_coupled_reset() {
    // ESP-IDF's C6/H2 `aes_ll_reset_register()` clears the Digital Signature
    // reset after resetting AES: the two blocks share reset logic, and AES can
    // otherwise remain held in reset even though its own clock guard is live.
    //
    // Use the HAL's chip-specific PAC accessor rather than a fixed PCR offset:
    // DS_CONF is at 0xe0 on C6 and 0xdc on H2.
    esp_hal::peripherals::SYSTEM::regs()
        .ds_conf()
        .modify(|_, w| w.ds_rst_en().clear_bit());
}

impl EspAesRegisters {
    #[inline(always)]
    fn read(offset: usize) -> u32 {
        // SAFETY: `EspAesEngine` retains the unique AES peripheral token.
        unsafe { core::ptr::read_volatile((AES_BASE + offset) as *const u32) }
    }

    #[inline(always)]
    fn write(offset: usize, value: u32) {
        // SAFETY: all offsets are fixed AES registers shared by C6 and H2.
        unsafe { core::ptr::write_volatile((AES_BASE + offset) as *mut u32, value) }
    }

    fn write_words(offset: usize, bytes: &[u8; 16]) {
        for index in 0..4 {
            let start = index * 4;
            Self::write(
                offset + start,
                u32::from_le_bytes([
                    bytes[start],
                    bytes[start + 1],
                    bytes[start + 2],
                    bytes[start + 3],
                ]),
            );
        }
    }
}

impl AesRegisters for EspAesRegisters {
    fn state(&mut self) -> u32 {
        Self::read(AES_STATE_OFFSET) & 0x03
    }

    fn configure_aes128_encrypt(&mut self) {
        Self::write(AES_ENDIAN_OFFSET, 0);
        Self::write(AES_MODE_OFFSET, 0);
    }

    fn write_key(&mut self, key: &[u8; 16]) {
        Self::write_words(AES_KEY_OFFSET, key);
    }

    fn write_input(&mut self, input: &[u8; 16]) {
        Self::write_words(AES_TEXT_IN_OFFSET, input);
    }

    fn trigger(&mut self) {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        Self::write(AES_TRIGGER_OFFSET, 1);
    }

    fn read_output(&mut self, output: &mut [u8; 16]) {
        for index in 0..4 {
            let start = index * 4;
            output[start..start + 4]
                .copy_from_slice(&Self::read(AES_TEXT_OUT_OFFSET + start).to_le_bytes());
        }
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}

struct EspAesEngine<'d> {
    _peripheral: esp_hal::aes::Aes<'d>,
    driver: AesDriver<EspAesRegisters>,
}

impl<'d> EspAesEngine<'d> {
    fn new(peripheral: esp_hal::peripherals::AES<'d>) -> Result<Self, AesEngineError> {
        let peripheral = esp_hal::aes::Aes::new(peripheral);
        release_aes_coupled_reset();
        let mut driver = AesDriver::new(EspAesRegisters, AES_WAIT_LIMIT);
        driver.self_test()?;
        Ok(Self {
            _peripheral: peripheral,
            driver,
        })
    }

    fn encrypt_block(
        &mut self,
        key: &[u8; 16],
        block: &mut [u8; 16],
    ) -> Result<(), AesEngineError> {
        let input = *block;
        self.driver.encrypt(key, &input, block)
    }
}

struct EspHardwareAes128<'engine, 'peripheral> {
    engine: &'engine mut EspAesEngine<'peripheral>,
    key: zigbee_crypto::AesKey,
}

impl<'engine, 'peripheral> EspHardwareAes128<'engine, 'peripheral> {
    fn new(engine: &'engine mut EspAesEngine<'peripheral>, key: zigbee_crypto::AesKey) -> Self {
        Self { engine, key }
    }
}

impl zigbee_crypto::Aes128Forward for EspHardwareAes128<'_, '_> {
    type Error = AesEngineError;

    #[inline(always)]
    fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
        self.engine.encrypt_block(&self.key, block)
    }
}
