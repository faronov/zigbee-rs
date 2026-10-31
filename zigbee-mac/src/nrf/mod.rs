//! nRF52833/nRF52840 radio backend.
//!
//! The 802.15.4 MAC itself is the shared [`crate::SoftMacCore`]; this module
//! provides only the nRF-specific pieces underneath it:
//!
//! - [`NrfRadioPhy`]: the thin [`crate::RadioPhy`] + [`crate::PlatformServices`]
//!   adapter over Embassy's ieee802154 radio driver and the hardware RNG;
//! - [`NrfSoftMac`]: the end-device `SoftMacCore` alias used by every nRF
//!   example;
//! - fail-closed AES-128 through the Nordic ECB EasyDMA peripheral;
//! - cancellation-safe transmit and the forced RADIO DISABLED used for
//!   sleepy-device idle.
//!
//! # Hardware features used
//! - Auto-CRC generation/checking
//! - Hardware CCA before transmission; unslotted CSMA-CA, retries, ACK
//!   matching and address filtering run in `SoftMacCore`
//! - Software ACK generation ([`NrfRadioPhy`]'s `send_ack`)
//!
//! # Software acknowledgement
//! Received data/command frames are acknowledged only when the AR bit is set
//! and the destination names this node exactly; broadcasts and frames for
//! other nodes are never acknowledged on their behalf. Embassy-nrf 0.3
//! exposes only a CCA-gated transmit, so a software ACK is sent from RX_IDLE
//! as `CCASTART → TXEN` (~128 µs CCA + TX ramp-up). That keeps it inside the
//! sender's macAckWaitDuration (864 µs) in practice but exceeds the nominal
//! 192 µs aTurnaroundTime, and a busy CCA drops the ACK (the sender retries).
//! ACK timing remains a hardware-in-the-loop gate.
//!
//! # Cancellation safety
//! `Radio::try_send` in embassy-nrf 0.3.1 leaves `PACKETPTR` pointing at the
//! caller's buffer with no drop guard. Every transmit here goes through
//! [`send_packet_cancel_safe`], which forces the radio to DISABLED if the
//! future is dropped mid-transmission, before the buffer can be reused.
//!
//! # Supported boards
//! - nRF52840-DK, nRF52840-Dongle, Seeed XIAO nRF52840
//! - nRF52833-DK

mod radio_phy;

pub use radio_phy::{NRF_PENDING_RX, NrfRadioPhy, NrfSoftMac};

use crate::nrf_aes::{EcbDataBlock, EcbRegisters, NrfAesDriver};
use core::sync::atomic::{AtomicBool, Ordering};

// Re-export embassy-nrf from the correct renamed dependency.
#[cfg(all(feature = "nrf52833", not(feature = "nrf52840")))]
use embassy_nrf52833 as embassy_nrf;
#[cfg(feature = "nrf52840")]
use embassy_nrf52840 as embassy_nrf;

use embassy_nrf::radio::Error as RadioError;
use embassy_nrf::radio::Instance as RadioInstance;
use embassy_nrf::radio::ieee802154::{Packet, Radio};
use embassy_nrf::rng::{Instance as RngInstance, Rng};

pub use crate::nrf_aes::NrfAesError;

const ECB_BASE: usize = 0x4000_E000;
const ECB_TASKS_STARTECB: usize = 0x000;
const ECB_TASKS_STOPECB: usize = 0x004;
const ECB_EVENTS_ENDECB: usize = 0x100;
const ECB_EVENTS_ERRORECB: usize = 0x104;
const ECB_INTENCLR: usize = 0x308;
const ECB_ECBDATAPTR: usize = 0x504;
const ECB_WAIT_LIMIT: u32 = 100_000;

const RADIO_BASE: usize = 0x4000_1000;
const RADIO_TASKS_STOP: usize = RADIO_BASE + 0x00C;
const RADIO_TASKS_DISABLE: usize = RADIO_BASE + 0x010;
const RADIO_TASKS_CCASTOP: usize = RADIO_BASE + 0x030;
const RADIO_SHORTS: usize = RADIO_BASE + 0x200;
const RADIO_INTENCLR: usize = RADIO_BASE + 0x308;
const RADIO_STATE: usize = RADIO_BASE + 0x550;
const RADIO_DISABLE_SPIN_LIMIT: u32 = 10_000;

/// Force the RADIO peripheral to DISABLED from any state.
///
/// Used where no Embassy `Radio` borrow can run (a dropped transmit future)
/// and by [`NrfRadioPhy::enter_low_power_idle`]. Returns `false` if the state
/// machine did not reach DISABLED within the spin budget.
fn force_radio_disabled() -> bool {
    // SAFETY: fixed RADIO registers of the nRF52833/nRF52840 RADIO instance.
    // Callers either hold `&mut` access to the owning driver or are running
    // the drop glue of the future that held it, so no other code is driving
    // the radio concurrently.
    unsafe {
        core::ptr::write_volatile(RADIO_SHORTS as *mut u32, 0);
        core::ptr::write_volatile(RADIO_INTENCLR as *mut u32, 0xFFFF_FFFF);
        if core::ptr::read_volatile(RADIO_STATE as *const u32) == 0 {
            core::sync::atomic::compiler_fence(Ordering::SeqCst);
            return true;
        }
        core::ptr::write_volatile(RADIO_TASKS_CCASTOP as *mut u32, 1);
        core::ptr::write_volatile(RADIO_TASKS_STOP as *mut u32, 1);
        core::ptr::write_volatile(RADIO_TASKS_DISABLE as *mut u32, 1);
    }
    for _ in 0..RADIO_DISABLE_SPIN_LIMIT {
        // SAFETY: read-only access to the fixed RADIO STATE register.
        if unsafe { core::ptr::read_volatile(RADIO_STATE as *const u32) } == 0 {
            // EasyDMA may have been reading the buffer; order that before the
            // caller releases or reuses it.
            core::sync::atomic::compiler_fence(Ordering::SeqCst);
            return true;
        }
        core::hint::spin_loop();
    }
    false
}

/// Disables the radio if dropped while a transmission is still armed.
struct TxCancelGuard {
    armed: bool,
}

impl TxCancelGuard {
    fn disarm(mut self) {
        self.armed = false;
    }
}

impl Drop for TxCancelGuard {
    fn drop(&mut self) {
        if self.armed && !force_radio_disabled() {
            log::error!("[nRF] radio did not reach DISABLED after cancelled TX");
        }
    }
}

/// Transmit `packet` (CCA-gated) without leaving EasyDMA pointed at a freed
/// buffer if this future is dropped before completion.
///
/// `Radio::try_send` has no drop guard of its own: cancelling it mid-CCA or
/// mid-TX leaves `PACKETPTR` aimed at the caller's (stack) `Packet`. The guard
/// stops the radio synchronously in the drop glue, before that storage can be
/// reused.
async fn send_packet_cancel_safe<T: RadioInstance>(
    radio: &mut Radio<'_, T>,
    packet: &mut Packet,
) -> Result<(), RadioError> {
    let guard = TxCancelGuard { armed: true };
    let result = radio.try_send(packet).await;
    guard.disarm();
    result
}

static ECB_TAKEN: AtomicBool = AtomicBool::new(false);

/// Unique process-wide ownership token for the Nordic ECB peripheral.
///
/// Embassy 0.3 does not expose ECB in its `Peripherals` struct, so this
/// backend provides equivalent singleton ownership locally. This token is
/// not cloneable, and a successful acquisition is never released.
pub struct NrfEcbToken {
    _private: (),
}

impl NrfEcbToken {
    /// Acquire the ECB peripheral exactly once.
    pub fn take() -> Option<Self> {
        ECB_TAKEN
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .ok()
            .map(|_| Self { _private: () })
    }
}

struct NrfEcbRegisters;

impl NrfEcbRegisters {
    fn prepare() -> Self {
        Self::write(ECB_INTENCLR, 0x03);
        Self::write(ECB_EVENTS_ENDECB, 0);
        Self::write(ECB_EVENTS_ERRORECB, 0);
        // Read-back prevents a following task write from overtaking event
        // clearing on the peripheral bus.
        let _ = Self::read(ECB_EVENTS_ERRORECB);
        Self
    }

    #[inline(always)]
    fn read(offset: usize) -> u32 {
        // SAFETY: offsets are fixed registers in the nRF52833/nRF52840 ECB
        // instance. The owning engine retains Embassy's unique ECB token.
        unsafe { core::ptr::read_volatile((ECB_BASE + offset) as *const u32) }
    }

    #[inline(always)]
    fn write(offset: usize, value: u32) {
        // SAFETY: same exclusive-token and fixed-register argument as `read`.
        unsafe { core::ptr::write_volatile((ECB_BASE + offset) as *mut u32, value) }
    }

    fn clear_event(offset: usize) {
        Self::write(offset, 0);
        let _ = Self::read(offset);
    }
}

impl EcbRegisters for NrfEcbRegisters {
    fn clear_end_event(&mut self) {
        Self::clear_event(ECB_EVENTS_ENDECB);
    }

    fn clear_error_event(&mut self) {
        Self::clear_event(ECB_EVENTS_ERRORECB);
    }

    fn set_data_ptr(&mut self, data: *mut EcbDataBlock) {
        Self::write(ECB_ECBDATAPTR, data as u32);
    }

    fn start(&mut self) {
        Self::write(ECB_TASKS_STARTECB, 1);
    }

    fn end_event(&mut self) -> bool {
        Self::read(ECB_EVENTS_ENDECB) != 0
    }

    fn error_event(&mut self) -> bool {
        Self::read(ECB_EVENTS_ERRORECB) != 0
    }

    fn stop(&mut self) {
        Self::write(ECB_TASKS_STOPECB, 1);
    }
}

struct NrfEcbEngine {
    _token: NrfEcbToken,
    driver: NrfAesDriver<NrfEcbRegisters>,
}

impl NrfEcbEngine {
    fn new(token: NrfEcbToken) -> Self {
        Self {
            _token: token,
            driver: NrfAesDriver::new(NrfEcbRegisters::prepare(), ECB_WAIT_LIMIT),
        }
    }

    fn self_test(&mut self) -> Result<(), NrfAesError> {
        self.driver.self_test()
    }

    #[inline(never)]
    fn encrypt_block(&mut self, key: &[u8; 16], block: &mut [u8; 16]) -> Result<(), NrfAesError> {
        let input = *block;
        self.driver.encrypt(key, &input, block)
    }
}

struct NrfAesState {
    installed: Option<NrfEcbEngine>,
    /// Retain the token and DMA storage after a rejected KAT. This leaves AES
    /// unavailable while ensuring a failed abort can never outlive its buffer.
    rejected: Option<NrfEcbEngine>,
}

impl NrfAesState {
    const fn new() -> Self {
        Self {
            installed: None,
            rejected: None,
        }
    }

    fn install(&mut self, token: NrfEcbToken) -> Result<(), NrfAesError> {
        if self.installed.is_some() || self.rejected.is_some() {
            return Err(NrfAesError::AlreadyInstalled);
        }

        let mut engine = NrfEcbEngine::new(token);
        match engine.self_test() {
            Ok(()) => {
                self.installed = Some(engine);
                Ok(())
            }
            Err(error) => {
                self.rejected = Some(engine);
                Err(error)
            }
        }
    }

    fn engine_mut(&mut self) -> Option<&mut NrfEcbEngine> {
        self.installed.as_mut()
    }
}

struct NrfHardwareAes128<'engine> {
    engine: Option<&'engine mut NrfEcbEngine>,
    key: zigbee_crypto::AesKey,
}

impl zigbee_crypto::Aes128Forward for NrfHardwareAes128<'_> {
    type Error = NrfAesError;

    fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
        self.engine
            .as_deref_mut()
            .ok_or(NrfAesError::NotInstalled)?
            .encrypt_block(&self.key, block)
    }
}
