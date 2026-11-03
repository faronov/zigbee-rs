//! LOW32K reset-on-wake platform hand-off.
//!
//! The cold path registers the statically placed MAC with
//! [`initialize_context`]. [`TelinkWake`](crate::TelinkWake) sleeps through the
//! PM record kept here, and the retention entry calls [`resume`] to restore
//! PM time, MAC/radio, AES/RNG, the flash voltage guard and the product
//! indicator with IRQs still masked before the fresh root future runs. Any
//! inconsistency ends in [`fail_closed_reset`], never in a cold-boot path
//! with retained joined state.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU8, Ordering};

use tlsr8258_hal::pm::{Low32kResumeRecord, PmError, WakeStatus};
use tlsr8258_rt::retention::RetainedRoot;
use zigbee_mac::telink::{TelinkMac, TelinkRetentionResumeError};

use crate::TelinkWakeError;

/// Product indicator (LEDs) restored after a retention wake.
///
/// The product records its semantic state with [`record_indicator`] whenever
/// it changes the outputs; that byte lives in retained RAM.
pub trait RetainedIndicator {
    /// State recorded for a fresh context and forced before a fail-closed
    /// reset.
    const RESET_STATE: u8;

    /// Drive the outputs to `state`. Runs with IRQs masked, before any
    /// future that owns the indicator exists.
    fn restore(state: u8) -> Result<(), tlsr8258_hal::gpio::GpioError>;
}

/// Record the indicator state that a retention wake must restore.
pub fn record_indicator(state: u8) {
    INDICATOR.store(state, Ordering::SeqCst);
}

const PLATFORM_MAGIC: u32 = 0x5452_5031; // "TRP1"

/// The MAC pointer is registered only after `ZigbeeDevice::build_into` has
/// placed it at its final static address.
#[repr(C)]
struct PlatformState {
    marker: u32,
    marker_inverse: u32,
    mac_pointer: usize,
    mac_pointer_inverse: usize,
    record: Low32kResumeRecord,
}

impl PlatformState {
    const fn invalid() -> Self {
        Self {
            marker: 0,
            marker_inverse: !0,
            mac_pointer: 0,
            mac_pointer_inverse: !0,
            record: Low32kResumeRecord::new(),
        }
    }

    fn valid(&self) -> bool {
        self.marker == PLATFORM_MAGIC
            && self.marker_inverse == !PLATFORM_MAGIC
            && self.mac_pointer != 0
            && self.mac_pointer_inverse == !self.mac_pointer
            && self.mac_pointer % core::mem::align_of::<TelinkMac>() == 0
            && (0x0084_0000..0x0084_8000).contains(&(self.mac_pointer as u32))
    }
}

struct PlatformCell(UnsafeCell<PlatformState>);
unsafe impl Sync for PlatformCell {}

#[unsafe(link_section = ".retained.platform")]
static PLATFORM: PlatformCell = PlatformCell(UnsafeCell::new(PlatformState::invalid()));

#[unsafe(link_section = ".retained.platform")]
static INDICATOR: AtomicU8 = AtomicU8::new(0);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RetentionPlatformError {
    ContextInvalid,
    PowerManagement(PmError),
    Mac(TelinkRetentionResumeError),
    Adc(tlsr8258_hal::adc::AdcError),
    Gpio(tlsr8258_hal::gpio::GpioError),
}

impl From<PmError> for RetentionPlatformError {
    fn from(error: PmError) -> Self {
        Self::PowerManagement(error)
    }
}

/// Register static MAC placement and invalidate any stale sleep hand-off.
pub fn initialize_context<I: RetainedIndicator>(mac: &mut TelinkMac) {
    use core::sync::atomic::compiler_fence;

    let pointer = mac as *mut TelinkMac as usize;
    let state = unsafe { &mut *PLATFORM.0.get() };
    *state = PlatformState::invalid();
    state.mac_pointer = pointer;
    state.mac_pointer_inverse = !pointer;
    state.marker_inverse = !PLATFORM_MAGIC;
    compiler_fence(Ordering::SeqCst);
    state.marker = PLATFORM_MAGIC;
    record_indicator(I::RESET_STATE);
}

fn state_mut() -> Result<&'static mut PlatformState, RetentionPlatformError> {
    let state = unsafe { &mut *PLATFORM.0.get() };
    if state.valid() {
        Ok(state)
    } else {
        Err(RetentionPlatformError::ContextInvalid)
    }
}

/// PM record for a retention sleep of the registered `mac`.
pub(crate) fn record_for(
    mac: &mut TelinkMac,
) -> Result<&'static mut Low32kResumeRecord, TelinkWakeError> {
    match state_mut() {
        Ok(state) if state.mac_pointer == mac as *mut TelinkMac as usize => Ok(&mut state.record),
        _ => Err(TelinkWakeError::RetentionContextInvalid),
    }
}

/// Restore PM time, retained MAC/radio, AES KAT, optional RNG state, the PC5
/// voltage guard and the indicator before IRQs are enabled.
#[inline(never)]
pub fn restore_platform<I: RetainedIndicator>() -> Result<WakeStatus, RetentionPlatformError> {
    let state = state_mut()?;
    let token = tlsr8258_hal::pm::begin_low32k_resume(&state.record)?;
    let mac = unsafe { &mut *(state.mac_pointer as *mut TelinkMac) };
    mac.resume_after_retention()
        .map_err(RetentionPlatformError::Mac)?;
    tlsr8258_hal::adc::restore_flash_voltage_guard().map_err(RetentionPlatformError::Adc)?;
    I::restore(INDICATOR.load(Ordering::SeqCst)).map_err(RetentionPlatformError::Gpio)?;
    let status = tlsr8258_hal::pm::complete_low32k_resume(&mut state.record, token)?;
    if !status.entered_low_power() || !status.woke_by_timer() {
        return Err(RetentionPlatformError::PowerManagement(
            PmError::RetentionWakeInvalid,
        ));
    }
    Ok(status)
}

/// Retention-entry gate: accept the wake only if `root` still names
/// `addresses()`, the platform restores, and the stack guard is intact.
/// Otherwise reset; never return into a stale or partial context.
///
/// Out of line so the address array lives in this short frame rather than in
/// the caller that goes on to enter the deep executor root.
#[inline(never)]
pub fn resume<I: RetainedIndicator, const N: usize>(
    root: &'static RetainedRoot<N>,
    addresses: fn() -> [u32; N],
) {
    if !root.is_valid(addresses())
        || restore_platform::<I>().is_err()
        || !tlsr8258_rt::retention::stack_guard_intact()
    {
        fail_closed_reset::<I>();
    }
}

/// Clear an unusable hand-off, show the reset indicator and force a real
/// reset. This never falls through into a cold-boot code path.
pub fn fail_closed_reset<I: RetainedIndicator>() -> ! {
    tlsr8258_hal::mmio::disable_all_irqs();
    let state = unsafe { &mut *PLATFORM.0.get() };
    tlsr8258_hal::pm::abandon_low32k_resume(&mut state.record);
    record_indicator(I::RESET_STATE);
    let _ = I::restore(I::RESET_STATE);
    tlsr8258_hal::reset::reboot()
}
