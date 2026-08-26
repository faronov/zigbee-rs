//! XT-ZB1 composition adapter for the reusable BL702 HAL.

use core::{cell::UnsafeCell, convert::Infallible};

use bl702_hal::adc::Gpadc;
use bl702_hal::clock::Clocks;
use bl702_hal::efuse::EfuseReader;
use bl702_hal::peripherals::Adc;
use bl702_hal::timer::Monotonic;
use bl702_hal::uart::{Uart0Tx, WriteError};
use bl702_xt_zb1::Resources;
use embassy_time::{Duration, Instant, Timer};
use sensor_sed_app::{
    BatteryReading, BlockingBatterySource, DiagnosticEvent, Diagnostics, SleepDepth, Supervisor,
    WaitRequest, WakeController, WakeReason,
};
use zigbee_mac::MacDriver;

pub use bl702_xt_zb1::ApplicationResources;

struct Global<T> {
    value: UnsafeCell<Option<T>>,
    borrowed: UnsafeCell<bool>,
}

// SAFETY: All access is serialized with the BL702 single-hart critical
// section. Values are initialized once before interrupts are enabled.
unsafe impl<T> Sync for Global<T> {}

impl<T> Global<T> {
    const fn empty() -> Self {
        Self {
            value: UnsafeCell::new(None),
            borrowed: UnsafeCell::new(false),
        }
    }

    fn set(&self, value: T) -> Result<(), T> {
        riscv::interrupt::free(|| {
            // SAFETY: The single-hart critical section excludes all other
            // accesses to this cell.
            let slot = unsafe { &mut *self.value.get() };
            if slot.is_some() {
                Err(value)
            } else {
                *slot = Some(value);
                Ok(())
            }
        })
    }

    fn with<R>(&self, operation: impl FnOnce(&T) -> R) -> Option<R> {
        riscv::interrupt::free(|| {
            // SAFETY: The single-hart critical section excludes mutation.
            if unsafe { *self.borrowed.get() } {
                None
            } else {
                unsafe { (&*self.value.get()).as_ref().map(operation) }
            }
        })
    }

    fn with_mut<R>(&self, operation: impl FnOnce(&mut T) -> R) -> Option<R> {
        let pointer = riscv::interrupt::free(|| {
            // SAFETY: The single-hart critical section serializes the borrow
            // flag and value access.
            unsafe {
                if *self.borrowed.get() {
                    None
                } else {
                    let pointer = (&mut *self.value.get())
                        .as_mut()
                        .map(|value| value as *mut T);
                    if pointer.is_some() {
                        *self.borrowed.get() = true;
                    }
                    pointer
                }
            }
        })?;

        // SAFETY: The borrow flag excludes every other access while the
        // operation runs, but interrupts remain enabled during bounded UART
        // polling.
        let result = operation(unsafe { &mut *pointer });
        riscv::interrupt::free(|| {
            // SAFETY: This is the matching release for the borrow above.
            unsafe { *self.borrowed.get() = false };
        });
        Some(result)
    }
}

static UART: Global<Uart0Tx<14, 15>> = Global::empty();
static TIMER: Global<Monotonic> = Global::empty();
static CHIP_ID: Global<[u8; 8]> = Global::empty();

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitError {
    ResourcesAlreadyTaken,
    Uart,
    AlreadyInitialized,
}

pub fn init() -> Result<ApplicationResources, InitError> {
    let resources = Resources::take().ok_or(InitError::ResourcesAlreadyTaken)?;
    let (runtime, application) = resources.split();
    let clocks = runtime.clocks;
    let uart = runtime
        .monitor
        .into_uart(clocks)
        .map_err(|_| InitError::Uart)?;
    let timer = Monotonic::new_1mhz(runtime.timer0, clocks);
    let chip_id = EfuseReader::new(runtime.efuse).chip_id();

    UART.set(uart).map_err(|_| InitError::AlreadyInitialized)?;
    TIMER
        .set(timer)
        .map_err(|_| InitError::AlreadyInitialized)?;
    CHIP_ID
        .set(chip_id)
        .map_err(|_| InitError::AlreadyInitialized)?;
    Ok(application)
}

pub fn delay_us(duration_us: u32) {
    bl702_hal::timer::delay_us(duration_us);
}

pub fn timer_ticks() -> u64 {
    TIMER
        .with_mut(Monotonic::ticks)
        .unwrap_or_else(|| panic!("BL702 timer is unavailable"))
}

pub fn chip_id() -> [u8; 8] {
    CHIP_ID.with(|id| *id).unwrap_or([0; 8])
}

pub fn uart_write(byte: u8) -> Result<(), WriteError> {
    UART.with_mut(|uart| uart.write_byte(byte))
        .unwrap_or(Err(WriteError::Timeout))
}

/// Polling-only wake adapter for the currently proven BL702 path.
///
/// PDS/HBN restoration is not hardware-proven, so any non-active request
/// fails explicitly rather than entering an unsafe sleep state.
#[derive(Debug, Default, Clone, Copy)]
pub struct ActiveOnlyWake;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActiveWakeError {
    UnsupportedSleepDepth,
}

impl<M: MacDriver> WakeController<M> for ActiveOnlyWake {
    type Mark = Instant;
    type Error = ActiveWakeError;

    fn mark(&self) -> Self::Mark {
        Instant::now()
    }

    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark {
        mark + Duration::from_millis(u64::from(duration_ms))
    }

    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32 {
        later
            .duration_since(earlier)
            .as_millis()
            .min(u64::from(u32::MAX)) as u32
    }

    async fn wait(
        &mut self,
        _mac: &mut M,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        if request.sleep_depth != SleepDepth::Active {
            return Err(ActiveWakeError::UnsupportedSleepDepth);
        }
        Timer::after(Duration::from_millis(u64::from(request.timeout_ms))).await;
        Ok(WakeReason::Timer)
    }

    async fn button_held_for(&mut self, duration_ms: u32) -> bool {
        Timer::after(Duration::from_millis(u64::from(duration_ms))).await;
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        Timer::after(Duration::from_millis(u64::from(duration_ms))).await;
    }
}

/// Blocking internal-VBAT source bound to the XT-ZB1 product battery curve.
pub struct SupplyBattery {
    adc: Option<Gpadc>,
}

impl SupplyBattery {
    pub fn new(token: Adc) -> Self {
        let adc = match Gpadc::new(token, Clocks::rom_boot_32mhz()) {
            Ok(adc) => {
                if !adc.gain_trim_valid() {
                    log::warn!("GPADC eFuse gain trim is invalid; using nominal conversion");
                }
                Some(adc)
            }
            Err(error) => {
                log::warn!("GPADC initialization failed; battery remains synthetic: {error:?}");
                None
            }
        };
        Self { adc }
    }

    fn sample_millivolts(&mut self) -> u16 {
        let Some(adc) = self.adc.as_mut() else {
            log::info!("battery sample: 3.000 V synthetic fallback");
            return bl702_xt_zb1_product::battery::SYNTHETIC_FALLBACK_MV;
        };

        match adc.read_supply_mv() {
            Ok(millivolts) if bl702_xt_zb1_product::battery::is_plausible_supply_mv(millivolts) => {
                log::info!(
                    "battery sample: {}.{:03} V nominal",
                    millivolts / 1_000,
                    millivolts % 1_000
                );
                millivolts
            }
            Ok(millivolts) => {
                log::warn!(
                    "GPADC supply reading is implausible ({millivolts} mV); \
                     battery remains synthetic"
                );
                bl702_xt_zb1_product::battery::SYNTHETIC_FALLBACK_MV
            }
            Err(error) => {
                log::warn!("GPADC supply read failed; battery remains synthetic: {error:?}");
                bl702_xt_zb1_product::battery::SYNTHETIC_FALLBACK_MV
            }
        }
    }
}

impl BlockingBatterySource for SupplyBattery {
    type Error = Infallible;

    fn sample(&mut self) -> Result<Option<BatteryReading>, Self::Error> {
        let millivolts = self.sample_millivolts();
        Ok(Some(BatteryReading {
            millivolts: u32::from(millivolts),
            measurement: bl702_xt_zb1_product::battery::measurement(millivolts),
        }))
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Bl702Supervisor;

impl Supervisor for Bl702Supervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        halt()
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Bl702Diagnostics;

impl Diagnostics for Bl702Diagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        match event {
            DiagnosticEvent::Environment(reading) => log::info!(
                "environment synthetic: {}.{:02} C, {}.{:02}% RH",
                reading.temperature_centi_celsius / 100,
                reading.temperature_centi_celsius.unsigned_abs() % 100,
                reading.humidity_centi_percent / 100,
                reading.humidity_centi_percent % 100
            ),
            DiagnosticEvent::Battery {
                millivolts,
                percentage,
            } => log::info!("battery report: {millivolts} mV, {percentage}%"),
            DiagnosticEvent::EnvironmentReadFailed => {
                log::warn!("synthetic environment source failed")
            }
            DiagnosticEvent::BatteryReadFailed => log::warn!("battery source failed"),
            other => log::info!("sensor lifecycle: {other:?}"),
        }
    }
}

pub fn halt() -> ! {
    loop {
        core::hint::spin_loop();
    }
}
