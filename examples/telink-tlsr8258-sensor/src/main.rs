//! # Telink TLSR8258 Zigbee Temperature Sensor — Pure Rust
//!
//! A `no_std` firmware for the Telink TLSR8258 (tc32 ISA).
//! Zero C dependencies, zero vendor SDK. Everything is Rust.
//!
//! ## Hardware
//! - TB-04-Kit: Telink TLSR8258 (tc32 core, 512KB Flash, 64KB SRAM)
//! - RGB LED: PC4 (Red), PC1 (Green), PB5 (Blue)
//! - Built-in IEEE 802.15.4 + BLE radio
//!
//! ## Building
//! ```bash
//! TC=/tmp/tc32-toolchain/tc32-stage2-x86_64-apple-darwin
//! cd zigbee-rs-fork/examples/telink-tlsr8258-sensor
//! $TC/bin/cargo build --release
//! ```

#![no_std]
#![no_main]

use panic_halt as _;

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::{MacDriver, MacError};
use zigbee_nwk::DeviceType;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::identify::IdentifyCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

// ── Boot vector table (tc32 ISA — native mnemonics) ──────────
//
// The tc32 LLVM assembler accepts standard Thumb-1 mnemonics and
// encodes them as tc32 opcodes automatically. No more .short!
//
//   0x00: tj __reset    (tc32 unconditional branch)
//   0x08: "KNLT" magic
//   0x0C: 0x00880000 + binary_size/16
//   0x10: tj __irq_stub (tc32 unconditional branch)
//   0x18: binary size
//   0x20: __irq_stub: bx lr
//   0x30: __reset: set SP, jump to Rust _start
core::arch::global_asm!(
    ".section .vectors, \"ax\"",
    ".balign 4",
    ".global _reset_vector",
    "_reset_vector:",
    // ─── Offset 0x00: Header ───
    "tj __reset",
    ".short 0x0000",            // 0x02: file version (0)
    ".word 0x00000000",         // 0x04: reserved
    ".byte 0x4B, 0x4E, 0x4C, 0x54",  // 0x08: "KNLT" magic
    ".word 0x00880000 + _bin_size_div_16",
    "tj __irq_stub",
    ".short 0x0000", ".short 0x0000", ".short 0x0000",
    ".word _bin_size_",
    ".word 0x00000000",         // 0x1C: reserved
    //
    // ─── Offset 0x20: IRQ stub ───
    "__irq_stub:",
    "bx lr",
    "nop", "nop", "nop", "nop", "nop", "nop", "nop",
    //
    // ─── Offset 0x30: __reset ───
    // Set stack pointer to top of RAM (0x850000), then jump to Rust
    "__reset:",
    "movs r0, #0x85",
    "lsls r0, r0, #16",        // r0 = 0x850000
    "mov sp, r0",
    "tj _start",
);

// ── Pure Rust startup ──────────────────────────────────────────

unsafe extern "C" {
    static mut _sdata: u8;
    static mut _edata: u8;
    static mut _sbss: u8;
    static mut _ebss: u8;
    static _etext: u8;
    static _stack_top: u8;
}

/// Entry point — called from __reset after SP setup.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _start() -> ! {
    unsafe {
        // Disable watchdog
        core::ptr::write_volatile((REG_BASE + 0x622) as *mut u8, 0x00);

        // Copy .data from flash to RAM
        let data_len = &raw const _edata as usize - &raw const _sdata as usize;
        let src = &raw const _etext as *const u8;
        let dst = &raw mut _sdata as *mut u8;
        core::ptr::copy_nonoverlapping(src, dst, data_len);

        // Zero .bss
        let bss_len = &raw const _ebss as usize - &raw const _sbss as usize;
        let bss = &raw mut _sbss as *mut u8;
        core::ptr::write_bytes(bss, 0, bss_len);
    }

    main_loop();
}

// ── TLSR8258 register base ─────────────────────────────────────

const REG_BASE: u32 = 0x800000;

// ── Multi-port GPIO driver ─────────────────────────────────────

mod gpio {
    //! Bare-metal GPIO for TLSR8258 — supports all ports (PA-PD).
    //!
    //! Register layout per port (8 bytes each):
    //!   +0: IN   (read input)
    //!   +1: IE   (input enable)
    //!   +2: OEN  (output enable, active-LOW: 0=output)
    //!   +3: OUT  (output data)
    //!   +4: POL  (polarity)
    //!   +5: DS   (drive strength)
    //!   +6: GPIO (GPIO function enable)
    //!   +7: IRQ  (interrupt)

    const REG_BASE: u32 = 0x800000;

    #[derive(Clone, Copy)]
    pub struct Pin {
        pub port_base: u32,
        pub bit: u8,
    }

    // Port base addresses
    pub const PA: u32 = REG_BASE + 0x580;
    pub const PB: u32 = REG_BASE + 0x588;
    pub const PC: u32 = REG_BASE + 0x590;
    pub const PD: u32 = REG_BASE + 0x598;

    impl Pin {
        pub const fn new(port_base: u32, bit: u8) -> Self {
            Self { port_base, bit }
        }

        fn mask(self) -> u8 {
            1u8 << self.bit
        }

        /// Configure pin as GPIO output
        pub fn set_output(self) {
            let mask = self.mask();
            unsafe {
                // Enable GPIO function
                let r = (self.port_base + 6) as *mut u8;
                core::ptr::write_volatile(r, core::ptr::read_volatile(r) | mask);
                // OEN active-low: clear bit = output enabled
                let r = (self.port_base + 2) as *mut u8;
                core::ptr::write_volatile(r, core::ptr::read_volatile(r) & !mask);
                // Disable input
                let r = (self.port_base + 1) as *mut u8;
                core::ptr::write_volatile(r, core::ptr::read_volatile(r) & !mask);
            }
        }

        /// Configure pin as GPIO input with input-enable
        pub fn set_input(self) {
            let mask = self.mask();
            unsafe {
                let r = (self.port_base + 6) as *mut u8;
                core::ptr::write_volatile(r, core::ptr::read_volatile(r) | mask);
                let r = (self.port_base + 1) as *mut u8;
                core::ptr::write_volatile(r, core::ptr::read_volatile(r) | mask);
                // OEN active-low: set bit = output disabled
                let r = (self.port_base + 2) as *mut u8;
                core::ptr::write_volatile(r, core::ptr::read_volatile(r) | mask);
            }
        }

        /// Write pin HIGH or LOW
        pub fn write(self, high: bool) {
            let mask = self.mask();
            unsafe {
                let r = (self.port_base + 3) as *mut u8;
                let v = core::ptr::read_volatile(r);
                core::ptr::write_volatile(r, if high { v | mask } else { v & !mask });
            }
        }

        /// Read input level
        pub fn read(self) -> bool {
            unsafe {
                let r = (self.port_base + 0) as *const u8;
                (core::ptr::read_volatile(r) >> self.bit) & 1 == 1
            }
        }
    }
}

// ── TB-04-Kit board definition ─────────────────────────────────

mod board {
    //! TB-04-Kit pinout (from official schematic)
    use super::gpio::{self, Pin};

    pub const LED_RED:   Pin = Pin::new(gpio::PC, 4);  // RGB Red (PWM2)
    pub const LED_GREEN: Pin = Pin::new(gpio::PC, 1);  // RGB Green (PWM0)
    pub const LED_BLUE:  Pin = Pin::new(gpio::PB, 5);  // RGB Blue (PWM5)
    pub const LED_WHITE: Pin = Pin::new(gpio::PD, 4);  // Cool White
    pub const LED_WARM:  Pin = Pin::new(gpio::PA, 0);  // Warm Yellow
}

// ── Blocking delay (system timer) ──────────────────────────────

fn delay_ms(ms: u32) {
    const REG_SYS_TIMER: u32 = 0x800740;
    // System timer runs at 16MHz after reset
    let ticks = ms * 16_000;
    let start = unsafe { core::ptr::read_volatile(REG_SYS_TIMER as *const u32) };
    while unsafe { core::ptr::read_volatile(REG_SYS_TIMER as *const u32) }
        .wrapping_sub(start) < ticks {}
}

// ── Stub MAC driver (radio bringup is separate track) ──────────

/// Minimal MAC stub for TLSR8258 — returns errors for all radio ops.
/// Allows the full Zigbee stack to compile and run its state machine
/// while radio hardware bringup continues separately.
pub struct Tlsr8258Mac;

impl Tlsr8258Mac {
    pub fn new() -> Self { Self }
}

use zigbee_mac::primitives::*;

impl MacDriver for Tlsr8258Mac {
    async fn mlme_scan(&mut self, _req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        Err(MacError::Unsupported)
    }

    async fn mlme_associate(&mut self, _req: MlmeAssociateRequest) -> Result<MlmeAssociateConfirm, MacError> {
        Err(MacError::Unsupported)
    }

    async fn mlme_associate_response(&mut self, _rsp: MlmeAssociateResponse) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }

    async fn mlme_disassociate(&mut self, _req: MlmeDisassociateRequest) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }

    async fn mlme_reset(&mut self, _set_default_pib: bool) -> Result<(), MacError> {
        Ok(())
    }

    async fn mlme_start(&mut self, _req: MlmeStartRequest) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }

    async fn mlme_get(&self, _attr: zigbee_mac::PibAttribute) -> Result<zigbee_mac::PibValue, MacError> {
        Err(MacError::Unsupported)
    }

    async fn mlme_set(&mut self, _attr: zigbee_mac::PibAttribute, _val: zigbee_mac::PibValue) -> Result<(), MacError> {
        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        Ok(None)
    }

    async fn mcps_data(&mut self, _req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        Err(MacError::Unsupported)
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        Err(MacError::Unsupported)
    }

    fn capabilities(&self) -> zigbee_mac::MacCapabilities {
        zigbee_mac::MacCapabilities {
            coordinator: false,
            router: false,
            hardware_security: false,
            max_payload: 100,
            tx_power_min: zigbee_types::TxPower(0),
            tx_power_max: zigbee_types::TxPower(8),
        }
    }
}

// ── Critical section (single-core, interrupt disable) ──────────

#[unsafe(no_mangle)]
pub unsafe extern "C" fn _critical_section_1_0_acquire() -> u8 {
    // tc32 doesn't have mrs/cpsid — use TLSR8258 IRQ enable register
    unsafe {
        let irq_en = (REG_BASE + 0x643) as *mut u8;
        let prev = core::ptr::read_volatile(irq_en);
        core::ptr::write_volatile(irq_en, 0);
        prev
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn _critical_section_1_0_release(restore: u8) {
    if restore != 0 {
        unsafe {
            let irq_en = (REG_BASE + 0x643) as *mut u8;
            core::ptr::write_volatile(irq_en, restore);
        }
    }
}

// ── Main loop (synchronous — no async executor) ────────────────

fn main_loop() -> ! {
    // Setup RGB LED pins
    board::LED_RED.set_output();
    board::LED_GREEN.set_output();
    board::LED_BLUE.set_output();

    // Stage 1: RED = GPIO init done
    board::LED_RED.write(true);
    delay_ms(500);
    board::LED_RED.write(false);
    delay_ms(200);

    // Stage 2: GREEN = MAC + clusters created
    let mac = Tlsr8258Mac::new();
    let mut basic_cluster = BasicCluster::new(
        b"Zigbee-RS", b"TLSR8258-Sensor", b"20260414", b"0.1.0",
    );
    basic_cluster.set_power_source(0x03);
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);
    let mut power_cluster = PowerConfigCluster::new();
    let mut identify_cluster = IdentifyCluster::new();
    temp_cluster.set_temperature(2250);
    hum_cluster.set_humidity(5000u16);
    power_cluster.set_battery_voltage(30);
    power_cluster.set_battery_percentage(200);

    board::LED_GREEN.write(true);
    delay_ms(500);
    board::LED_GREEN.write(false);
    delay_ms(200);

    // Stage 3: BLUE = device builder
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer("Zigbee-RS")
        .model("TLSR8258-Sensor")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0003) // Identify
                .cluster_server(0x0001) // Power Configuration
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
        })
        .build();

    board::LED_BLUE.write(true);
    delay_ms(500);
    board::LED_BLUE.write(false);
    delay_ms(200);

    // Stage 4: WHITE (all RGB) = main loop entered
    // Skip user_action(Join) for now — it triggers BDB scan which may hang
    board::LED_RED.write(true);
    board::LED_GREEN.write(true);
    board::LED_BLUE.write(true);
    delay_ms(500);
    board::LED_RED.write(false);
    board::LED_GREEN.write(false);
    board::LED_BLUE.write(false);
    delay_ms(200);

    let mut tick_counter: u32 = 0;

    loop {
        // Alternate RED/GREEN heartbeat to prove loop is running
        if tick_counter % 20 == 0 {
            if (tick_counter / 20) % 2 == 0 {
                board::LED_RED.write(true);
                delay_ms(50);
                board::LED_RED.write(false);
            } else {
                board::LED_GREEN.write(true);
                delay_ms(50);
                board::LED_GREEN.write(false);
            }
        }

        // Tick the Zigbee stack
        let mut clusters = [
            ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
            ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
            ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
            ClusterRef { endpoint: 1, cluster: &mut power_cluster },
            ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
        ];

        let waker = noop_waker();
        let mut cx = core::task::Context::from_waker(&waker);
        let mut fut = core::pin::pin!(device.tick(1, &mut clusters));
        let _ = fut.as_mut().poll(&mut cx);

        tick_counter = tick_counter.wrapping_add(1);
        delay_ms(100);
    }
}

/// No-op waker for polling futures in a bare-metal blocking loop.
fn noop_waker() -> core::task::Waker {
    use core::task::{RawWaker, RawWakerVTable};
    const VTABLE: RawWakerVTable = RawWakerVTable::new(
        |p| RawWaker::new(p, &VTABLE),
        |_| {},
        |_| {},
        |_| {},
    );
    unsafe { core::task::Waker::new(core::ptr::null(), &VTABLE) }
}
