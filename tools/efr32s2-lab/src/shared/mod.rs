//! Lab plumbing per board: HFXO, LED, log transport, panic output.
//!
//! * `wstk` -- BRD4001A + BRD4183A (GSDK 4.5.0
//!   `hardware/board/config/brd4183a_brd4001a`): LED0 PB1, BTN0 PB0, VCOM
//!   USART TX PA5 / RX PA6, VCOM enable PA4, HFXO 38.4 MHz CTUNE 120,
//!   PTI DOUT PC4 / DFRAME PC5. Log over VCOM.
//! * `zbt1` -- Home Assistant Connect ZBT-1: HFXO 38.4 MHz with the CTUNE
//!   of the stock firmware (XTALCTRL read over SWD: 140), no user LED wired
//!   to the EFR32. Log through a SEGGER RTT up-buffer (`JLinkRTTLogger`).

#![allow(dead_code)]

use core::cell::RefCell;
use core::fmt::Write;

use cortex_m::interrupt::Mutex;
use efr32s2_hal::{
    clock::HfxoConfig,
    gpio::{Gpio, Pin, PushPull},
    i2c::I2c0Token,
    peripherals::Peripherals,
    radio::RadioToken,
    rtcc::RtccToken,
    usart::Usart,
};

#[cfg(feature = "wstk")]
pub const HFXO: HfxoConfig = HfxoConfig {
    frequency_hz: 38_400_000,
    ctune: 120,
};
#[cfg(feature = "zbt1")]
pub const HFXO: HfxoConfig = HfxoConfig {
    frequency_hz: 38_400_000,
    ctune: 140,
};

#[cfg(feature = "wstk")]
type Sink = efr32s2_hal::usart::UartTx;
#[cfg(feature = "zbt1")]
type Sink = rtt::Rtt;

static LOG: Mutex<RefCell<Option<Sink>>> = Mutex::new(RefCell::new(None));

/// Write formatted text to the log sink (no-op before [`init`]).
pub fn log_fmt(args: core::fmt::Arguments<'_>) {
    cortex_m::interrupt::free(|cs| {
        if let Some(sink) = LOG.borrow(cs).borrow_mut().as_mut() {
            let _ = sink.write_fmt(args);
            let _ = sink.write_str("\r\n");
        }
    });
}

/// Wait until the VCOM UART has shifted out every queued byte (before EM2).
pub fn log_flush() {
    #[cfg(feature = "wstk")]
    cortex_m::interrupt::free(|cs| {
        if let Some(uart) = LOG.borrow(cs).borrow_mut().as_mut() {
            uart.flush();
        }
    });
}

#[cfg(feature = "zbt1")]
mod rtt {
    //! Minimal SEGGER RTT control block with one non-blocking up-buffer.
    //! The probe finds `_SEGGER_RTT` by its ID string in RAM.

    use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};

    const SIZE: usize = 48 * 1024;

    #[repr(C)]
    struct Up {
        name: *const u8,
        buf: *mut u8,
        size: u32,
        wr: u32,
        rd: u32,
        flags: u32,
    }

    #[repr(C)]
    struct Cb {
        id: [u8; 16],
        max_up: i32,
        max_down: i32,
        up: Up,
        down: Up,
    }

    static mut BUF: [u8; SIZE] = [0; SIZE];
    #[unsafe(no_mangle)]
    static mut _SEGGER_RTT: Cb = Cb {
        id: [0; 16],
        max_up: 1,
        max_down: 1,
        up: Up {
            name: core::ptr::null(),
            buf: core::ptr::null_mut(),
            size: 0,
            wr: 0,
            rd: 0,
            flags: 0,
        },
        down: Up {
            name: core::ptr::null(),
            buf: core::ptr::null_mut(),
            size: 0,
            wr: 0,
            rd: 0,
            flags: 0,
        },
    };

    pub struct Rtt;

    impl Rtt {
        pub fn init() -> Self {
            // SAFETY: called once from `init` before any log output; the
            // ID is written last so the probe never sees a partial block.
            unsafe {
                let cb = addr_of_mut!(_SEGGER_RTT);
                (*cb).up.name = c"Terminal".as_ptr().cast();
                (*cb).up.buf = addr_of_mut!(BUF).cast();
                (*cb).up.size = SIZE as u32;
                (*cb).down.name = c"Terminal".as_ptr().cast();
                core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
                let id = b"SEGGER RTT\0\0\0\0\0\0";
                for (i, b) in id.iter().enumerate() {
                    write_volatile(addr_of_mut!((*cb).id[i]), *b);
                }
            }
            Rtt
        }
    }

    impl core::fmt::Write for Rtt {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            // SAFETY: single producer (callers hold the LOG critical
            // section); the probe only advances `rd`.
            unsafe {
                let cb = addr_of_mut!(_SEGGER_RTT);
                let buf = addr_of_mut!(BUF).cast::<u8>();
                let mut wr = read_volatile(addr_of!((*cb).up.wr)) as usize;
                for &b in s.as_bytes() {
                    let next = (wr + 1) % SIZE;
                    if next == read_volatile(addr_of!((*cb).up.rd)) as usize {
                        break; // full: drop (no host attached)
                    }
                    write_volatile(buf.add(wr), b);
                    wr = next;
                }
                write_volatile(addr_of_mut!((*cb).up.wr), wr as u32);
            }
            Ok(())
        }
    }
}

#[macro_export]
macro_rules! log {
    ($($arg:tt)*) => { $crate::shared::log_fmt(format_args!($($arg)*)) };
}

pub struct Lab {
    pub gpio: Gpio,
    pub i2c0: I2c0Token,
    pub radio: RadioToken,
    pub rtcc: RtccToken,
    pub usart0: Usart,
    pub led: Led,
    #[cfg(feature = "wstk")]
    pub vcom_enable: Pin<PushPull>,
}

/// Board LED, or a no-op where none is wired to the EFR32 (ZBT-1).
pub struct Led(Option<Pin<PushPull>>);

impl Led {
    pub fn toggle(&mut self) {
        if let Some(p) = self.0.as_mut() {
            p.toggle();
        }
    }
    pub fn set_high(&mut self) {
        if let Some(p) = self.0.as_mut() {
            p.set_high();
        }
    }
    pub fn set_low(&mut self) {
        if let Some(p) = self.0.as_mut() {
            p.set_low();
        }
    }
}

/// Bring up HFXO, the LED and the log sink; returns the remaining resources.
pub fn init(name: &str) -> Lab {
    let Peripherals {
        mut clocks,
        #[cfg_attr(feature = "zbt1", allow(unused_mut))]
        mut gpio,
        usart0,
        usart1,
        i2c0,
        radio,
        rtcc,
        ..
    } = Peripherals::take().expect("peripherals taken once");
    let hfxo = clocks.configure_hfxo(HFXO);
    #[cfg(feature = "wstk")]
    let (led, vcom_enable) = {
        // SAFETY: each pin is claimed exactly once here.
        use efr32s2_hal::gpio::{Level, Port};
        let (led, vcom_enable, tx) = unsafe {
            (
                gpio.claim_pin(Port::B, 1).unwrap(),
                gpio.claim_pin(Port::A, 4).unwrap(),
                gpio.claim_pin(Port::A, 5).unwrap(),
            )
        };
        let led = led.into_push_pull(Level::Low);
        let vcom_enable = vcom_enable.into_push_pull(Level::High);
        let uart = usart1.into_uart_tx(tx.into_push_pull(Level::High), 115_200);
        cortex_m::interrupt::free(|cs| *LOG.borrow(cs).borrow_mut() = Some(uart));
        (Led(Some(led)), vcom_enable)
    };
    #[cfg(feature = "zbt1")]
    let led = {
        let _ = usart1;
        let rtt = rtt::Rtt::init();
        cortex_m::interrupt::free(|cs| *LOG.borrow(cs).borrow_mut() = Some(rtt));
        Led(None)
    };
    efr32s2_hal::delay::enable_cycle_counter();
    log!("");
    log!("== efr32s2-lab {} ({:?}) ==", name, efr32s2_hal::chip::CHIP);
    match hfxo {
        Ok(c) => log!("HFXO ok, HCLK {} Hz, CTUNE {}", c.hclk_hz(), HFXO.ctune),
        Err(e) => log!("HFXO FAILED: {:?}", e),
    }
    log!("EUI64 {:016X}", eui64());
    Lab {
        gpio,
        i2c0,
        radio,
        rtcc,
        usart0,
        led,
        #[cfg(feature = "wstk")]
        vcom_enable,
    }
}

/// Factory EUI-64 from DEVINFO (`EUI64L` 0x0FE08048, `EUI64H` 0x0FE0804C).
pub fn eui64() -> u64 {
    unsafe {
        let lo = core::ptr::read_volatile(0x0FE0_8048 as *const u32) as u64;
        let hi = core::ptr::read_volatile(0x0FE0_804C as *const u32) as u64;
        (hi << 32) | lo
    }
}

pub fn delay_ms(ms: u32) {
    for _ in 0..ms {
        efr32s2_hal::delay::spin_cycles(efr32s2_hal::delay::us_to_cycles(1000));
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    log!("PANIC: {}", info);
    loop {
        cortex_m::asm::bkpt();
    }
}

#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    log!("HARDFAULT pc={:08X} lr={:08X}", frame.pc(), frame.lr());
    loop {
        cortex_m::asm::bkpt();
    }
}

/// The RTCC time driver's queue (embassy-time-queue-utils) links against
/// embassy-executor's pender; the lab has no executor, so wakes are no-ops.
#[unsafe(no_mangle)]
fn __pender(_context: *mut ()) {}
