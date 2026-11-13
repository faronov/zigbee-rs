//! Start-up, VCOM output and fault handling shared by the lab binaries.

use core::fmt::{self, Write};

use stm32wb_hal::embassy_stm32::mode::Blocking;
use stm32wb_hal::embassy_stm32::usart::UartTx;
use stm32wb_hal::embassy_stm32::{self, Peri, Peripherals, pac, peripherals};

/// Blocking VCOM writer (USART1, 115200 8N1).
pub struct Vcom(UartTx<'static, Blocking>);

impl Vcom {
    pub fn new(
        usart1: Peri<'static, peripherals::USART1>,
        tx: Peri<'static, peripherals::PB6>,
    ) -> Self {
        Self(stm32wb5mm_dk::vcom_tx_blocking(usart1, tx))
    }
}

impl Write for Vcom {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.0.blocking_write(s.as_bytes()).map_err(|_| fmt::Error)
    }
}

/// One VCOM line.
#[macro_export]
macro_rules! say {
    ($vcom:expr, $($arg:tt)*) => {{
        let _ = core::fmt::Write::write_fmt(&mut $vcom, format_args!($($arg)*));
        let _ = core::fmt::Write::write_str(&mut $vcom, "\r\n");
    }};
}

/// The board clock tree (with the SAI1 kernel clock) and Embassy time.
pub fn init() -> Peripherals {
    let mut config = embassy_stm32::Config::default();
    config.rcc = stm32wb5mm_dk::rcc_config();
    embassy_stm32::init(config)
}

/// Stop: interrupts off, wait for the debugger.
pub fn halt() -> ! {
    cortex_m::interrupt::disable();
    loop {
        cortex_m::asm::wfi();
    }
}

struct RawUsart1;

impl Write for RawUsart1 {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.bytes() {
            while !pac::USART1.isr().read().txe() {}
            pac::USART1.tdr().write(|w| w.set_dr(u16::from(byte)));
        }
        Ok(())
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    let _ = write!(RawUsart1, "\r\n[DK] PANIC {info}\r\n");
    halt()
}
