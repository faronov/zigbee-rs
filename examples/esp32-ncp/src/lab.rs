//! Lab-only boot diagnostics (feature `boot-trace`; never in shipped
//! images). Every function is a no-op without the feature.
//!
//! On the Thread BR board the H2's UART0 is both the ZRHP link and the only
//! console, so a hang or panic before the link works is invisible. With
//! `boot-trace`:
//!
//! * [`boot`] prints a marker into the UART0 FIFO at the ROM rate
//!   (115 200), then reports what the previous run left in RTC fast memory
//!   (kept across software and watchdog resets, not across EN): its panic
//!   message, its last [`stage`] and its UART transport counters; it arms
//!   the RTC watchdog (3 s);
//! * [`tick`] (run loop) feeds the watchdog for the first 6 s of uptime and
//!   snapshots the transport counters, so a stalled link ends in a watchdog
//!   reset that reports them;
//! * [`panic`] keeps the panic message for the next boot.
//!
//! Raw FIFO writes bypass the link framing; the host sees them as noise.

#[cfg(feature = "boot-trace")]
mod imp {
    use core::fmt::Write;

    const PANIC_MAGIC_VALUE: u32 = 0x5041_4E43;
    const COUNTERS_MAGIC: u32 = 0x4C41_4231;

    #[esp_hal::ram(unstable(rtc_fast, persistent))]
    static mut PANIC_MAGIC: u32 = 0;
    #[esp_hal::ram(unstable(rtc_fast, persistent))]
    static mut PANIC_LEN: usize = 0;
    #[esp_hal::ram(unstable(rtc_fast, persistent))]
    static mut PANIC_BUF: [u8; 384] = [0; 384];
    #[esp_hal::ram(unstable(rtc_fast, persistent))]
    static mut STAGE: u8 = 0;
    #[esp_hal::ram(unstable(rtc_fast, persistent))]
    static mut COUNTERS: [u32; 8] = [0; 8];
    static mut WATCHDOG: Option<esp_hal::rtc_cntl::Rtc<'static>> = None;

    /// Raw bytes into the UART0 TX FIFO at its current rate.
    pub fn trace(text: &[u8]) {
        let uart = esp_hal::peripherals::UART0::regs();
        for &byte in text {
            while uart.status().read().txfifo_cnt().bits() >= 120 {}
            uart.fifo()
                .write(|w| unsafe { w.rxfifo_rd_byte().bits(byte) });
        }
    }

    struct Console;
    impl Write for Console {
        fn write_str(&mut self, text: &str) -> core::fmt::Result {
            trace(text.as_bytes());
            Ok(())
        }
    }

    struct Keep;
    impl Write for Keep {
        fn write_str(&mut self, text: &str) -> core::fmt::Result {
            // SAFETY: single-threaded panic path.
            unsafe {
                let len = core::ptr::addr_of_mut!(PANIC_LEN);
                let buf = core::ptr::addr_of_mut!(PANIC_BUF).cast::<u8>();
                for &byte in text.as_bytes() {
                    if *len < 384 {
                        buf.add(*len).write(byte);
                        *len += 1;
                    }
                }
            }
            Ok(())
        }
    }

    pub fn boot() {
        let _ = write!(Console, "\r\n[ncp] boot\r\n");
        // SAFETY: before anything else runs; RTC memory is plain data.
        unsafe {
            if core::ptr::addr_of!(PANIC_MAGIC).read_volatile() == PANIC_MAGIC_VALUE {
                let len = core::ptr::addr_of!(PANIC_LEN).read_volatile().min(384);
                let buf =
                    core::slice::from_raw_parts(core::ptr::addr_of!(PANIC_BUF).cast::<u8>(), len);
                trace(b"[ncp] previous panic: ");
                trace(buf);
                trace(b"\r\n");
            }
            core::ptr::addr_of_mut!(PANIC_MAGIC).write_volatile(0);
            let stage = core::ptr::addr_of!(STAGE).read_volatile();
            let _ = write!(Console, "[ncp] previous stage {}\r\n", stage);
            core::ptr::addr_of_mut!(STAGE).write_volatile(0);
            let counters = core::ptr::addr_of!(COUNTERS).read_volatile();
            if counters[7] == COUNTERS_MAGIC {
                let _ = write!(
                    Console,
                    "[ncp] previous uart: rx {} tx {} transfers {} refused {} completed {} \
                     overflow {} line errors {}\r\n",
                    counters[0],
                    counters[1],
                    counters[2],
                    counters[3],
                    counters[4],
                    counters[5],
                    counters[6]
                );
            }
            core::ptr::addr_of_mut!(COUNTERS).write_volatile([0; 8]);
            let mut rtc = esp_hal::rtc_cntl::Rtc::new(esp_hal::peripherals::LPWR::steal());
            rtc.rwdt.set_timeout(
                esp_hal::rtc_cntl::RwdtStage::Stage0,
                esp_hal::time::Duration::from_millis(3000),
            );
            rtc.rwdt.enable();
            *core::ptr::addr_of_mut!(WATCHDOG) = Some(rtc);
        }
    }

    pub fn stage(stage: u8) {
        // SAFETY: plain byte store into RTC memory.
        unsafe { core::ptr::addr_of_mut!(STAGE).write_volatile(stage) };
    }

    pub fn tick(_host: &crate::product::host::Host) {
        // SAFETY: only the run loop touches the watchdog after boot.
        if let Some(rtc) = unsafe { (*core::ptr::addr_of_mut!(WATCHDOG)).as_mut() }
            && crate::now_ms() < 6_000
        {
            rtc.rwdt.feed();
        }
        #[cfg(feature = "transport-uart")]
        {
            let c = _host.counters();
            // SAFETY: plain stores into RTC memory.
            unsafe {
                core::ptr::addr_of_mut!(COUNTERS).write_volatile([
                    c.rx_bytes,
                    c.tx_bytes,
                    c.tx_transfers,
                    c.tx_refused,
                    c.tx_completed,
                    c.rx_overflow,
                    c.rx_line_errors,
                    COUNTERS_MAGIC,
                ]);
            }
        }
    }

    pub fn panic(info: &core::panic::PanicInfo) {
        // SAFETY: single-threaded panic path.
        unsafe { core::ptr::addr_of_mut!(PANIC_LEN).write_volatile(0) };
        let _ = write!(Keep, "{}", info);
        // SAFETY: as above.
        unsafe { core::ptr::addr_of_mut!(PANIC_MAGIC).write_volatile(PANIC_MAGIC_VALUE) };
        let _ = write!(Console, "\r\n[ncp] PANIC {}\r\n", info);
    }
}

#[cfg(feature = "boot-trace")]
pub use imp::{boot, panic, stage, tick};

#[cfg(not(feature = "boot-trace"))]
#[inline(always)]
pub fn boot() {}

#[cfg(not(feature = "boot-trace"))]
#[inline(always)]
pub fn stage(_stage: u8) {}

#[cfg(not(feature = "boot-trace"))]
#[inline(always)]
pub fn tick(_host: &crate::product::host::Host) {}

#[cfg(not(feature = "boot-trace"))]
#[inline(always)]
pub fn panic(_info: &core::panic::PanicInfo) {}
