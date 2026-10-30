//! Target-only boot flow: install, validate, jump, or report and halt.

use bl702_hal::flash::{XipFlash, invalidate_xip_cache};

use crate::{
    BootFlash, Config, Console, Reporter, check_entry, image_error_name, write_hex, write_version,
};

/// Run one boot decision and start the application, or report the failure
/// and halt.
///
/// `flash` must be the physical XIP flash (ROM routines, physical offsets).
/// Clocks are expected in the ROM boot state, which the application assumes
/// as well; the loader changes no clock or power setting.
pub fn run(flash: XipFlash, config: &Config, console: &mut impl Console) -> ! {
    let mut flash = BootFlash::new(flash, &config.layout);
    let mut reporter = Reporter::default();
    let result = mcuboot_image::boot(&mut flash, &config.layout, &config.key, |event| {
        reporter.event(console, event)
    });
    reporter.end_line(console);
    match result {
        Ok(bootable) => {
            reporter.update(console, bootable.update);
            let image_end = config.layout.primary
                + u32::from(bootable.image.header.hdr_size)
                + bootable.image.header.img_size;
            let entry = bootable.vector_table;
            match (
                check_entry(config.layout.primary, entry, image_end),
                config.xip_address(entry),
            ) {
                (Ok(()), Some(xip_entry)) => {
                    console.write(b"bl702-boot: app ");
                    write_version(console, &bootable.image.header.version);
                    console.write(b" @ ");
                    write_hex(console, xip_entry);
                    console.write(b"\r\n");
                    // A rewritten primary may still be cached from the
                    // validation reads through XIP (none today; the ROM
                    // routines read physically) or a previous boot; drop
                    // every line before fetching the new code.
                    if invalidate_xip_cache().is_err() {
                        console.write(b"bl702-boot: cache flush failed\r\n");
                    } else {
                        drain_console();
                        // SAFETY: the image was validated, its entry checked
                        // to be the address the application is linked at,
                        // and the cache invalidated; no interrupt has been
                        // enabled by the loader.
                        unsafe { jump(xip_entry) }
                    }
                }
                _ => console.write(b"bl702-boot: bad entry\r\n"),
            }
        }
        Err(crate::BootError::NoBootableImage { primary, update }) => {
            reporter.update(console, update);
            console.write(b"bl702-boot: no bootable image: ");
            console.write(image_error_name(primary));
            console.write(b"\r\n");
        }
        Err(crate::BootError::CopyCorrupt) => {
            console.write(b"bl702-boot: installed copy corrupt, request kept\r\n");
        }
        Err(crate::BootError::Flash) => console.write(b"bl702-boot: flash error\r\n"),
    }
    console.write(b"bl702-boot: halted\r\n");
    halt()
}

/// Give the UART FIFO time to drain before the application resets UART0
/// (32 bytes at 2 Mbaud is 160 us).
fn drain_console() {
    bl702_hal::timer::delay_us(1_000);
}

fn halt() -> ! {
    // SAFETY: masking machine interrupts has no memory-safety effect.
    unsafe { core::arch::asm!("csrci mstatus, 8") };
    loop {
        // SAFETY: `wfi` is a hint; with MIE clear it simply idles.
        unsafe { core::arch::asm!("wfi") };
    }
}

/// Branch to the application's riscv-rt `_start`.
///
/// `_start` itself disables and clears `mie`/`mip`, installs its pre-init
/// trap vector, loads `gp` and `sp` from its own link-time symbols and
/// initializes `.data`/`.bss`, so only the interrupt state is handed over:
/// global interrupts off and nothing pending from the loader.
unsafe fn jump(entry: u32) -> ! {
    // SAFETY: the caller guarantees `entry` is a validated application entry
    // in the XIP window; control never returns.
    unsafe {
        core::arch::asm!(
            "csrci mstatus, 8",
            "csrw mie, zero",
            "fence",
            "fence.i",
            "jr {entry}",
            entry = in(reg) entry,
            options(noreturn),
        )
    }
}
