use core::mem::MaybeUninit;

#[repr(C)]
struct FaultLog {
    hardfault_magic: u32,
    hardfault_pc: u32,
    hardfault_lr: u32,
    hardfault_xpsr: u32,
    hardfault_msp: u32,
    hardfault_r0: u32,
    hardfault_r12: u32,
    panic_magic: u32,
    panic_line: u32,
    panic_column: u32,
    panic_file_ptr: u32,
    panic_file_len: u32,
}

#[unsafe(link_section = ".uninit.fault_log")]
static mut FAULT_LOG: MaybeUninit<FaultLog> = MaybeUninit::uninit();

#[inline(always)]
unsafe fn fault_log_mut() -> *mut FaultLog {
    core::ptr::addr_of_mut!(FAULT_LOG).cast::<FaultLog>()
}

pub fn clear() {
    unsafe {
        fault_log_mut().write_volatile(FaultLog {
            hardfault_magic: 0,
            hardfault_pc: 0,
            hardfault_lr: 0,
            hardfault_xpsr: 0,
            hardfault_msp: 0,
            hardfault_r0: 0,
            hardfault_r12: 0,
            panic_magic: 0,
            panic_line: 0,
            panic_column: 0,
            panic_file_ptr: 0,
            panic_file_len: 0,
        });
    }
}

#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    unsafe {
        let msp: u32;
        core::arch::asm!("mrs {}, msp", out(reg) msp);
        let log = fault_log_mut();
        core::ptr::addr_of_mut!((*log).hardfault_magic).write_volatile(0xDEAD_BEEF);
        core::ptr::addr_of_mut!((*log).hardfault_pc).write_volatile(frame.pc());
        core::ptr::addr_of_mut!((*log).hardfault_lr).write_volatile(frame.lr());
        core::ptr::addr_of_mut!((*log).hardfault_xpsr).write_volatile(frame.xpsr());
        core::ptr::addr_of_mut!((*log).hardfault_msp).write_volatile(msp);
        core::ptr::addr_of_mut!((*log).hardfault_r0).write_volatile(frame.r0());
        core::ptr::addr_of_mut!((*log).hardfault_r12).write_volatile(frame.r12());
    }
    loop {
        cortex_m::asm::nop();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    unsafe {
        let log = fault_log_mut();
        core::ptr::addr_of_mut!((*log).panic_magic).write_volatile(0x5041_4E49);
        if let Some(location) = info.location() {
            core::ptr::addr_of_mut!((*log).panic_line).write_volatile(location.line());
            core::ptr::addr_of_mut!((*log).panic_column).write_volatile(location.column());
            core::ptr::addr_of_mut!((*log).panic_file_ptr)
                .write_volatile(location.file().as_ptr() as u32);
            core::ptr::addr_of_mut!((*log).panic_file_len)
                .write_volatile(location.file().len() as u32);
        }
    }
    loop {
        cortex_m::asm::nop();
    }
}
