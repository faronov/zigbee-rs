//! PTA (packet traffic arbitration): external Wi-Fi/802.15.4 coexistence
//! of the ESP32-H2 on the Espressif Thread Border Router / Zigbee Gateway
//! board (spec 11.5; config `0x0009 coex_enable`, `0x000A
//! coex_tx_priority_mask`; `SYS_GET_STATUS.flags` bit 2).
//!
//! # Mechanism
//!
//! The ESP32-S3 Wi-Fi is the arbitration leader; the H2 is the follower on
//! four wires (request, priority: H2 → S3; grant, tx_line: S3 → H2). The
//! arbitration itself is hardware: the 802.15.4 MAC asks the modem
//! coexistence block, which drives request/priority and gates the radio on
//! grant. A transmission without grant ends in the MAC's `TxCoexBreak`
//! abort.
//!
//! [`Pta::set`] reproduces ESP-IDF **v5.5.1**
//! `components/esp_coex/src/coexist.c` `esp_enable_extern_coex_gpio_pin`
//! for `EXTERNAL_COEX_FOLLOWER_ROLE` and `EXTERN_COEX_WIRE_4` (lines
//! 161-276; follower branch 215-257) and `esp_disable_extern_coex_gpio_pin`
//! (278-283), with the pins of `examples/openthread/ot_rcp` (request 0,
//! grant 1, priority 2, tx_line 3, wire type 3), which are this board's:
//!
//! | Wire | H2 pin | Direction | GPIO-matrix signal (H2 `gpio_sig_map.h`) |
//! |---|---|---|---|
//! | request | GPIO0 | out | `EXTERN_ACTIVE_O_IDX` 82 |
//! | grant | GPIO1 | in | `EXTERN_ACTIVE_I_IDX` 82 |
//! | priority | GPIO2 | out | `EXTERN_PRIORITY_O_IDX` 81 |
//! | tx_line | GPIO3 | in | `EXTERN_PRIORITY_I_IDX` 81 |
//!
//! The spec's "drive tx_line for the duration of each TX" does not match
//! ESP-IDF: in the 4-wire follower role tx_line is an *input* (the S3
//! leader's Wi-Fi TX indication, `SOC_EXTERNAL_COEX_LEADER_TX_LINE` on the
//! S3, `BB_DIAG9_IDX`).
//!
//! The arbiter configuration is the closed `libcoexist.a` that
//! `esp-wifi-sys` 0.8.1 links for the radio (`libs/esp32h2`, SHA-1
//! `d67b0a16d30eb2d876b6a717a84840ea14f18b82`, byte-identical to ESP-IDF
//! v5.5.1's `components/esp_coex/lib/esp32h2/libcoexist.a`). Used symbols
//! (`external_coex.o`): `esp_coex_external_set_wire_type`,
//! `esp_coex_external_params`, `esp_coex_external_set`,
//! `esp_coex_external_stop`. Their disassembly shows register writes to
//! `0x600AD49C..0x600AD4D8` (coexistence block) and to IEEE802154 `0x04`
//! (bit 26, coex delay); `ic_set_extern_coex`/`ic_stop_extern_coex` also
//! call `coex_core_enable/disable` and `coex_schm_status_bit_set/clear`,
//! which are software bookkeeping guarded by locks whose handles
//! (`coex_env[0]`, `coex_schm_env+0x10`) stay zero because ESP-IDF's
//! `coex_pre_init` (`esp_system/startup_funcs.c` `init_coexist`) is not run
//! here: they return `ESP_ERR_INVALID_STATE` without side effects, and the
//! blob ignores those results. The GPIO matrix is set with the ROM routines
//! ESP-IDF calls (`esp_rom_gpio_connect_{in,out}_signal` =
//! `gpio_matrix_{in,out}`); esp-hal's signal enums have no 81/82.
//!
//! The radio side (ESP-IDF's per-operation `esp_coex_ieee802154_extcoex_
//! {tx,rx}_stage` calls and configurable scene PTIs) is the esp-radio local
//! patch `ieee802154-extcoex` (`vendor/esp-radio/LOCAL-PATCHES.md`).
//!
//! # Priority mask (`coex_tx_priority_mask`)
//!
//! The only priority control the blob offers is the PTI of each radio
//! scene (`esp_coex_ieee802154_txrx_pti_set`) and of the hardware ACK
//! (`esp_coex_ieee802154_ack_pti_set`); see [`priorities`]. There is no
//! per-frame PTI, so bit 2 (TC/key transport) alone is refused with
//! [`PtaError::UnsupportedClass`]; together with bit 3 (all transmissions)
//! it is honoured. Bits 4-7 are refused with [`PtaError::ReservedBits`].
//! Whether the follower asserts the priority wire for `High` only is not
//! documented; the lab binary `pta_lab` measures it.
//!
//! # State
//!
//! Disabled at construction (the arbiter is reset-disabled); request and
//! priority are then driven low (deasserted) as plain GPIO outputs instead
//! of being left floating. Disabling a running arbiter stops it, parks the
//! outputs low again and restores the ESP-IDF default priorities;
//! disabling while disabled touches no register. The state is volatile, as
//! `coex_enable` is.
//!
//! Suggested `PlatformError` mapping for `NcpPlatform::set_pta`:
//! `ReservedBits`/`UnsupportedClass` → `InvalidParameter`, `RadioOff` →
//! `InvalidState`, `Arbiter` → `InvalidState`.

/// `coex_tx_priority_mask` bits (spec 7.1, config `0x000A`).
pub mod mask {
    /// MAC ACK frames (hardware automatic ACK).
    pub const MAC_ACK: u8 = 0x01;
    /// Green Power timed TX (TempMaster reply at an instant).
    pub const GP_TIMED_TX: u8 = 0x02;
    /// Trust Center / key transport frames.
    pub const TC_KEY_TRANSPORT: u8 = 0x04;
    /// All transmissions.
    pub const ALL: u8 = 0x08;
    /// Bits defined by ZRHP v1.
    pub const DEFINED: u8 = MAC_ACK | GP_TIMED_TX | TC_KEY_TRANSPORT | ALL;
}

/// Why [`Pta::set`] (or [`priorities`]) refused. Nothing was changed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PtaError {
    /// Mask bits not defined by ZRHP v1 (the offending bits).
    ReservedBits(u8),
    /// Frame classes the radio cannot prioritise on their own (the
    /// offending bits): the 802.15.4 driver has one PTI per scene, not per
    /// frame.
    UnsupportedClass(u8),
    /// The 802.15.4/coexistence clocks are off (radio not started); the
    /// blob's register writes would be lost.
    RadioOff,
    /// `esp_coex_external_params`/`esp_coex_external_set` returned this
    /// `esp_err_t`; the arbiter was stopped again.
    Arbiter(i32),
}

/// Coexistence priority of a radio operation (ESP-IDF
/// `ieee802154_coex_event_t`; through `coex_pti_tab` of the H2 blob:
/// `High` 0x0C, `Middle` 0x08, `Low` 0x03, `Idle` 0x01).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Priority {
    High,
    Middle,
    Low,
    Idle,
}

/// Priority of each radio scene, mirroring esp-radio's `CoexPti`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScenePriorities {
    pub idle: Priority,
    pub rx: Priority,
    pub tx: Priority,
    /// Transmissions at an instant (GP TempMaster reply; ESP-IDF `TX_AT`).
    pub timed: Priority,
    /// Hardware automatic ACK.
    pub ack: Priority,
}

impl ScenePriorities {
    /// ESP-IDF v5.5.1 defaults (`esp_ieee802154_util.c` `s_coex_config`,
    /// `esp_ieee802154_dev.c` `ieee802154_mac_init` ACK PTI).
    pub const DEFAULT: Self = Self {
        idle: Priority::Idle,
        rx: Priority::Low,
        tx: Priority::Low,
        timed: Priority::Middle,
        ack: Priority::Middle,
    };
}

/// Scene priorities for a `coex_tx_priority_mask`:
///
/// * bit 0 → ACK `High` (default `Middle`);
/// * bit 1 → timed TX `High` (default `Middle`);
/// * bit 3 → TX, timed TX and ACK `High` (TX default `Low`);
/// * bit 2 → covered only by bit 3, otherwise refused.
///
/// Reception and idle keep the ESP-IDF defaults.
pub const fn priorities(priority_mask: u8) -> Result<ScenePriorities, PtaError> {
    let reserved = priority_mask & !mask::DEFINED;
    if reserved != 0 {
        return Err(PtaError::ReservedBits(reserved));
    }
    let all = priority_mask & mask::ALL != 0;
    if priority_mask & mask::TC_KEY_TRANSPORT != 0 && !all {
        return Err(PtaError::UnsupportedClass(mask::TC_KEY_TRANSPORT));
    }
    let d = ScenePriorities::DEFAULT;
    Ok(ScenePriorities {
        idle: d.idle,
        rx: d.rx,
        tx: raise(all, d.tx),
        timed: raise(all || priority_mask & mask::GP_TIMED_TX != 0, d.timed),
        ack: raise(all || priority_mask & mask::MAC_ACK != 0, d.ack),
    })
}

const fn raise(raised: bool, default: Priority) -> Priority {
    if raised { Priority::High } else { default }
}

#[cfg(target_os = "none")]
pub use self::hw::{Pta, adapter_stats};

#[cfg(target_os = "none")]
mod hw {
    use esp_hal::peripherals::{
        GPIO, GPIO0, GPIO1, GPIO2, GPIO3, IO_MUX, MODEM_LPCON, MODEM_SYSCON,
    };
    use esp_radio::ieee802154::{CoexPriority, CoexPti, set_coex_pti};
    pub(super) use esp_radio::ieee802154::{coex_breaks, reset_coex_breaks};
    use esp_thread_br_h2::THREAD_BR_H2_PTA as WIRING;

    use super::{Priority, PtaError, ScenePriorities, priorities};

    const REQUEST: u32 = WIRING.request_gpio as u32;
    const GRANT: u32 = WIRING.grant_gpio as u32;
    const PRIORITY: u32 = WIRING.priority_gpio as u32;
    const TX_LINE: u32 = WIRING.tx_line_gpio as u32;
    // `Pta::new` takes these pins by type.
    const _: () = assert!(REQUEST == 0 && GRANT == 1 && PRIORITY == 2 && TX_LINE == 3);

    /// ESP32-H2 `soc/gpio_sig_map.h`.
    const EXTERN_PRIORITY_IDX: u32 = 81;
    const EXTERN_ACTIVE_IDX: u32 = 82;
    /// `SIG_GPIO_OUT_IDX`: the pin follows `GPIO_OUT`.
    const SIG_GPIO_OUT_IDX: u32 = 128;
    /// `PIN_FUNC_GPIO` of the H2 IO_MUX.
    const PIN_FUNC_GPIO: u8 = 1;
    /// `EXTERN_COEX_WIRE_4` (`esp_coexist.h`).
    const EXTERN_COEX_WIRE_4: u32 = 3;
    /// `EXTERNAL_COEX_FOLLOWER_ROLE`.
    const FOLLOWER_ROLE: u32 = 2;
    /// `esp_coex_pti_level_t`.
    const EXTERN_COEX_PTI_MID: u32 = 0;
    const EXTERN_COEX_PTI_HIGH: u32 = 1;

    /// `esp_external_coex_advance_t` (passed by value: two registers).
    #[repr(C)]
    struct ExternalCoexAdvance {
        work_mode: u32,
        delay_us: u8,
        is_high_valid: bool,
    }

    unsafe extern "C" {
        // libcoexist.a, external_coex.o (ESP-IDF v5.5.1
        // esp_coex/include/private/esp_coexist_internal.h).
        fn esp_coex_external_set_wire_type(wire_type: u32);
        fn esp_coex_external_params(
            coex_info: ExternalCoexAdvance,
            out_pti1: u32,
            out_pti2: u32,
        ) -> i32;
        fn esp_coex_external_set(level1: u32, level2: u32, level3: u32) -> i32;
        fn esp_coex_external_stop();
        // libcoexist.a, coexist.o: the OS adapter and the pre-init ESP-IDF
        // runs at start-up (`esp_system/startup_funcs.c` `init_coexist`:
        // `esp_coex_adapter_register(&g_coex_adapter_funcs);
        // coex_pre_init();`) whenever external coexistence is configured.
        fn esp_coex_adapter_register(funcs: *mut adapter::CoexAdapterFuncs) -> i32;
        fn coex_pre_init() -> i32;
        // ESP32-H2 ROM (esp-rom-sys `esp32h2.rom.api.ld`: gpio_matrix_out /
        // gpio_matrix_in at 0x400006a8 / 0x400006a4).
        fn esp_rom_gpio_connect_out_signal(gpio: u32, signal: u32, out_inv: bool, oen_inv: bool);
        fn esp_rom_gpio_connect_in_signal(gpio: u32, signal: u32, inv: bool);
    }

    /// The board's PTA wires, owned exclusively. Disabled after [`Self::new`].
    pub struct Pta<'d> {
        _request: GPIO0<'d>,
        _grant: GPIO1<'d>,
        _priority: GPIO2<'d>,
        _tx_line: GPIO3<'d>,
        enabled: bool,
        priority_mask: u8,
    }

    impl<'d> Pta<'d> {
        /// Take the four PTA pins; the arbiter stays disabled, request and
        /// priority are driven low, grant and tx_line are inputs.
        pub fn new(
            request: GPIO0<'d>,
            grant: GPIO1<'d>,
            priority: GPIO2<'d>,
            tx_line: GPIO3<'d>,
        ) -> Self {
            critical_section::with(|_| {
                park_outputs();
                input_pad(GRANT);
                input_pad(TX_LINE);
            });
            Self {
                _request: request,
                _grant: grant,
                _priority: priority,
                _tx_line: tx_line,
                enabled: false,
                priority_mask: 0,
            }
        }

        /// Enable or disable arbitration with `priority_mask`
        /// (`coex_tx_priority_mask`). Validates the mask first; on any
        /// error nothing was changed (except [`PtaError::Arbiter`], after
        /// which PTA is disabled). Disabling while disabled only records the
        /// mask and touches no register (the blob's stop path rewrites
        /// IEEE802154 `CTRL_CFG` bit 26 and the coex delay). Call after the
        /// radio is started; a radio re-initialisation needs another call.
        pub fn set(&mut self, enable: bool, priority_mask: u8) -> Result<(), PtaError> {
            let scenes = priorities(priority_mask)?;
            if !enable && !self.enabled {
                self.priority_mask = priority_mask;
                return Ok(());
            }
            if !radio_clocked() {
                return Err(PtaError::RadioOff);
            }
            if enable {
                set_coex_pti(coex_pti(scenes));
                let result = critical_section::with(|_| unsafe { start() });
                if let Err(error) = result {
                    critical_section::with(|_| stop());
                    set_coex_pti(CoexPti::DEFAULT);
                    self.enabled = false;
                    return Err(error);
                }
            } else {
                critical_section::with(|_| stop());
                set_coex_pti(CoexPti::DEFAULT);
            }
            self.enabled = enable;
            self.priority_mask = priority_mask;
            Ok(())
        }

        /// Arbitration is on (`SYS_GET_STATUS.flags` bit 2).
        pub fn enabled(&self) -> bool {
            self.enabled
        }

        /// The last accepted `coex_tx_priority_mask`.
        pub fn priority_mask(&self) -> u8 {
            self.priority_mask
        }
    }

    fn coex_pti(s: ScenePriorities) -> CoexPti {
        let p = |p: Priority| match p {
            Priority::High => CoexPriority::High,
            Priority::Middle => CoexPriority::Middle,
            Priority::Low => CoexPriority::Low,
            Priority::Idle => CoexPriority::Idle,
        };
        CoexPti {
            idle: p(s.idle),
            rx: p(s.rx),
            tx: p(s.tx),
            timed: p(s.timed),
            ack: p(s.ack),
        }
    }

    /// The coexistence block (`MODEM_LPCON` `CLK_COEX_EN`, ESP-IDF
    /// `PERIPH_COEX_MODULE`) and the 802.15.4 registers the blob also
    /// writes (`MODEM_SYSCON` `CLK_ZB_APB_EN`) are clocked; esp-hal sets
    /// both while the radio is enabled.
    fn radio_clocked() -> bool {
        MODEM_LPCON::regs()
            .clk_conf()
            .read()
            .clk_coex_en()
            .bit_is_set()
            && MODEM_SYSCON::regs()
                .clk_conf()
                .read()
                .clk_zb_apb_en()
                .bit_is_set()
    }

    /// ESP-IDF's start-up `init_coexist`, once: register the OS adapter and
    /// run `coex_pre_init`, which sets up the blob's state (`coex_env`) that
    /// `esp_coex_external_set` needs to actually enable the arbiter. Without
    /// it the arbiter stays off (Thread BR board, 2026-11-21: grant held low
    /// did not stop transmissions, request never rose).
    fn pre_init() -> Result<(), PtaError> {
        use core::sync::atomic::{AtomicI32, Ordering};
        /// 1 not yet run, 0 done, other: the error it returned.
        static STATE: AtomicI32 = AtomicI32::new(1);
        match STATE.load(Ordering::Relaxed) {
            0 => return Ok(()),
            1 => {}
            error => return Err(PtaError::Arbiter(error)),
        }
        let ret = unsafe {
            let ret = esp_coex_adapter_register(core::ptr::addr_of_mut!(adapter::FUNCS));
            if ret != 0 { ret } else { coex_pre_init() }
        };
        STATE.store(ret, Ordering::Relaxed);
        if ret != 0 {
            return Err(PtaError::Arbiter(ret));
        }
        Ok(())
    }

    /// Diagnostics of the OS adapter (lab): semaphores created, timer
    /// calls, allocation bytes, failed allocations.
    pub fn adapter_stats() -> (u32, u32, u32, u32) {
        adapter::stats()
    }

    /// `esp_enable_extern_coex_gpio_pin(EXTERN_COEX_WIRE_4, ...)` in the
    /// follower role (ESP-IDF v5.5.1 `coexist.c` 168, 215-257, 265-268).
    unsafe fn start() -> Result<(), PtaError> {
        pre_init()?;
        unsafe {
            esp_coex_external_set_wire_type(EXTERN_COEX_WIRE_4);
            // tx_line: leader Wi-Fi TX → EXTERN_PRIORITY_I.
            input_pad(TX_LINE);
            esp_rom_gpio_connect_in_signal(TX_LINE, EXTERN_PRIORITY_IDX, false);
            sync_bypass(TX_LINE);
            // priority: EXTERN_PRIORITY_O → leader.
            output_pad(PRIORITY);
            GPIO::regs().enable_w1tc().write(|w| w.bits(1 << PRIORITY));
            esp_rom_gpio_connect_out_signal(PRIORITY, EXTERN_PRIORITY_IDX, false, false);
            // grant: leader → EXTERN_ACTIVE_I.
            input_pad(GRANT);
            esp_rom_gpio_connect_in_signal(GRANT, EXTERN_ACTIVE_IDX, false);
            sync_bypass(GRANT);
            // request: EXTERN_ACTIVE_O → leader.
            output_pad(REQUEST);
            GPIO::regs().enable_w1tc().write(|w| w.bits(1 << REQUEST));
            esp_rom_gpio_connect_out_signal(REQUEST, EXTERN_ACTIVE_IDX, false, false);

            // `g_external_coex_params` default { LEADER, 0, true } with the
            // work mode set to follower (`esp_external_coex_set_work_mode`).
            let params = ExternalCoexAdvance {
                work_mode: FOLLOWER_ROLE,
                delay_us: 0,
                is_high_valid: true,
            };
            let ret = esp_coex_external_params(params, 0, 0);
            if ret != 0 {
                return Err(PtaError::Arbiter(ret));
            }
            let ret = esp_coex_external_set(
                EXTERN_COEX_PTI_MID,
                EXTERN_COEX_PTI_MID,
                EXTERN_COEX_PTI_HIGH,
            );
            if ret != 0 {
                return Err(PtaError::Arbiter(ret));
            }
        }
        Ok(())
    }

    /// `esp_disable_extern_coex_gpio_pin`, then request/priority low.
    fn stop() {
        unsafe { esp_coex_external_stop() };
        park_outputs();
    }

    /// Request and priority as plain GPIO outputs driven low (deasserted;
    /// ESP-IDF leaves them on the arbiter signals).
    fn park_outputs() {
        let gpio = GPIO::regs();
        for pin in [REQUEST, PRIORITY] {
            gpio.out_w1tc().write(|w| unsafe { w.bits(1 << pin) });
            output_pad(pin);
            unsafe { esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false) };
            gpio.enable_w1ts().write(|w| unsafe { w.bits(1 << pin) });
        }
    }

    /// `gpio_func_sel(GPIO)` + `gpio_set_direction(INPUT)`.
    fn input_pad(pin: u32) {
        IO_MUX::regs().gpio(pin as usize).modify(|_, w| unsafe {
            w.mcu_sel().bits(PIN_FUNC_GPIO);
            w.fun_ie().set_bit()
        });
        GPIO::regs()
            .enable_w1tc()
            .write(|w| unsafe { w.bits(1 << pin) });
        GPIO::regs()
            .pin(pin as usize)
            .modify(|_, w| w.pad_driver().clear_bit());
    }

    /// `gpio_func_sel(GPIO)` + `gpio_set_direction(OUTPUT)`.
    fn output_pad(pin: u32) {
        IO_MUX::regs().gpio(pin as usize).modify(|_, w| unsafe {
            w.mcu_sel().bits(PIN_FUNC_GPIO);
            w.fun_ie().clear_bit()
        });
        GPIO::regs()
            .enable_w1ts()
            .write(|w| unsafe { w.bits(1 << pin) });
        GPIO::regs()
            .pin(pin as usize)
            .modify(|_, w| w.pad_driver().clear_bit());
    }

    /// `GPIO_PINn_SYNC1/2_BYPASS = 2` on the arbiter inputs.
    fn sync_bypass(pin: u32) {
        GPIO::regs().pin(pin as usize).modify(|_, w| unsafe {
            w.sync1_bypass().bits(2);
            w.sync2_bypass().bits(2)
        });
    }

    /// The OS adapter `libcoexist.a` calls (`coex_adapter_funcs_t`, ESP-IDF
    /// v5.5.1 `esp_coex/esp32h2/esp_coex_adapter.c`, adapter version 2).
    /// Bare metal, one core, no scheduler: semaphores are counters taken
    /// and given under a critical section (nothing ever waits: with one
    /// context a held semaphore could not be released, so a blocking take
    /// fails instead of hanging), memory comes from a small arena that is
    /// never freed (the blob allocates its state once in `coex_pre_init`),
    /// and the software timers are not provided (the external-coexistence
    /// path does not schedule; calls are counted for the lab).
    pub(super) mod adapter {
        use core::ffi::c_void;
        use core::sync::atomic::{AtomicU32, Ordering};

        #[repr(C)]
        pub struct CoexAdapterFuncs {
            version: i32,
            task_yield_from_isr: unsafe extern "C" fn(),
            semphr_create: unsafe extern "C" fn(u32, u32) -> *mut c_void,
            semphr_delete: unsafe extern "C" fn(*mut c_void),
            semphr_take_from_isr: unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32,
            semphr_give_from_isr: unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32,
            semphr_take: unsafe extern "C" fn(*mut c_void, u32) -> i32,
            semphr_give: unsafe extern "C" fn(*mut c_void) -> i32,
            is_in_isr: unsafe extern "C" fn() -> i32,
            malloc_internal: unsafe extern "C" fn(usize) -> *mut c_void,
            free: unsafe extern "C" fn(*mut c_void),
            esp_timer_get_time: unsafe extern "C" fn() -> i64,
            env_is_chip: unsafe extern "C" fn() -> bool,
            timer_disarm: unsafe extern "C" fn(*mut c_void),
            timer_done: unsafe extern "C" fn(*mut c_void),
            timer_setfn: unsafe extern "C" fn(*mut c_void, *mut c_void, *mut c_void),
            timer_arm_us: unsafe extern "C" fn(*mut c_void, u32, bool),
            debug_matrix_init: unsafe extern "C" fn(i32, i32, bool) -> i32,
            xtal_freq_get: unsafe extern "C" fn() -> i32,
            magic: i32,
        }

        /// `COEX_ADAPTER_VERSION`, `COEX_ADAPTER_MAGIC`.
        const VERSION: i32 = 2;
        const MAGIC: i32 = 0xDEAD_BEAFu32 as i32;
        /// `ESP_ERR_NOT_SUPPORTED`.
        const NOT_SUPPORTED: i32 = 0x106;

        pub static mut FUNCS: CoexAdapterFuncs = CoexAdapterFuncs {
            version: VERSION,
            task_yield_from_isr,
            semphr_create,
            semphr_delete,
            semphr_take_from_isr,
            semphr_give_from_isr,
            semphr_take,
            semphr_give,
            is_in_isr,
            malloc_internal,
            free,
            esp_timer_get_time,
            env_is_chip,
            timer_disarm,
            timer_done,
            timer_setfn,
            timer_arm_us,
            debug_matrix_init,
            xtal_freq_get,
            magic: MAGIC,
        };

        struct Semaphore {
            count: AtomicU32,
            max: AtomicU32,
        }
        const SEMAPHORES: usize = 8;
        static POOL: [Semaphore; SEMAPHORES] = [const {
            Semaphore {
                count: AtomicU32::new(0),
                max: AtomicU32::new(0),
            }
        }; SEMAPHORES];
        static CREATED: AtomicU32 = AtomicU32::new(0);
        static TIMER_CALLS: AtomicU32 = AtomicU32::new(0);
        static ALLOC_FAILED: AtomicU32 = AtomicU32::new(0);

        const ARENA_BYTES: usize = 2048;
        #[repr(align(8))]
        struct Arena([u8; ARENA_BYTES]);
        static mut ARENA: Arena = Arena([0; ARENA_BYTES]);
        static USED: AtomicU32 = AtomicU32::new(0);

        pub fn stats() -> (u32, u32, u32, u32) {
            (
                CREATED.load(Ordering::Relaxed),
                TIMER_CALLS.load(Ordering::Relaxed),
                USED.load(Ordering::Relaxed),
                ALLOC_FAILED.load(Ordering::Relaxed),
            )
        }

        unsafe extern "C" fn task_yield_from_isr() {}

        unsafe extern "C" fn semphr_create(max: u32, init: u32) -> *mut c_void {
            let index = CREATED.fetch_add(1, Ordering::Relaxed) as usize;
            let Some(slot) = POOL.get(index) else {
                return core::ptr::null_mut();
            };
            slot.max.store(max, Ordering::Relaxed);
            slot.count.store(init.min(max), Ordering::Relaxed);
            core::ptr::from_ref(slot).cast_mut().cast()
        }

        unsafe extern "C" fn semphr_delete(_semphr: *mut c_void) {}

        fn take(semphr: *mut c_void) -> i32 {
            let Some(slot) = (unsafe { semphr.cast::<Semaphore>().as_ref() }) else {
                return 0;
            };
            critical_section::with(|_| {
                let count = slot.count.load(Ordering::Relaxed);
                if count == 0 {
                    0
                } else {
                    slot.count.store(count - 1, Ordering::Relaxed);
                    1
                }
            })
        }

        fn give(semphr: *mut c_void) -> i32 {
            let Some(slot) = (unsafe { semphr.cast::<Semaphore>().as_ref() }) else {
                return 0;
            };
            critical_section::with(|_| {
                let count = slot.count.load(Ordering::Relaxed);
                if count >= slot.max.load(Ordering::Relaxed) {
                    0
                } else {
                    slot.count.store(count + 1, Ordering::Relaxed);
                    1
                }
            })
        }

        unsafe extern "C" fn semphr_take_from_isr(semphr: *mut c_void, _hptw: *mut c_void) -> i32 {
            take(semphr)
        }

        unsafe extern "C" fn semphr_give_from_isr(semphr: *mut c_void, _hptw: *mut c_void) -> i32 {
            give(semphr)
        }

        unsafe extern "C" fn semphr_take(semphr: *mut c_void, _block_time_tick: u32) -> i32 {
            take(semphr)
        }

        unsafe extern "C" fn semphr_give(semphr: *mut c_void) -> i32 {
            give(semphr)
        }

        unsafe extern "C" fn is_in_isr() -> i32 {
            // Only selects the `_from_isr` variants, which are the same here.
            0
        }

        unsafe extern "C" fn malloc_internal(size: usize) -> *mut c_void {
            let size = (size + 7) & !7;
            let offset = USED.fetch_add(size as u32, Ordering::Relaxed) as usize;
            if offset + size > ARENA_BYTES {
                USED.fetch_sub(size as u32, Ordering::Relaxed);
                ALLOC_FAILED.fetch_add(1, Ordering::Relaxed);
                return core::ptr::null_mut();
            }
            // SAFETY: each range of the arena is handed out once.
            unsafe {
                core::ptr::addr_of_mut!(ARENA.0)
                    .cast::<u8>()
                    .add(offset)
                    .cast()
            }
        }

        unsafe extern "C" fn free(_p: *mut c_void) {}

        unsafe extern "C" fn esp_timer_get_time() -> i64 {
            esp_hal::time::Instant::now()
                .duration_since_epoch()
                .as_micros() as i64
        }

        unsafe extern "C" fn env_is_chip() -> bool {
            true
        }

        unsafe extern "C" fn timer_disarm(_timer: *mut c_void) {
            TIMER_CALLS.fetch_add(1, Ordering::Relaxed);
        }

        unsafe extern "C" fn timer_done(_timer: *mut c_void) {
            TIMER_CALLS.fetch_add(1, Ordering::Relaxed);
        }

        unsafe extern "C" fn timer_setfn(_timer: *mut c_void, _f: *mut c_void, _arg: *mut c_void) {
            TIMER_CALLS.fetch_add(1, Ordering::Relaxed);
        }

        unsafe extern "C" fn timer_arm_us(_timer: *mut c_void, _us: u32, _repeat: bool) {
            TIMER_CALLS.fetch_add(1, Ordering::Relaxed);
        }

        unsafe extern "C" fn debug_matrix_init(_event: i32, _signal: i32, _rev: bool) -> i32 {
            NOT_SUPPORTED
        }

        unsafe extern "C" fn xtal_freq_get() -> i32 {
            // ESP32-H2: 32 MHz crystal.
            32
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_mask_is_the_esp_idf_default() {
        assert_eq!(priorities(0), Ok(ScenePriorities::DEFAULT));
    }

    #[test]
    fn single_classes_raise_only_their_scene() {
        let ack = priorities(mask::MAC_ACK).unwrap();
        assert_eq!(ack.ack, Priority::High);
        assert_eq!((ack.tx, ack.timed), (Priority::Low, Priority::Middle));
        let gp = priorities(mask::GP_TIMED_TX).unwrap();
        assert_eq!(gp.timed, Priority::High);
        assert_eq!((gp.tx, gp.ack), (Priority::Low, Priority::Middle));
    }

    #[test]
    fn all_raises_every_transmission_but_not_reception() {
        for m in [mask::ALL, mask::ALL | mask::TC_KEY_TRANSPORT, mask::DEFINED] {
            let p = priorities(m).unwrap();
            assert_eq!(
                (p.tx, p.timed, p.ack),
                (Priority::High, Priority::High, Priority::High)
            );
            assert_eq!((p.rx, p.idle), (Priority::Low, Priority::Idle));
        }
    }

    #[test]
    fn unsupported_and_reserved_bits_are_refused() {
        assert_eq!(
            priorities(mask::TC_KEY_TRANSPORT),
            Err(PtaError::UnsupportedClass(mask::TC_KEY_TRANSPORT))
        );
        assert_eq!(
            priorities(mask::TC_KEY_TRANSPORT | mask::MAC_ACK),
            Err(PtaError::UnsupportedClass(mask::TC_KEY_TRANSPORT))
        );
        assert_eq!(priorities(0x10), Err(PtaError::ReservedBits(0x10)));
        // Reserved bits are reported even when a defined bit is also wrong.
        assert_eq!(priorities(0x84), Err(PtaError::ReservedBits(0x80)));
    }

    #[test]
    fn every_mask_is_either_accepted_or_refused_explicitly() {
        for m in 0..=u8::MAX {
            let accepted = priorities(m).is_ok();
            let expected =
                m & !mask::DEFINED == 0 && (m & mask::TC_KEY_TRANSPORT == 0 || m & mask::ALL != 0);
            assert_eq!(accepted, expected, "mask {m:#04x}");
        }
    }
}

#[cfg(target_os = "none")]
impl crate::platform::CoexControl for Pta<'static> {
    fn set(
        &mut self,
        enable: bool,
        priority_mask: u8,
    ) -> Result<(), ncp_coordinator_app::platform::PlatformError> {
        use ncp_coordinator_app::platform::PlatformError;
        Pta::set(self, enable, priority_mask).map_err(|error| {
            log::warn!("[PTA] refused: {:?}", error);
            match error {
                PtaError::ReservedBits(_) => PlatformError::InvalidParameter,
                PtaError::UnsupportedClass(_) => PlatformError::Unsupported,
                PtaError::RadioOff => PlatformError::InvalidState,
                PtaError::Arbiter(_) => PlatformError::Hardware,
            }
        })
    }

    fn counters(&self) -> crate::platform::CoexCounters {
        let breaks = hw::coex_breaks();
        crate::platform::CoexCounters {
            tx_denied: breaks.tx,
            ack_rx_denied: breaks.ack_rx,
            ack_tx_denied: breaks.ack_tx,
        }
    }

    fn reset_counters(&mut self) {
        hw::reset_coex_breaks();
    }
}
