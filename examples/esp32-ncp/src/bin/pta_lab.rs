//! # PTA lab (Thread BR / Zigbee GW board, ESP32-H2 side)
//!
//! Lab image that proves the PTA follower primitive (`product::pta::Pta`)
//! without the NCP: it starts the radio alone, cycles through PTA phases and
//! for each sends broadcast data frames on channel 26 (lab PAN `0x0B7A`, no
//! ACK request) while sampling the four PTA pads. Never a product image; it
//! owns no persistence and speaks no ZRHP.
//!
//! ```bash
//! cargo build --release --no-default-features --features esp32h2,transport-uart,pta-lab --bin pta_lab
//! espflash flash --monitor target/riscv32imac-unknown-none-elf/release/pta_lab   # H2 USB-C port
//! ```
//!
//! Without a USB host on the H2 (esp-println `auto`) the output goes to
//! UART0 at 115 200, which the S3 bridge forwards to CDC 0; the S3 console
//! (`pta`, `pta grant high|low|in`, `pta sample <ms>`) drives grant and
//! samples request/priority from the leader's side.
//!
//! Phases (repeated):
//!
//! | Phase | PTA | mask | GPIO1 (grant) pad |
//! |---|---|---|---|
//! | A | off | 0x00 | no pull |
//! | B | on | 0x00 | pull-up (granted, if the S3 does not drive GPIO13) |
//! | C | on | 0x00 | pull-down (denied, if the S3 does not drive GPIO13) |
//! | D | on | 0x08 | pull-up (all TX `High`) |
//! | E | on | 0x00 | no pull (the S3 leader drives grant) |
//! | R | refusals: mask 0x04 and 0x10 must be refused, state unchanged |
//!
//! Per phase it prints the coexistence-block registers the blob writes
//! (`0x600AD49C/4A0/4A4/4D0/4D8`), IEEE802154 `CTRL_CFG`, `COEX_PTI`,
//! `TX_BREAK_COEX_CNT`, the GPIO-matrix routing of the four pins, the
//! frames completed and timed out (grant denied ends in `TxCoexBreak`, which
//! the driver does not report: a 10 ms timeout), and how many pad samples
//! saw request/grant/priority/tx_line high during the transmissions.
//!
//! Lab-only pad changes: input buffers are enabled on GPIO0/GPIO2 so the
//! outputs can be read back, and phases B-D set pulls on GPIO1.

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

use core::sync::atomic::{AtomicBool, Ordering};

use esp_hal::delay::Delay;
use esp_hal::peripherals::{GPIO, IEEE802154, IO_MUX};
use esp_hal::time::{Duration, Instant};
use esp_println::println;
use esp_radio::ieee802154::{Config, Ieee802154};
use esp32_zigbee_ncp_product::pta::{Pta, PtaError, mask};

const CHANNEL: u8 = 26;
const PAN_ID: u16 = 0x0B7A;
const SHORT_ADDR: u16 = 0x0B01;
const FRAMES_PER_PHASE: u32 = 40;
const TX_TIMEOUT: Duration = Duration::from_millis(10);

const REQUEST: u32 = 0;
const GRANT: u32 = 1;
const PRIORITY: u32 = 2;
const TX_LINE: u32 = 3;

static TX_DONE: AtomicBool = AtomicBool::new(false);

fn on_tx_done() {
    TX_DONE.store(true, Ordering::Release);
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("PANIC {info}");
    loop {
        core::hint::spin_loop();
    }
}

#[derive(Clone, Copy)]
enum Pull {
    None,
    Up,
    Down,
}

#[derive(Default)]
struct Stats {
    sent: u32,
    denied: u32,
    timed_out: u32,
    refused: u32,
    samples: u32,
    request_high: u32,
    grant_high: u32,
    priority_high: u32,
    tx_line_high: u32,
}

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    // esp-radio allocates its receive queue.
    esp_alloc::heap_allocator!(size: 16384);
    let delay = Delay::new();
    delay.delay_millis(1500);

    let mut radio = Ieee802154::new(peripherals.IEEE802154);
    radio.set_config(Config {
        channel: CHANNEL,
        pan_id: Some(PAN_ID),
        short_addr: Some(SHORT_ADDR),
        rx_when_idle: true,
        auto_ack_rx: true,
        auto_ack_tx: true,
        ..Config::default()
    });
    radio.set_tx_done_callback_fn(on_tx_done);
    radio.start_receive();

    let mut pta = Pta::new(
        peripherals.GPIO0,
        peripherals.GPIO1,
        peripherals.GPIO2,
        peripherals.GPIO3,
    );
    println!("pta_lab: ESP32-H2 PTA follower, 4-wire (req 0, grant 1, prio 2, tx_line 3)");
    println!("pta_lab: after Pta::new (disabled, outputs parked low)");
    dump(&pta);

    let mut seq = 0u8;
    loop {
        let mut run = |name: &str, enable: bool, priority_mask: u8, pull: Pull| {
            phase(
                name,
                &mut pta,
                &mut radio,
                &mut seq,
                enable,
                priority_mask,
                pull,
            )
        };
        run("A off", false, 0, Pull::None);
        run("B on, grant pull-up", true, 0, Pull::Up);
        run("C on, grant pull-down", true, 0, Pull::Down);
        run(
            "D on mask 0x08 (all High), grant pull-up",
            true,
            mask::ALL,
            Pull::Up,
        );
        run("E on, S3 drives grant", true, 0, Pull::None);
        refusals(&mut pta);
        let off = pta.set(false, 0);
        set_pull(GRANT, Pull::None);
        println!("pta_lab: cycle done, set(false, 0) -> {off:?}; next cycle in 5 s");
        delay.delay_millis(5000);
    }
}

fn phase(
    name: &str,
    pta: &mut Pta<'_>,
    radio: &mut Ieee802154<'_>,
    seq: &mut u8,
    enable: bool,
    priority_mask: u8,
    grant_pull: Pull,
) {
    set_pull(GRANT, grant_pull);
    let result = pta.set(enable, priority_mask);
    // Lab only: read the outputs back through their input buffers.
    input_buffer(REQUEST);
    input_buffer(PRIORITY);
    println!(
        "\n== phase {name}: set({enable}, {priority_mask:#04x}) -> {result:?}, enabled={}",
        pta.enabled()
    );
    println!("idle pads: {}", PadLevels(GPIO::regs().in_().read().bits()));
    dump(pta);

    let breaks_before = IEEE802154::regs().tx_break_coex_cnt().read().bits();
    let mut stats = Stats::default();
    for _ in 0..FRAMES_PER_PHASE {
        transmit_one(radio, seq, &mut stats);
        while radio.raw_received().is_some() {}
        Delay::new().delay_millis(5);
    }
    let breaks = IEEE802154::regs()
        .tx_break_coex_cnt()
        .read()
        .bits()
        .wrapping_sub(breaks_before);
    let coex = esp_radio::ieee802154::coex_breaks();
    println!(
        "tx: sent {} denied {} timed_out {} refused {} | TX_BREAK_COEX_CNT +{} | coex_breaks tx {} ack_rx {} ack_tx {}",
        stats.sent,
        stats.denied,
        stats.timed_out,
        stats.refused,
        breaks,
        coex.tx,
        coex.ack_rx,
        coex.ack_tx
    );
    println!(
        "pad samples during TX: {} | request {} grant {} priority {} tx_line {}",
        stats.samples,
        stats.request_high,
        stats.grant_high,
        stats.priority_high,
        stats.tx_line_high
    );
}

fn refusals(pta: &mut Pta<'_>) {
    let before = (pta.enabled(), pta.priority_mask());
    let tc = pta.set(true, mask::TC_KEY_TRANSPORT);
    let reserved = pta.set(true, 0x10);
    let after = (pta.enabled(), pta.priority_mask());
    let ok = tc == Err(PtaError::UnsupportedClass(mask::TC_KEY_TRANSPORT))
        && reserved == Err(PtaError::ReservedBits(0x10))
        && before == after;
    println!(
        "\n== phase R: set(true, 0x04) -> {tc:?}; set(true, 0x10) -> {reserved:?}; \
         state {before:?} -> {after:?}: {}",
        if ok { "PASS" } else { "FAIL" }
    );
}

fn transmit_one(radio: &mut Ieee802154<'_>, seq: &mut u8, stats: &mut Stats) {
    *seq = seq.wrapping_add(1);
    let [pan_lo, pan_hi] = PAN_ID.to_le_bytes();
    let [src_lo, src_hi] = SHORT_ADDR.to_le_bytes();
    // Data, PAN ID compression, short dst/src, no ACK request; 2 FCS octets.
    let frame = [
        0x41, 0x88, *seq, pan_lo, pan_hi, 0xFF, 0xFF, src_lo, src_hi, b'P', b'T', b'A', 0, 0,
    ];
    TX_DONE.store(false, Ordering::Release);
    if radio.transmit_raw(&frame).is_err() {
        stats.refused += 1;
        return;
    }
    let deadline = Instant::now() + TX_TIMEOUT;
    loop {
        let pads = GPIO::regs().in_().read().bits();
        stats.samples += 1;
        stats.request_high += pads >> REQUEST & 1;
        stats.grant_high += pads >> GRANT & 1;
        stats.priority_high += pads >> PRIORITY & 1;
        stats.tx_line_high += pads >> TX_LINE & 1;
        if TX_DONE.load(Ordering::Acquire) {
            // esp-radio `ieee802154-extcoex`: a TxCoexBreak completes too.
            if radio.take_tx_coex_denied() {
                stats.denied += 1;
            } else {
                stats.sent += 1;
            }
            return;
        }
        if Instant::now() > deadline {
            radio.cancel_transmit();
            stats.timed_out += 1;
            return;
        }
    }
}

fn dump(pta: &Pta<'_>) {
    // Coexistence block of the modem (written by libcoexist.a's
    // hal_*_extern_* functions).
    let coex =
        |offset: usize| unsafe { core::ptr::read_volatile((0x600A_D000 + offset) as *const u32) };
    println!(
        "enabled={} mask={:#04x} adapter(sem, timer calls, arena, alloc fails)={:?} | coex 49C={:08x} 4A0={:08x} 4A4={:08x} 4D0={:08x} 4D8={:08x}",
        pta.enabled(),
        pta.priority_mask(),
        esp32_zigbee_ncp_product::pta::adapter_stats(),
        coex(0x49C),
        coex(0x4A0),
        coex(0x4A4),
        coex(0x4D0),
        coex(0x4D8)
    );
    let ieee = IEEE802154::regs();
    let gpio = GPIO::regs();
    println!(
        "ieee CTRL_CFG={:08x} COEX_PTI={:08x} | out_sel req={:03x} prio={:03x} | \
         in_sel sig81={:02x} sig82={:02x} | GPIO_ENABLE={:08x}",
        ieee.ctrl_cfg().read().bits(),
        ieee.coex_pti().read().bits(),
        gpio.func_out_sel_cfg(REQUEST as usize).read().bits(),
        gpio.func_out_sel_cfg(PRIORITY as usize).read().bits(),
        gpio.func_in_sel_cfg(81).read().bits(),
        gpio.func_in_sel_cfg(82).read().bits(),
        gpio.enable().read().bits()
    );
}

struct PadLevels(u32);

impl core::fmt::Display for PadLevels {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let level = |pin: u32| self.0 >> pin & 1;
        write!(
            f,
            "request={} grant={} priority={} tx_line={}",
            level(REQUEST),
            level(GRANT),
            level(PRIORITY),
            level(TX_LINE)
        )
    }
}

fn input_buffer(pin: u32) {
    IO_MUX::regs()
        .gpio(pin as usize)
        .modify(|_, w| w.fun_ie().set_bit());
}

fn set_pull(pin: u32, pull: Pull) {
    IO_MUX::regs().gpio(pin as usize).modify(|_, w| {
        w.fun_wpu().bit(matches!(pull, Pull::Up));
        w.fun_wpd().bit(matches!(pull, Pull::Down))
    });
}
