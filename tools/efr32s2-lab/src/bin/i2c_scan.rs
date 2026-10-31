//! Gate: find the WSTK Si7021 wiring for BRD4183A and read it with I2C0.
//!
//! GSDK does not declare a sensor I2C bus for BRD4183A (BRD4182A uses
//! SCL PB2 / SDA PB3 / enable PC7, but PB2 is VCOM CTS on BRD4183A). This
//! binary releases every candidate pin to the internal pull-up (never drives
//! a line high), so a sensor-enable load switch sees a weak high, then
//! bit-bangs an address-0x40 probe on every ordered (SCL, SDA) pair. A hit is
//! confirmed with the hardware I2C0 master: Si7021 electronic ID and one
//! no-hold temperature/humidity measurement.

#![no_std]
#![no_main]

#[path = "../shared/mod.rs"]
mod shared;

use efr32s2_hal::{
    delay::{spin_cycles, us_to_cycles},
    gpio::{OpenDrain, Pin, Port},
};
use embedded_hal::i2c::I2c as _;

const SI7021: u8 = 0x40;

/// GPIOs bonded out on the BRD4183A part (EFR32MG22C224F512**IM32**, QFN32:
/// PA0-PA6, PB0-PB4, PC0-PC5, PD0-PD3) except SWD (PA1/PA2), VCOM TX/enable
/// (PA5/PA4), LED0 (PB1) and LFXO (PD0/PD1).
///
/// Hardware result: no pair ACKs 0x40. Every QFN32 pin is already assigned to
/// VCOM, buttons, LEDs, PTI, SPI flash or the display, so the WSTK Si7021 is
/// not reachable from this radio board.
const CANDIDATES: [(Port, u8); 13] = [
    (Port::A, 0),
    (Port::A, 3),
    (Port::A, 6),
    (Port::B, 0),
    (Port::B, 2),
    (Port::B, 3),
    (Port::B, 4),
    (Port::C, 0),
    (Port::C, 1),
    (Port::C, 2),
    (Port::C, 3),
    (Port::D, 2),
    (Port::D, 3),
];

fn half() {
    spin_cycles(us_to_cycles(10));
}

/// Bit-banged START + address(write) + ACK check + STOP.
fn probe(scl: &mut Pin<OpenDrain>, sda: &mut Pin<OpenDrain>, addr: u8) -> bool {
    scl.set_released(true);
    sda.set_released(true);
    half();
    if !scl.read_input() || !sda.read_input() {
        return false;
    }
    sda.set_released(false);
    half();
    scl.set_released(false);
    half();
    let byte = addr << 1;
    for bit in (0..8).rev() {
        let one = byte & (1 << bit) != 0;
        sda.set_released(one);
        half();
        // A released line must rise within one half-period: rejects
        // RC-filtered button/strap nets that only look like an ACK.
        if one && !sda.read_input() {
            sda.set_released(true);
            scl.set_released(true);
            return false;
        }
        scl.set_released(true);
        half();
        scl.set_released(false);
        half();
    }
    sda.set_released(true);
    half();
    scl.set_released(true);
    half();
    let ack = !sda.read_input();
    scl.set_released(false);
    half();
    // After the ACK clock the slave releases SDA: it must read high again.
    let released = sda.read_input();
    sda.set_released(false);
    half();
    scl.set_released(true);
    half();
    sda.set_released(true);
    half();
    ack && released
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut lab = shared::init("i2c-scan");
    let mut pins: [Option<Pin<OpenDrain>>; 17] = Default::default();
    for (slot, &(port, n)) in pins.iter_mut().zip(CANDIDATES.iter()) {
        // SAFETY: candidates are distinct and not claimed by `shared::init`.
        *slot = unsafe { lab.gpio.claim_pin(port, n) }
            .ok()
            .map(|p| p.into_open_drain());
    }
    shared::delay_ms(100); // let a pulled-up sensor enable settle
    let mut found = None;
    for i in 0..CANDIDATES.len() {
        for j in 0..CANDIDATES.len() {
            if i == j {
                continue;
            }
            let (a, b) = if i < j {
                let (l, r) = pins.split_at_mut(j);
                (&mut l[i], &mut r[0])
            } else {
                let (l, r) = pins.split_at_mut(i);
                (&mut r[0], &mut l[j])
            };
            let (Some(scl), Some(sda)) = (a.as_mut(), b.as_mut()) else {
                continue;
            };
            if probe(scl, sda, SI7021) {
                log!("ACK 0x40: SCL={:?} SDA={:?}", CANDIDATES[i], CANDIDATES[j]);
                found.get_or_insert((i, j));
            }
        }
    }
    let Some((i, j)) = found else {
        log!("no device ACKed 0x40 on any candidate pair");
        loop {
            lab.led.toggle();
            shared::delay_ms(100);
        }
    };
    let scl = pins[i].take().unwrap();
    let sda = pins[j].take().unwrap();
    let mut bus = lab.i2c0.into_master(scl, sda, 100_000);
    let mut id = [0u8; 8];
    match bus.write_read(SI7021, &[0xFA, 0x0F], &mut id) {
        Ok(()) => log!("Si70xx ID1 {:02X?}", id),
        Err(e) => log!("ID read failed: {:?}", e),
    }
    loop {
        let rh = measure(&mut bus, 0xF5).map(|raw| ((125 * raw as i32 * 100) >> 16) - 600);
        let t = measure(&mut bus, 0xF3).map(|raw| ((17572 * raw as i32) >> 16) - 4685);
        log!("T={:?} c°C RH={:?} c%", t, rh);
        lab.led.toggle();
        shared::delay_ms(2000);
    }
}

/// No-hold measurement: start, poll for the result (NACK while busy).
fn measure(bus: &mut efr32s2_hal::i2c::I2c, cmd: u8) -> Option<u16> {
    bus.write(SI7021, &[cmd]).ok()?;
    for _ in 0..50 {
        shared::delay_ms(5);
        let mut b = [0u8; 2];
        if bus.read(SI7021, &mut b).is_ok() {
            return Some(u16::from_be_bytes(b) & !3);
        }
    }
    None
}
