//! Low-level ESP32 802.15.4 radio driver wrapper.
//!
//! Provides TX with completion signaling and polling-based RX on top of
//! `esp-radio::ieee802154`.

use core::sync::atomic::{AtomicBool, Ordering};
use esp_radio::ieee802154::{Config, Error, Ieee802154};

/// TX completion flag — set by ISR callback, cleared before TX.
static TX_COMPLETE: AtomicBool = AtomicBool::new(false);

/// TX-done callback — called from radio ISR when TX finishes.
fn on_tx_done() {
    TX_COMPLETE.store(true, Ordering::Release);
}

/// Received frame data (copied out of radio buffer).
pub struct RxFrame {
    pub data: [u8; 127],
    pub len: usize,
    pub lqi: u8,
}

/// Wrapper around the ESP32 ieee802154 radio peripheral.
pub struct Ieee802154Driver<'a> {
    driver: Ieee802154<'a>,
    config: Config,
}

impl<'a> Ieee802154Driver<'a> {
    pub fn new(mut ieee802154: Ieee802154<'a>, config: Config) -> Self {
        ieee802154.set_config(config);
        // Register TX-done callback for completion signaling
        ieee802154.set_tx_done_callback_fn(on_tx_done);
        Self {
            driver: ieee802154,
            config,
        }
    }

    /// Update radio configuration (channel, PAN ID, short address, etc.)
    pub fn update_config(&mut self, update_fn: impl FnOnce(&mut Config)) {
        update_fn(&mut self.config);
        self.driver.set_config(self.config);
    }

    /// Transmit a raw 802.15.4 frame and wait for TX completion.
    ///
    /// The ESP32 802.15.4 hardware interprets PHR as total OTA length
    /// INCLUDING FCS. We add +2 padding so the full frame is transmitted.
    /// Uses TX-done callback for precise completion detection (~500µs typical).
    pub fn transmit(&mut self, frame: &[u8]) -> Result<(), Error> {
        let mut padded = [0u8; 129];
        padded[..frame.len()].copy_from_slice(frame);

        // Clear TX-done flag before starting TX
        TX_COMPLETE.store(false, Ordering::Release);

        self.driver.transmit_raw(&padded[..frame.len() + 2])?;

        // Wait for TX completion via ISR callback (max 10ms safety timeout)
        let start = esp_hal::time::Instant::now();
        while !TX_COMPLETE.load(Ordering::Acquire) {
            if start.elapsed() > esp_hal::time::Duration::from_millis(10) {
                break; // Safety timeout — TX should never take this long
            }
            core::hint::spin_loop();
        }

        Ok(())
    }

    /// Transmit and wait for ACK from recipient.
    /// Returns true if ACK received, false if timeout.
    pub fn transmit_with_ack(&mut self, frame: &[u8], seq: u8) -> bool {
        if self.transmit(frame).is_err() {
            return false;
        }

        // Immediately switch to RX to catch the ACK
        self.driver.start_receive();

        // ACK window: aTurnaroundTime (192µs) + ACK duration (~352µs) + margin
        let deadline = esp_hal::time::Instant::now()
            + esp_hal::time::Duration::from_millis(2);

        while esp_hal::time::Instant::now() < deadline {
            if let Some(raw) = self.driver.raw_received() {
                let phr = raw.data[0] as usize;
                if phr >= 5 {
                    let fc = u16::from_le_bytes([raw.data[1], raw.data[2]]);
                    let frame_type = fc & 0x07;
                    let ack_seq = raw.data[3];
                    // ACK frame: type=2, matching sequence number
                    if frame_type == 0x02 && ack_seq == seq {
                        return true;
                    }
                }
                // Not our ACK — restart RX
                self.driver.start_receive();
            }
        }

        false
    }

    /// Put radio into receive mode.
    pub fn start_receive(&mut self) {
        self.driver.start_receive();
    }

    /// Poll for a received frame. Returns None if nothing available yet.
    pub fn poll_receive(&mut self) -> Option<Result<RxFrame, Error>> {
        if let Some(raw) = self.driver.raw_received() {
            let mut rx = RxFrame {
                data: [0u8; 127],
                len: 0,
                lqi: 128,
            };
            let phr = raw.data[0] as usize;
            let mac_len = if phr >= 2 { phr - 2 } else { phr };
            let len = mac_len.min(125);
            if len > 0 {
                rx.data[..len].copy_from_slice(&raw.data[1..][..len]);
            }
            rx.len = len;
            return Some(Ok(rx));
        }
        None
    }
}
