//! Polling BL702 IEEE 802.15.4 radio access without vendor archives.

use core::hint::spin_loop;

use embassy_time::{Duration, Instant, Timer};

use super::{registers::*, rf};
use crate::{
    MAX_PHY_FRAME_LEN, MacError, PhyAddressFilter, PhyCapabilities, PhyError, PhyRxFrame,
    PlatformServices, RadioPhy, SoftMacCore,
};

const TX_IDLE_TIMEOUT_TICKS: u32 = 0x7a12;
const TX_DONE_TIMEOUT: Duration = Duration::from_millis(10);
const CCA_READY_SPINS: usize = 100_000;
const MIN_FRAME_LEN: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TxResult {
    Finished,
    ChannelBusy,
    Aborted,
    HardwareError,
}

/// BL702 direct-register radio exposed through the shared software MAC.
pub struct Bl702RadioPhy {
    channel: u8,
    tx_power: i8,
    /// SEC_ENG hardware AES-128 accelerator, present only under the
    /// `hardware-aes-bl702` feature. Installed once by the composition root
    /// via [`Bl702RadioPhy::install_aes_engine`] from the board's exclusive
    /// `Aes` token, so there is exactly one engine and no global mutable
    /// alias. `None` only in the window between construction and that single
    /// install call; a missing engine at cipher time traps loudly rather
    /// than silently falling back to software.
    #[cfg(feature = "hardware-aes-bl702")]
    aes_engine: Option<bl702_hal::aes::AesEngine>,
}

/// Shared software MAC over the BL702 direct-register radio.
pub type Bl702SoftMac = SoftMacCore<Bl702RadioPhy>;

impl Bl702RadioPhy {
    /// Reset and calibrate the BL702 radio, then construct the M154 driver.
    ///
    /// `delay_us` must busy-wait for at least the requested number of
    /// microseconds. RF calibration takes several milliseconds.
    ///
    /// # Safety
    ///
    /// The caller must guarantee exclusive access to the BL702 radio and must
    /// configure the chip clocks before calling this function.
    pub unsafe fn initialize(mut delay_us: impl FnMut(u32)) -> Result<Self, PhyError> {
        rf::initialize(&mut delay_us).map_err(|_| PhyError::Hardware)?;
        Ok(Self::finish_initialization())
    }

    /// Construct the M154 driver after the analog RF block has been initialized.
    ///
    /// # Safety
    ///
    /// The caller must guarantee exclusive access to the BL702 radio and must
    /// have completed the RF power-up and per-die calibration sequence.
    pub unsafe fn from_preinitialized_rf() -> Self {
        Self::finish_initialization()
    }

    fn finish_initialization() -> Self {
        initialize_m154();
        configure_promiscuous_rx();
        disable_rx();
        set_cca_defaults();
        set_channel_register(11);
        rf::set_tx_power(0);

        Self {
            channel: 11,
            tx_power: 0,
            #[cfg(feature = "hardware-aes-bl702")]
            aes_engine: None,
        }
    }

    /// Take ownership of the SEC_ENG AES-128 accelerator and use it for all
    /// subsequent CCM* and AES-MMO operations (via the `ForwardAesProvider`
    /// override below, forwarded through [`crate::SoftMacCore`]).
    ///
    /// The composition root calls this exactly once, right after
    /// construction, handing over the board's single exclusive `Aes` token.
    /// Before accepting the engine this runs back-to-back AES-128
    /// known-answer tests against the real peripheral, including a
    /// re-key/reuse cycle. Only compiled under `hardware-aes-bl702`.
    #[cfg(feature = "hardware-aes-bl702")]
    pub fn install_aes_engine(
        &mut self,
        aes: bl702_hal::peripherals::Aes,
    ) -> Result<(), bl702_hal::aes::AesError> {
        let mut engine = bl702_hal::aes::AesEngine::new(
            aes,
            bl702_hal::aes::AesEngine::DEFAULT_TIMEOUT_ITERATIONS,
        )?;
        engine.self_test()?;
        self.aes_engine = Some(engine);
        Ok(())
    }

    async fn transmit(&mut self, frame: &[u8], run_cca: bool) -> Result<(), PhyError> {
        validate_frame(frame)?;
        disable_rx();

        if run_cca {
            let (busy, _) = run_cca_once()?;
            if busy {
                return Err(PhyError::ChannelBusy);
            }
        }

        clear_tx_events();
        update32(M154_CSMA_CONTROL, 1, 0);
        write_tx_frame(frame);
        trigger_tx()?;

        let deadline = Instant::now() + TX_DONE_TIMEOUT;
        loop {
            let status = read32(M154_IRQ_STATUS);
            if let Some(result) = decode_tx_result(status) {
                write32(M154_IRQ_CLEAR, status & IRQ_TX_MASK);
                return match result {
                    TxResult::Finished => Ok(()),
                    TxResult::ChannelBusy => Err(PhyError::ChannelBusy),
                    TxResult::Aborted | TxResult::HardwareError => Err(PhyError::Hardware),
                };
            }
            if Instant::now() >= deadline {
                reset_tx();
                return Err(PhyError::Hardware);
            }
            yield_now().await;
        }
    }
}

impl RadioPhy for Bl702RadioPhy {
    fn capabilities(&self) -> PhyCapabilities {
        PhyCapabilities {
            hardware_auto_ack: false,
            hardware_address_filter: false,
            tx_power_min: 0,
            tx_power_max: 14,
        }
    }

    async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
        self.transmit(frame, true).await
    }

    async fn send_ack(&mut self, sequence: u8, frame_pending: bool) -> Result<(), PhyError> {
        let frame = [0x02 | if frame_pending { 0x10 } else { 0 }, 0x00, sequence];
        self.transmit(&frame, false).await
    }

    async fn receive(&mut self, timeout_us: u32) -> Result<Option<PhyRxFrame>, PhyError> {
        let mut rx_started = enable_rx();
        let _guard = RxGuard;
        let deadline = Instant::now() + Duration::from_micros(u64::from(timeout_us));

        loop {
            if !rx_started {
                rx_started = enable_rx();
            }
            let status = read32(M154_IRQ_STATUS);
            let rx_status = status & IRQ_RX_MASK;
            if rx_status != 0 {
                write32(M154_IRQ_CLEAR, rx_status);
                if rx_status & IRQ_RX_CRC != 0 && read32(M154_RX_STATUS) & 1 != 0 {
                    return Err(PhyError::CrcFailed);
                }
                return read_rx_frame().map(Some);
            }
            if Instant::now() >= deadline {
                return Ok(None);
            }
            yield_now().await;
        }
    }

    fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
        if !(11..=26).contains(&channel) {
            return Err(PhyError::InvalidChannel);
        }
        set_channel_register(channel);
        self.channel = channel;
        Ok(())
    }

    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
        if !(0..=14).contains(&dbm) {
            return Err(PhyError::Unsupported);
        }
        rf::set_tx_power(dbm as u8);
        self.tx_power = dbm;
        Ok(())
    }

    async fn energy_detect(&mut self, duration_us: u32) -> Result<u8, PhyError> {
        let deadline = Instant::now() + Duration::from_micros(u64::from(duration_us));
        let mut strongest = i8::MIN;
        loop {
            let (_, rssi) = run_cca_once()?;
            strongest = strongest.max(rssi);
            if duration_us == 0 || Instant::now() >= deadline {
                return Ok(rssi_to_ed(strongest));
            }
            yield_now().await;
        }
    }

    fn set_address_filter(&mut self, _filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
        Ok(())
    }
}

/// Software AES (the default) when `hardware-aes-bl702` is off — the
/// standard BL702 image keeps the RustCrypto software core, exactly as
/// before.
#[cfg(not(feature = "hardware-aes-bl702"))]
impl zigbee_crypto::ForwardAesProvider for Bl702RadioPhy {}

/// Hardware AES-128 backend for CCM* and AES-MMO. Hands back a
/// [`zigbee_crypto::bl702::HardwareAes128`] borrowing this phy's
/// exclusively-owned SEC_ENG [`bl702_hal::aes::AesEngine`], so the RustCrypto
/// software core is dead-code-eliminated from the image. A missing engine
/// (composition root failed to call [`Bl702RadioPhy::install_aes_engine`])
/// is a firmware wiring bug and traps loudly rather than silently falling
/// back to software.
#[cfg(feature = "hardware-aes-bl702")]
impl zigbee_crypto::ForwardAesProvider for Bl702RadioPhy {
    fn forward_cipher(
        &mut self,
        key: &zigbee_crypto::AesKey,
    ) -> impl zigbee_crypto::Aes128Forward + '_ {
        let engine = self
            .aes_engine
            .as_mut()
            .expect("AES engine not installed: call Bl702RadioPhy::install_aes_engine()");
        zigbee_crypto::bl702::HardwareAes128::new(engine, *key)
    }
}

impl PlatformServices for Bl702RadioPhy {
    fn monotonic_micros(&self) -> u32 {
        Instant::now().as_micros() as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        Timer::after_micros(u64::from(duration_us)).await;
    }

    fn fill_random(&mut self, _output: &mut [u8]) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }
}

struct RxGuard;

impl Drop for RxGuard {
    fn drop(&mut self) {
        disable_rx();
    }
}

fn initialize_m154() {
    set32(GLB_CGEN_CFG2, 1);
    set32(GLB_MIX_RESET, 1 << 4);
    clear32(GLB_MIX_RESET, 1 << 4);
    set32(GLB_CLK_CFG1, 1 << 25);

    update32(M154_CONTROL, 0x3, 0x1);
    update32(M154_BASE + 0x0c0, 0xff, 0x82);
    write32(M154_STATE, (read32(M154_STATE) & 0x00ff_ffff) | 0xc001_0000);
    set32(M154_BASE + 0x104, 1);
    clear32(M154_RX_CONTROL, 1 << 5);
    update32(M154_TX_CONTROL, 0xff00, 0x4000);
    set32(M154_FILTER_CONTROL, 1 << 8);

    write32(M154_IRQ_ENABLE, 0);
    write32(M154_BASE + 0x028, 0);
    set32(M154_IRQ_ENABLE, 0x00f0_0000);
    set32(M154_IRQ_ENABLE, 0x0c00_0000);
    set32(M154_IRQ_ENABLE, 0x3000_0000);
    if read32(M154_FILTER_CONTROL) & (1 << 16) != 0 {
        set32(M154_IRQ_ENABLE, IRQ_RX_DONE);
    }

    write32(
        M154_COEX_CONTROL,
        (read32(M154_COEX_CONTROL) & 0x3fff_0000) | 0x0001_cb31,
    );
    set32(GLB_COEX_CTRL, 1);
}

fn configure_promiscuous_rx() {
    update32(M154_RX_CONTROL, 0xff00_0000, 0x8f00_0000);
    set32(M154_FILTER_CONTROL, 1 << 16);
    set32(M154_IRQ_ENABLE, IRQ_RX_MASK);
    clear32(M154_TX_CONTROL, 1 << 4);
}

fn set_cca_defaults() {
    update32(PHY_CCA_CTRL, 0x1800_0000, 2 << 27);
    update32(PHY_CCA_CTRL, 0x007f_0000, 76 << 16);
}

fn set_channel_register(channel: u8) {
    update32(M154_CONTROL, 0x0fff_0000, channel_frequency_word(channel));
}

const fn channel_frequency_word(channel: u8) -> u32 {
    (0x965 + (channel as u32 - 11) * 5) << 16
}

fn run_cca_once() -> Result<(bool, i8), PhyError> {
    set32(PHY_CCA_CTRL, 1 << 1);
    clear32(PHY_CCA_CTRL, 1 << 1);
    set32(PHY_CCA_ENABLE, 1 << 27);

    for _ in 0..CCA_READY_SPINS {
        let result = read32(PHY_CCA_CTRL);
        if result & (1 << 2) != 0 {
            clear32(PHY_CCA_ENABLE, 1 << 27);
            let raw = ((result >> 8) & 0x7f) as u8;
            return Ok((result & (1 << 7) != 0, 5 - raw as i8));
        }
        spin_loop();
    }

    clear32(PHY_CCA_ENABLE, 1 << 27);
    Err(PhyError::Hardware)
}

fn validate_frame(frame: &[u8]) -> Result<(), PhyError> {
    if frame.len() > MAX_PHY_FRAME_LEN {
        return Err(PhyError::FrameTooLong);
    }
    if frame.len() < MIN_FRAME_LEN {
        return Err(PhyError::Hardware);
    }
    Ok(())
}

fn write_tx_frame(frame: &[u8]) {
    for (word_index, bytes) in frame.chunks(4).enumerate() {
        let mut packed = 0u32;
        for (byte_index, byte) in bytes.iter().enumerate() {
            packed |= u32::from(*byte) << (byte_index * 8);
        }
        write32(M154_TX_BUFFER + (word_index as u32 * 4), packed);
    }
    update32(
        M154_TX_LENGTH,
        0xff00_0000,
        ((frame.len() as u32 + 2) & 0xff) << 24,
    );
}

fn read_rx_frame() -> Result<PhyRxFrame, PhyError> {
    let phy_len = (read32(M154_RX_METADATA) >> 24) as u8;
    let payload_len = rx_payload_len(phy_len)?;
    let mut data = [0u8; MAX_PHY_FRAME_LEN];

    for (word_index, chunk) in data[..payload_len].chunks_mut(4).enumerate() {
        let word = read32(M154_RX_BUFFER + word_index as u32 * 4).to_le_bytes();
        chunk.copy_from_slice(&word[..chunk.len()]);
    }

    let raw_rssi = ((read32(PHY_RSSI) >> 8) & 0xff) as u8;
    PhyRxFrame::from_slice(&data[..payload_len], lqi_from_raw_rssi(raw_rssi))
}

fn rx_payload_len(phy_len: u8) -> Result<usize, PhyError> {
    if !(5..=127).contains(&phy_len) {
        return Err(PhyError::Hardware);
    }
    Ok(usize::from(phy_len - 2))
}

fn enable_rx() -> bool {
    if is_idle() {
        set32(M154_RX_CONTROL, 1 << 12);
        true
    } else {
        false
    }
}

fn disable_rx() {
    set32(M154_RX_CONTROL, 1 << 8);
}

fn is_idle() -> bool {
    if read32(M154_RX_CONTROL) & (1 << 18) != 0 {
        return false;
    }
    if read32(M154_BUSY_STATUS) & (1 << 11) != 0 {
        return false;
    }
    if read32(M154_STATE) & 0x7ff != 1 {
        return false;
    }
    read32(M154_FILTER_CONTROL) & (1 << 16) != 0 || ((read32(RF_FSM) >> 12) & 0x7) == 0
}

fn trigger_tx() -> Result<(), PhyError> {
    let started = read32(M154_TIMER);
    loop {
        if is_idle() {
            if read32(GLB_COEX_CTRL) & 1 != 0 {
                set32(M154_IRQ_ENABLE, 1 << 31);
            }
            set32(M154_TX_CONTROL, 1);
            return Ok(());
        }

        if read32(M154_TIMER).wrapping_sub(started) > TX_IDLE_TIMEOUT_TICKS {
            reset_tx();
            return Err(PhyError::Hardware);
        }
        spin_loop();
    }
}

fn reset_tx() {
    write32(M154_RESET, 24);
    write32(M154_RECOVERY_TIME, read32(M154_TIMER).wrapping_add(20));
    set32(M154_IRQ_ENABLE, 1 << 1);
}

fn clear_tx_events() {
    let pending = read32(M154_IRQ_STATUS) & IRQ_TX_MASK;
    if pending != 0 {
        write32(M154_IRQ_CLEAR, pending);
    }
}

fn decode_tx_result(status: u32) -> Option<TxResult> {
    if status & IRQ_TX_ABORTED != 0 {
        Some(TxResult::Aborted)
    } else if status & IRQ_TX_HW_ERROR != 0 {
        Some(TxResult::HardwareError)
    } else if status & IRQ_TX_CSMA_FAILED != 0 {
        Some(TxResult::ChannelBusy)
    } else if status & IRQ_TX_FINISHED != 0 {
        Some(TxResult::Finished)
    } else {
        None
    }
}

const fn lqi_from_raw_rssi(raw: u8) -> u8 {
    if raw <= 49 {
        255
    } else {
        255u8.saturating_sub(raw.saturating_sub(50).saturating_mul(5))
    }
}

const fn rssi_to_ed(rssi: i8) -> u8 {
    if rssi <= -100 {
        0
    } else if rssi >= 5 {
        255
    } else {
        (((rssi as i16 + 100) * 255) / 105) as u8
    }
}

async fn yield_now() {
    let mut yielded = false;
    core::future::poll_fn(|context| {
        if yielded {
            core::task::Poll::Ready(())
        } else {
            yielded = true;
            context.waker().wake_by_ref();
            core::task::Poll::Pending
        }
    })
    .await;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_ieee_channels_to_mhz_field() {
        assert_eq!(channel_frequency_word(11), 2405 << 16);
        assert_eq!(channel_frequency_word(26), 2480 << 16);
    }

    #[test]
    fn strips_fcs_from_received_length() {
        assert_eq!(rx_payload_len(5), Ok(3));
        assert_eq!(rx_payload_len(127), Ok(125));
        assert_eq!(rx_payload_len(4), Err(PhyError::Hardware));
        assert_eq!(rx_payload_len(128), Err(PhyError::Hardware));
    }

    #[test]
    fn receive_irq_mask_includes_crc_filtering() {
        assert_eq!(IRQ_RX_MASK, 0x01f8_0000);
    }

    #[test]
    fn decodes_tx_completion_priority() {
        assert_eq!(decode_tx_result(IRQ_TX_FINISHED), Some(TxResult::Finished));
        assert_eq!(
            decode_tx_result(IRQ_TX_CSMA_FAILED),
            Some(TxResult::ChannelBusy)
        );
        assert_eq!(
            decode_tx_result(IRQ_TX_FINISHED | IRQ_TX_ABORTED),
            Some(TxResult::Aborted)
        );
        assert_eq!(decode_tx_result(0), None);
    }

    #[test]
    fn matches_vendor_lqi_curve() {
        assert_eq!(lqi_from_raw_rssi(49), 255);
        assert_eq!(lqi_from_raw_rssi(50), 255);
        assert_eq!(lqi_from_raw_rssi(51), 250);
        assert_eq!(lqi_from_raw_rssi(101), 0);
    }

    #[test]
    fn maps_rssi_monotonically_to_ed() {
        assert_eq!(rssi_to_ed(-100), 0);
        assert!(rssi_to_ed(-71) < rssi_to_ed(-40));
        assert_eq!(rssi_to_ed(5), 255);
    }
}
