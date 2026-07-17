use crate::{
    MAX_PHY_FRAME_LEN, MacError, PhyAddressFilter, PhyCapabilities, PhyError, PhyRxFrame,
    PlatformServices, RadioPhy, SoftMacCore,
};
use embassy_futures::select::{Either, select};
use embassy_time::Timer;

use super::embassy_nrf::radio::Error as RadioError;
use super::{Packet, Radio, RadioInstance, Rng, RngInstance};

/// nRF52833/nRF52840 radio and RNG exposed through the portable PHY contract.
pub struct NrfRadioPhy<'a, T: RadioInstance, R: RngInstance> {
    radio: Radio<'a, T>,
    rng: Rng<'a, R>,
}

/// Shared software MAC using the nRF radio adapter.
pub type NrfSoftMac<'a, T, R> = SoftMacCore<NrfRadioPhy<'a, T, R>>;

impl<'a, T: RadioInstance, R: RngInstance> NrfRadioPhy<'a, T, R> {
    pub fn new(radio: Radio<'a, T>, rng: Rng<'a, R>) -> Self {
        Self { radio, rng }
    }
}

impl<T: RadioInstance, R: RngInstance> RadioPhy for NrfRadioPhy<'_, T, R> {
    fn capabilities(&self) -> PhyCapabilities {
        PhyCapabilities {
            hardware_auto_ack: false,
            hardware_address_filter: false,
        }
    }

    async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
        if frame.len() > MAX_PHY_FRAME_LEN {
            return Err(PhyError::FrameTooLong);
        }

        let mut packet = Packet::new();
        packet.copy_from_slice(frame);
        self.radio.try_send(&mut packet).await.map_err(|error| {
            if error == RadioError::ChannelInUse {
                PhyError::ChannelBusy
            } else {
                Self::map_radio_error(error)
            }
        })
    }

    async fn send_ack(&mut self, _sequence: u8, _frame_pending: bool) -> Result<(), PhyError> {
        // Embassy currently exposes only CCA-gated TX. Returning Unsupported
        // keeps the PHY contract honest until a no-CCA ACK path is added.
        Err(PhyError::Unsupported)
    }

    async fn receive(&mut self, timeout_us: u32) -> Result<Option<PhyRxFrame>, PhyError> {
        let mut packet = Packet::new();
        match select(
            Timer::after_micros(u64::from(timeout_us)),
            self.radio.receive(&mut packet),
        )
        .await
        {
            Either::First(()) => Ok(None),
            Either::Second(Ok(())) => {
                PhyRxFrame::from_slice(packet.as_ref(), packet.lqi()).map(Some)
            }
            Either::Second(Err(error)) => Err(Self::map_radio_error(error)),
        }
    }

    fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
        if !(11..=26).contains(&channel) {
            return Err(PhyError::InvalidChannel);
        }
        self.radio.set_channel(channel);
        Ok(())
    }

    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
        if !matches!(
            dbm,
            -40 | -30 | -20 | -16 | -12 | -8 | -4 | 0 | 2 | 3 | 4 | 5 | 6 | 7 | 8
        ) {
            return Err(PhyError::Unsupported);
        }
        self.radio.set_transmission_power(dbm);
        Ok(())
    }

    async fn energy_detect(&mut self, _duration_us: u32) -> Result<u8, PhyError> {
        Err(PhyError::Unsupported)
    }

    fn set_address_filter(&mut self, _filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
        Ok(())
    }
}

impl<T: RadioInstance, R: RngInstance> PlatformServices for NrfRadioPhy<'_, T, R> {
    fn monotonic_micros(&self) -> u32 {
        embassy_time::Instant::now().as_micros() as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        Timer::after_micros(u64::from(duration_us)).await;
    }

    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        self.rng.blocking_fill_bytes(output);
        Ok(())
    }
}

impl<T: RadioInstance, R: RngInstance> NrfRadioPhy<'_, T, R> {
    fn map_radio_error(error: RadioError) -> PhyError {
        match error {
            RadioError::BufferTooLong => PhyError::FrameTooLong,
            RadioError::ChannelInUse => PhyError::ChannelBusy,
            RadioError::CrcFailed(_) => PhyError::CrcFailed,
            _ => PhyError::Hardware,
        }
    }
}
