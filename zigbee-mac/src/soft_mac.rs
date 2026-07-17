//! Portable MAC state layered over a hardware radio PHY.

use zigbee_types::MacAddress;

use crate::{
    MacError, MacPib, PhyAddressFilter, PhyError, PibAttribute, PibError, PibValue,
    PlatformServices, RadioPhy,
};

/// Shared software-MAC state for one radio.
///
/// Protocol operations are added incrementally; existing hardware backends
/// continue implementing [`crate::MacDriver`] directly during migration.
pub struct SoftMacCore<P: RadioPhy> {
    phy: P,
    pib: MacPib,
}

impl<P: RadioPhy> SoftMacCore<P> {
    pub fn new(mut phy: P, pib: MacPib) -> Result<Self, MacError> {
        Self::apply_full_config(&mut phy, &pib)?;
        Ok(Self { phy, pib })
    }

    pub fn phy(&self) -> &P {
        &self.phy
    }

    pub fn phy_mut(&mut self) -> &mut P {
        &mut self.phy
    }

    pub fn into_phy(self) -> P {
        self.phy
    }

    pub fn pib(&self) -> &MacPib {
        &self.pib
    }

    pub fn get_pib(&self, attr: PibAttribute) -> PibValue {
        self.pib.get(attr)
    }

    pub fn set_pib(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        let mut next = self.pib.clone();
        next.set(attr, value).map_err(Self::map_pib_error)?;
        self.apply_changed_config(attr, &next)?;
        self.pib = next;
        Ok(())
    }

    pub fn reset_pib(&mut self, dsn: u8, bsn: u8) -> Result<(), MacError> {
        let mut next = self.pib.clone();
        next.reset(dsn, bsn);
        Self::apply_full_config(&mut self.phy, &next)?;
        self.pib = next;
        Ok(())
    }

    pub fn accepts_destination(&self, destination: &MacAddress) -> bool {
        if self.pib.promiscuous() {
            return true;
        }

        match destination {
            MacAddress::Short(pan_id, address) => {
                (*pan_id == self.pib.pan_id() || pan_id.0 == 0xFFFF)
                    && (*address == self.pib.short_address() || address.0 == 0xFFFF)
            }
            MacAddress::Extended(pan_id, address) => {
                (*pan_id == self.pib.pan_id() || pan_id.0 == 0xFFFF)
                    && *address == self.pib.extended_address()
            }
        }
    }

    pub fn is_exact_destination(&self, destination: &MacAddress) -> bool {
        match destination {
            MacAddress::Short(pan_id, address) => {
                address.0 != 0xFFFF
                    && *pan_id == self.pib.pan_id()
                    && *address == self.pib.short_address()
            }
            MacAddress::Extended(pan_id, address) => {
                *pan_id == self.pib.pan_id() && *address == self.pib.extended_address()
            }
        }
    }

    fn apply_changed_config(&mut self, attr: PibAttribute, next: &MacPib) -> Result<(), MacError> {
        use PibAttribute::*;
        match attr {
            PhyCurrentChannel => self
                .phy
                .set_channel(next.current_channel())
                .map_err(Self::map_phy_error),
            PhyTransmitPower => self
                .phy
                .set_tx_power(next.transmit_power())
                .map_err(Self::map_phy_error),
            MacShortAddress | MacPanId | MacExtendedAddress | MacPromiscuousMode => self
                .phy
                .set_address_filter(Self::address_filter(next))
                .map_err(Self::map_phy_error),
            _ => Ok(()),
        }
    }

    fn apply_full_config(phy: &mut P, pib: &MacPib) -> Result<(), MacError> {
        phy.set_channel(pib.current_channel())
            .map_err(Self::map_phy_error)?;
        phy.set_tx_power(pib.transmit_power())
            .map_err(Self::map_phy_error)?;
        phy.set_address_filter(Self::address_filter(pib))
            .map_err(Self::map_phy_error)
    }

    fn address_filter(pib: &MacPib) -> Option<PhyAddressFilter> {
        (!pib.promiscuous()).then_some(PhyAddressFilter {
            pan_id: pib.pan_id(),
            short_address: pib.short_address(),
            extended_address: pib.extended_address(),
        })
    }

    fn map_pib_error(error: PibError) -> MacError {
        match error {
            PibError::InvalidValue | PibError::ReadOnly => MacError::InvalidParameter,
        }
    }

    fn map_phy_error(error: PhyError) -> MacError {
        match error {
            PhyError::ChannelBusy => MacError::ChannelAccessFailure,
            PhyError::InvalidChannel => MacError::InvalidParameter,
            PhyError::FrameTooLong => MacError::FrameTooLong,
            PhyError::CrcFailed | PhyError::Hardware => MacError::RadioError,
            PhyError::Unsupported => MacError::Unsupported,
        }
    }
}

impl<P: RadioPhy + PlatformServices> PlatformServices for SoftMacCore<P> {
    fn monotonic_micros(&self) -> u32 {
        self.phy.monotonic_micros()
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        self.phy.delay_micros(duration_us).await;
    }

    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        self.phy.fill_random(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{PhyCapabilities, PhyRxFrame};
    use zigbee_types::{PanId, ShortAddress};

    const IEEE: [u8; 8] = [0x02, 1, 2, 3, 4, 5, 6, 7];

    struct TestPhy {
        channel: u8,
        tx_power: i8,
        filter: Option<PhyAddressFilter>,
        fail_channel: Option<u8>,
    }

    impl TestPhy {
        fn new() -> Self {
            Self {
                channel: 0,
                tx_power: i8::MIN,
                filter: None,
                fail_channel: None,
            }
        }
    }

    impl RadioPhy for TestPhy {
        fn capabilities(&self) -> PhyCapabilities {
            PhyCapabilities::default()
        }

        async fn try_transmit(&mut self, _frame: &[u8]) -> Result<(), PhyError> {
            Ok(())
        }

        async fn send_ack(&mut self, _sequence: u8, _frame_pending: bool) -> Result<(), PhyError> {
            Ok(())
        }

        async fn receive(&mut self, _timeout_us: u32) -> Result<Option<PhyRxFrame>, PhyError> {
            Ok(None)
        }

        fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
            if self.fail_channel == Some(channel) {
                return Err(PhyError::Hardware);
            }
            self.channel = channel;
            Ok(())
        }

        fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
            self.tx_power = dbm;
            Ok(())
        }

        async fn energy_detect(&mut self, _duration_us: u32) -> Result<u8, PhyError> {
            Ok(0)
        }

        fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
            self.filter = filter;
            Ok(())
        }
    }

    impl PlatformServices for TestPhy {
        fn monotonic_micros(&self) -> u32 {
            123
        }

        async fn delay_micros(&mut self, _duration_us: u32) {}

        fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
            output.fill(0xA5);
            Ok(())
        }
    }

    fn core() -> SoftMacCore<TestPhy> {
        SoftMacCore::new(TestPhy::new(), MacPib::new(IEEE, 1, 2)).unwrap()
    }

    #[test]
    fn constructor_applies_pib_to_phy() {
        let core = core();
        assert_eq!(core.phy().channel, 11);
        assert_eq!(core.phy().tx_power, 0);
        assert_eq!(
            core.phy().filter,
            Some(PhyAddressFilter {
                pan_id: PanId(0xFFFF),
                short_address: ShortAddress(0xFFFF),
                extended_address: IEEE,
            })
        );
    }

    #[test]
    fn failed_phy_update_does_not_commit_pib_state() {
        let mut core = core();
        core.phy_mut().fail_channel = Some(26);

        assert_eq!(
            core.set_pib(PibAttribute::PhyCurrentChannel, PibValue::U8(26)),
            Err(MacError::RadioError)
        );
        assert_eq!(
            core.get_pib(PibAttribute::PhyCurrentChannel),
            PibValue::U8(11)
        );
    }

    #[test]
    fn destination_filter_accepts_exact_and_broadcast_addresses() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPanId, PibValue::PanId(PanId(0x1234)))
            .unwrap();
        core.set_pib(
            PibAttribute::MacShortAddress,
            PibValue::ShortAddress(ShortAddress(0x5678)),
        )
        .unwrap();

        let exact = MacAddress::Short(PanId(0x1234), ShortAddress(0x5678));
        let broadcast = MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF));
        let other = MacAddress::Short(PanId(0x1234), ShortAddress(0x9ABC));

        assert!(core.accepts_destination(&exact));
        assert!(core.is_exact_destination(&exact));
        assert!(core.accepts_destination(&broadcast));
        assert!(!core.is_exact_destination(&broadcast));
        assert!(!core.accepts_destination(&other));
    }

    #[test]
    fn unassociated_address_is_not_an_exact_broadcast_match() {
        let core = core();
        let broadcast = MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF));

        assert!(core.accepts_destination(&broadcast));
        assert!(!core.is_exact_destination(&broadcast));
    }

    #[test]
    fn promiscuous_mode_disables_hardware_and_software_filtering() {
        let mut core = core();
        core.set_pib(PibAttribute::MacPromiscuousMode, PibValue::Bool(true))
            .unwrap();

        assert_eq!(core.phy().filter, None);
        assert!(core.accepts_destination(&MacAddress::Short(PanId(0x1111), ShortAddress(0x2222))));
    }

    #[test]
    fn platform_services_delegate_to_phy() {
        let mut core = core();
        let mut random = [0; 4];
        core.fill_random(&mut random).unwrap();

        assert_eq!(core.monotonic_micros(), 123);
        assert_eq!(random, [0xA5; 4]);
    }
}
