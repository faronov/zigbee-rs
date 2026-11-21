//! GPD Channel Request on the sink (feature `channel-request`).
//!
//! A GPD that does not know the network's channel sends Channel Request
//! GPDFs (SrcID 0, Maintenance frame, one byte: the channels it will listen
//! on next, low nibble first) while toggling channels. A sink in
//! commissioning mode answers with a GP Response carrying a Channel
//! Configuration (the operational channel, Basic subfield set) for the
//! TempMaster: the proxy that tunnelled the request, or this combo's own
//! proxy for a request heard directly. The TempMaster transmits it on the
//! GPD's next receive channel when the GPD's next Channel Request arrives.
//!
//! Behaviour follows gecko_sdk `handleChannelRequest`, with two
//! differences: the sink answers only in commissioning mode (gecko_sdk
//! answers tunnelled requests at any time; proxies forward them only in
//! commissioning mode), and the same answer for the same TempMaster is not
//! repeated within [`REPEAT_SUPPRESSION_MS`] (a GPD sends several Channel
//! Requests per channel; one queued GP Response serves them all).

use super::{Out, Sink, SinkDrop, SinkOutcome};
use crate::cluster::ProxyInfo;
use crate::frame::{FrameType, Gpdf};
use crate::types::{GpdId, SecurityLevel, cmd};

/// Repeated Channel Requests for the same TempMaster and channel within
/// this window share one GP Response.
pub const REPEAT_SUPPRESSION_MS: u32 = 1_000;

/// Channel Configuration payload: operational channel - 11, Basic bit.
const BASIC: u8 = 0x10;

impl<const N: usize, const Q: usize> Sink<N, Q> {
    /// A Maintenance GPDF without a GPD ID heard directly (combo device).
    pub(super) fn on_maintenance_gpdf(&mut self, frame: &Gpdf<'_>, now_ms: u32) -> SinkOutcome {
        let h = &frame.header;
        if h.frame_type != FrameType::Maintenance || h.security_level != SecurityLevel::None {
            return SinkOutcome::Dropped(SinkDrop::Invalid);
        }
        let Some((&command_id, payload)) = frame.payload.split_first() else {
            return SinkOutcome::Dropped(SinkDrop::Invalid);
        };
        if command_id != cmd::CHANNEL_REQUEST {
            return SinkOutcome::Dropped(SinkDrop::Unsupported);
        }
        if self
            .dup
            .check_and_record(&GpdId::SrcId(0), u32::from(frame.mac_seq), now_ms)
        {
            return SinkOutcome::Dropped(SinkDrop::Duplicate);
        }
        // This combo's own proxy is the TempMaster; it needs timed
        // transmission (`set_direct_bidirectional`).
        let temp_master = self.direct_bidirectional.then_some(self.own_nwk);
        self.channel_request(temp_master, payload, now_ms)
    }

    /// A Channel Request tunnelled in a GP Notification or GP
    /// Commissioning Notification.
    pub(super) fn on_tunnelled_channel_request(
        &mut self,
        gpd: &GpdId,
        rx_after_tx: bool,
        proxy_info: ProxyInfo,
        payload: Option<&[u8]>,
        counter: u32,
        now_ms: u32,
    ) -> SinkOutcome {
        // gecko_sdk: a Channel Request comes from GPD ID 0 only, and the
        // proxy must have signalled the GPD's receive window.
        if *gpd != GpdId::SrcId(0) {
            return SinkOutcome::Dropped(SinkDrop::Invalid);
        }
        let Some(payload) = payload else {
            return SinkOutcome::Dropped(SinkDrop::Invalid);
        };
        let temp_master = match proxy_info {
            ProxyInfo::Link { gpp_short, .. } | ProxyInfo::Distance { gpp_short, .. }
                if rx_after_tx =>
            {
                Some(gpp_short)
            }
            _ => None,
        };
        if self.dup.check_and_record(gpd, counter, now_ms) {
            return SinkOutcome::Dropped(SinkDrop::Duplicate);
        }
        self.channel_request(temp_master, payload, now_ms)
    }

    /// Queue the Channel Configuration for `temp_master` (gecko_sdk
    /// `handleChannelRequest`).
    fn channel_request(
        &mut self,
        temp_master: Option<u16>,
        payload: &[u8],
        now_ms: u32,
    ) -> SinkOutcome {
        if self.commissioning.is_none() {
            return SinkOutcome::Dropped(SinkDrop::NotCommissioning);
        }
        let [request] = payload else {
            return SinkOutcome::Dropped(SinkDrop::Invalid);
        };
        let Some(temp_master) = temp_master else {
            return SinkOutcome::Dropped(SinkDrop::NoTempMaster);
        };
        if !(11..=26).contains(&self.operational_channel) {
            return SinkOutcome::Dropped(SinkDrop::NoTempMaster);
        }
        // The GPD listens next on the channel of the low nibble.
        let tx_channel = request & 0x0F;
        if let Some((tm, ch, at)) = self.last_channel_configuration
            && tm == temp_master
            && ch == tx_channel
            && now_ms.wrapping_sub(at) < REPEAT_SUPPRESSION_MS
        {
            return SinkOutcome::ChannelConfigurationSent;
        }
        let config = (self.operational_channel - 11) | BASIC;
        if !self.queue(Out::ChannelConfiguration {
            temp_master,
            tx_channel,
            config,
        }) {
            return SinkOutcome::Dropped(SinkDrop::QueueFull);
        }
        self.last_channel_configuration = Some((temp_master, tx_channel, now_ms));
        SinkOutcome::ChannelConfigurationSent
    }
}

#[cfg(test)]
mod tests;
