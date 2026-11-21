//! GP Pairing Search sent by the proxy (`green-power-pairing-search`).
//!
//! A GPDF from a GPD without a Proxy Table entry (outside commissioning
//! mode) is dropped by the proxy. With this feature the proxy also asks the
//! sinks for the GPD's pairings with a broadcast GP Pairing Search (every
//! sink kind, frame counter and key); a sink that has the GPD answers with
//! GP Pairing (`zigbee-gp/pairing-search`) and the proxy stores the entry
//! as for any GP Pairing, so the GPD's next GPDF is tunnelled. This
//! recovers a proxy whose table was lost or that joined after the GPD was
//! commissioned.
//!
//! Searches are bounded, since GPDs of neighbouring networks are heard too:
//! at most one per [`SEARCH_GAP_MS`] from this node, and the same GPD again
//! only after [`SEARCH_REPEAT_MS`] (the last [`RECENT`] GPDs are
//! remembered). The search waits for the proxy's transmit path and goes
//! out after the proxy's own frames.
//!
//! Without the feature the hooks return the proxy's own results and compile
//! to the previous code.

use zigbee_gp::cluster::GppGpdLink;
use zigbee_gp::frame::Gpdf;
use zigbee_gp::proxy::{GpTx, GpdfOutcome};
use zigbee_mac::ForwardAesProvider;

use super::GreenPowerState;

#[cfg(not(feature = "green-power-pairing-search"))]
#[inline(always)]
pub(super) fn on_gpdf<P: ForwardAesProvider>(
    gp: &mut GreenPowerState,
    aes: &mut P,
    frame: &Gpdf<'_>,
    link: GppGpdLink,
    now_ms: u32,
) -> GpdfOutcome {
    gp.proxy.on_gpdf(aes, frame, link, now_ms)
}

#[cfg(not(feature = "green-power-pairing-search"))]
#[inline(always)]
pub(super) fn poll(gp: &mut GreenPowerState, now_ms: u32) -> Option<GpTx> {
    gp.proxy.poll(now_ms)
}

#[cfg(feature = "green-power-pairing-search")]
pub(super) use imp::{SearchState, on_gpdf, poll};

#[cfg(feature = "green-power-pairing-search")]
mod imp {
    use heapless::Deque;
    use zigbee_gp::cluster::{PairingSearch, server_cmd};
    use zigbee_gp::frame::FrameType;
    use zigbee_gp::proxy::{
        BROADCAST_RX_ON_WHEN_IDLE, DestAddr, DropReason, MAX_GP_COMMAND_LEN, TxFrame,
    };
    use zigbee_gp::types::GpdId;

    use super::*;

    /// Shortest interval between two searches from this node.
    pub const SEARCH_GAP_MS: u32 = 60_000;
    /// Shortest interval between two searches for the same GPD.
    pub const SEARCH_REPEAT_MS: u32 = 15 * 60_000;
    /// GPDs whose last search is remembered.
    pub const RECENT: usize = 4;

    /// The proxy's searches.
    pub(in crate::green_power) struct SearchState {
        pending: Option<GpdId>,
        last_ms: Option<u32>,
        recent: Deque<(GpdId, u32), RECENT>,
    }

    impl SearchState {
        pub(in crate::green_power) const fn new() -> Self {
            Self {
                pending: None,
                last_ms: None,
                recent: Deque::new(),
            }
        }
    }

    fn within(now: u32, since: u32, interval: u32) -> bool {
        now.wrapping_sub(since) < interval
    }

    /// The proxy's GPDF processing; a frame dropped for want of a Proxy
    /// Table entry queues a search.
    pub(in crate::green_power) fn on_gpdf<P: ForwardAesProvider>(
        gp: &mut GreenPowerState,
        aes: &mut P,
        frame: &Gpdf<'_>,
        link: GppGpdLink,
        now_ms: u32,
    ) -> GpdfOutcome {
        let outcome = gp.proxy.on_gpdf(aes, frame, link, now_ms);
        let h = &frame.header;
        let Some(gpd) = h.gpd_id else {
            return outcome;
        };
        let s = &mut gp.search;
        if outcome != GpdfOutcome::Dropped(DropReason::NoEntry)
            || h.to_gpd
            || h.frame_type != FrameType::Data
            || gpd.is_reserved()
            || s.pending.is_some()
            || s.last_ms.is_some_and(|t| within(now_ms, t, SEARCH_GAP_MS))
            || s.recent
                .iter()
                .any(|(g, t)| g.matches(&gpd) && within(now_ms, *t, SEARCH_REPEAT_MS))
        {
            return outcome;
        }
        s.pending = Some(gpd);
        s.last_ms = Some(now_ms);
        if s.recent.is_full() {
            s.recent.pop_front();
        }
        let _ = s.recent.push_back((gpd, now_ms));
        outcome
    }

    /// The proxy's next frame, else the queued search.
    pub(in crate::green_power) fn poll(gp: &mut GreenPowerState, now_ms: u32) -> Option<GpTx> {
        if let Some(tx) = gp.proxy.poll(now_ms) {
            return Some(tx);
        }
        let gpd_id = gp.search.pending.take()?;
        let search = PairingSearch {
            gpd_id,
            request_unicast: true,
            request_derived_groupcast: true,
            request_commissioned_groupcast: true,
            request_frame_counter: true,
            request_key: true,
        };
        let mut payload = heapless::Vec::<u8, MAX_GP_COMMAND_LEN>::new();
        payload.resize_default(MAX_GP_COMMAND_LEN).ok()?;
        let len = search.encode(&mut payload).ok()?;
        payload.truncate(len);
        log::info!("[GP] Pairing Search for {:?}", gpd_id);
        Some(GpTx {
            dest: DestAddr::Broadcast(BROADCAST_RX_ON_WHEN_IDLE),
            alias: None,
            radius: 0,
            frame: TxFrame::GpCommand {
                command_id: server_cmd::PAIRING_SEARCH,
                payload,
            },
        })
    }
}

#[cfg(all(
    test,
    feature = "router",
    feature = "green-power-pairing-search",
    feature = "green-power-sink-pairing-search"
))]
#[path = "pairing_search/tests.rs"]
mod tests;
