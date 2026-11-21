//! GP Pairing Search on the sink: a proxy without a Proxy Table entry for
//! a GPD asks the sinks for their pairings; a sink with a Sink Table entry
//! in one of the requested communication modes answers with a GP Pairing
//! (AddSink) for it, sent to the searching proxy. The GPD security frame
//! counter and key are included only when requested (and known).
//!
//! gecko_sdk leaves the command to a weak application callback; the
//! answer is its GP Pairing for a Sink Table entry
//! (`sendGpPairingFromSinkEntry`).

use super::*;
use crate::cluster::PairingSearch;

impl<const N: usize, const Q: usize> Sink<N, Q> {
    /// GP Pairing Search from `src_nwk`.
    pub(super) fn on_pairing_search(
        &mut self,
        src_nwk: u16,
        payload: &[u8],
    ) -> Result<(u8, Option<SinkOutcome>)> {
        let search = PairingSearch::decode(payload)?;
        if src_nwk == self.own_nwk {
            return Ok((zcl_status::SUCCESS, None));
        }
        let gpd_id = search.gpd_id;
        if let Some(i) = self.find(&gpd_id)
            && wants(&search, self.table[i].communication_mode)
        {
            self.queue(Out::PairingSearchResponse {
                gpd_id,
                dst: src_nwk,
                frame_counter: search.request_frame_counter,
                key: search.request_key,
            });
        }
        Ok((zcl_status::SUCCESS, None))
    }
}

/// Whether `search` asks for pairings in `mode`.
fn wants(search: &PairingSearch, mode: CommunicationMode) -> bool {
    match mode {
        CommunicationMode::FullUnicast | CommunicationMode::LightweightUnicast => {
            search.request_unicast
        }
        CommunicationMode::DerivedGroupcast => search.request_derived_groupcast,
        CommunicationMode::CommissionedGroupcast => search.request_commissioned_groupcast,
    }
}

#[cfg(test)]
#[path = "pairing_search/tests.rs"]
mod tests;
