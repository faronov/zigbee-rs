//! gpsSecurityLevel InvolveTC (feature `involve-tc`).
//!
//! GP 1.1 marks the InvolveTC sub-field (bit 3) for a commissioning that
//! involves the Trust Center; no GP 1.1 sink implements such a procedure.
//! gecko_sdk (`writeInvolveTCBit`, `updateInvolveTC`) maintains the bit
//! from how the node joined and refuses GPD commissioning while it is set:
//!
//! * a node joined to a distributed network (no Trust Center) clears it;
//! * a node joined to a centralized network with a non-default preconfigured
//!   Trust Center link key (an install code) sets it: such a network does
//!   not trust a GPD key protected only with the well-known key;
//! * a join with the default ZigBeeAlliance09 key leaves it as configured.
//!
//! While set, GP Sink Commissioning Mode is answered with UNSUP_COMMAND, the
//! local commissioning API fails and GPD Commissioning frames are dropped
//! with [`SinkDrop::InvolveTc`](super::SinkDrop::InvolveTc).
//!
//! Difference from gecko_sdk: a sink that is itself the network's Trust
//! Center (the ZRHP NCP) is not stopped by the bit. The Trust Center is
//! involved by construction; the host application that runs it decides
//! which GPDs to keep.

use super::{Sink, security_level_attr};

/// How this node joined its network, for [`Sink::derive_involve_tc`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JoinKind {
    /// Centralized network, default global Trust Center link key.
    CentralizedDefaultKey,
    /// Centralized network, an install-code-derived or other non-default
    /// preconfigured Trust Center link key.
    CentralizedPreconfiguredKey,
    /// Distributed network (no Trust Center).
    Distributed,
    /// This node is the network's Trust Center.
    TrustCenter,
}

impl<const N: usize, const Q: usize> Sink<N, Q> {
    /// Tell the sink whether this node is the network's Trust Center.
    pub fn set_trust_center(&mut self, trust_center: bool) {
        self.trust_center = trust_center;
    }

    /// Whether this sink is the network's Trust Center.
    pub fn is_trust_center(&self) -> bool {
        self.trust_center
    }

    /// Update InvolveTC from how this node joined (gecko_sdk
    /// `writeInvolveTCBit`); call once per join. Returns whether the bit
    /// is now set.
    pub fn derive_involve_tc(&mut self, join: JoinKind) -> bool {
        use security_level_attr::INVOLVE_TC;
        let level = &mut self.config.security_level;
        match join {
            JoinKind::Distributed => *level &= !INVOLVE_TC,
            JoinKind::CentralizedPreconfiguredKey => *level |= INVOLVE_TC,
            JoinKind::CentralizedDefaultKey => {}
            JoinKind::TrustCenter => self.trust_center = true,
        }
        if join != JoinKind::TrustCenter {
            self.trust_center = false;
        }
        *level & INVOLVE_TC != 0
    }
}

#[cfg(test)]
mod tests;
