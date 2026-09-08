//! Router application errors.

use zigbee_mac::MacError;
use zigbee_nwk::DeviceType;
use zigbee_runtime::aps_table_store::ApsTableStoreError;
use zigbee_runtime::child_store::ChildStoreError;
use zigbee_runtime::event_loop::StartError;
use zigbee_runtime::node::NodeError;
use zigbee_runtime::security_store::SecurityStoreError;
#[cfg(feature = "trust-center")]
use zigbee_runtime::trust_center_runtime::TrustCenterRuntimeError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouterAppError {
    InvalidPolicy,
    /// An always-on lifecycle was given a sleepy or receiver-off device.
    NotAlwaysOnDevice,
    /// The selected APS-table lifecycle requires receiving application link
    /// keys, but this product did not select that optional capability.
    ApplicationLinkKeyInstallationUnavailable,
    WrongDeviceType {
        expected: DeviceType,
        actual: DeviceType,
    },
    AlreadyInitialized,
    NotInitialized,
    InvalidRunAgainDelay {
        delay_ms: u32,
    },
    Start(StartError),
    Node(NodeError),
    Security(SecurityStoreError),
    ApsTables(ApsTableStoreError),
    ChildStore(ChildStoreError),
    #[cfg(feature = "trust-center")]
    TrustCenter(TrustCenterRuntimeError),
    Mac(MacError),
}

impl From<NodeError> for RouterAppError {
    fn from(error: NodeError) -> Self {
        Self::Node(error)
    }
}

impl From<SecurityStoreError> for RouterAppError {
    fn from(error: SecurityStoreError) -> Self {
        Self::Security(error)
    }
}

impl From<ChildStoreError> for RouterAppError {
    fn from(error: ChildStoreError) -> Self {
        Self::ChildStore(error)
    }
}

impl From<ApsTableStoreError> for RouterAppError {
    fn from(error: ApsTableStoreError) -> Self {
        Self::ApsTables(error)
    }
}

impl From<MacError> for RouterAppError {
    fn from(error: MacError) -> Self {
        Self::Mac(error)
    }
}

#[cfg(feature = "trust-center")]
impl From<TrustCenterRuntimeError> for RouterAppError {
    fn from(error: TrustCenterRuntimeError) -> Self {
        Self::TrustCenter(error)
    }
}
