//! Lab only (`lab-gp-translation`, never a product image): exercise the GP
//! sink's Pairing Configuration and Translation Table on a combo light that
//! is already in a network and already has the lab GPD in its Sink Table,
//! without a commissioning tool.
//!
//! Ten seconds after boot, once joined, the image issues locally, through
//! [`zigbee_runtime::ZigbeeDevice::gp_sink_configure_locally`]:
//!
//! * a GP Pairing Configuration (Extend) rebuilt from the GPD's current Sink
//!   Table entry (same mode, alias, key and frame counter, paired endpoints
//!   0xFE), and
//! * a GP Translation Table Update that retargets the GPD's Toggle (0x22) to
//!   On/Off On (0x01) on the light endpoint, so repeated Toggles leave the
//!   light on.
//!
//! Both are journaled like remote commands. When the translation already
//! exists (restored from the GP journal after a reset) nothing is sent: the
//! next Toggle shows whether the persisted translation still applies.
//! `lab-gp-translation-remove` builds the clean-up variant, which removes
//! the translation instead.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_time::Instant;
use zigbee_gp::cluster::server_cmd;
use zigbee_gp::config::{
    pairing_action, tt_action, PairingConfiguration, PairingSecurity, TranslationRecord,
    TranslationTableUpdate, TtPayload,
};
use zigbee_gp::types::GpdId;

use super::App;
use esp32_zigbee_devkit_product as product;

/// The lab GPD: the EFR32MG22 GP switch.
const GPD: GpdId = GpdId::SrcId(0x7A6B_9879);
/// GPD Toggle.
const GPD_TOGGLE: u8 = 0x22;
/// Let the stack resume and restore the GP journal first (seconds after
/// boot).
const SETTLE_S: u64 = 10;
/// The clean-up variant.
const REMOVE: bool = cfg!(feature = "lab-gp-translation-remove");

const TOGGLE_TO_ON: TranslationRecord<'static> = TranslationRecord {
    index: 0xFF,
    gpd_command: GPD_TOGGLE,
    endpoint: product::router::COMBO_SINK_ENDPOINT,
    profile: 0x0104,
    cluster: 0x0006,
    zcl_command: 0x01,
    payload: TtPayload::Fixed(&[]),
    additional_info: None,
};

/// The lab step ran (once per boot).
static DONE: AtomicBool = AtomicBool::new(false);

/// Call once per run-loop step; acts once, [`SETTLE_S`] after boot.
#[inline(never)]
pub fn service(app: &mut App) {
    if DONE.load(Ordering::Relaxed) || Instant::now() < Instant::from_secs(SETTLE_S) {
        return;
    }
    let device = app.node_mut().device_mut();
    if !device.is_joined() {
        return;
    }
    DONE.store(true, Ordering::Relaxed);
    let present = device
        .gp_sink()
        .translations()
        .iter()
        .any(|t| t.gpd_id == GPD && t.gpd_command == GPD_TOGGLE);
    log::info!(
            "[LAB] GP translation ({}): sink entries {}, translations {}, Toggle translation present {}",
            if REMOVE { "remove" } else { "apply" },
            device.gp_sink().entries().len(),
            device.gp_sink().translations().len(),
            present
        );
    let mut buf = [0u8; 96];
    if REMOVE {
        if present {
            let Ok(n) =
                TranslationTableUpdate::encode(&GPD, tt_action::REMOVE, &[TOGGLE_TO_ON], &mut buf)
            else {
                log::error!("[LAB] TT Update encode failed");
                return;
            };
            let sent =
                device.gp_sink_configure_locally(server_cmd::TRANSLATION_TABLE_UPDATE, &buf[..n]);
            log::info!("[LAB] TT Update remove issued {}", sent);
        }
    } else if !present {
        let Some(entry) = device.gp_sink().entries().iter().find(|e| e.gpd_id == GPD) else {
            log::warn!(
                "[LAB] {:?} is not in the Sink Table; nothing to configure",
                GPD
            );
            return;
        };
        let security = entry.key.map(|key| PairingSecurity {
            level: entry.security_level,
            key_type: entry.key_type,
            key,
        });
        let pc = PairingConfiguration {
            actions: pairing_action::EXTEND,
            gpd_id: GPD,
            communication_mode: entry.communication_mode,
            mac_seq_cap: entry.mac_seq_cap,
            rx_on_cap: entry.rx_on_cap,
            fixed_location: entry.fixed_location,
            device_id: entry.device_id,
            groups: &[],
            assigned_alias: entry.assigned_alias,
            groupcast_radius: entry.groupcast_radius,
            frame_counter: (security.is_some() || entry.mac_seq_cap).then_some(entry.frame_counter),
            security,
            number_of_paired_endpoints: 0xFE,
            paired_endpoints: &[],
            application_information: None,
        };
        log::info!(
            "[LAB] Sink Table entry: mode {:?} level {:?} counter {} alias {:?}",
            entry.communication_mode,
            entry.security_level,
            entry.frame_counter,
            entry.assigned_alias
        );
        match pc.encode(&mut buf) {
            Ok(n) => {
                let sent =
                    device.gp_sink_configure_locally(server_cmd::PAIRING_CONFIGURATION, &buf[..n]);
                log::info!("[LAB] Pairing Configuration (Extend) issued {}", sent);
            }
            Err(error) => {
                log::error!("[LAB] Pairing Configuration encode failed: {:?}", error);
                return;
            }
        }
        match TranslationTableUpdate::encode(&GPD, tt_action::ADD, &[TOGGLE_TO_ON], &mut buf) {
            Ok(n) => {
                let sent = device
                    .gp_sink_configure_locally(server_cmd::TRANSLATION_TABLE_UPDATE, &buf[..n]);
                log::info!("[LAB] TT Update add (Toggle -> On) issued {}", sent);
            }
            Err(error) => log::error!("[LAB] TT Update encode failed: {:?}", error),
        }
    }
    for t in device.gp_sink().translations() {
        log::info!("[LAB] translation {:?}", t);
    }
    log::info!(
        "[LAB] translations now {}",
        device.gp_sink().translations().len()
    );
}
