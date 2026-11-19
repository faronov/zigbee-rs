//! Built-in GPD command → ZCL command translation of a GP sink.
//!
//! The mapping is the default translation table of the certified Silicon
//! Labs GP sink (gecko_sdk `protocol/zigbee/app/framework/plugin/
//! green-power-server/green-power-server-gpd-cmd-maps.c`,
//! `emberGpDefaultTranslationTable` and
//! `sli_zigbee_af_gp_forward_gpd_to_mapped_endpoint`): On/Off/Toggle go to
//! the On/Off cluster, the Level Control commands to the Level Control
//! cluster with a preconfigured move/step mode followed by the GPD payload.
//! Only commands with an unambiguous ZCL counterpart are translated; every
//! other command (scenes, colour, generic switch, attribute reports, ...)
//! is passed to the application untranslated.
//!
//! Per-GPD translations written by GP Translation Table Update (0x07) take
//! precedence over this mapping with the `translation-table` feature
//! ([`crate::translation_table`]).

/// GPD command identifiers handled here (GP spec Table 49; values from
/// gecko_sdk `app/zcl/green-power.xml`, enum `GpGpdf`).
pub mod gpd_cmd {
    /// Off.
    pub const OFF: u8 = 0x20;
    /// On.
    pub const ON: u8 = 0x21;
    /// Toggle.
    pub const TOGGLE: u8 = 0x22;
    /// Release.
    pub const RELEASE: u8 = 0x23;
    /// Move Up.
    pub const MOVE_UP: u8 = 0x30;
    /// Move Down.
    pub const MOVE_DOWN: u8 = 0x31;
    /// Step Up.
    pub const STEP_UP: u8 = 0x32;
    /// Step Down.
    pub const STEP_DOWN: u8 = 0x33;
    /// Level Control/Stop.
    pub const LEVEL_CONTROL_STOP: u8 = 0x34;
    /// Move Up (with On/Off).
    pub const MOVE_UP_WITH_ON_OFF: u8 = 0x35;
    /// Move Down (with On/Off).
    pub const MOVE_DOWN_WITH_ON_OFF: u8 = 0x36;
    /// Step Up (with On/Off).
    pub const STEP_UP_WITH_ON_OFF: u8 = 0x37;
    /// Step Down (with On/Off).
    pub const STEP_DOWN_WITH_ON_OFF: u8 = 0x38;
}

/// ZCL On/Off cluster.
pub const ON_OFF_CLUSTER: u16 = 0x0006;
/// ZCL Level Control cluster.
pub const LEVEL_CONTROL_CLUSTER: u16 = 0x0008;

/// Largest translated ZCL payload: mode, step size, transition time.
#[cfg(not(feature = "translation-table"))]
pub const MAX_TRANSLATED_PAYLOAD: usize = 4;
/// Largest translated ZCL payload: a Translation Table entry's fixed or
/// copied payload ([`crate::translation_table::MAX_TT_PAYLOAD`]).
#[cfg(feature = "translation-table")]
pub const MAX_TRANSLATED_PAYLOAD: usize = crate::translation_table::MAX_TT_PAYLOAD;

/// A ZCL cluster-specific command (client → server) produced from a GPD
/// command.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Translation {
    /// ZCL cluster.
    pub cluster_id: u16,
    /// ZCL cluster-specific command identifier.
    pub command_id: u8,
    /// ZCL command payload.
    pub payload: heapless::Vec<u8, MAX_TRANSLATED_PAYLOAD>,
}

fn level(command_id: u8, mode: u8, gpd_payload: &[u8], step: bool) -> Option<Translation> {
    let mut payload = heapless::Vec::new();
    payload.push(mode).ok()?;
    if step {
        // GPD Step: step size, then an optional 2-byte transition time.
        // Without it the ZCL Step gets 0xFFFF, "as fast as able" (ZCL r8
        // §3.10.2.4.3.3).
        match gpd_payload.len() {
            1 => {
                payload.push(gpd_payload[0]).ok()?;
                payload.extend_from_slice(&[0xFF, 0xFF]).ok()?;
            }
            3 => payload.extend_from_slice(gpd_payload).ok()?,
            _ => return None,
        }
    } else {
        // GPD Move: optional rate; without it 0xFF, the default rate
        // (gecko_sdk `sli_zigbee_af_gp_forward_gpd_to_mapped_endpoint`).
        match gpd_payload {
            [] => payload.push(0xFF).ok()?,
            [rate] => payload.push(*rate).ok()?,
            _ => return None,
        }
    }
    Some(Translation {
        cluster_id: LEVEL_CONTROL_CLUSTER,
        command_id,
        payload,
    })
}

/// Translate a GPD command into the ZCL command for the sink's application
/// endpoint, or `None` when it has no built-in translation (or a payload
/// that does not match the GPD command format).
pub fn translate(gpd_command: u8, gpd_payload: &[u8]) -> Option<Translation> {
    use gpd_cmd::*;
    const MOVE: u8 = 0x01;
    const STEP: u8 = 0x02;
    const STOP: u8 = 0x03;
    const MOVE_WITH_ON_OFF: u8 = 0x05;
    const STEP_WITH_ON_OFF: u8 = 0x06;
    const UP: u8 = 0x00;
    const DOWN: u8 = 0x01;
    let on_off = |command_id| {
        gpd_payload.is_empty().then(|| Translation {
            cluster_id: ON_OFF_CLUSTER,
            command_id,
            payload: heapless::Vec::new(),
        })
    };
    match gpd_command {
        OFF => on_off(0x00),
        ON => on_off(0x01),
        TOGGLE => on_off(0x02),
        RELEASE | LEVEL_CONTROL_STOP => gpd_payload.is_empty().then(|| Translation {
            cluster_id: LEVEL_CONTROL_CLUSTER,
            command_id: STOP,
            payload: heapless::Vec::new(),
        }),
        MOVE_UP => level(MOVE, UP, gpd_payload, false),
        MOVE_DOWN => level(MOVE, DOWN, gpd_payload, false),
        STEP_UP => level(STEP, UP, gpd_payload, true),
        STEP_DOWN => level(STEP, DOWN, gpd_payload, true),
        MOVE_UP_WITH_ON_OFF => level(MOVE_WITH_ON_OFF, UP, gpd_payload, false),
        MOVE_DOWN_WITH_ON_OFF => level(MOVE_WITH_ON_OFF, DOWN, gpd_payload, false),
        STEP_UP_WITH_ON_OFF => level(STEP_WITH_ON_OFF, UP, gpd_payload, true),
        STEP_DOWN_WITH_ON_OFF => level(STEP_WITH_ON_OFF, DOWN, gpd_payload, true),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(cmd: u8, payload: &[u8]) -> Option<(u16, u8, heapless::Vec<u8, MAX_TRANSLATED_PAYLOAD>)> {
        translate(cmd, payload).map(|t| (t.cluster_id, t.command_id, t.payload))
    }

    #[test]
    fn switch_commands_map_to_on_off() {
        assert_eq!(t(0x20, &[]).unwrap().0, ON_OFF_CLUSTER);
        assert_eq!(t(0x20, &[]).unwrap().1, 0x00);
        assert_eq!(t(0x21, &[]).unwrap().1, 0x01);
        assert_eq!(t(0x22, &[]).unwrap().1, 0x02);
        // Off/On/Toggle carry no payload.
        assert_eq!(t(0x22, &[0x01]), None);
    }

    #[test]
    fn level_commands_carry_mode_and_gpd_payload() {
        let (cluster, cmd, payload) = t(gpd_cmd::MOVE_DOWN, &[0x20]).unwrap();
        assert_eq!((cluster, cmd), (LEVEL_CONTROL_CLUSTER, 0x01));
        assert_eq!(&payload[..], &[0x01, 0x20]);
        // Missing rate: default rate 0xFF.
        assert_eq!(&t(gpd_cmd::MOVE_UP, &[]).unwrap().2[..], &[0x00, 0xFF]);
        assert_eq!(
            &t(gpd_cmd::STEP_UP, &[0x10, 0x0A, 0x00]).unwrap().2[..],
            &[0x00, 0x10, 0x0A, 0x00]
        );
        assert_eq!(
            &t(gpd_cmd::STEP_DOWN_WITH_ON_OFF, &[0x10]).unwrap().2[..],
            &[0x01, 0x10, 0xFF, 0xFF]
        );
        assert_eq!(t(gpd_cmd::STEP_UP_WITH_ON_OFF, &[0x10]).unwrap().1, 0x06);
        assert_eq!(t(gpd_cmd::MOVE_UP_WITH_ON_OFF, &[]).unwrap().1, 0x05);
        assert_eq!(t(gpd_cmd::LEVEL_CONTROL_STOP, &[]).unwrap().1, 0x03);
        assert_eq!(t(gpd_cmd::RELEASE, &[]).unwrap().1, 0x03);
        assert_eq!(t(gpd_cmd::STEP_UP, &[0x10, 0x00]), None);
        assert_eq!(t(gpd_cmd::MOVE_UP, &[1, 2]), None);
    }

    #[test]
    fn reports_and_unknown_commands_are_not_translated() {
        assert_eq!(t(0xA2, &[0x02, 0x04]), None);
        assert_eq!(t(0x60, &[]), None);
        assert_eq!(t(0x10, &[]), None);
    }
}
