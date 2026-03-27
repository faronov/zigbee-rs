//! Comprehensive cluster tests — exercises every cluster's handle_command(),
//! attribute store, setters, and edge cases (malformed payloads, unsupported
//! commands, capacity limits, state machines).

use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::data_types::ZclValue;
use zigbee_zcl::{ClusterId, CommandId, ZclStatus};

// ────────────────────────────────────────────────────────────────────────
// Groups cluster (0x0004)
// ────────────────────────────────────────────────────────────────────────

mod groups {
    use super::*;
    use zigbee_zcl::clusters::groups::*;

    fn gid_payload(gid: u16) -> [u8; 2] {
        gid.to_le_bytes()
    }

    #[test]
    fn cluster_id_is_groups() {
        let c = GroupsCluster::new();
        assert_eq!(c.cluster_id(), ClusterId::GROUPS);
    }

    #[test]
    fn add_group_success_and_response() {
        let mut c = GroupsCluster::new();
        let resp = c
            .handle_command(CMD_ADD_GROUP, &gid_payload(0x0001))
            .unwrap();
        assert_eq!(resp[0], ZclStatus::Success as u8);
        assert_eq!(u16::from_le_bytes([resp[1], resp[2]]), 0x0001);
    }

    #[test]
    fn add_duplicate_group_returns_duplicate_exists() {
        let mut c = GroupsCluster::new();
        c.handle_command(CMD_ADD_GROUP, &gid_payload(0x0001))
            .unwrap();
        let resp = c
            .handle_command(CMD_ADD_GROUP, &gid_payload(0x0001))
            .unwrap();
        assert_eq!(resp[0], 0x8A); // DUPLICATE_EXISTS
    }

    #[test]
    fn add_group_triggers_added_action() {
        let mut c = GroupsCluster::new();
        c.handle_command(CMD_ADD_GROUP, &gid_payload(0x0042))
            .unwrap();
        match c.take_action() {
            GroupAction::Added(gid) => assert_eq!(gid, 0x0042),
            other => panic!("expected Added, got {:?}", other),
        }
        // Second take should be None
        assert!(matches!(c.take_action(), GroupAction::None));
    }

    #[test]
    fn view_group_found_and_not_found() {
        let mut c = GroupsCluster::new();
        c.handle_command(CMD_ADD_GROUP, &gid_payload(0x000A))
            .unwrap();

        // Found
        let resp = c
            .handle_command(CMD_VIEW_GROUP, &gid_payload(0x000A))
            .unwrap();
        assert_eq!(resp[0], ZclStatus::Success as u8);

        // Not found
        let resp = c
            .handle_command(CMD_VIEW_GROUP, &gid_payload(0x00FF))
            .unwrap();
        assert_eq!(resp[0], ZclStatus::NotFound as u8);
    }

    #[test]
    fn remove_group_success_and_not_found() {
        let mut c = GroupsCluster::new();
        c.handle_command(CMD_ADD_GROUP, &gid_payload(0x0005))
            .unwrap();

        let resp = c
            .handle_command(CMD_REMOVE_GROUP, &gid_payload(0x0005))
            .unwrap();
        assert_eq!(resp[0], ZclStatus::Success as u8);
        match c.take_action() {
            GroupAction::Removed(gid) => assert_eq!(gid, 0x0005),
            other => panic!("expected Removed, got {:?}", other),
        }

        // Already removed
        let resp = c
            .handle_command(CMD_REMOVE_GROUP, &gid_payload(0x0005))
            .unwrap();
        assert_eq!(resp[0], ZclStatus::NotFound as u8);
    }

    #[test]
    fn remove_all_groups_clears_and_triggers_action() {
        let mut c = GroupsCluster::new();
        c.handle_command(CMD_ADD_GROUP, &gid_payload(0x0001))
            .unwrap();
        c.handle_command(CMD_ADD_GROUP, &gid_payload(0x0002))
            .unwrap();
        c.handle_command(CMD_ADD_GROUP, &gid_payload(0x0003))
            .unwrap();

        c.handle_command(CMD_REMOVE_ALL_GROUPS, &[]).unwrap();
        assert!(matches!(c.take_action(), GroupAction::RemovedAll));

        // Verify all gone
        let resp = c
            .handle_command(CMD_VIEW_GROUP, &gid_payload(0x0001))
            .unwrap();
        assert_eq!(resp[0], ZclStatus::NotFound as u8);
    }

    #[test]
    fn get_group_membership_all() {
        let mut c = GroupsCluster::new();
        c.handle_command(CMD_ADD_GROUP, &gid_payload(0x000A))
            .unwrap();
        c.handle_command(CMD_ADD_GROUP, &gid_payload(0x000B))
            .unwrap();

        // Empty request → return all groups
        let resp = c.handle_command(CMD_GET_GROUP_MEMBERSHIP, &[0]).unwrap();
        assert_eq!(resp[0], (MAX_GROUPS - 2) as u8); // capacity
        assert_eq!(resp[1], 2); // count
    }

    #[test]
    fn get_group_membership_filtered() {
        let mut c = GroupsCluster::new();
        c.handle_command(CMD_ADD_GROUP, &gid_payload(0x000A))
            .unwrap();
        c.handle_command(CMD_ADD_GROUP, &gid_payload(0x000B))
            .unwrap();

        // Request membership for [0x000A, 0x00FF]
        let mut payload = vec![2u8]; // count=2
        payload.extend_from_slice(&0x000Au16.to_le_bytes());
        payload.extend_from_slice(&0x00FFu16.to_le_bytes());

        let resp = c
            .handle_command(CMD_GET_GROUP_MEMBERSHIP, &payload)
            .unwrap();
        assert_eq!(resp[1], 1); // only 0x000A matched
    }

    #[test]
    fn add_group_malformed_payload() {
        let mut c = GroupsCluster::new();
        let result = c.handle_command(CMD_ADD_GROUP, &[0x01]); // only 1 byte
        assert_eq!(result, Err(ZclStatus::MalformedCommand));
    }

    #[test]
    fn add_group_if_identifying_no_response() {
        let mut c = GroupsCluster::new();
        let resp = c
            .handle_command(CMD_ADD_GROUP_IF_IDENTIFYING, &gid_payload(0x0010))
            .unwrap();
        assert!(resp.is_empty()); // no response per spec
        assert!(matches!(c.take_action(), GroupAction::None));
    }

    #[test]
    fn add_group_external_duplicate() {
        let mut c = GroupsCluster::new();
        assert_eq!(c.add_group_external(0x0001), ZclStatus::Success as u8);
        assert_eq!(c.add_group_external(0x0001), 0x8A); // DUPLICATE_EXISTS
    }

    #[test]
    fn group_table_full_returns_insufficient_space() {
        let mut c = GroupsCluster::new();
        for i in 0..MAX_GROUPS as u16 {
            c.handle_command(CMD_ADD_GROUP, &gid_payload(i + 1))
                .unwrap();
        }
        // Table is full
        let resp = c
            .handle_command(CMD_ADD_GROUP, &gid_payload(0xFFFF))
            .unwrap();
        assert_eq!(resp[0], ZclStatus::InsufficientSpace as u8);
    }

    #[test]
    fn unsupported_command_rejected() {
        let mut c = GroupsCluster::new();
        assert_eq!(
            c.handle_command(CommandId(0xFF), &[]),
            Err(ZclStatus::UnsupClusterCommand)
        );
    }
}

// ────────────────────────────────────────────────────────────────────────
// Identify cluster (0x0003)
// ────────────────────────────────────────────────────────────────────────

mod identify {
    use super::*;
    use zigbee_zcl::clusters::identify::*;

    #[test]
    fn initial_state_not_identifying() {
        let c = IdentifyCluster::new();
        assert!(!c.is_identifying());
    }

    #[test]
    fn identify_command_starts_identifying() {
        let mut c = IdentifyCluster::new();
        let time: u16 = 60;
        c.handle_command(CMD_IDENTIFY, &time.to_le_bytes()).unwrap();
        assert!(c.is_identifying());
    }

    #[test]
    fn identify_tick_counts_down() {
        let mut c = IdentifyCluster::new();
        c.handle_command(CMD_IDENTIFY, &30u16.to_le_bytes())
            .unwrap();
        assert!(c.is_identifying());

        c.tick(10);
        assert!(c.is_identifying());

        c.tick(20);
        assert!(!c.is_identifying()); // 30 - 10 - 20 = 0
    }

    #[test]
    fn tick_saturates_at_zero() {
        let mut c = IdentifyCluster::new();
        c.handle_command(CMD_IDENTIFY, &5u16.to_le_bytes()).unwrap();
        c.tick(100); // way more than 5
        assert!(!c.is_identifying());

        // Verify attribute is 0, not wrapped around
        if let Some(ZclValue::U16(t)) = c.attributes().get(ATTR_IDENTIFY_TIME.into()) {
            assert_eq!(*t, 0);
        } else {
            panic!("expected U16 attribute");
        }
    }

    #[test]
    fn identify_query_responds_when_identifying() {
        let mut c = IdentifyCluster::new();
        c.handle_command(CMD_IDENTIFY, &120u16.to_le_bytes())
            .unwrap();

        let resp = c.handle_command(CMD_IDENTIFY_QUERY, &[]).unwrap();
        assert_eq!(resp.len(), 2);
        let remaining = u16::from_le_bytes([resp[0], resp[1]]);
        assert_eq!(remaining, 120);
    }

    #[test]
    fn identify_query_empty_when_not_identifying() {
        let mut c = IdentifyCluster::new();
        let resp = c.handle_command(CMD_IDENTIFY_QUERY, &[]).unwrap();
        assert!(resp.is_empty());
    }

    #[test]
    fn trigger_effect_accepted() {
        let mut c = IdentifyCluster::new();
        let resp = c.handle_command(CMD_TRIGGER_EFFECT, &[0x01, 0x00]).unwrap();
        assert!(resp.is_empty());
    }

    #[test]
    fn identify_malformed_payload() {
        let mut c = IdentifyCluster::new();
        assert_eq!(
            c.handle_command(CMD_IDENTIFY, &[0x01]), // only 1 byte
            Err(ZclStatus::MalformedCommand)
        );
    }

    #[test]
    fn unsupported_command() {
        let mut c = IdentifyCluster::new();
        assert_eq!(
            c.handle_command(CommandId(0xFE), &[]),
            Err(ZclStatus::UnsupClusterCommand)
        );
    }
}

// ────────────────────────────────────────────────────────────────────────
// Level Control cluster (0x0008)
// ────────────────────────────────────────────────────────────────────────

mod level_control {
    use super::*;
    use zigbee_zcl::clusters::level_control::*;

    #[test]
    fn initial_level_is_zero() {
        let c = LevelControlCluster::new();
        assert_eq!(c.current_level(), 0);
    }

    #[test]
    fn move_to_level_sets_level() {
        let mut c = LevelControlCluster::new();
        // payload: level(u8) + transition_time(u16 LE)
        c.handle_command(CMD_MOVE_TO_LEVEL, &[128, 0x00, 0x00])
            .unwrap();
        assert_eq!(c.current_level(), 128);
    }

    #[test]
    fn move_to_level_with_on_off_also_sets_level() {
        let mut c = LevelControlCluster::new();
        // transition_time=10 starts a transition; tick to completion
        c.handle_command(CMD_MOVE_TO_LEVEL_WITH_ON_OFF, &[200, 0x0A, 0x00])
            .unwrap();
        c.tick(10); // complete the transition
        assert_eq!(c.current_level(), 200);
    }

    #[test]
    fn move_up_jumps_to_max() {
        let mut c = LevelControlCluster::new();
        // mode=0 (up), rate=10 — starts transition
        c.handle_command(CMD_MOVE, &[0x00, 10]).unwrap();
        c.tick(0xFFFF); // complete
        assert_eq!(c.current_level(), 0xFE);
    }

    #[test]
    fn move_down_jumps_to_min() {
        let mut c = LevelControlCluster::new();
        c.handle_command(CMD_MOVE_TO_LEVEL, &[100, 0, 0]).unwrap();
        c.handle_command(CMD_MOVE, &[0x01, 10]).unwrap(); // mode=1 (down)
        c.tick(0xFFFF); // complete
        assert_eq!(c.current_level(), 0x00);
    }

    #[test]
    fn step_up_adds_step_size() {
        let mut c = LevelControlCluster::new();
        c.handle_command(CMD_MOVE_TO_LEVEL, &[100, 0, 0]).unwrap();
        // mode=0 (up), step=20, transition=0
        c.handle_command(CMD_STEP, &[0x00, 20, 0x00, 0x00]).unwrap();
        assert_eq!(c.current_level(), 120);
    }

    #[test]
    fn step_down_subtracts_step_size() {
        let mut c = LevelControlCluster::new();
        c.handle_command(CMD_MOVE_TO_LEVEL, &[100, 0, 0]).unwrap();
        c.handle_command(CMD_STEP, &[0x01, 30, 0x00, 0x00]).unwrap();
        assert_eq!(c.current_level(), 70);
    }

    #[test]
    fn step_up_saturates_at_max() {
        let mut c = LevelControlCluster::new();
        c.handle_command(CMD_MOVE_TO_LEVEL, &[250, 0, 0]).unwrap();
        c.handle_command(CMD_STEP, &[0x00, 100, 0x00, 0x00])
            .unwrap();
        assert_eq!(c.current_level(), 0xFE); // capped at max
    }

    #[test]
    fn step_down_saturates_at_min() {
        let mut c = LevelControlCluster::new();
        c.handle_command(CMD_MOVE_TO_LEVEL, &[5, 0, 0]).unwrap();
        c.handle_command(CMD_STEP, &[0x01, 100, 0x00, 0x00])
            .unwrap();
        assert_eq!(c.current_level(), 0x01); // capped at min
    }

    #[test]
    fn stop_clears_remaining_time() {
        let mut c = LevelControlCluster::new();
        c.handle_command(CMD_STOP, &[]).unwrap();
        if let Some(ZclValue::U16(t)) = c.attributes().get(ATTR_REMAINING_TIME.into()) {
            assert_eq!(*t, 0);
        }
    }

    #[test]
    fn step_with_on_off_variant_works() {
        let mut c = LevelControlCluster::new();
        c.handle_command(CMD_MOVE_TO_LEVEL, &[50, 0, 0]).unwrap();
        c.handle_command(CMD_STEP_WITH_ON_OFF, &[0x00, 10, 0x00, 0x00])
            .unwrap();
        assert_eq!(c.current_level(), 60);
    }

    #[test]
    fn move_to_level_malformed() {
        let mut c = LevelControlCluster::new();
        assert_eq!(
            c.handle_command(CMD_MOVE_TO_LEVEL, &[128, 0x00]),
            Err(ZclStatus::MalformedCommand)
        );
    }

    #[test]
    fn step_malformed() {
        let mut c = LevelControlCluster::new();
        assert_eq!(
            c.handle_command(CMD_STEP, &[0, 10, 0]),
            Err(ZclStatus::MalformedCommand)
        );
    }
}

// ────────────────────────────────────────────────────────────────────────
// On/Off cluster (0x0006) — advanced commands
// ────────────────────────────────────────────────────────────────────────

mod on_off {
    use super::*;
    use zigbee_zcl::clusters::on_off::*;

    #[test]
    fn off_with_effect_turns_off_and_clears_global_scene() {
        let mut c = OnOffCluster::new();
        c.handle_command(CMD_ON, &[]).unwrap();
        assert!(c.is_on());

        // effect_id=0, effect_variant=0
        c.handle_command(CMD_OFF_WITH_EFFECT, &[0x00, 0x00])
            .unwrap();
        assert!(!c.is_on());

        // GlobalSceneControl should be false
        if let Some(ZclValue::Bool(v)) = c.attributes().get(ATTR_GLOBAL_SCENE_CONTROL.into()) {
            assert!(!v);
        }
    }

    #[test]
    fn on_with_recall_global_scene_restores() {
        let mut c = OnOffCluster::new();
        // Start off with global scene cleared
        c.handle_command(CMD_OFF_WITH_EFFECT, &[0x00, 0x00])
            .unwrap();

        c.handle_command(CMD_ON_WITH_RECALL_GLOBAL_SCENE, &[])
            .unwrap();
        assert!(c.is_on());
        if let Some(ZclValue::Bool(v)) = c.attributes().get(ATTR_GLOBAL_SCENE_CONTROL.into()) {
            assert!(v);
        }
    }

    #[test]
    fn on_with_timed_off_sets_timers() {
        let mut c = OnOffCluster::new();
        // control=0, on_time=100, off_wait=50
        let mut payload = vec![0x00u8];
        payload.extend_from_slice(&100u16.to_le_bytes());
        payload.extend_from_slice(&50u16.to_le_bytes());

        c.handle_command(CMD_ON_WITH_TIMED_OFF, &payload).unwrap();
        assert!(c.is_on());
        if let Some(ZclValue::U16(v)) = c.attributes().get(ATTR_ON_TIME.into()) {
            assert_eq!(*v, 100);
        }
        if let Some(ZclValue::U16(v)) = c.attributes().get(ATTR_OFF_WAIT_TIME.into()) {
            assert_eq!(*v, 50);
        }
    }

    #[test]
    fn on_with_timed_off_accept_only_when_on_rejected_when_off() {
        let mut c = OnOffCluster::new();
        assert!(!c.is_on());

        // control bit 0 set = accept only when on
        let mut payload = vec![0x01u8];
        payload.extend_from_slice(&100u16.to_le_bytes());
        payload.extend_from_slice(&50u16.to_le_bytes());

        c.handle_command(CMD_ON_WITH_TIMED_OFF, &payload).unwrap();
        assert!(!c.is_on()); // should still be off
    }

    #[test]
    fn on_with_timed_off_accept_only_when_on_accepted_when_on() {
        let mut c = OnOffCluster::new();
        c.handle_command(CMD_ON, &[]).unwrap();

        let mut payload = vec![0x01u8]; // accept only when on
        payload.extend_from_slice(&200u16.to_le_bytes());
        payload.extend_from_slice(&100u16.to_le_bytes());

        c.handle_command(CMD_ON_WITH_TIMED_OFF, &payload).unwrap();
        assert!(c.is_on());
        if let Some(ZclValue::U16(v)) = c.attributes().get(ATTR_ON_TIME.into()) {
            assert_eq!(*v, 200);
        }
    }

    #[test]
    fn off_with_effect_malformed() {
        let mut c = OnOffCluster::new();
        assert_eq!(
            c.handle_command(CMD_OFF_WITH_EFFECT, &[0x00]),
            Err(ZclStatus::MalformedCommand)
        );
    }

    #[test]
    fn on_with_timed_off_malformed() {
        let mut c = OnOffCluster::new();
        assert_eq!(
            c.handle_command(CMD_ON_WITH_TIMED_OFF, &[0, 0, 0, 0]),
            Err(ZclStatus::MalformedCommand)
        );
    }

    #[test]
    fn toggle_flips_state() {
        let mut c = OnOffCluster::new();
        assert!(!c.is_on());
        c.handle_command(CMD_TOGGLE, &[]).unwrap();
        assert!(c.is_on());
        c.handle_command(CMD_TOGGLE, &[]).unwrap();
        assert!(!c.is_on());
    }
}

// ────────────────────────────────────────────────────────────────────────
// Color Control cluster (0x0300)
// ────────────────────────────────────────────────────────────────────────

mod color_control {
    use super::*;
    use zigbee_zcl::clusters::color_control::*;

    #[test]
    fn defaults() {
        let c = ColorControlCluster::new();
        assert_eq!(c.cluster_id(), ClusterId::COLOR_CONTROL);
        if let Some(ZclValue::U16(x)) = c.attributes().get(ATTR_CURRENT_X.into()) {
            assert_eq!(*x, 0x616B);
        }
        if let Some(ZclValue::Enum8(m)) = c.attributes().get(ATTR_COLOR_MODE.into()) {
            assert_eq!(*m, COLOR_MODE_XY);
        }
    }

    #[test]
    fn move_to_hue_sets_hue_and_mode() {
        let mut c = ColorControlCluster::new();
        // hue=180, direction=0, transition=0
        c.handle_command(CMD_MOVE_TO_HUE, &[180, 0, 0, 0]).unwrap();
        if let Some(ZclValue::U8(h)) = c.attributes().get(ATTR_CURRENT_HUE.into()) {
            assert_eq!(*h, 180);
        }
        if let Some(ZclValue::Enum8(m)) = c.attributes().get(ATTR_COLOR_MODE.into()) {
            assert_eq!(*m, COLOR_MODE_HUE_SAT);
        }
    }

    #[test]
    fn move_to_saturation_sets_saturation() {
        let mut c = ColorControlCluster::new();
        c.handle_command(CMD_MOVE_TO_SATURATION, &[200, 0, 0])
            .unwrap();
        if let Some(ZclValue::U8(s)) = c.attributes().get(ATTR_CURRENT_SATURATION.into()) {
            assert_eq!(*s, 200);
        }
    }

    #[test]
    fn move_to_hue_and_saturation() {
        let mut c = ColorControlCluster::new();
        c.handle_command(CMD_MOVE_TO_HUE_AND_SATURATION, &[120, 240, 0, 0])
            .unwrap();
        if let Some(ZclValue::U8(h)) = c.attributes().get(ATTR_CURRENT_HUE.into()) {
            assert_eq!(*h, 120);
        }
        if let Some(ZclValue::U8(s)) = c.attributes().get(ATTR_CURRENT_SATURATION.into()) {
            assert_eq!(*s, 240);
        }
    }

    #[test]
    fn move_to_color_sets_xy() {
        let mut c = ColorControlCluster::new();
        let x: u16 = 0x1234;
        let y: u16 = 0x5678;
        let mut payload = Vec::new();
        payload.extend_from_slice(&x.to_le_bytes());
        payload.extend_from_slice(&y.to_le_bytes());
        payload.extend_from_slice(&0u16.to_le_bytes()); // transition
        c.handle_command(CMD_MOVE_TO_COLOR, &payload).unwrap();

        if let Some(ZclValue::U16(vx)) = c.attributes().get(ATTR_CURRENT_X.into()) {
            assert_eq!(*vx, 0x1234);
        }
        if let Some(ZclValue::U16(vy)) = c.attributes().get(ATTR_CURRENT_Y.into()) {
            assert_eq!(*vy, 0x5678);
        }
        if let Some(ZclValue::Enum8(m)) = c.attributes().get(ATTR_COLOR_MODE.into()) {
            assert_eq!(*m, COLOR_MODE_XY);
        }
    }

    #[test]
    fn move_to_color_temperature() {
        let mut c = ColorControlCluster::new();
        let mireds: u16 = 370;
        let mut payload = Vec::new();
        payload.extend_from_slice(&mireds.to_le_bytes());
        payload.extend_from_slice(&0u16.to_le_bytes()); // transition
        c.handle_command(CMD_MOVE_TO_COLOR_TEMPERATURE, &payload)
            .unwrap();

        if let Some(ZclValue::U16(m)) = c.attributes().get(ATTR_COLOR_TEMPERATURE_MIREDS.into()) {
            assert_eq!(*m, 370);
        }
        if let Some(ZclValue::Enum8(m)) = c.attributes().get(ATTR_COLOR_MODE.into()) {
            assert_eq!(*m, COLOR_MODE_TEMPERATURE);
        }
    }

    #[test]
    fn enhanced_move_to_hue() {
        let mut c = ColorControlCluster::new();
        let ehue: u16 = 0xBEEF;
        let mut payload = Vec::new();
        payload.extend_from_slice(&ehue.to_le_bytes());
        payload.push(0); // direction
        payload.extend_from_slice(&0u16.to_le_bytes()); // transition
        c.handle_command(CMD_ENHANCED_MOVE_TO_HUE, &payload)
            .unwrap();

        if let Some(ZclValue::U16(h)) = c.attributes().get(ATTR_ENHANCED_CURRENT_HUE.into()) {
            assert_eq!(*h, 0xBEEF);
        }
    }

    #[test]
    fn color_loop_set_updates_flags() {
        let mut c = ColorControlCluster::new();
        // update_flags=0x07 (all), action=1, direction=1, time=50, start_hue=0
        let payload = [0x07, 1, 1, 50, 0, 0, 0];
        c.handle_command(CMD_COLOR_LOOP_SET, &payload).unwrap();

        if let Some(ZclValue::U8(a)) = c.attributes().get(ATTR_COLOR_LOOP_ACTIVE.into()) {
            assert_eq!(*a, 1);
        }
        if let Some(ZclValue::U8(d)) = c.attributes().get(ATTR_COLOR_LOOP_DIRECTION.into()) {
            assert_eq!(*d, 1);
        }
        if let Some(ZclValue::U16(t)) = c.attributes().get(ATTR_COLOR_LOOP_TIME.into()) {
            assert_eq!(*t, 50);
        }
    }

    #[test]
    fn color_loop_set_partial_flags() {
        let mut c = ColorControlCluster::new();
        // Only update direction (flag 0x02), action=0, direction=1, time=0, start=0
        let payload = [0x02, 0, 1, 0, 0, 0, 0];
        c.handle_command(CMD_COLOR_LOOP_SET, &payload).unwrap();

        // Active should still be default (0)
        if let Some(ZclValue::U8(a)) = c.attributes().get(ATTR_COLOR_LOOP_ACTIVE.into()) {
            assert_eq!(*a, 0);
        }
        if let Some(ZclValue::U8(d)) = c.attributes().get(ATTR_COLOR_LOOP_DIRECTION.into()) {
            assert_eq!(*d, 1); // updated
        }
    }

    #[test]
    fn malformed_payloads() {
        let mut c = ColorControlCluster::new();
        assert_eq!(
            c.handle_command(CMD_MOVE_TO_HUE, &[1, 2, 3]),
            Err(ZclStatus::MalformedCommand)
        );
        assert_eq!(
            c.handle_command(CMD_MOVE_TO_COLOR, &[1, 2, 3, 4, 5]),
            Err(ZclStatus::MalformedCommand)
        );
        assert_eq!(
            c.handle_command(CMD_COLOR_LOOP_SET, &[0; 6]),
            Err(ZclStatus::MalformedCommand)
        );
        assert_eq!(
            c.handle_command(CMD_STEP_COLOR_TEMPERATURE, &[0; 8]),
            Err(ZclStatus::MalformedCommand)
        );
    }

    #[test]
    fn unsupported_command() {
        let mut c = ColorControlCluster::new();
        assert_eq!(
            c.handle_command(CommandId(0xFE), &[]),
            Err(ZclStatus::UnsupClusterCommand)
        );
    }
}

// ────────────────────────────────────────────────────────────────────────
// Basic cluster (0x0000)
// ────────────────────────────────────────────────────────────────────────

mod basic {
    use super::*;
    use zigbee_zcl::clusters::basic::*;

    #[test]
    fn attributes_set_from_constructor() {
        let c = BasicCluster::new(b"TestMfr", b"Model-X", b"20260101", b"1.0.0");
        if let Some(ZclValue::CharString(s)) = c.attributes().get(ATTR_MANUFACTURER_NAME.into()) {
            assert_eq!(s.as_slice(), b"TestMfr");
        } else {
            panic!("missing ManufacturerName");
        }
        if let Some(ZclValue::CharString(s)) = c.attributes().get(ATTR_MODEL_IDENTIFIER.into()) {
            assert_eq!(s.as_slice(), b"Model-X");
        }
        if let Some(ZclValue::U8(v)) = c.attributes().get(ATTR_ZCL_VERSION.into()) {
            assert_eq!(*v, 8);
        }
    }

    #[test]
    fn set_power_source() {
        let mut c = BasicCluster::new(b"", b"", b"", b"");
        c.set_power_source(0x03); // Battery
        if let Some(ZclValue::Enum8(v)) = c.attributes().get(ATTR_POWER_SOURCE.into()) {
            assert_eq!(*v, 0x03);
        }
    }

    #[test]
    fn reset_to_factory_defaults_accepted() {
        let mut c = BasicCluster::new(b"", b"", b"", b"");
        let resp = c
            .handle_command(CMD_RESET_TO_FACTORY_DEFAULTS, &[])
            .unwrap();
        assert!(resp.is_empty());
    }

    #[test]
    fn unsupported_command() {
        let mut c = BasicCluster::new(b"", b"", b"", b"");
        assert_eq!(
            c.handle_command(CommandId(0x01), &[]),
            Err(ZclStatus::UnsupClusterCommand)
        );
    }
}

// ────────────────────────────────────────────────────────────────────────
// Measurement clusters: Humidity, Pressure, Temperature, PowerConfig
// ────────────────────────────────────────────────────────────────────────

mod measurement {
    use super::*;

    #[test]
    fn humidity_cluster_set_and_read() {
        use zigbee_zcl::clusters::humidity::*;
        let mut c = HumidityCluster::new(0, 10000);
        c.set_humidity(5500);
        if let Some(ZclValue::U16(v)) = c.attributes().get(ATTR_MEASURED_VALUE.into()) {
            assert_eq!(*v, 5500);
        }
        assert_eq!(c.cluster_id(), ClusterId::HUMIDITY);

        // No commands supported
        assert_eq!(
            c.handle_command(CommandId(0x00), &[]),
            Err(ZclStatus::UnsupClusterCommand)
        );
    }

    #[test]
    fn humidity_min_max_from_constructor() {
        use zigbee_zcl::clusters::humidity::*;
        let c = HumidityCluster::new(500, 9500);
        if let Some(ZclValue::U16(v)) = c.attributes().get(ATTR_MIN_MEASURED_VALUE.into()) {
            assert_eq!(*v, 500);
        }
        if let Some(ZclValue::U16(v)) = c.attributes().get(ATTR_MAX_MEASURED_VALUE.into()) {
            assert_eq!(*v, 9500);
        }
    }

    #[test]
    fn pressure_cluster_set_and_read() {
        use zigbee_zcl::clusters::pressure::*;
        let mut c = PressureCluster::new(300, 1100);
        c.set_pressure(1013);
        if let Some(ZclValue::I16(v)) = c.attributes().get(ATTR_MEASURED_VALUE.into()) {
            assert_eq!(*v, 1013);
        }
        assert_eq!(c.cluster_id(), ClusterId::PRESSURE);
    }

    #[test]
    fn pressure_no_commands() {
        use zigbee_zcl::clusters::pressure::*;
        let mut c = PressureCluster::new(0, 2000);
        assert_eq!(
            c.handle_command(CommandId(0x00), &[]),
            Err(ZclStatus::UnsupClusterCommand)
        );
    }

    #[test]
    fn temperature_cluster_set_and_read() {
        use zigbee_zcl::clusters::temperature::*;
        let mut c = TemperatureCluster::new(-4000, 12500);
        c.set_temperature(2350);
        if let Some(ZclValue::I16(v)) = c.attributes().get(ATTR_MEASURED_VALUE.into()) {
            assert_eq!(*v, 2350);
        }
        assert_eq!(c.cluster_id(), ClusterId::TEMPERATURE);
    }

    #[test]
    fn power_config_battery_setters() {
        use zigbee_zcl::clusters::power_config::*;
        let mut c = PowerConfigCluster::new();
        assert_eq!(c.cluster_id(), ClusterId::POWER_CONFIG);

        c.set_battery_voltage(33); // 3.3V
        c.set_battery_percentage(200); // 100%
        c.set_battery_size(3); // AA
        c.set_battery_quantity(2);
        c.set_battery_rated_voltage(15); // 1.5V
        c.set_battery_voltage_min_threshold(10); // 1.0V

        if let Some(ZclValue::U8(v)) = c.attributes().get(ATTR_BATTERY_VOLTAGE.into()) {
            assert_eq!(*v, 33);
        }
        if let Some(ZclValue::U8(v)) = c.attributes().get(ATTR_BATTERY_PERCENTAGE_REMAINING.into())
        {
            assert_eq!(*v, 200);
        }
        if let Some(ZclValue::Enum8(v)) = c.attributes().get(ATTR_BATTERY_SIZE.into()) {
            assert_eq!(*v, 3);
        }
        if let Some(ZclValue::U8(v)) = c.attributes().get(ATTR_BATTERY_QUANTITY.into()) {
            assert_eq!(*v, 2);
        }
    }

    #[test]
    fn power_config_no_commands() {
        use zigbee_zcl::clusters::power_config::*;
        let mut c = PowerConfigCluster::new();
        assert_eq!(
            c.handle_command(CommandId(0x00), &[]),
            Err(ZclStatus::UnsupClusterCommand)
        );
    }
}

// ────────────────────────────────────────────────────────────────────────
// Default Response parsing/serialization
// ────────────────────────────────────────────────────────────────────────

mod default_response {
    use zigbee_zcl::foundation::default_response::DefaultResponse;
    use zigbee_zcl::ZclStatus;

    #[test]
    fn parse_success() {
        let data = [0x02, 0x00]; // command_id=2, status=Success
        let dr = DefaultResponse::parse(&data).unwrap();
        assert_eq!(dr.command_id, 0x02);
        assert_eq!(dr.status, ZclStatus::Success);
    }

    #[test]
    fn parse_failure_status() {
        let data = [0x0B, 0x86]; // command_id=0x0B, status=UnsupportedAttribute
        let dr = DefaultResponse::parse(&data).unwrap();
        assert_eq!(dr.command_id, 0x0B);
        assert_eq!(dr.status, ZclStatus::UnsupportedAttribute);
    }

    #[test]
    fn parse_too_short() {
        assert!(DefaultResponse::parse(&[0x01]).is_none());
        assert!(DefaultResponse::parse(&[]).is_none());
    }

    #[test]
    fn serialize_roundtrip() {
        let dr = DefaultResponse {
            command_id: 0x42,
            status: ZclStatus::MalformedCommand,
        };
        let mut buf = [0u8; 4];
        let len = dr.serialize(&mut buf);
        assert_eq!(len, 2);

        let parsed = DefaultResponse::parse(&buf[..len]).unwrap();
        assert_eq!(parsed.command_id, 0x42);
        assert_eq!(parsed.status, ZclStatus::MalformedCommand);
    }

    #[test]
    fn serialize_buffer_too_small() {
        let dr = DefaultResponse {
            command_id: 0,
            status: ZclStatus::Success,
        };
        let mut buf = [0u8; 1];
        assert_eq!(dr.serialize(&mut buf), 0);
    }
}

// ────────────────────────────────────────────────────────────────────────
// Analog/Binary Input clusters — measurement-style, no commands
// ────────────────────────────────────────────────────────────────────────

mod analog_binary {
    use super::*;

    #[test]
    fn analog_input_defaults_and_set() {
        use zigbee_zcl::clusters::analog_input::*;
        let mut c = AnalogInputCluster::new();
        assert_eq!(c.cluster_id().0, 0x000C);

        // Set present value
        c.set_present_value(42.5);
        if let Some(ZclValue::Float32(v)) = c.attributes().get(ATTR_PRESENT_VALUE.into()) {
            assert!((*v - 42.5).abs() < f32::EPSILON);
        }

        // No commands supported
        assert_eq!(
            c.handle_command(CommandId(0x00), &[]),
            Err(ZclStatus::UnsupClusterCommand)
        );
    }

    #[test]
    fn binary_input_defaults_and_set() {
        use zigbee_zcl::clusters::binary_input::*;
        let mut c = BinaryInputCluster::new();
        assert_eq!(c.cluster_id(), ClusterId::BINARY_INPUT);

        // Present value starts false
        if let Some(ZclValue::Bool(v)) = c.attributes().get(ATTR_PRESENT_VALUE.into()) {
            assert!(!v);
        }

        c.set_present_value(true);
        if let Some(ZclValue::Bool(v)) = c.attributes().get(ATTR_PRESENT_VALUE.into()) {
            assert!(v);
        }

        // No commands
        assert_eq!(
            c.handle_command(CommandId(0x00), &[]),
            Err(ZclStatus::UnsupClusterCommand)
        );
    }

    #[test]
    fn analog_output_set_and_read() {
        use zigbee_zcl::clusters::analog_output::*;
        let mut c = AnalogOutputCluster::new();
        assert_eq!(c.cluster_id(), ClusterId::ANALOG_OUTPUT);

        c.set_present_value(99.9);
        if let Some(ZclValue::Float32(v)) = c.attributes().get(ATTR_PRESENT_VALUE.into()) {
            assert!((*v - 99.9).abs() < 0.01);
        }

        assert_eq!(
            c.handle_command(CommandId(0x00), &[]),
            Err(ZclStatus::UnsupClusterCommand)
        );
    }

    #[test]
    fn binary_output_set_and_read() {
        use zigbee_zcl::clusters::binary_output::*;
        let mut c = BinaryOutputCluster::new();
        assert_eq!(c.cluster_id(), ClusterId::BINARY_OUTPUT);

        c.set_present_value(true);
        if let Some(ZclValue::Bool(v)) = c.attributes().get(ATTR_PRESENT_VALUE.into()) {
            assert!(v);
        }

        c.set_present_value(false);
        if let Some(ZclValue::Bool(v)) = c.attributes().get(ATTR_PRESENT_VALUE.into()) {
            assert!(!v);
        }
    }

    #[test]
    fn analog_value_set_and_read() {
        use zigbee_zcl::clusters::analog_value::*;
        let mut c = AnalogValueCluster::new();
        assert_eq!(c.cluster_id(), ClusterId::ANALOG_VALUE);

        c.set_present_value(-17.5);
        if let Some(ZclValue::Float32(v)) = c.attributes().get(ATTR_PRESENT_VALUE.into()) {
            assert!((*v - (-17.5)).abs() < f32::EPSILON);
        }
    }

    #[test]
    fn binary_value_set_and_read() {
        use zigbee_zcl::clusters::binary_value::*;
        let mut c = BinaryValueCluster::new();
        assert_eq!(c.cluster_id(), ClusterId::BINARY_VALUE);

        c.set_present_value(true);
        if let Some(ZclValue::Bool(v)) = c.attributes().get(ATTR_PRESENT_VALUE.into()) {
            assert!(v);
        }
    }
}

// ────────────────────────────────────────────────────────────────────────
// Environmental sensor clusters (CO2, PM2.5, Soil Moisture, Illuminance)
// ────────────────────────────────────────────────────────────────────────

mod environmental {
    use super::*;

    #[test]
    fn carbon_dioxide_set_and_read() {
        use zigbee_zcl::clusters::carbon_dioxide::*;
        let mut c = CarbonDioxideCluster::new();
        assert_eq!(c.cluster_id(), ClusterId::CARBON_DIOXIDE);

        c.set_co2_ppm(412.5);
        if let Some(ZclValue::Float32(v)) = c.attributes().get(ATTR_MEASURED_VALUE.into()) {
            assert!((*v - 412.5).abs() < f32::EPSILON);
        }

        assert_eq!(
            c.handle_command(CommandId(0x00), &[]),
            Err(ZclStatus::UnsupClusterCommand)
        );
    }

    #[test]
    fn pm25_set_and_read() {
        use zigbee_zcl::clusters::pm25::*;
        let mut c = Pm25Cluster::new();
        assert_eq!(c.cluster_id(), ClusterId::PM25_MEASUREMENT);

        c.set_pm25(35.0);
        if let Some(ZclValue::Float32(v)) = c.attributes().get(ATTR_MEASURED_VALUE.into()) {
            assert!((*v - 35.0).abs() < f32::EPSILON);
        }
    }

    #[test]
    fn soil_moisture_set_and_read() {
        use zigbee_zcl::clusters::soil_moisture::*;
        let mut c = SoilMoistureCluster::new();
        assert_eq!(c.cluster_id(), ClusterId::SOIL_MOISTURE);

        c.set_moisture(4500); // 45%
        if let Some(ZclValue::U16(v)) = c.attributes().get(ATTR_MEASURED_VALUE.into()) {
            assert_eq!(*v, 4500);
        }
    }

    #[test]
    fn illuminance_level_set_and_read() {
        use zigbee_zcl::clusters::illuminance_level::*;
        let mut c = IlluminanceLevelCluster::new();
        assert_eq!(c.cluster_id(), ClusterId::ILLUMINANCE_LEVEL_SENSING);

        c.set_level_status(2); // "on target"
        if let Some(ZclValue::U8(v)) = c.attributes().get(ATTR_LEVEL_STATUS.into()) {
            assert_eq!(*v, 2);
        }
    }
}

// ────────────────────────────────────────────────────────────────────────
// Ballast Configuration and Device Temperature Configuration
// ────────────────────────────────────────────────────────────────────────

mod config_clusters {
    use super::*;

    #[test]
    fn ballast_config_defaults_and_setter() {
        use zigbee_zcl::clusters::ballast_config::*;
        let mut c = BallastConfigCluster::new();
        assert_eq!(c.cluster_id(), ClusterId::BALLAST_CONFIG);

        // Physical min/max defaults
        if let Some(ZclValue::U8(v)) = c.attributes().get(ATTR_PHYSICAL_MIN_LEVEL.into()) {
            assert_eq!(*v, 1);
        }
        if let Some(ZclValue::U8(v)) = c.attributes().get(ATTR_PHYSICAL_MAX_LEVEL.into()) {
            assert_eq!(*v, 254);
        }

        c.set_lamp_burn_hours(5000);
        if let Some(ZclValue::U32(v)) = c.attributes().get(ATTR_LAMP_BURN_HOURS.into()) {
            assert_eq!(*v, 5000);
        }

        // burn hours is masked to 24 bits
        c.set_lamp_burn_hours(0x01FFFFFF);
        if let Some(ZclValue::U32(v)) = c.attributes().get(ATTR_LAMP_BURN_HOURS.into()) {
            assert_eq!(*v, 0x00FFFFFF);
        }

        assert_eq!(
            c.handle_command(CommandId(0x00), &[]),
            Err(ZclStatus::UnsupClusterCommand)
        );
    }

    #[test]
    fn device_temp_config_set_and_read() {
        use zigbee_zcl::clusters::device_temp_config::*;
        let mut c = DeviceTempConfigCluster::new();
        assert_eq!(c.cluster_id(), ClusterId::DEVICE_TEMP_CONFIG);

        c.set_temperature(42);
        if let Some(ZclValue::I16(v)) = c.attributes().get(ATTR_CURRENT_TEMPERATURE.into()) {
            assert_eq!(*v, 42);
        }

        assert_eq!(
            c.handle_command(CommandId(0x00), &[]),
            Err(ZclStatus::UnsupClusterCommand)
        );
    }
}
