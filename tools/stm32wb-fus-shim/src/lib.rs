//! Pure parts of the flash-resident STM32WB coprocessor sequencer: the
//! decision table, the flash log record format and CRC-32. They have no
//! hardware access, so the host tests cover them (`cargo test --lib
//! --target <host triple>`).

#![cfg_attr(not(test), no_std)]

/// Major.minor.sub of an ST version word (`major << 24 | minor << 16 |
/// sub << 8 | branch << 4 | type`).
pub const fn mms(v: u32) -> u32 {
    v >> 8
}

pub const V_FUS_053: u32 = 0x00_05_03;
pub const V_FUS_120: u32 = 0x01_02_00;
pub const V_MAC_124: u32 = 0x01_18_00;
/// `InfoStack` of the 802.15.4 MAC coprocessor binary (measured on the
/// STM32WB5MM-DK with MAC 1.24.0).
pub const INFO_STACK_MAC: u32 = 0x40;
/// Secure flash start page after MAC 1.24.0 on a 1 MiB part (DK, Nucleo).
pub const SFSA_MAC: u32 = 0xE1;

pub const FLASH_BASE: u32 = 0x0800_0000;
pub const PAGE: u32 = 0x1000;

/// Staged ST images: (address, length, CRC-32 of the file). The addresses
/// are the hwlab DK precedent (FUS 0x8000 below SFSA 0xCB) and ST's MAC
/// install address for a FUS-only 1 MiB part.
pub const FUS_STAGE: u32 = 0x080C_3000;
pub const FUS_LEN: u32 = 24_492;
pub const CRC_FUS_FOR_053: u32 = 0x5905_719D; // stm32wb5x_FUS_fw_for_fus_0_5_3.bin
pub const CRC_FUS_120: u32 = 0x3998_DD30; // stm32wb5x_FUS_fw_1_2_0.bin
pub const MAC_STAGE: u32 = 0x080E_1000;
pub const MAC_LEN: u32 = 74_100;
pub const CRC_MAC_124: u32 = 0x3683_569C; // stm32wb5x_Mac_802_15_4_fw.bin

/// One flash page of log records, below every staging area.
pub const LOG: u32 = 0x080C_0000;
pub const LOG_SLOTS: u32 = PAGE / 8;

pub const MAX_BOOTS: u16 = 48;
pub const MAX_ENTER_FUS: u16 = 6;
pub const MAX_FUS_UPGRADES: u16 = 3;
pub const MAX_DELETES: u16 = 2;
pub const MAX_INSTALLS: u16 = 2;
pub const MAX_STARTS: u16 = 3;
pub const MAX_RESETS: u16 = 2;

pub const FUS_STATE_IDLE: u8 = 0x00;
pub const FUS_STATE_ERROR: u8 = 0xFF;
/// FUS error code on the first FUS start in a part's life (AN5185): one
/// reset initialises the FUS.
pub const FUS_ERR_UNKNOWN: u8 = 0xFF;

/// Which coprocessor firmware announced itself.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ready {
    WirelessStack,
    Fus,
}

/// Content of the FUS staging area.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Staged {
    Empty,
    FusFor053,
    Fus120,
    Other,
}

/// What the sequencer saw at this boot.
#[derive(Clone, Copy, Debug)]
pub struct Observed {
    pub ready: Ready,
    /// FLASH_SFR.SFSA.
    pub sfsa: u32,
    pub fus_version: u32,
    pub stack_version: u32,
    /// Wireless-stack layout only.
    pub info_stack: u32,
    /// FUS layout only: FUS_GET_STATE once the FUS stopped being busy.
    pub fus_state: u8,
    pub fus_error: u8,
    pub fus_stage: Staged,
    /// The MAC image lies wholly below the secure start and matches its CRC.
    pub mac_staged: bool,
    pub counts: Counts,
}

/// Actions already started, from the flash log.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counts {
    pub boots: u16,
    pub enter_fus: u16,
    pub fus_upgrades: u16,
    pub deletes: u16,
    pub installs: u16,
    pub starts: u16,
    pub resets: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Action {
    EnterFus = 1,
    UpgradeFus = 2,
    DeleteStack = 3,
    InstallMac = 4,
    StartWs = 5,
    /// A plain system reset (the FUS's first start).
    Reset = 6,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Stop {
    /// MAC 1.24.0 runs; nothing left to do.
    Done = 0,
    /// FUS >= 1.2.0 and no stack: stage the MAC image over DFU.
    WaitMacStaging = 1,
    /// FUS < 1.2.0 and the staged FUS image is not the one for it.
    NeedFusImage = 2,
    /// An action was started too often.
    Cap = 3,
    /// A state the table does not cover.
    Unexpected = 4,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    Act(Action),
    Stop(Stop),
}

/// The whole update is this table. It only looks at what the coprocessor
/// and the flash report now, so any reset (FUS, power, button) resumes it.
pub fn decide(o: &Observed) -> Decision {
    use Decision::{Act, Stop as S};
    let c = &o.counts;
    match o.ready {
        Ready::WirelessStack => {
            if o.info_stack == INFO_STACK_MAC
                && mms(o.stack_version) == V_MAC_124
                && o.sfsa == SFSA_MAC
            {
                S(Stop::Done)
            } else if c.enter_fus >= MAX_ENTER_FUS {
                S(Stop::Cap)
            } else {
                Act(Action::EnterFus)
            }
        }
        Ready::Fus => {
            if o.fus_state == FUS_STATE_ERROR {
                if o.fus_error == FUS_ERR_UNKNOWN && c.resets < MAX_RESETS {
                    Act(Action::Reset)
                } else {
                    S(Stop::Unexpected)
                }
            } else if o.fus_state != FUS_STATE_IDLE {
                S(Stop::Unexpected)
            } else if mms(o.fus_version) < V_FUS_120 {
                let need = if mms(o.fus_version) == V_FUS_053 {
                    Staged::FusFor053
                } else {
                    Staged::Fus120
                };
                if c.fus_upgrades >= MAX_FUS_UPGRADES {
                    S(Stop::Cap)
                } else if o.fus_stage == need {
                    Act(Action::UpgradeFus)
                } else {
                    S(Stop::NeedFusImage)
                }
            } else if o.stack_version != 0 {
                if mms(o.stack_version) == V_MAC_124 {
                    if o.sfsa != SFSA_MAC {
                        S(Stop::Unexpected)
                    } else if c.starts >= MAX_STARTS {
                        S(Stop::Cap)
                    } else {
                        Act(Action::StartWs)
                    }
                } else if c.deletes >= MAX_DELETES {
                    S(Stop::Cap)
                } else {
                    Act(Action::DeleteStack)
                }
            } else if o.fus_stage != Staged::Empty || !o.mac_staged {
                // A FUS image still staged would be found by the next
                // FUS_FW_UPGRADE; the DFU round erases it and stages the MAC.
                S(Stop::WaitMacStaging)
            } else if c.installs >= MAX_INSTALLS {
                S(Stop::Cap)
            } else {
                Act(Action::InstallMac)
            }
        }
    }
}

// --- flash log ----------------------------------------------------------------

/// Low byte of every record's first word.
pub const TAG: u32 = 0x5A;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Kind {
    /// arg: boot number, val: RCC_CSR.
    Boot = 1,
    /// arg: ready code | SFSA << 8, val: device-info word 0.
    Ready = 2,
    /// arg: FUS staging class | MAC staged << 8, val: FUS version.
    FusVersion = 3,
    /// arg: InfoStack, val: stack version.
    StackVersion = 4,
    /// arg: state | error << 8, val: elapsed ms of the poll.
    State = 5,
    /// arg: action, val: 0. Written before the action runs.
    Action = 6,
    /// arg: stop reason, val: detail.
    Stop = 7,
    /// arg: error code, val: detail.
    Error = 8,
}

pub const fn encode(kind: Kind, arg: u16, val: u32) -> (u32, u32) {
    (TAG | (kind as u32) << 8 | (arg as u32) << 16, val)
}

/// (kind, arg, val) of a written record; None for an erased slot or a
/// foreign word.
pub fn decode(w0: u32, w1: u32) -> Option<(u8, u16, u32)> {
    if w0 & 0xFF != TAG {
        return None;
    }
    let _ = w1;
    Some(((w0 >> 8) as u8, (w0 >> 16) as u16, w1))
}

pub const fn is_erased(w0: u32, w1: u32) -> bool {
    w0 == u32::MAX && w1 == u32::MAX
}

/// Counts from the records in order; also returns the index of the first
/// erased slot (None when the page is full or holds a foreign word).
pub fn scan(words: impl Iterator<Item = (u32, u32)>) -> (Counts, Option<u32>) {
    let mut c = Counts::default();
    for (i, (w0, w1)) in words.enumerate() {
        if is_erased(w0, w1) {
            return (c, Some(i as u32));
        }
        let Some((kind, arg, _)) = decode(w0, w1) else {
            return (c, None);
        };
        if kind == Kind::Boot as u8 {
            c.boots = c.boots.saturating_add(1);
        } else if kind == Kind::Action as u8 {
            let n = match arg {
                1 => &mut c.enter_fus,
                2 => &mut c.fus_upgrades,
                3 => &mut c.deletes,
                4 => &mut c.installs,
                5 => &mut c.starts,
                6 => &mut c.resets,
                _ => continue,
            };
            *n = n.saturating_add(1);
        }
    }
    (c, None)
}

// --- CRC-32 (IEEE, as zlib.crc32) ----------------------------------------------

pub fn crc32_update(mut crc: u32, bytes: impl Iterator<Item = u8>) -> u32 {
    for b in bytes {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xEDB8_8320 & (crc & 1).wrapping_neg());
        }
    }
    crc
}

pub fn crc32(bytes: impl Iterator<Item = u8>) -> u32 {
    !crc32_update(!0, bytes)
}

/// Classifies the FUS staging bytes.
pub fn classify_fus_stage(bytes: impl Iterator<Item = u8> + Clone) -> Staged {
    if bytes.clone().all(|b| b == 0xFF) {
        return Staged::Empty;
    }
    match crc32(bytes) {
        CRC_FUS_FOR_053 => Staged::FusFor053,
        CRC_FUS_120 => Staged::Fus120,
        _ => Staged::Other,
    }
}

const _: () = assert!(FUS_STAGE + FUS_LEN <= 0x080C_B000);
const _: () = assert!(LOG + PAGE <= FUS_STAGE);
const _: () = assert!(MAC_STAGE.is_multiple_of(PAGE) && FUS_STAGE.is_multiple_of(PAGE));
// The FUS installs a stack where it was downloaded: MAC 1.24.0 at its
// install address ends up with SFSA 0xE1, the router layout's boundary.
const _: () = assert!(MAC_STAGE == FLASH_BASE + SFSA_MAC * PAGE);

#[cfg(test)]
mod tests {
    use super::*;

    fn obs(ready: Ready) -> Observed {
        Observed {
            ready,
            sfsa: 0xCB,
            fus_version: 0x0005_0300,
            stack_version: 0x0100_0043,
            info_stack: 0x01,
            fus_state: FUS_STATE_IDLE,
            fus_error: 0,
            fus_stage: Staged::FusFor053,
            mac_staged: false,
            counts: Counts::default(),
        }
    }

    #[test]
    fn crc_matches_zlib() {
        assert_eq!(crc32(b"123456789".iter().copied()), 0xCBF4_3926);
        assert_eq!(crc32([].into_iter()), 0);
    }

    #[test]
    fn image_crcs_match_the_staged_files() {
        let Some(home) = std::env::var_os("HOME") else {
            return;
        };
        let dir = std::path::Path::new(&home).join(".hwlab/artifacts/stm32wb-copro");
        for (f, len, crc) in [
            (
                "stm32wb5x_FUS_fw_for_fus_0_5_3.bin",
                FUS_LEN,
                CRC_FUS_FOR_053,
            ),
            ("stm32wb5x_FUS_fw_1_2_0.bin", FUS_LEN, CRC_FUS_120),
            ("stm32wb5x_Mac_802_15_4_fw.bin", MAC_LEN, CRC_MAC_124),
        ] {
            let Ok(b) = std::fs::read(dir.join(f)) else {
                continue;
            };
            assert_eq!(b.len() as u32, len, "{f}");
            assert_eq!(crc32(b.iter().copied()), crc, "{f}");
        }
    }

    #[test]
    fn factory_dongle_walks_to_the_mac() {
        // Factory BLE stack running.
        let mut o = obs(Ready::WirelessStack);
        assert_eq!(decide(&o), Decision::Act(Action::EnterFus));
        // FUS 0.5.3 with the matching image.
        o.ready = Ready::Fus;
        assert_eq!(decide(&o), Decision::Act(Action::UpgradeFus));
        o.fus_stage = Staged::Fus120;
        assert_eq!(decide(&o), Decision::Stop(Stop::NeedFusImage));
        // FUS 1.2.0, BLE still installed.
        o.fus_version = 0x0102_0000;
        assert_eq!(decide(&o), Decision::Act(Action::DeleteStack));
        // No stack, stale FUS image or no MAC: wait for the DFU round.
        o.stack_version = 0;
        o.sfsa = 0xF4;
        assert_eq!(decide(&o), Decision::Stop(Stop::WaitMacStaging));
        o.fus_stage = Staged::Empty;
        assert_eq!(decide(&o), Decision::Stop(Stop::WaitMacStaging));
        o.mac_staged = true;
        assert_eq!(decide(&o), Decision::Act(Action::InstallMac));
        // Installed, FUS idle.
        o.stack_version = 0x0118_0000;
        o.sfsa = SFSA_MAC;
        assert_eq!(decide(&o), Decision::Act(Action::StartWs));
        // Running.
        o.ready = Ready::WirelessStack;
        o.info_stack = INFO_STACK_MAC;
        assert_eq!(decide(&o), Decision::Stop(Stop::Done));
    }

    #[test]
    fn first_fus_start_resets_twice_then_stops() {
        let mut o = obs(Ready::Fus);
        o.fus_state = FUS_STATE_ERROR;
        o.fus_error = FUS_ERR_UNKNOWN;
        assert_eq!(decide(&o), Decision::Act(Action::Reset));
        o.counts.resets = MAX_RESETS;
        assert_eq!(decide(&o), Decision::Stop(Stop::Unexpected));
        o.counts.resets = 0;
        o.fus_error = 0x02; // IMG_CORRUPT
        assert_eq!(decide(&o), Decision::Stop(Stop::Unexpected));
        o.fus_state = 0x20; // still busy
        assert_eq!(decide(&o), Decision::Stop(Stop::Unexpected));
    }

    #[test]
    fn fus_1x_needs_the_normal_image() {
        let mut o = obs(Ready::Fus);
        o.fus_version = 0x0101_0100;
        assert_eq!(decide(&o), Decision::Stop(Stop::NeedFusImage));
        o.fus_stage = Staged::Fus120;
        assert_eq!(decide(&o), Decision::Act(Action::UpgradeFus));
    }

    #[test]
    fn caps_stop_every_loop() {
        let mut o = obs(Ready::WirelessStack);
        o.counts.enter_fus = MAX_ENTER_FUS;
        assert_eq!(decide(&o), Decision::Stop(Stop::Cap));
        let mut o = obs(Ready::Fus);
        o.counts.fus_upgrades = MAX_FUS_UPGRADES;
        assert_eq!(decide(&o), Decision::Stop(Stop::Cap));
        o.fus_version = 0x0102_0000;
        o.counts.deletes = MAX_DELETES;
        assert_eq!(decide(&o), Decision::Stop(Stop::Cap));
        o.stack_version = 0;
        o.fus_stage = Staged::Empty;
        o.mac_staged = true;
        o.counts.installs = MAX_INSTALLS;
        assert_eq!(decide(&o), Decision::Stop(Stop::Cap));
        o.stack_version = 0x0118_0000;
        o.sfsa = SFSA_MAC;
        o.counts.starts = MAX_STARTS;
        assert_eq!(decide(&o), Decision::Stop(Stop::Cap));
    }

    #[test]
    fn mac_version_under_another_boundary_is_not_deleted() {
        let mut o = obs(Ready::Fus);
        o.fus_version = 0x0102_0000;
        o.stack_version = 0x0118_0000;
        o.sfsa = 0xB8;
        assert_eq!(decide(&o), Decision::Stop(Stop::Unexpected));
    }

    #[test]
    fn mac_running_on_the_wrong_boundary_is_not_done() {
        let mut o = obs(Ready::WirelessStack);
        o.info_stack = INFO_STACK_MAC;
        o.stack_version = 0x0118_0000;
        o.sfsa = 0xCB;
        assert_eq!(decide(&o), Decision::Act(Action::EnterFus));
    }

    #[test]
    fn log_round_trip_and_counts() {
        let recs = [
            encode(Kind::Boot, 0, 0x0C00_0000),
            encode(Kind::Action, Action::EnterFus as u16, 0),
            encode(Kind::Boot, 1, 0),
            encode(Kind::Action, Action::UpgradeFus as u16, 0),
            encode(Kind::Action, Action::UpgradeFus as u16, 0),
            encode(Kind::State, 0x00, 1234),
            (u32::MAX, u32::MAX),
            (u32::MAX, u32::MAX),
        ];
        let (c, free) = scan(recs.iter().copied());
        assert_eq!(free, Some(6));
        assert_eq!(
            c,
            Counts {
                boots: 2,
                enter_fus: 1,
                fus_upgrades: 2,
                ..Counts::default()
            }
        );
        assert_eq!(
            decode(recs[5].0, recs[5].1),
            Some((Kind::State as u8, 0, 1234))
        );
    }

    #[test]
    fn full_or_foreign_page_has_no_free_slot() {
        let full = core::iter::repeat(encode(Kind::Boot, 0, 0)).take(LOG_SLOTS as usize);
        assert_eq!(scan(full).1, None);
        let foreign = [(0x1234_5678, 0), (u32::MAX, u32::MAX)];
        assert_eq!(scan(foreign.into_iter()).1, None);
    }

    #[test]
    fn stage_classification() {
        assert_eq!(classify_fus_stage([0xFFu8; 64].into_iter()), Staged::Empty);
        assert_eq!(classify_fus_stage([0u8; 64].into_iter()), Staged::Other);
    }
}
