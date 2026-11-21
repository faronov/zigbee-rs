//! HSE (32 MHz crystal) load-capacitor tuning from the factory OTP record.
//!
//! The 32 MHz HSE clocks both the 2.4 GHz radio and (through the PLL) the
//! system clock. Its frequency depends on the crystal load capacitance,
//! which the STM32WB provides on-chip (RM0434 "HSE clock"; AN5042 "HSE
//! trimming for RF applications"). ST measures every board at production
//! and stores the matching `RCC_HSECR.HSETUNE` value in the OTP area; ST's
//! applications apply it before `SystemClock_Config` enables HSE
//! (STM32CubeWB `app_entry.c`, `Config_HSE()`):
//!
//! ```c
//! OTP_ID0_t *p = (OTP_ID0_t *)OTP_Read(0);
//! if (p) LL_RCC_HSE_SetCapacitorTuning(p->hse_tuning);
//! ```
//!
//! # OTP format (STM32CubeWB `utilities/otp.{c,h}`)
//!
//! The OTP area is 1 KiB at `0x1FFF_7000` (`OTP_AREA_BASE` ..
//! `OTP_AREA_END_ADDR`), programmed in 64-bit double words. Each 8-byte
//! record carries its id in the last byte; `OTP_Read(id)` scans from the
//! highest record down to the base and returns the first match, so the
//! **last** record written with an id wins (records are appended upwards).
//! Record id 0 is `OTP_ID0_t { bd_address[6], hse_tuning, id }`.
//! Erased OTP reads `0xFF`, so an erased slot never matches id 0.
//!
//! [`find_id0`] is that search; unlike `OTP_Read` it also rejects a
//! `hse_tuning` outside the 6-bit `HSETUNE` field (ST's
//! `LL_RCC_HSE_SetCapacitorTuning` would shift such a value into the
//! neighbouring reserved bits). A rejected last record is not replaced by
//! an older one: the newest declaration is the one `OTP_Read` returns, so
//! an invalid one leaves the reset tuning in place.
//!
//! # `RCC_HSECR` access (RM0434 "RCC clock HSE register")
//!
//! Write access is key-protected: a word write of `0xCAFE_CAFE` unlocks the
//! register (`UNLOCKED` reads 1) for exactly one following write, after
//! which it locks again. [`program_tuning`] performs ST's sequence (key,
//! then read-modify-write of `HSETUNE[13:8]` keeping `HSES`/`HSEGMC`) but
//! polls both the unlock and the read-back within [`VERIFY_READS`]: one
//! Nucleo-WB55 boot read the old value back right after the write.
//! `HSETUNE` may be changed while HSE runs (AN5042 trims a running
//! oscillator), but like ST the composition roots apply it before
//! `embassy_stm32::init` starts HSE, so the PLL, the system clock and the
//! radio never run on the untuned crystal. A system reset restores the
//! reset value (observed: HSETUNE 0 after a debugger system reset), so the
//! tuning is applied on every boot.

/// Base of the OTP area (`OTP_AREA_BASE`).
pub const OTP_BASE: usize = 0x1FFF_7000;
/// Size of the OTP area (`OTP_AREA_END_ADDR + 1 - OTP_AREA_BASE`).
pub const OTP_SIZE: usize = 1024;
/// One OTP record: a 64-bit double word, id in the last byte.
pub const OTP_RECORD_SIZE: usize = 8;
/// Id of the `OTP_ID0_t` record (BD address + HSE tuning).
pub const OTP_ID0: u8 = 0;
/// Largest value of the 6-bit `RCC_HSECR.HSETUNE` field.
pub const HSETUNE_MAX: u8 = 0x3F;

/// `RCC_HSECR` unlock key (`HSE_CONTROL_UNLOCK_KEY`).
pub const HSECR_UNLOCK_KEY: u32 = 0xCAFE_CAFE;
/// `RCC_HSECR.UNLOCKED`.
pub const HSECR_UNLOCKED: u32 = 1 << 0;
/// `RCC_HSECR.HSETUNE[5:0]` position.
pub const HSECR_HSETUNE_POS: u32 = 8;
/// `RCC_HSECR.HSETUNE[5:0]` mask.
pub const HSECR_HSETUNE_MASK: u32 = (HSETUNE_MAX as u32) << HSECR_HSETUNE_POS;

/// The decoded factory `OTP_ID0_t` record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OtpId0 {
    /// Byte offset of the record inside the OTP area.
    pub offset: usize,
    /// Public BD address as stored (least significant byte first).
    pub bd_address: [u8; 6],
    /// `RCC_HSECR.HSETUNE` value, `0..=HSETUNE_MAX`.
    pub hse_tuning: u8,
}

/// Why the OTP does not yield a usable HSE tuning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtpError {
    /// No record with id 0 (an unprogrammed OTP).
    NoId0Record,
    /// The newest id-0 record holds a value that does not fit `HSETUNE`.
    TuningOutOfRange {
        /// Byte offset of the rejected record.
        offset: usize,
        /// The stored value.
        value: u8,
    },
}

/// Find the newest id-0 record in an OTP image, like ST's `OTP_Read(0)`.
///
/// Only whole, 8-byte-aligned records inside `otp` are considered; a
/// trailing partial record is ignored. The highest-addressed match wins.
pub fn find_id0(otp: &[u8]) -> Result<OtpId0, OtpError> {
    let records = otp.len() / OTP_RECORD_SIZE;
    for index in (0..records).rev() {
        let offset = index * OTP_RECORD_SIZE;
        let record = &otp[offset..offset + OTP_RECORD_SIZE];
        if record[7] != OTP_ID0 {
            continue;
        }
        let hse_tuning = record[6];
        if hse_tuning > HSETUNE_MAX {
            return Err(OtpError::TuningOutOfRange {
                offset,
                value: hse_tuning,
            });
        }
        let mut bd_address = [0; 6];
        bd_address.copy_from_slice(&record[..6]);
        return Ok(OtpId0 {
            offset,
            bd_address,
            hse_tuning,
        });
    }
    Err(OtpError::NoId0Record)
}

/// Raw access to `RCC_HSECR` (the chip register, or a model in tests).
pub trait HsecrPort {
    fn read(&mut self) -> u32;
    fn write(&mut self, value: u32);
}

/// `RCC_HSECR` did not take the tuning value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WriteRejected {
    /// The value that was written.
    pub wanted: u8,
    /// The `HSETUNE` value read back.
    pub read_back: u8,
}

/// Extract `HSETUNE` from an `RCC_HSECR` value.
pub const fn hsetune(hsecr: u32) -> u8 {
    ((hsecr & HSECR_HSETUNE_MASK) >> HSECR_HSETUNE_POS) as u8
}

/// Bounded reads of `RCC_HSECR` while waiting for `UNLOCKED` after the
/// key, and for the new `HSETUNE` after the write. On the Nucleo-WB55 one
/// boot logged a first read-back that still showed the old `HSETUNE` while
/// a later debugger read showed the new value; four later boots saw both
/// on the first read. The sequence therefore never relies on one read.
pub const VERIFY_READS: u32 = 1000;

/// A successful [`program_tuning`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Programmed {
    /// `HSETUNE` before the write.
    pub previous: u8,
    /// Reads until `UNLOCKED` showed after the key (1 = the first;
    /// [`VERIFY_READS`] + 1 = never, the value was written anyway, as ST
    /// does).
    pub unlock_reads: u32,
    /// Reads until `HSETUNE` showed the new value (1 = the first).
    pub verify_reads: u32,
}

/// Program `HSETUNE` with ST's unlock sequence (key, then one
/// read-modify-write) and verify it. Both the unlock and the result are
/// polled for at most [`VERIFY_READS`] reads. Other `RCC_HSECR` fields are
/// kept.
pub fn program_tuning<P: HsecrPort>(port: &mut P, tuning: u8) -> Result<Programmed, WriteRejected> {
    let tuning = tuning & HSETUNE_MAX;
    let previous = hsetune(port.read());
    port.write(HSECR_UNLOCK_KEY);
    let mut unlock_reads = 1;
    let mut current = port.read();
    while current & HSECR_UNLOCKED == 0 && unlock_reads <= VERIFY_READS {
        unlock_reads += 1;
        current = port.read();
    }
    port.write((current & !HSECR_HSETUNE_MASK) | (u32::from(tuning) << HSECR_HSETUNE_POS));
    let mut read_back = previous;
    for verify_reads in 1..=VERIFY_READS {
        read_back = hsetune(port.read());
        if read_back == tuning {
            return Ok(Programmed {
                previous,
                unlock_reads,
                verify_reads,
            });
        }
    }
    Err(WriteRejected {
        wanted: tuning,
        read_back,
    })
}

/// What [`apply_otp_tuning`] did; log it once the logger runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HseTuning {
    /// The factory value is now in `HSETUNE`.
    Applied {
        /// The value applied.
        tuning: u8,
        /// `HSETUNE` before (the reset value after a reset).
        previous: u8,
        /// HSE was already on or ready when the value was applied (not the
        /// case when called before `embassy_stm32::init`).
        hse_was_running: bool,
        /// Reads until `UNLOCKED` showed after the key.
        unlock_reads: u32,
        /// Read-backs until the register showed the value.
        verify_reads: u32,
    },
    /// The OTP holds no id-0 record; `HSETUNE` keeps its reset value.
    NoOtpRecord {
        /// The `HSETUNE` value left in place.
        current: u8,
    },
}

/// Why the factory tuning could not be applied; `HSETUNE` keeps its
/// previous value and HSE still works, only less accurately.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HseTuneError {
    /// The newest id-0 record holds a value that does not fit `HSETUNE`.
    OutOfRange {
        /// Byte offset of the record inside the OTP area.
        offset: u16,
        /// The stored value.
        value: u8,
    },
    /// `RCC_HSECR` did not accept the value.
    WriteRejected {
        /// The value that was written.
        wanted: u8,
        /// The `HSETUNE` value read back.
        read_back: u8,
    },
}

#[cfg(feature = "_chip")]
struct ChipHsecr;

#[cfg(feature = "_chip")]
impl HsecrPort for ChipHsecr {
    fn read(&mut self) -> u32 {
        embassy_stm32::pac::RCC.hsecr().read().0
    }

    fn write(&mut self, value: u32) {
        embassy_stm32::pac::RCC
            .hsecr()
            .write_value(embassy_stm32::pac::rcc::regs::Hsecr(value));
    }
}

/// Apply the factory HSE load-capacitor tuning from OTP (ST's
/// `Config_HSE()`). Call first thing in `main`, before
/// `embassy_stm32::init` enables HSE; it only touches `RCC_HSECR` and reads
/// the OTP and `RCC_CR`, so it needs no clock or driver set-up.
#[cfg(feature = "_chip")]
pub fn apply_otp_tuning() -> Result<HseTuning, HseTuneError> {
    // SAFETY: the OTP area is always-mapped, read-only system memory of
    // OTP_SIZE bytes; nothing writes it while this slice lives.
    let otp = unsafe { core::slice::from_raw_parts(OTP_BASE as *const u8, OTP_SIZE) };
    let mut port = ChipHsecr;
    let record = match find_id0(otp) {
        Ok(record) => record,
        Err(OtpError::NoId0Record) => {
            return Ok(HseTuning::NoOtpRecord {
                current: hsetune(port.read()),
            });
        }
        Err(OtpError::TuningOutOfRange { offset, value }) => {
            return Err(HseTuneError::OutOfRange {
                offset: offset as u16,
                value,
            });
        }
    };
    let cr = embassy_stm32::pac::RCC.cr().read();
    let hse_was_running = cr.hseon() || cr.hserdy();
    let programmed = program_tuning(&mut port, record.hse_tuning).map_err(|rejected| {
        HseTuneError::WriteRejected {
            wanted: rejected.wanted,
            read_back: rejected.read_back,
        }
    })?;
    Ok(HseTuning::Applied {
        tuning: record.hse_tuning,
        previous: programmed.previous,
        hse_was_running,
        unlock_reads: programmed.unlock_reads,
        verify_reads: programmed.verify_reads,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The Nucleo-WB55 (MB1355C) lab board's factory OTP, read over SWD:
    /// one id-0 record, BD address 00:80:E1:05:13:55, `hse_tuning` 0x14.
    const NUCLEO_RECORD: [u8; 8] = [0x55, 0x13, 0x05, 0xE1, 0x80, 0x00, 0x14, 0x00];

    fn erased() -> [u8; OTP_SIZE] {
        [0xFF; OTP_SIZE]
    }

    fn put(otp: &mut [u8], index: usize, record: [u8; 8]) {
        otp[index * 8..index * 8 + 8].copy_from_slice(&record);
    }

    #[test]
    fn the_factory_record_is_decoded() {
        let mut otp = erased();
        put(&mut otp, 0, NUCLEO_RECORD);
        assert_eq!(
            find_id0(&otp),
            Ok(OtpId0 {
                offset: 0,
                bd_address: [0x55, 0x13, 0x05, 0xE1, 0x80, 0x00],
                hse_tuning: 0x14,
            })
        );
    }

    #[test]
    fn the_last_id0_record_wins() {
        let mut otp = erased();
        put(&mut otp, 0, NUCLEO_RECORD);
        // A user record with another id in between does not count.
        put(&mut otp, 1, [1, 2, 3, 4, 5, 6, 7, 0x01]);
        put(
            &mut otp,
            2,
            [0x55, 0x13, 0x05, 0xE1, 0x80, 0x00, 0x18, 0x00],
        );
        put(&mut otp, 3, [9, 9, 9, 9, 9, 9, 9, 0x02]);
        let record = find_id0(&otp).unwrap();
        assert_eq!(record.offset, 16);
        assert_eq!(record.hse_tuning, 0x18);
    }

    #[test]
    fn the_topmost_record_is_found() {
        let mut otp = erased();
        put(&mut otp, 0, NUCLEO_RECORD);
        put(&mut otp, OTP_SIZE / 8 - 1, [0, 0, 0, 0, 0, 0, 0x3F, 0x00]);
        let record = find_id0(&otp).unwrap();
        assert_eq!(record.offset, OTP_SIZE - 8);
        assert_eq!(record.hse_tuning, HSETUNE_MAX);
    }

    #[test]
    fn erased_otp_has_no_record() {
        assert_eq!(find_id0(&erased()), Err(OtpError::NoId0Record));
        assert_eq!(find_id0(&[]), Err(OtpError::NoId0Record));
    }

    #[test]
    fn other_ids_are_not_id0() {
        let mut otp = erased();
        put(
            &mut otp,
            0,
            [0x55, 0x13, 0x05, 0xE1, 0x80, 0x00, 0x14, 0x01],
        );
        put(&mut otp, 1, [0, 0, 0, 0, 0, 0, 0, 0xFE]);
        assert_eq!(find_id0(&otp), Err(OtpError::NoId0Record));
    }

    #[test]
    fn an_out_of_range_newest_record_is_rejected_without_fallback() {
        let mut otp = erased();
        put(&mut otp, 0, NUCLEO_RECORD);
        put(&mut otp, 1, [0, 0, 0, 0, 0, 0, 0x40, 0x00]);
        assert_eq!(
            find_id0(&otp),
            Err(OtpError::TuningOutOfRange {
                offset: 8,
                value: 0x40
            })
        );
    }

    #[test]
    fn a_partial_trailing_record_is_ignored() {
        let mut otp = [0xFF; 12];
        otp[..8].copy_from_slice(&NUCLEO_RECORD);
        // Bytes 8..12 would read as id 0 at index 11 if not bounded.
        otp[8..12].fill(0);
        assert_eq!(find_id0(&otp).unwrap().offset, 0);
        assert_eq!(find_id0(&otp[..7]), Err(OtpError::NoId0Record));
    }

    /// `RCC_HSECR` model: writable HSES (bit 3), HSEGMC (6:4) and HSETUNE
    /// (13:8); writes count only right after the unlock key.
    struct HsecrModel {
        value: u32,
        unlocked: bool,
        writes: usize,
        ignore_tune: bool,
        /// Reads that still return the old value after a write (the
        /// silicon's delayed update).
        latency: u32,
        stale: Option<(u32, u32)>,
        /// Reads after the key that do not show `UNLOCKED` yet.
        unlock_latency: u32,
        unlock_pending: u32,
    }

    const WRITABLE: u32 = (1 << 3) | (0x7 << 4) | HSECR_HSETUNE_MASK;

    impl HsecrModel {
        fn reset() -> Self {
            Self {
                value: 0x30,
                unlocked: false,
                writes: 0,
                ignore_tune: false,
                latency: 0,
                stale: None,
                unlock_latency: 0,
                unlock_pending: 0,
            }
        }
    }

    impl HsecrPort for HsecrModel {
        fn read(&mut self) -> u32 {
            if self.unlock_pending > 0 {
                self.unlock_pending -= 1;
                return self.value;
            }
            if let Some((old, reads)) = self.stale {
                self.stale = (reads > 1).then(|| (old, reads - 1));
                return old;
            }
            self.value | u32::from(self.unlocked)
        }

        fn write(&mut self, value: u32) {
            self.writes += 1;
            if !self.unlocked {
                self.unlocked = value == HSECR_UNLOCK_KEY;
                self.unlock_pending = self.unlock_latency;
                return;
            }
            let mut mask = WRITABLE;
            if self.ignore_tune {
                mask &= !HSECR_HSETUNE_MASK;
            }
            if self.latency > 0 {
                self.stale = Some((self.value, self.latency));
            }
            self.value = (self.value & !mask) | (value & mask);
            self.unlocked = false;
        }
    }

    #[test]
    fn tuning_is_programmed_with_the_unlock_key_and_keeps_other_fields() {
        let mut model = HsecrModel::reset();
        assert_eq!(
            program_tuning(&mut model, 0x14),
            Ok(Programmed {
                previous: 0,
                unlock_reads: 1,
                verify_reads: 1
            })
        );
        assert_eq!(model.value, 0x1430, "HSEGMC = 3 kept, HSETUNE = 0x14");
        assert!(!model.unlocked, "locked again after the one write");
        assert_eq!(model.writes, 2);
        assert_eq!(program_tuning(&mut model, 0x3F).unwrap().previous, 0x14);
        assert_eq!(hsetune(model.value), 0x3F);
        assert_eq!(model.value & !HSECR_HSETUNE_MASK, 0x30);
    }

    #[test]
    fn a_delayed_register_update_is_polled_within_the_bound() {
        let mut model = HsecrModel::reset();
        model.latency = 3;
        assert_eq!(
            program_tuning(&mut model, 0x14),
            Ok(Programmed {
                previous: 0,
                unlock_reads: 1,
                verify_reads: 4
            })
        );
        let mut model = HsecrModel::reset();
        model.latency = VERIFY_READS;
        assert_eq!(
            program_tuning(&mut model, 0x14),
            Err(WriteRejected {
                wanted: 0x14,
                read_back: 0
            })
        );
    }

    #[test]
    fn a_delayed_unlock_is_polled_before_the_write() {
        let mut model = HsecrModel::reset();
        model.unlock_latency = 2;
        assert_eq!(
            program_tuning(&mut model, 0x14),
            Ok(Programmed {
                previous: 0,
                unlock_reads: 3,
                verify_reads: 1
            })
        );
        assert_eq!(model.value, 0x1430);
    }

    #[test]
    fn a_rejected_write_is_reported() {
        let mut model = HsecrModel::reset();
        model.ignore_tune = true;
        assert_eq!(
            program_tuning(&mut model, 0x14),
            Err(WriteRejected {
                wanted: 0x14,
                read_back: 0
            })
        );
    }
}
