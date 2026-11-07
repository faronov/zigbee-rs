//! Swap-move install with revert (feature `swap-move`, opt-in).
//!
//! The primary and secondary slots are exchanged sector by sector through
//! one spare sector, the primary slot's last sector, so after an install
//! the previous image sits in the secondary slot and can be swapped back.
//!
//! # Exchange of `n` sectors (`P` primary, `S` secondary)
//!
//! * move, steps `0..n`: step `k` copies `P[n-1-k]` to `P[n-k]`, shifting
//!   the primary image up by one sector;
//! * swap, steps `n..3n`: for sector `i`, step `n+2i` copies `S[i]` to
//!   `P[i]`, then step `n+2i+1` copies `P[i+1]` (the old `P[i]`) to `S[i]`.
//!
//! Every step erases its destination and copies from a sector no earlier
//! step overwrote and no later step needs intact before it runs, so a step
//! can be repeated after a power cut. The exchange is symmetric: running it
//! again with the same `n` reverts it.
//!
//! # Status area
//!
//! Two sectors outside both slots, used alternately. A sector starts with a
//! header (magic, generation, revert flag and `n`, each word followed by its
//! complement) programmed in one write, so a torn header never validates;
//! the valid header with the highest generation is the active exchange. A
//! new exchange erases the *other* sector and writes generation + 1, so the
//! active record is never erased. The header is followed by a
//! [status log](crate::status): one `Step` record per finished step, `Done`,
//! `Cleared` once the request magic was dropped, and the trial records.
//!
//! # Boot
//!
//! 1. Resume an unfinished exchange.
//! 2. After a finished install, drop the request magic (secondary trailer
//!    sector only: the secondary slot now holds the previous image).
//! 3. If no exchange ran, process a request like overwrite mode, but
//!    install by exchange.
//! 4. With trial boots configured, an unconfirmed install that used up its
//!    boots, or whose copy does not validate, is reverted when the previous
//!    image validates.
//! 5. Validate the primary and account the boot.

use embedded_storage::nor_flash::NorFlash;

use crate::boot::{
    BootConfig, BootError, BootEvent, Bootable, Layout, Request, Trial, Update, account_boot,
    bootable, copy, erase_sector, examine_request, validate_primary,
};
use crate::image::{ImageError, validate};
use crate::status::{Log, LogState, MAX_STEP, MAX_UNIT, Record, unit};

const MAGIC: [u8; 8] = *b"ZRSSWAP1";
const HEADER_LEN: usize = 24;
const REVERT: u32 = 1 << 31;
/// Log units kept free for records damaged by power cuts.
const SLACK: u32 = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Exchange {
    generation: u32,
    revert: bool,
    sectors: u32,
}

#[derive(Debug, Clone, Copy)]
struct Active {
    sector: u32,
    exchange: Exchange,
    log: Log,
    state: LogState,
}

const fn header_len<F: NorFlash>() -> u32 {
    let unit = unit::<F>();
    (HEADER_LEN as u32).div_ceil(unit) * unit
}

const fn max_sectors(layout: &Layout) -> u32 {
    layout.slot_size / layout.sector_size - 1
}

/// Check the status area against the layout; nothing is written before
/// this passes.
fn check<F: NorFlash>(config: &BootConfig<'_>, status: u32) -> Result<(), BootError> {
    let layout = &config.layout;
    let end = status + 2 * layout.sector_size;
    let overlaps = |slot: u32| status < slot + layout.slot_size && slot < end;
    let units = (layout.sector_size - header_len::<F>()) / unit::<F>();
    let need = 3 * max_sectors(layout) + 3 + u32::from(config.trial_boots) + SLACK;
    if overlaps(layout.primary)
        || overlaps(layout.secondary)
        || !status.is_multiple_of(layout.sector_size)
        || 3 * max_sectors(layout) > u32::from(MAX_STEP) + 1
        || units < need
    {
        return Err(BootError::Config);
    }
    Ok(())
}

fn read_header<F: NorFlash>(
    flash: &mut F,
    layout: &Layout,
    sector: u32,
) -> Result<Option<Exchange>, BootError> {
    let mut bytes = [0u8; HEADER_LEN];
    flash
        .read(sector, &mut bytes)
        .map_err(|_| BootError::Flash)?;
    let word =
        |at: usize| u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]]);
    let (generation, desc) = (word(8), word(12));
    let sectors = desc & !REVERT;
    Ok((bytes[..8] == MAGIC
        && word(16) == !generation
        && word(20) == !desc
        && (1..=max_sectors(layout)).contains(&sectors))
    .then_some(Exchange {
        generation,
        revert: desc & REVERT != 0,
        sectors,
    }))
}

fn log_of<F: NorFlash>(layout: &Layout, sector: u32) -> Log {
    Log {
        start: sector + header_len::<F>(),
        end: sector + layout.sector_size,
    }
}

fn read_active<F: NorFlash>(
    flash: &mut F,
    layout: &Layout,
    status: u32,
) -> Result<Option<Active>, BootError> {
    let mut best: Option<(u32, Exchange)> = None;
    for sector in [status, status + layout.sector_size] {
        if let Some(exchange) = read_header(flash, layout, sector)?
            && best.is_none_or(|(_, other)| exchange.generation > other.generation)
        {
            best = Some((sector, exchange));
        }
    }
    let Some((sector, exchange)) = best else {
        return Ok(None);
    };
    let log = log_of::<F>(layout, sector);
    let state = log.read(flash).map_err(|_| BootError::Flash)?;
    Ok(Some(Active {
        sector,
        exchange,
        log,
        state,
    }))
}

/// Start a new exchange in the inactive status sector.
fn start<F: NorFlash>(
    flash: &mut F,
    layout: &Layout,
    status: u32,
    current: Option<&Active>,
    revert: bool,
    sectors: u32,
) -> Result<Active, BootError> {
    let (sector, generation) = match current {
        Some(active) if active.sector == status => {
            (status + layout.sector_size, active.exchange.generation + 1)
        }
        Some(active) => (status, active.exchange.generation + 1),
        None => (status, 1),
    };
    erase_sector(flash, sector, layout.sector_size)?;
    let desc = sectors | if revert { REVERT } else { 0 };
    let mut bytes = [0xFFu8; HEADER_LEN + MAX_UNIT];
    bytes[..8].copy_from_slice(&MAGIC);
    for (at, word) in [(8, generation), (12, desc), (16, !generation), (20, !desc)] {
        bytes[at..at + 4].copy_from_slice(&word.to_le_bytes());
    }
    flash
        .write(sector, &bytes[..header_len::<F>() as usize])
        .map_err(|_| BootError::Flash)?;
    let log = log_of::<F>(layout, sector);
    Ok(Active {
        sector,
        exchange: Exchange {
            generation,
            revert,
            sectors,
        },
        log,
        state: LogState {
            next: log.start,
            ..LogState::default()
        },
    })
}

/// Run the remaining steps of `active` and record it done.
fn run<F: NorFlash>(
    flash: &mut F,
    layout: &Layout,
    active: &mut Active,
    progress: &mut impl FnMut(BootEvent),
) -> Result<(), BootError> {
    let size = layout.sector_size;
    let n = active.exchange.sectors;
    let total = 3 * n;
    let primary = |i: u32| layout.primary + i * size;
    let secondary = |i: u32| layout.secondary + i * size;
    for k in u32::from(active.state.steps)..total {
        let (from, to) = if k < n {
            (primary(n - 1 - k), primary(n - k))
        } else {
            let (i, odd) = ((k - n) / 2, (k - n) % 2 == 1);
            if odd {
                (primary(i + 1), secondary(i))
            } else {
                (secondary(i), primary(i))
            }
        };
        erase_sector(flash, to, size)?;
        copy(flash, from, to, size, k * size, total * size, progress)?;
        active
            .log
            .append(flash, &mut active.state, Record::Step(k as u16))?;
    }
    active.log.append(flash, &mut active.state, Record::Done)
}

/// Drop the request magic after a finished install.
fn clear<F: NorFlash>(
    flash: &mut F,
    layout: &Layout,
    active: &mut Active,
    progress: &mut impl FnMut(BootEvent),
) -> Result<(), BootError> {
    progress(BootEvent::Dropping);
    erase_sector(
        flash,
        layout.trailer_sector(layout.secondary),
        layout.sector_size,
    )?;
    active.log.append(flash, &mut active.state, Record::Cleared)
}

pub(crate) fn boot<F: NorFlash>(
    flash: &mut F,
    config: &BootConfig<'_>,
    status: u32,
    progress: &mut impl FnMut(BootEvent),
) -> Result<Bootable, BootError> {
    check::<F>(config, status)?;
    let layout = &config.layout;
    let mut current = read_active(flash, layout, status)?;
    let mut update = Update::None;
    let mut exchanged = false;

    if let Some(active) = current.as_mut()
        && !active.state.done
    {
        if active.exchange.revert {
            progress(BootEvent::Reverting);
        }
        run(flash, layout, active, progress)?;
        update = if active.exchange.revert {
            Update::Reverted
        } else {
            Update::Installed
        };
        exchanged = true;
    }
    if let Some(active) = current.as_mut()
        && !active.exchange.revert
        && !active.state.cleared
    {
        clear(flash, layout, active, progress)?;
    }

    if !exchanged {
        match examine_request(flash, config, progress)? {
            Request::None => {}
            Request::Drop(dropped) => {
                progress(BootEvent::Dropping);
                erase_sector(
                    flash,
                    layout.trailer_sector(layout.secondary),
                    layout.sector_size,
                )?;
                update = dropped;
            }
            Request::Install(candidate) => {
                let extent =
                    match validate(flash, layout.primary, layout.max_image_len(), config.keys) {
                        Ok(primary) => primary.end,
                        Err(ImageError::Flash) => return Err(BootError::Flash),
                        Err(_) => layout.max_image_len(),
                    };
                let sectors = candidate.end.max(extent).div_ceil(layout.sector_size);
                progress(BootEvent::Erasing);
                let mut active = start(flash, layout, status, current.as_ref(), false, sectors)?;
                run(flash, layout, &mut active, progress)?;
                clear(flash, layout, &mut active, progress)?;
                current = Some(active);
                update = Update::Installed;
            }
        }
    }

    if config.trial_boots != 0
        && let Some(active) = current
        && !active.exchange.revert
        && !active.state.confirmed
    {
        let expired = active.state.attempts >= u16::from(config.trial_boots);
        let broken = !expired
            && match validate(flash, layout.primary, layout.max_image_len(), config.keys) {
                Ok(_) => false,
                Err(ImageError::Flash) => return Err(BootError::Flash),
                Err(_) => true,
            };
        if expired || broken {
            match validate(flash, layout.secondary, layout.max_image_len(), config.keys) {
                Ok(_) => {
                    progress(BootEvent::Reverting);
                    let mut revert = start(
                        flash,
                        layout,
                        status,
                        Some(&active),
                        true,
                        active.exchange.sectors,
                    )?;
                    run(flash, layout, &mut revert, progress)?;
                    current = Some(revert);
                    update = Update::Reverted;
                }
                Err(ImageError::Flash) => return Err(BootError::Flash),
                Err(_) => {}
            }
        }
    }

    let image = validate_primary(flash, config, update, progress)?;
    let trial = match current.as_mut() {
        Some(active) if config.trial_boots != 0 && !active.exchange.revert => {
            account_boot(flash, &active.log, &mut active.state, config.trial_boots)
        }
        _ => Trial::Permanent,
    };
    Ok(bootable(layout, image, update, trial))
}

/// The trial log of the running image: the active exchange's log when that
/// exchange was an install.
pub(crate) fn trial_log<F: NorFlash>(
    flash: &mut F,
    layout: &Layout,
    status: u32,
) -> Result<Option<(Log, LogState)>, BootError> {
    Ok(read_active(flash, layout, status)?
        .filter(|active| !active.exchange.revert)
        .map(|active| (active.log, active.state)))
}
