//! Text command protocol of the nRF52840 IEEE 802.15.4 radio lab.
//!
//! The lab firmware (`tools/nrf52840-radio-lab`) replaces Nordic's
//! *nRF Sniffer for 802.15.4* on a PCA10059 dongle. It keeps that firmware's
//! USB CDC shell grammar, so the Wireshark extcap and hwlab's capture code
//! work unchanged, and adds raw transmission for lab tests.
//!
//! # Nordic-compatible commands
//!
//! | Command            | Effect                                         |
//! |--------------------|------------------------------------------------|
//! | `sleep`            | disable the radio                              |
//! | `receive`          | promiscuous receive, no auto-ACK               |
//! | `channel`          | print the current channel (`15`)               |
//! | `channel <11..26>` | retune (the receiver stays in its state)       |
//! | `shell echo on/off`| line echo (on after reset, like Zephyr)        |
//! | `shell colors ..`  | accepted and ignored (no VT100 output)         |
//!
//! Every received frame with a valid FCS is printed as
//!
//! ```text
//! received: <psdu hex incl. FCS> power: <rssi dBm> lqi: <0..255> time: <us>
//! ```
//!
//! exactly like Nordic's firmware: lower-case hex of the PSDU including its
//! two FCS octets, RSSI in dBm, LQI, and microseconds since boot at the end
//! of the SFD. Unlike Nordic's driver, which overwrites the FCS with RSSI/LQI
//! data, the lab prints the real FCS. Consumers strip the last two octets in
//! both cases.
//!
//! # Lab extensions
//!
//! | Command | Effect |
//! |---------|--------|
//! | `version` | `version: <name> <ver> protocol: <n> caps: <list>` |
//! | `power` / `power <dBm>` | query or set the TX power (default 0 dBm) |
//! | `stats` | receive and transmit counters |
//! | `transmit <hex> [cca] [ack] [repeat <n> <interval_ms>]` | raw TX |
//!
//! `transmit` takes the PSDU *without* its FCS (1..=125 octets); the radio
//! appends the FCS. Without options it sends once, without CCA and without
//! waiting for an acknowledgement. Each attempt that reaches the air is
//! echoed as
//!
//! ```text
//! transmitted: <psdu hex incl. FCS> channel: <ch> power: <dBm> time: <us>
//! ```
//!
//! followed by one `tx: <i>/<n> status: <status> time: <us>` result per
//! attempt and a final `tx_done: ...` summary. `transmitted:` does not contain
//! `received:`, so receive-only parsers ignore it. Any command line received
//! during a repeat burst aborts the burst before it is executed.
//!
//! Firmware that answers `version: command not found` is Nordic's receive-only
//! sniffer.

#![no_std]

use core::fmt;

/// Firmware name reported by `version`.
pub const FIRMWARE_NAME: &str = "zigbee-rs-radio-lab";
/// Version of this text protocol (bumped on incompatible grammar changes).
pub const PROTOCOL_VERSION: u32 = 1;
/// Capability list reported by `version`.
pub const CAPABILITIES: &str = "receive,transmit,cca,ack-wait,repeat,power,stats";

/// aMaxPHYPacketSize: the largest PSDU, FCS included.
pub const MAX_PSDU_LEN: usize = 127;
/// IEEE 802.15.4 FCS length.
pub const FCS_LEN: usize = 2;
/// Largest `transmit` payload (PSDU without FCS).
pub const MAX_TX_LEN: usize = MAX_PSDU_LEN - FCS_LEN;
/// Largest `repeat` count of one `transmit` command.
pub const MAX_REPEAT: u16 = 1000;
/// Smallest interval between repeated attempts.
pub const MIN_REPEAT_INTERVAL_MS: u32 = 5;
/// Largest interval between repeated attempts.
pub const MAX_REPEAT_INTERVAL_MS: u32 = 60_000;
/// First 2.4 GHz O-QPSK channel.
pub const MIN_CHANNEL: u8 = 11;
/// Last 2.4 GHz O-QPSK channel.
pub const MAX_CHANNEL: u8 = 26;
/// Channel after reset (Nordic's driver default).
pub const DEFAULT_CHANNEL: u8 = 11;
/// TX power after reset.
pub const DEFAULT_TX_POWER_DBM: i8 = 0;
/// TX power levels of the nRF52840 RADIO.TXPOWER register.
pub const TX_POWER_LEVELS_DBM: [i8; 14] = [-40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8];
/// Longest accepted command line (a full `transmit` with every option fits).
pub const MAX_LINE_LEN: usize = 320;

/// Lines printed by `help`.
pub const HELP_LINES: &[&str] = &[
    "Available commands:",
    "  channel [11..26]       query or set the radio channel",
    "  receive                promiscuous receive (no auto-ACK)",
    "  sleep                  disable the radio",
    "  power [dBm]            query or set the TX power",
    "  transmit <hex> [cca] [ack] [repeat <n> <interval_ms>]",
    "                         send a PSDU without FCS (the radio appends it)",
    "  stats                  receive/transmit counters",
    "  version                firmware, protocol and capabilities",
    "  shell echo on|off      line echo",
];

// ── Commands ─────────────────────────────────────────────────

/// One `transmit` request.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TxRequest {
    psdu: [u8; MAX_TX_LEN],
    len: u8,
    /// Clear-channel assessment before each attempt.
    pub cca: bool,
    /// Wait for an Imm-Ack with the frame's sequence number.
    pub ack_wait: bool,
    /// Number of attempts (1..=[`MAX_REPEAT`]).
    pub repeat: u16,
    /// Delay between the starts of consecutive attempts.
    pub interval_ms: u32,
}

impl TxRequest {
    /// Single-shot request without CCA or ACK wait.
    pub fn new(psdu: &[u8]) -> Option<Self> {
        if psdu.is_empty() || psdu.len() > MAX_TX_LEN {
            return None;
        }
        let mut buffer = [0; MAX_TX_LEN];
        buffer[..psdu.len()].copy_from_slice(psdu);
        Some(Self {
            psdu: buffer,
            len: psdu.len() as u8,
            cca: false,
            ack_wait: false,
            repeat: 1,
            interval_ms: 0,
        })
    }

    /// The PSDU without FCS.
    pub fn psdu(&self) -> &[u8] {
        &self.psdu[..usize::from(self.len)]
    }

    /// MAC sequence number (third octet), when present.
    pub fn sequence(&self) -> Option<u8> {
        self.psdu().get(2).copied()
    }
}

/// A parsed command line.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Command {
    /// Blank line.
    Empty,
    Sleep,
    Receive,
    ChannelGet,
    ChannelSet(u8),
    PowerGet,
    PowerSet(i8),
    Transmit(TxRequest),
    Version,
    Help,
    Stats,
    ShellEcho(bool),
    ShellColors(bool),
}

/// A rejected command line; [`fmt::Display`] gives the reply line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandError<'a> {
    /// Unknown command (Zephyr shell wording).
    NotFound(&'a str),
    /// Known command with the wrong number of arguments.
    WrongArguments(&'static str),
    /// Argument out of range or malformed.
    Invalid(&'static str),
}

impl fmt::Display for CommandError<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotFound(command) => write!(f, "{command}: command not found"),
            Self::WrongArguments(command) => write!(f, "error: {command}: wrong parameter count"),
            Self::Invalid(message) => write!(f, "error: {message}"),
        }
    }
}

/// Parse one command line (without its terminator).
pub fn parse(line: &str) -> Result<Command, CommandError<'_>> {
    let mut words = line.split_ascii_whitespace();
    let Some(command) = words.next() else {
        return Ok(Command::Empty);
    };
    match command {
        "sleep" => no_arguments(words, "sleep", Command::Sleep),
        "receive" => no_arguments(words, "receive", Command::Receive),
        "version" => no_arguments(words, "version", Command::Version),
        "help" => no_arguments(words, "help", Command::Help),
        "stats" => no_arguments(words, "stats", Command::Stats),
        "channel" => match (words.next(), words.next()) {
            (None, _) => Ok(Command::ChannelGet),
            (Some(value), None) => parse_channel(value).map(Command::ChannelSet),
            _ => Err(CommandError::WrongArguments("channel")),
        },
        "power" => match (words.next(), words.next()) {
            (None, _) => Ok(Command::PowerGet),
            (Some(value), None) => parse_power(value).map(Command::PowerSet),
            _ => Err(CommandError::WrongArguments("power")),
        },
        "shell" => parse_shell(words),
        "transmit" => parse_transmit(words),
        other => Err(CommandError::NotFound(other)),
    }
}

fn no_arguments<'a>(
    mut words: impl Iterator<Item = &'a str>,
    name: &'static str,
    command: Command,
) -> Result<Command, CommandError<'a>> {
    match words.next() {
        None => Ok(command),
        Some(_) => Err(CommandError::WrongArguments(name)),
    }
}

fn parse_channel(value: &str) -> Result<u8, CommandError<'static>> {
    match value.parse::<u8>() {
        Ok(channel) if (MIN_CHANNEL..=MAX_CHANNEL).contains(&channel) => Ok(channel),
        _ => Err(CommandError::Invalid("channel must be 11..26")),
    }
}

fn parse_power(value: &str) -> Result<i8, CommandError<'static>> {
    match value.parse::<i8>() {
        Ok(dbm) if tx_power_register(dbm).is_some() => Ok(dbm),
        _ => Err(CommandError::Invalid(
            "power must be one of -40 -20 -16 -12 -8 -4 0 2 3 4 5 6 7 8",
        )),
    }
}

fn parse_switch(value: Option<&str>) -> Option<bool> {
    match value {
        Some("on") => Some(true),
        Some("off") => Some(false),
        _ => None,
    }
}

fn parse_shell<'a>(mut words: impl Iterator<Item = &'a str>) -> Result<Command, CommandError<'a>> {
    let command = match words.next() {
        Some("echo") => parse_switch(words.next()).map(Command::ShellEcho),
        Some("colors") => parse_switch(words.next()).map(Command::ShellColors),
        _ => None,
    };
    match (command, words.next()) {
        (Some(command), None) => Ok(command),
        _ => Err(CommandError::Invalid("usage: shell echo|colors on|off")),
    }
}

fn parse_transmit<'a>(
    mut words: impl Iterator<Item = &'a str>,
) -> Result<Command, CommandError<'a>> {
    let Some(hex) = words.next() else {
        return Err(CommandError::WrongArguments("transmit"));
    };
    let mut psdu = [0u8; MAX_TX_LEN];
    let len = decode_hex(hex, &mut psdu).ok_or(CommandError::Invalid(
        "transmit needs 1..125 octets of even-length hex",
    ))?;
    let mut request = TxRequest::new(&psdu[..len]).ok_or(CommandError::Invalid(
        "transmit needs 1..125 octets of even-length hex",
    ))?;
    let mut repeat_seen = false;
    while let Some(option) = words.next() {
        match option {
            "cca" => request.cca = true,
            "ack" => request.ack_wait = true,
            "repeat" if !repeat_seen => {
                repeat_seen = true;
                let count = words.next().and_then(|value| value.parse::<u16>().ok());
                let interval = words.next().and_then(|value| value.parse::<u32>().ok());
                let (Some(count), Some(interval)) = (count, interval) else {
                    return Err(CommandError::Invalid("usage: repeat <count> <interval_ms>"));
                };
                if !(1..=MAX_REPEAT).contains(&count) {
                    return Err(CommandError::Invalid("repeat count must be 1..1000"));
                }
                if interval > MAX_REPEAT_INTERVAL_MS
                    || (count > 1 && interval < MIN_REPEAT_INTERVAL_MS)
                {
                    return Err(CommandError::Invalid("repeat interval must be 5..60000 ms"));
                }
                request.repeat = count;
                request.interval_ms = interval;
            }
            _ => return Err(CommandError::Invalid("unknown transmit option")),
        }
    }
    if request.ack_wait && request.sequence().is_none() {
        return Err(CommandError::Invalid(
            "ack needs a MAC header with a sequence number",
        ));
    }
    Ok(Command::Transmit(request))
}

// ── Output lines ─────────────────────────────────────────────

/// Lower-case hex rendering of a byte slice (Zephyr `bin2hex`).
pub struct Hex<'a>(pub &'a [u8]);

impl fmt::Display for Hex<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        const DIGITS: &[u8; 16] = b"0123456789abcdef";
        for byte in self.0 {
            let pair = [
                DIGITS[usize::from(byte >> 4)],
                DIGITS[usize::from(byte & 0x0f)],
            ];
            // Both octets are ASCII hex digits.
            f.write_str(core::str::from_utf8(&pair).map_err(|_| fmt::Error)?)?;
        }
        Ok(())
    }
}

/// `received:` line of one sniffed frame.
pub struct Received<'a> {
    /// PSDU including the FCS.
    pub psdu: &'a [u8],
    pub rssi_dbm: i8,
    pub lqi: u8,
    pub time_us: u64,
}

impl fmt::Display for Received<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "received: {} power: {} lqi: {} time: {}",
            Hex(self.psdu),
            self.rssi_dbm,
            self.lqi,
            self.time_us
        )
    }
}

/// `transmitted:` echo of one frame put on the air.
pub struct Transmitted<'a> {
    /// PSDU including the FCS the radio appended.
    pub psdu: &'a [u8],
    pub channel: u8,
    pub power_dbm: i8,
    /// End of the transmitted SFD, same clock as `received:`.
    pub time_us: u64,
}

impl fmt::Display for Transmitted<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "transmitted: {} channel: {} power: {} time: {}",
            Hex(self.psdu),
            self.channel,
            self.power_dbm,
            self.time_us
        )
    }
}

/// Outcome of one transmit attempt.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxStatus {
    /// On the air; no ACK requested.
    Sent,
    /// On the air and the matching Imm-Ack arrived.
    Acked,
    /// On the air; no matching Imm-Ack within macAckWaitDuration.
    NoAck,
    /// CCA reported a busy channel; nothing was sent.
    CcaBusy,
    /// The radio did not complete the sequence in time.
    Error,
}

impl TxStatus {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Sent => "sent",
            Self::Acked => "acked",
            Self::NoAck => "no_ack",
            Self::CcaBusy => "cca_busy",
            Self::Error => "error",
        }
    }

    /// Whether the frame went out on the air.
    pub const fn on_air(self) -> bool {
        matches!(self, Self::Sent | Self::Acked | Self::NoAck)
    }
}

/// `tx:` result line of one attempt.
pub struct TxResult {
    pub attempt: u16,
    pub total: u16,
    pub status: TxStatus,
    pub time_us: u64,
}

impl fmt::Display for TxResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "tx: {}/{} status: {} time: {}",
            self.attempt,
            self.total,
            self.status.as_str(),
            self.time_us
        )
    }
}

/// Counters of one `transmit` command; `Display` gives the `tx_done:` line.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TxSummary {
    pub attempts: u16,
    pub sent: u16,
    pub acked: u16,
    pub no_ack: u16,
    pub cca_busy: u16,
    pub error: u16,
    pub aborted: bool,
}

impl TxSummary {
    /// Count one attempt.
    pub fn record(&mut self, status: TxStatus) {
        self.attempts = self.attempts.saturating_add(1);
        let counter = match status {
            TxStatus::Sent => &mut self.sent,
            TxStatus::Acked => &mut self.acked,
            TxStatus::NoAck => &mut self.no_ack,
            TxStatus::CcaBusy => &mut self.cca_busy,
            TxStatus::Error => &mut self.error,
        };
        *counter = counter.saturating_add(1);
    }
}

impl fmt::Display for TxSummary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "tx_done: attempts: {} sent: {} acked: {} no_ack: {} cca_busy: {} error: {} result: {}",
            self.attempts,
            self.sent,
            self.acked,
            self.no_ack,
            self.cca_busy,
            self.error,
            if self.aborted { "aborted" } else { "ok" }
        )
    }
}

/// `version:` reply.
pub struct Version<'a> {
    pub firmware_version: &'a str,
}

impl fmt::Display for Version<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "version: {} {} protocol: {} caps: {}",
            FIRMWARE_NAME, self.firmware_version, PROTOCOL_VERSION, CAPABILITIES
        )
    }
}

/// `power:` reply.
pub struct Power(pub i8);

impl fmt::Display for Power {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "power: {}", self.0)
    }
}

/// Radio counters since reset; `Display` gives the `stats:` line.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Frames with a valid FCS queued for output.
    pub rx: u32,
    /// Frames dropped for a bad FCS.
    pub crc_error: u32,
    /// Valid frames dropped because the output ring was full.
    pub overflow: u32,
    /// Frames put on the air.
    pub tx: u32,
}

impl fmt::Display for Stats {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "stats: rx: {} crc_error: {} overflow: {} tx: {}",
            self.rx, self.crc_error, self.overflow, self.tx
        )
    }
}

/// `rx_overflow:` notice: valid frames lost since the previous notice.
pub struct RxOverflow(pub u32);

impl fmt::Display for RxOverflow {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "rx_overflow: {}", self.0)
    }
}

// ── Codecs and helpers ───────────────────────────────────────

/// Decode even-length hex into `out`; `None` for odd length, bad digits,
/// empty input or overflow.
pub fn decode_hex(hex: &str, out: &mut [u8]) -> Option<usize> {
    let digits = hex.as_bytes();
    if digits.is_empty() || !digits.len().is_multiple_of(2) || digits.len() / 2 > out.len() {
        return None;
    }
    for (slot, pair) in out.iter_mut().zip(digits.chunks_exact(2)) {
        *slot = (nibble(pair[0])? << 4) | nibble(pair[1])?;
    }
    Some(digits.len() / 2)
}

fn nibble(digit: u8) -> Option<u8> {
    match digit {
        b'0'..=b'9' => Some(digit - b'0'),
        b'a'..=b'f' => Some(digit - b'a' + 10),
        b'A'..=b'F' => Some(digit - b'A' + 10),
        _ => None,
    }
}

/// IEEE 802.15.4 FCS (ITU-T CRC-16, LSB first: CRC-16/KERMIT).
pub fn fcs(mpdu: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in mpdu {
        crc ^= u16::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0x8408
            } else {
                crc >> 1
            };
        }
    }
    crc
}

/// Write the FCS of `psdu[..len - 2]` into its last two octets.
pub fn append_fcs(psdu: &mut [u8]) {
    if psdu.len() < FCS_LEN {
        return;
    }
    let body = psdu.len() - FCS_LEN;
    let crc = fcs(&psdu[..body]).to_le_bytes();
    psdu[body..].copy_from_slice(&crc);
}

/// Whether `mpdu` (no FCS) is the Imm-Ack for sequence number `sequence`.
pub fn is_ack_for(mpdu: &[u8], sequence: u8) -> bool {
    mpdu.len() == 3 && mpdu[0] & 0x07 == 0x02 && mpdu[2] == sequence
}

/// RADIO.TXPOWER value for a supported level.
pub fn tx_power_register(dbm: i8) -> Option<u32> {
    TX_POWER_LEVELS_DBM
        .contains(&dbm)
        .then_some(u32::from(dbm as u8))
}

/// RADIO.FREQUENCY value of an O-QPSK channel (2405 + 5 (ch - 11) MHz).
pub fn channel_frequency(channel: u8) -> Option<u32> {
    (MIN_CHANNEL..=MAX_CHANNEL)
        .contains(&channel)
        .then(|| (u32::from(channel) - 10) * 5)
}

/// RSSI in dBm from RADIO.RSSISAMPLE (a positive magnitude).
pub fn rssi_dbm(sample: u32) -> i8 {
    -(sample.min(127) as i8)
}

/// LQI from the nRF energy-detect octet the RADIO writes over the first FCS
/// octet; identical to `zigbee_mac::lqi::nrf_from_hardware`.
pub const fn lqi_from_hardware(raw: u8) -> u8 {
    raw.saturating_mul(4)
}

/// Zephyr's USB serial number of an nRF52: `DEVICEID[1]` then `DEVICEID[0]`
/// as upper-case big-endian hex, so hwlab keeps the dongle's identity when
/// the lab firmware replaces Nordic's.
pub fn usb_serial(device_id: [u32; 2]) -> [u8; 16] {
    const DIGITS: &[u8; 16] = b"0123456789ABCDEF";
    let value = (u64::from(device_id[1]) << 32) | u64::from(device_id[0]);
    let mut out = [0u8; 16];
    for (index, slot) in out.iter_mut().enumerate() {
        let shift = 60 - 4 * index;
        *slot = DIGITS[((value >> shift) & 0xf) as usize];
    }
    out
}

/// Extends a free-running 32-bit microsecond counter to 64 bits.
///
/// [`Self::extend`] must run at least once per counter period (71.6 min at
/// 1 MHz); the firmware calls it from a 250 ms ticker and from every radio
/// event, so captures never see a backwards jump that hwlab would treat as a
/// device reset.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TimestampExtender {
    high: u32,
    last: u32,
}

impl TimestampExtender {
    pub const fn new() -> Self {
        Self { high: 0, last: 0 }
    }

    /// 64-bit time of the current counter value `now`.
    pub fn extend(&mut self, now: u32) -> u64 {
        if now < self.last {
            self.high = self.high.wrapping_add(1);
        }
        self.last = now;
        (u64::from(self.high) << 32) | u64::from(now)
    }

    /// 64-bit time of an earlier capture `then`, given the extended time
    /// `now64` of the counter value `now32` (within one counter period).
    pub fn earlier(now64: u64, now32: u32, then: u32) -> u64 {
        now64.saturating_sub(u64::from(now32.wrapping_sub(then)))
    }
}

/// What [`LineAssembler::push`] produced.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Push {
    /// The line is still being received.
    Pending,
    /// A non-empty line is complete; read it with [`LineAssembler::line`].
    Line,
    /// A line longer than the buffer was discarded.
    Overflow,
}

/// Collects shell input into lines (`\r` or `\n` terminated, backspace aware).
pub struct LineAssembler<const N: usize> {
    buffer: [u8; N],
    len: usize,
    complete: bool,
    overflow: bool,
}

impl<const N: usize> Default for LineAssembler<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> LineAssembler<N> {
    pub const fn new() -> Self {
        Self {
            buffer: [0; N],
            len: 0,
            complete: false,
            overflow: false,
        }
    }

    /// Feed one input octet.
    pub fn push(&mut self, byte: u8) -> Push {
        if self.complete {
            self.complete = false;
            self.len = 0;
        }
        match byte {
            b'\r' | b'\n' => {
                if self.overflow {
                    self.overflow = false;
                    self.len = 0;
                    Push::Overflow
                } else if self.len == 0 {
                    Push::Pending
                } else {
                    self.complete = true;
                    Push::Line
                }
            }
            0x08 | 0x7f => {
                self.len = self.len.saturating_sub(1);
                Push::Pending
            }
            0x20..=0x7e | b'\t' => {
                if self.len == N {
                    self.overflow = true;
                } else if !self.overflow {
                    self.buffer[self.len] = byte;
                    self.len += 1;
                }
                Push::Pending
            }
            _ => Push::Pending,
        }
    }

    /// The completed line after [`Push::Line`] (printable ASCII only).
    pub fn line(&self) -> &str {
        core::str::from_utf8(&self.buffer[..self.len]).unwrap_or("")
    }
}

#[cfg(test)]
mod tests;
