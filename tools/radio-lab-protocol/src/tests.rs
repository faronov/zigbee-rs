extern crate std;

use std::string::{String, ToString};

use super::*;

fn transmit(line: &str) -> TxRequest {
    match parse(line) {
        Ok(Command::Transmit(request)) => request,
        other => panic!("{line}: {other:?}"),
    }
}

fn error(line: &str) -> String {
    parse(line).expect_err(line).to_string()
}

#[test]
fn nordic_commands_parse() {
    assert_eq!(parse(""), Ok(Command::Empty));
    assert_eq!(parse("   "), Ok(Command::Empty));
    assert_eq!(parse("sleep"), Ok(Command::Sleep));
    assert_eq!(parse("receive"), Ok(Command::Receive));
    assert_eq!(parse("channel"), Ok(Command::ChannelGet));
    assert_eq!(parse("channel 15"), Ok(Command::ChannelSet(15)));
    assert_eq!(parse("  channel   26 "), Ok(Command::ChannelSet(26)));
    assert_eq!(parse("shell echo off"), Ok(Command::ShellEcho(false)));
    assert_eq!(parse("shell echo on"), Ok(Command::ShellEcho(true)));
    assert_eq!(parse("shell colors off"), Ok(Command::ShellColors(false)));
}

#[test]
fn lab_commands_parse() {
    assert_eq!(parse("version"), Ok(Command::Version));
    assert_eq!(parse("help"), Ok(Command::Help));
    assert_eq!(parse("stats"), Ok(Command::Stats));
    assert_eq!(parse("power"), Ok(Command::PowerGet));
    assert_eq!(parse("power -20"), Ok(Command::PowerSet(-20)));
    assert_eq!(parse("power 8"), Ok(Command::PowerSet(8)));
    assert_eq!(parse("power +4"), Ok(Command::PowerSet(4)));
}

#[test]
fn errors_use_words_hwlab_detects() {
    // hwlab rejects a `channel` reply containing "not found", "error" or
    // "invalid"; the extcap ignores every non-`received:` line.
    assert_eq!(error("version2"), "version2: command not found");
    assert_eq!(error("channel 10"), "error: channel must be 11..26");
    assert_eq!(error("channel 27"), "error: channel must be 11..26");
    assert_eq!(error("channel x"), "error: channel must be 11..26");
    assert_eq!(
        error("channel 15 16"),
        "error: channel: wrong parameter count"
    );
    assert_eq!(error("sleep now"), "error: sleep: wrong parameter count");
    assert!(error("power 1").starts_with("error: power must be one of"));
    assert!(error("power -30").starts_with("error: power must be one of"));
    assert_eq!(
        error("shell echo"),
        "error: usage: shell echo|colors on|off"
    );
    assert_eq!(
        error("shell echo on off"),
        "error: usage: shell echo|colors on|off"
    );
}

#[test]
fn transmit_defaults_are_single_shot_without_cca_or_ack() {
    let request = transmit("transmit 0308c1ffffffff07");
    assert_eq!(
        request.psdu(),
        &[0x03, 0x08, 0xc1, 0xff, 0xff, 0xff, 0xff, 0x07]
    );
    assert!(!request.cca);
    assert!(!request.ack_wait);
    assert_eq!(request.repeat, 1);
    assert_eq!(request.interval_ms, 0);
    assert_eq!(request.sequence(), Some(0xc1));
}

#[test]
fn transmit_options_in_any_order() {
    let request = transmit("transmit 0308C1FFFFFFFF07 repeat 10 100 ack cca");
    assert!(request.cca);
    assert!(request.ack_wait);
    assert_eq!(request.repeat, 10);
    assert_eq!(request.interval_ms, 100);

    let single = transmit("transmit 41 repeat 1 0");
    assert_eq!(single.repeat, 1);
    assert_eq!(single.psdu(), &[0x41]);
}

#[test]
fn transmit_length_limits() {
    let max = "ab".repeat(MAX_TX_LEN);
    assert_eq!(transmit(&std::format!("transmit {max}")).psdu().len(), 125);
    let too_long = "ab".repeat(MAX_TX_LEN + 1);
    assert_eq!(
        error(&std::format!("transmit {too_long}")),
        "error: transmit needs 1..125 octets of even-length hex"
    );
    for bad in ["transmit 0", "transmit 0g", "transmit xyz1"] {
        assert_eq!(
            error(bad),
            "error: transmit needs 1..125 octets of even-length hex",
            "{bad}"
        );
    }
    assert_eq!(error("transmit"), "error: transmit: wrong parameter count");
}

#[test]
fn transmit_repeat_limits() {
    assert_eq!(transmit("transmit 41 repeat 1000 5").repeat, 1000);
    assert_eq!(transmit("transmit 41 repeat 2 60000").interval_ms, 60000);
    assert_eq!(
        error("transmit 41 repeat 1001 5"),
        "error: repeat count must be 1..1000"
    );
    assert_eq!(
        error("transmit 41 repeat 0 5"),
        "error: repeat count must be 1..1000"
    );
    assert_eq!(
        error("transmit 41 repeat 2 4"),
        "error: repeat interval must be 5..60000 ms"
    );
    assert_eq!(
        error("transmit 41 repeat 2 60001"),
        "error: repeat interval must be 5..60000 ms"
    );
    assert_eq!(
        error("transmit 41 repeat 1 60001"),
        "error: repeat interval must be 5..60000 ms"
    );
    assert_eq!(
        error("transmit 41 repeat 2"),
        "error: usage: repeat <count> <interval_ms>"
    );
    assert_eq!(
        error("transmit 41 repeat 2 5 repeat 3 5"),
        "error: unknown transmit option"
    );
    assert_eq!(error("transmit 41 fast"), "error: unknown transmit option");
}

#[test]
fn ack_wait_needs_a_sequence_number() {
    assert_eq!(
        error("transmit 4188 ack"),
        "error: ack needs a MAC header with a sequence number"
    );
    assert!(transmit("transmit 418801 ack").ack_wait);
}

#[test]
fn received_line_matches_nordic_and_extcap_grammar() {
    let line = Received {
        psdu: &[0x02, 0x00, 0x2a, 0x12, 0x34],
        rssi_dbm: -61,
        lqi: 220,
        time_us: 18_446_744_073_709_551_615,
    }
    .to_string();
    assert_eq!(
        line,
        "received: 02002a1234 power: -61 lqi: 220 time: 18446744073709551615"
    );
}

#[test]
fn transmitted_and_result_lines_do_not_look_received() {
    let echo = Transmitted {
        psdu: &[0x03, 0x08, 0x01],
        channel: 26,
        power_dbm: -4,
        time_us: 42,
    }
    .to_string();
    assert_eq!(echo, "transmitted: 030801 channel: 26 power: -4 time: 42");
    assert!(!echo.contains("received:"));

    let result = TxResult {
        attempt: 2,
        total: 5,
        status: TxStatus::NoAck,
        time_us: 7,
    }
    .to_string();
    assert_eq!(result, "tx: 2/5 status: no_ack time: 7");

    let mut summary = TxSummary::default();
    for status in [
        TxStatus::Sent,
        TxStatus::Acked,
        TxStatus::NoAck,
        TxStatus::CcaBusy,
        TxStatus::Error,
        TxStatus::Sent,
    ] {
        summary.record(status);
    }
    summary.aborted = true;
    assert_eq!(
        summary.to_string(),
        "tx_done: attempts: 6 sent: 2 acked: 1 no_ack: 1 cca_busy: 1 error: 1 result: aborted"
    );
    assert!(TxStatus::NoAck.on_air());
    assert!(!TxStatus::CcaBusy.on_air());
}

#[test]
fn version_power_stats_lines() {
    assert_eq!(
        Version {
            firmware_version: "0.1.0"
        }
        .to_string(),
        "version: zigbee-rs-radio-lab 0.1.0 protocol: 1 caps: receive,transmit,cca,ack-wait,repeat,power,stats"
    );
    assert_eq!(Power(-8).to_string(), "power: -8");
    assert_eq!(
        Stats {
            rx: 1,
            crc_error: 2,
            overflow: 3,
            tx: 4
        }
        .to_string(),
        "stats: rx: 1 crc_error: 2 overflow: 3 tx: 4"
    );
    assert_eq!(RxOverflow(9).to_string(), "rx_overflow: 9");
}

#[test]
fn fcs_matches_kermit_and_a_captured_frame() {
    assert_eq!(fcs(b"123456789"), 0x2189);
    // Imm-Ack for sequence 0x2a has FCS bytes ... appended LSB first; the
    // residue of a frame with its FCS is zero.
    let mut ack = [0x02, 0x00, 0x2a, 0, 0];
    append_fcs(&mut ack);
    assert_eq!(fcs(&ack), 0);
    assert!(is_ack_for(&ack[..3], 0x2a));
    assert!(!is_ack_for(&ack[..3], 0x2b));
    assert!(!is_ack_for(&[0x41, 0x00, 0x2a], 0x2a));
    // MAC beacon request, sequence 0x8e (IEEE 802.15.4 Annex golden).
    let mut beacon_request = [0x03, 0x08, 0x8e, 0xff, 0xff, 0xff, 0xff, 0x07, 0, 0];
    append_fcs(&mut beacon_request);
    assert_eq!(fcs(&beacon_request), 0);
}

#[test]
fn hex_codec_round_trip() {
    let mut out = [0u8; 4];
    assert_eq!(decode_hex("00Ff7a10", &mut out), Some(4));
    assert_eq!(out, [0x00, 0xff, 0x7a, 0x10]);
    assert_eq!(Hex(&out).to_string(), "00ff7a10");
    assert_eq!(decode_hex("0011223344", &mut out), None);
    assert_eq!(decode_hex("", &mut out), None);
}

#[test]
fn radio_register_helpers() {
    assert_eq!(channel_frequency(11), Some(5));
    assert_eq!(channel_frequency(26), Some(80));
    assert_eq!(channel_frequency(10), None);
    assert_eq!(tx_power_register(0), Some(0));
    assert_eq!(tx_power_register(-4), Some(0xfc));
    assert_eq!(tx_power_register(-40), Some(0xd8));
    assert_eq!(tx_power_register(8), Some(8));
    assert_eq!(tx_power_register(1), None);
    assert_eq!(rssi_dbm(61), -61);
    assert_eq!(rssi_dbm(500), -127);
}

#[test]
fn lqi_matches_the_stack_mapping() {
    for raw in 0..=u8::MAX {
        assert_eq!(
            lqi_from_hardware(raw),
            zigbee_mac::lqi::nrf_from_hardware(raw)
        );
    }
}

#[test]
fn usb_serial_matches_zephyr_hwinfo_order() {
    // Zephyr hwinfo_nrf: big-endian DEVICEID[1] then DEVICEID[0].
    let serial = usb_serial([0x8DD4_0A98, 0x7E2F_90FB]);
    assert_eq!(&serial, b"7E2F90FB8DD40A98");
}

#[test]
fn timestamps_extend_monotonically_across_wrap() {
    let mut extender = TimestampExtender::new();
    assert_eq!(extender.extend(10), 10);
    assert_eq!(extender.extend(u32::MAX - 5), u64::from(u32::MAX - 5));
    let after_wrap = extender.extend(3);
    assert_eq!(after_wrap, (1u64 << 32) + 3);
    assert_eq!(extender.extend(3), after_wrap);
    // A capture taken just before the wrap maps to the previous epoch.
    assert_eq!(
        TimestampExtender::earlier(after_wrap, 3, u32::MAX - 1),
        u64::from(u32::MAX - 1)
    );
    assert_eq!(TimestampExtender::earlier(1_000, 100, 40), 940);
}

#[test]
fn line_assembler_handles_crlf_backspace_and_overflow() {
    let mut lines = LineAssembler::<8>::new();
    let feed = |bytes: &[u8], lines: &mut LineAssembler<8>| {
        let mut out = std::vec::Vec::new();
        for &byte in bytes {
            match lines.push(byte) {
                Push::Pending => {}
                Push::Line => out.push(lines.line().to_string()),
                Push::Overflow => out.push("<overflow>".to_string()),
            }
        }
        out
    };
    assert_eq!(
        feed(b"sleep\r\n\r\nreceive\n", &mut lines),
        ["sleep", "receive"]
    );
    assert_eq!(feed(b"chx\x08annel\r", &mut lines), ["channel"]);
    assert_eq!(feed(b"0123456789\nok\n", &mut lines), ["<overflow>", "ok"]);
    assert_eq!(feed(b"\x1b[Aa\n", &mut lines), ["[Aa"]);
}

#[test]
fn help_mentions_every_command() {
    let help = HELP_LINES.join("\n");
    for command in [
        "channel", "receive", "sleep", "power", "transmit", "stats", "version", "shell",
    ] {
        assert!(help.contains(command), "{command}");
    }
    assert!(
        MAX_LINE_LEN >= "transmit ".len() + 2 * MAX_TX_LEN + " cca ack repeat 1000 60000".len()
    );
}
