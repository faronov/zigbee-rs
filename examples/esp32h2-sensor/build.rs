//! Build-time firmware version and the optional thermometer build profile.
//!
//! `ESP32_OTA_VERSION` (decimal or `0x`-prefixed) becomes the Zigbee OTA file
//! version and the Basic cluster software build ID. The shared OTA packager
//! (`tools/create-ota.py`) sets it to the version it stamps into the container.
//!
//! With the `thermometer` feature, `thermometer.toml` (or the file named by
//! `H2_THERMO_CONFIG`) is validated and emitted as constants; see that file.

use std::{env, fs, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-env-changed=ESP32_OTA_VERSION");

    let raw = env::var("ESP32_OTA_VERSION").unwrap_or_else(|_| "1".to_owned());
    let version = parse_u32(&raw).expect("ESP32_OTA_VERSION must be a decimal or 0x-prefixed u32");
    assert!(version != u32::MAX, "0xFFFFFFFF is reserved by Zigbee OTA");

    let generated = format!(
        "pub const FIRMWARE_VERSION: u32 = {version};\n\
         pub const FIRMWARE_VERSION_STR: &str = \"{version}\";\n"
    );
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR"));
    fs::write(out.join("firmware_version.rs"), generated)
        .expect("write generated firmware version");

    if env::var_os("CARGO_FEATURE_THERMOMETER").is_some() {
        fs::write(out.join("thermometer_config.rs"), thermometer::generate())
            .expect("write generated thermometer configuration");
    }
}

fn parse_u32(raw: &str) -> Result<u32, std::num::ParseIntError> {
    let raw = raw.trim();
    raw.strip_prefix("0x")
        .or_else(|| raw.strip_prefix("0X"))
        .map_or_else(|| raw.parse(), |hex| u32::from_str_radix(hex, 16))
}

mod thermometer {
    use super::parse_u32;
    use std::{
        collections::BTreeMap,
        env, fs,
        path::{Path, PathBuf},
    };

    /// Longest joined wait the shared sleepy application accepts
    /// (`sensor_sed_app::policy::MAX_WAIT_MS`, checked again at compile time).
    const MAX_SLOW_POLL_MS: u32 = 60_000;
    /// Largest rejoin backoff cap (`sensor_sed_app::JOIN_RETRY_MAX_MS`,
    /// checked again at compile time through `POLICY`).
    const JOIN_RETRY_MAX_MS: u32 = 30 * 60 * 1_000;
    /// ZCL Poll Control intervals are quarter-seconds.
    const QS_MS: i64 = 250;

    pub fn generate() -> String {
        println!("cargo:rerun-if-env-changed=H2_THERMO_CONFIG");
        let path = env::var_os("H2_THERMO_CONFIG").map_or_else(
            || {
                PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"))
                    .join("thermometer.toml")
            },
            PathBuf::from,
        );
        println!("cargo:rerun-if-changed={}", path.display());
        let text = fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("read thermometer config {}: {e}", path.display()));
        let mut values = parse(&text, &path);

        let c = Config {
            model: string(&mut values, "model"),
            ota_image_type: int(&mut values, "ota_image_type", 0, 0xFFBF),
            role_tag: int(&mut values, "role_tag", 1, 0xFFFE),
            min_measured: int(&mut values, "min_measured_centi_c", -27_315, 32_767),
            max_measured: int(&mut values, "max_measured_centi_c", -27_314, 32_767),
            report_min_s: int(&mut values, "report_min_s", 0, 0xFFFF),
            report_max_s: int(&mut values, "report_max_s", 0, 0xFFFF),
            report_change: int(&mut values, "report_change_centi_c", 1, 32_767),
            slow_poll_ms: int(
                &mut values,
                "slow_poll_ms",
                1_000,
                i64::from(MAX_SLOW_POLL_MS),
            ),
            fast_poll_ms: int(&mut values, "fast_poll_ms", 250, 7_500),
            fast_poll_light_sleep: boolean(&mut values, "fast_poll_light_sleep"),
            check_in_interval_s: int(&mut values, "check_in_interval_s", 0, 0x006E_0000 / 4),
            check_in_interval_min_s: int(
                &mut values,
                "check_in_interval_min_s",
                0,
                0x006E_0000 / 4,
            ),
            long_poll_min_ms: int(
                &mut values,
                "long_poll_min_ms",
                1_000,
                i64::from(MAX_SLOW_POLL_MS),
            ),
            long_poll_max_ms: int(
                &mut values,
                "long_poll_max_ms",
                1_000,
                i64::from(MAX_SLOW_POLL_MS),
            ),
            fast_poll_timeout_ms: int(&mut values, "fast_poll_timeout_ms", 250, 0xFFFF * 250),
            fast_poll_timeout_max_ms: int(
                &mut values,
                "fast_poll_timeout_max_ms",
                250,
                0xFFFF * 250,
            ),
            check_in_response_ms: int(&mut values, "check_in_response_ms", 250, 0xFFFF * 250),
            join_retry_ms: int(
                &mut values,
                "join_retry_ms",
                1_000,
                i64::from(JOIN_RETRY_MAX_MS),
            ),
            join_retry_max_ms: int(
                &mut values,
                "join_retry_max_ms",
                1_000,
                i64::from(JOIN_RETRY_MAX_MS),
            ),
            join_retry_jitter_pct: int(&mut values, "join_retry_jitter_pct", 0, 50),
            parent_loss_strikes: int(&mut values, "parent_loss_strikes", 0, 255),
            max_active_ms: int(&mut values, "max_active_ms", 0, i64::from(MAX_SLOW_POLL_MS)),
            report_aps_ack: boolean(&mut values, "report_aps_ack"),
            max_ota_awake_ms: int(&mut values, "max_ota_awake_ms", 0, 86_400_000),
            max_identify_awake_ms: int(&mut values, "max_identify_awake_ms", 0, 86_400_000),
            max_fast_poll_ms: int(&mut values, "max_fast_poll_ms", 0, 86_400_000),
            missed_report_grace_ms: int(&mut values, "missed_report_grace_ms", 0, 86_400_000),
            anomaly_max_writes_per_boot: int(&mut values, "anomaly_max_writes_per_boot", 0, 255),
            sleep_failure_limit: int(&mut values, "sleep_failure_limit", 0, 255),
            rapid_reset_limit: int(&mut values, "rapid_reset_limit", 0, 255),
            rapid_reset_window_ms: int(&mut values, "rapid_reset_window_ms", 1_000, 3_600_000),
            sleep_xtal_off: boolean(&mut values, "sleep_xtal_off"),
            sleep_rc_fast_off: boolean(&mut values, "sleep_rc_fast_off"),
            fresh_join_fast_ms: int(&mut values, "fresh_join_fast_ms", 1, 3_600_000),
            restored_fast_ms: int(&mut values, "restored_fast_ms", 1, 3_600_000),
            button_fast_poll_ms: int(&mut values, "button_fast_poll_ms", 1, 600_000),
            sample_every_polls: int(&mut values, "sample_every_polls", 1, 1_000),
            usb_host_wait_ms: int(&mut values, "usb_host_wait_ms", 0, 10_000),
            log_level: log_level(&mut values, "log_level"),
            diag_summary_ms: int(&mut values, "diag_summary_ms", 0, 3_600_000),
        };
        if let Some(unknown) = values.keys().next() {
            panic!("{}: unknown key `{unknown}`", path.display());
        }
        c.validate();
        c.render()
    }

    struct Config {
        model: String,
        ota_image_type: i64,
        role_tag: i64,
        min_measured: i64,
        max_measured: i64,
        report_min_s: i64,
        report_max_s: i64,
        report_change: i64,
        slow_poll_ms: i64,
        fast_poll_ms: i64,
        fast_poll_light_sleep: bool,
        check_in_interval_s: i64,
        check_in_interval_min_s: i64,
        long_poll_min_ms: i64,
        long_poll_max_ms: i64,
        fast_poll_timeout_ms: i64,
        fast_poll_timeout_max_ms: i64,
        check_in_response_ms: i64,
        join_retry_ms: i64,
        join_retry_max_ms: i64,
        join_retry_jitter_pct: i64,
        parent_loss_strikes: i64,
        max_active_ms: i64,
        report_aps_ack: bool,
        max_ota_awake_ms: i64,
        max_identify_awake_ms: i64,
        max_fast_poll_ms: i64,
        missed_report_grace_ms: i64,
        anomaly_max_writes_per_boot: i64,
        sleep_failure_limit: i64,
        rapid_reset_limit: i64,
        rapid_reset_window_ms: i64,
        sleep_xtal_off: bool,
        sleep_rc_fast_off: bool,
        fresh_join_fast_ms: i64,
        restored_fast_ms: i64,
        button_fast_poll_ms: i64,
        sample_every_polls: i64,
        usb_host_wait_ms: i64,
        log_level: &'static str,
        diag_summary_ms: i64,
    }

    impl Config {
        fn validate(&self) {
            assert!(
                !self.model.is_empty()
                    && self.model.len() <= 32
                    && self
                        .model
                        .bytes()
                        .all(|b| (0x20..0x7F).contains(&b) && b != b'"' && b != b'\\'),
                "model must be 1..=32 printable ASCII bytes without quotes/backslashes"
            );
            assert!(
                self.min_measured < self.max_measured,
                "min_measured_centi_c must be below max_measured_centi_c"
            );
            assert!(
                self.report_max_s == 0
                    || self.report_max_s == 0xFFFF
                    || self.report_min_s <= self.report_max_s,
                "report_min_s must not exceed report_max_s"
            );
            assert!(
                self.fast_poll_ms < self.slow_poll_ms,
                "fast_poll_ms must be shorter than slow_poll_ms"
            );
            assert!(
                self.slow_poll_ms * self.sample_every_polls <= u32::MAX as i64,
                "sample interval overflows"
            );
            for (key, value) in [
                ("slow_poll_ms", self.slow_poll_ms),
                ("fast_poll_ms", self.fast_poll_ms),
                ("long_poll_min_ms", self.long_poll_min_ms),
                ("long_poll_max_ms", self.long_poll_max_ms),
                ("fast_poll_timeout_ms", self.fast_poll_timeout_ms),
                ("fast_poll_timeout_max_ms", self.fast_poll_timeout_max_ms),
                ("check_in_response_ms", self.check_in_response_ms),
            ] {
                assert!(
                    value % QS_MS == 0,
                    "`{key}` = {value} must be a multiple of 250 ms (ZCL quarter-seconds)"
                );
            }
            assert!(
                self.long_poll_min_ms <= self.slow_poll_ms
                    && self.slow_poll_ms <= self.long_poll_max_ms,
                "slow_poll_ms must lie within long_poll_min_ms..=long_poll_max_ms"
            );
            assert!(
                self.check_in_interval_s == 0
                    || (self.check_in_interval_s >= self.check_in_interval_min_s
                        && self.check_in_interval_s * 1_000 >= self.slow_poll_ms),
                "check_in_interval_s must be 0, or at least check_in_interval_min_s and slow_poll_ms"
            );
            assert!(
                self.fast_poll_timeout_ms <= self.fast_poll_timeout_max_ms,
                "fast_poll_timeout_ms must not exceed fast_poll_timeout_max_ms"
            );
            assert!(
                self.join_retry_ms <= self.join_retry_max_ms,
                "join_retry_ms must not exceed join_retry_max_ms"
            );
            assert!(
                self.missed_report_grace_ms == 0 || self.report_aps_ack,
                "missed_report_grace_ms needs report_aps_ack (the ACK is the delivery proof)"
            );
            assert!(
                self.max_fast_poll_ms == 0
                    || [
                        self.fresh_join_fast_ms,
                        self.restored_fast_ms,
                        self.button_fast_poll_ms,
                        self.fast_poll_timeout_max_ms,
                        self.check_in_response_ms,
                    ]
                    .iter()
                    .all(|window| *window <= self.max_fast_poll_ms),
                "max_fast_poll_ms must be 0 or cover every configured fast-poll window"
            );
        }

        fn render(&self) -> String {
            format!(
                "// Generated by build.rs from the thermometer build profile.\n\
                 pub const MODEL: &str = \"{model}\";\n\
                 pub const OTA_IMAGE_TYPE: u16 = {ota:#06X};\n\
                 pub const ROLE_TAG: u16 = {role:#06X};\n\
                 pub const MIN_MEASURED_CENTI_C: i16 = {min_m};\n\
                 pub const MAX_MEASURED_CENTI_C: i16 = {max_m};\n\
                 pub const REPORT_MIN_S: u16 = {rmin};\n\
                 pub const REPORT_MAX_S: u16 = {rmax};\n\
                 pub const REPORT_CHANGE_CENTI_C: i16 = {rchg};\n\
                 pub const SLOW_POLL_MS: u32 = {slow};\n\
                 pub const FAST_POLL_MS: u32 = {fast};\n\
                 pub const FRESH_JOIN_FAST_MS: u32 = {fresh};\n\
                 pub const RESTORED_FAST_MS: u32 = {restored};\n\
                 pub const BUTTON_FAST_POLL_MS: u32 = {button};\n\
                 pub const SAMPLE_EVERY_POLLS: u32 = {every};\n\
                 pub const USB_HOST_WAIT_MS: u32 = {usb};\n\
                 pub const FAST_POLL_LIGHT_SLEEP: bool = {fast_light};\n\
                 pub const CHECK_IN_INTERVAL_QS: u32 = {check_in};\n\
                 pub const CHECK_IN_INTERVAL_MIN_QS: u32 = {check_in_min};\n\
                 pub const LONG_POLL_INTERVAL_QS: u32 = {long_qs};\n\
                 pub const LONG_POLL_MIN_QS: u32 = {long_min};\n\
                 pub const LONG_POLL_MAX_QS: u32 = {long_max};\n\
                 pub const SHORT_POLL_INTERVAL_QS: u16 = {short_qs};\n\
                 pub const FAST_POLL_TIMEOUT_QS: u16 = {fpt};\n\
                 pub const FAST_POLL_TIMEOUT_MAX_QS: u16 = {fpt_max};\n\
                 pub const CHECK_IN_RESPONSE_QS: u16 = {resp};\n\
                 pub const JOIN_RETRY_MS: u32 = {jr};\n\
                 pub const JOIN_RETRY_MAX_MS: u32 = {jr_max};\n\
                 pub const JOIN_RETRY_JITTER_PCT: u8 = {jitter};\n\
                 pub const PARENT_LOSS_STRIKES: u8 = {strikes};\n\
                 pub const MAX_ACTIVE_MS: u32 = {max_active};\n\
                 pub const REPORT_APS_ACK: bool = {aps_ack};\n\
                 pub const MAX_OTA_AWAKE_MS: u32 = {max_ota};\n\
                 pub const MAX_IDENTIFY_AWAKE_MS: u32 = {max_identify};\n\
                 pub const MAX_FAST_POLL_MS: u32 = {max_fast};\n\
                 pub const MISSED_REPORT_GRACE_MS: u32 = {missed_grace};\n\
                 #[allow(dead_code)]\n\
                 pub const ANOMALY_MAX_WRITES_PER_BOOT: u8 = {anomaly_writes};\n\
                 pub const SLEEP_FAILURE_LIMIT: u8 = {sleep_failures};\n\
                 pub const RAPID_RESET_LIMIT: u8 = {rapid_limit};\n\
                 pub const RAPID_RESET_WINDOW_MS: u32 = {rapid_window};\n\
                 pub const SLEEP_XTAL_OFF: bool = {xtal_off};\n\
                 pub const SLEEP_RC_FAST_OFF: bool = {fosc_off};\n\
                 #[allow(dead_code)]\n\
                 pub const LOG_LEVEL: log::LevelFilter = log::LevelFilter::{log_level};\n\
                 #[allow(dead_code)]\n\
                 pub const DIAG_SUMMARY_MS: u32 = {diag_summary_ms};\n",
                model = self.model,
                ota = self.ota_image_type,
                role = self.role_tag,
                min_m = self.min_measured,
                max_m = self.max_measured,
                rmin = self.report_min_s,
                rmax = self.report_max_s,
                rchg = self.report_change,
                slow = self.slow_poll_ms,
                fast = self.fast_poll_ms,
                fresh = self.fresh_join_fast_ms,
                restored = self.restored_fast_ms,
                button = self.button_fast_poll_ms,
                every = self.sample_every_polls,
                usb = self.usb_host_wait_ms,
                fast_light = self.fast_poll_light_sleep,
                check_in = self.check_in_interval_s * 4,
                check_in_min = self.check_in_interval_min_s * 4,
                long_qs = self.slow_poll_ms / QS_MS,
                long_min = self.long_poll_min_ms / QS_MS,
                long_max = self.long_poll_max_ms / QS_MS,
                short_qs = self.fast_poll_ms / QS_MS,
                fpt = self.fast_poll_timeout_ms / QS_MS,
                fpt_max = self.fast_poll_timeout_max_ms / QS_MS,
                resp = self.check_in_response_ms / QS_MS,
                jr = self.join_retry_ms,
                jr_max = self.join_retry_max_ms,
                jitter = self.join_retry_jitter_pct,
                strikes = self.parent_loss_strikes,
                max_active = self.max_active_ms,
                aps_ack = self.report_aps_ack,
                max_ota = self.max_ota_awake_ms,
                max_identify = self.max_identify_awake_ms,
                max_fast = self.max_fast_poll_ms,
                missed_grace = self.missed_report_grace_ms,
                anomaly_writes = self.anomaly_max_writes_per_boot,
                sleep_failures = self.sleep_failure_limit,
                rapid_limit = self.rapid_reset_limit,
                rapid_window = self.rapid_reset_window_ms,
                xtal_off = self.sleep_xtal_off,
                fosc_off = self.sleep_rc_fast_off,
                log_level = self.log_level,
                diag_summary_ms = self.diag_summary_ms,
            )
        }
    }

    /// `key = value` lines; `#` starts a comment outside strings.
    fn parse(text: &str, path: &Path) -> BTreeMap<String, String> {
        let mut values = BTreeMap::new();
        for (number, line) in text.lines().enumerate() {
            let line = strip_comment(line).trim();
            if line.is_empty() {
                continue;
            }
            let (key, value) = line.split_once('=').unwrap_or_else(|| {
                panic!("{}:{}: expected `key = value`", path.display(), number + 1)
            });
            let key = key.trim().to_owned();
            if values
                .insert(key.clone(), value.trim().to_owned())
                .is_some()
            {
                panic!("{}:{}: duplicate key `{key}`", path.display(), number + 1);
            }
        }
        values
    }

    fn strip_comment(line: &str) -> &str {
        let mut quoted = false;
        for (index, byte) in line.bytes().enumerate() {
            match byte {
                b'"' => quoted = !quoted,
                b'#' if !quoted => return &line[..index],
                _ => {}
            }
        }
        line
    }

    /// File value, overridden by `H2_THERMO_<KEY>`.
    fn take(values: &mut BTreeMap<String, String>, key: &str) -> String {
        let variable = format!("H2_THERMO_{}", key.to_ascii_uppercase());
        println!("cargo:rerun-if-env-changed={variable}");
        let from_file = values.remove(key);
        env::var(&variable)
            .ok()
            .or(from_file)
            .unwrap_or_else(|| panic!("thermometer config is missing `{key}`"))
    }

    fn string(values: &mut BTreeMap<String, String>, key: &str) -> String {
        let raw = take(values, key);
        let raw = raw.trim();
        raw.strip_prefix('"')
            .and_then(|s| s.strip_suffix('"'))
            .unwrap_or(raw)
            .to_owned()
    }

    fn boolean(values: &mut BTreeMap<String, String>, key: &str) -> bool {
        match take(values, key).trim() {
            "true" => true,
            "false" => false,
            other => panic!("`{key}` = `{other}` must be true or false"),
        }
    }

    /// `log::LevelFilter` variant name for a console log level.
    fn log_level(values: &mut BTreeMap<String, String>, key: &str) -> &'static str {
        match string(values, key).as_str() {
            "off" => "Off",
            "error" => "Error",
            "warn" => "Warn",
            "info" => "Info",
            "debug" => "Debug",
            "trace" => "Trace",
            other => panic!("`{key}` = `{other}` must be off, error, warn, info, debug or trace"),
        }
    }

    fn int(values: &mut BTreeMap<String, String>, key: &str, min: i64, max: i64) -> i64 {
        let raw = take(values, key);
        let raw = raw.trim();
        let value = match raw.strip_prefix('-') {
            Some(rest) => parse_u32(rest).map(|v| -i64::from(v)),
            None => parse_u32(raw).map(i64::from),
        }
        .unwrap_or_else(|_| panic!("`{key}` = `{raw}` is not an integer"));
        assert!(
            (min..=max).contains(&value),
            "`{key}` = {value} is outside {min}..={max}"
        );
        value
    }
}
