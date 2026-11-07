//! Status lines on a byte console (typically the product's UART).

use crate::{BootEvent, ImageError, ImageVersion, Trial, Update};

/// Byte console for loader status lines.
pub trait Console {
    fn write(&mut self, bytes: &[u8]);
}

/// Console that discards everything.
pub struct NoConsole;

impl Console for NoConsole {
    fn write(&mut self, _bytes: &[u8]) {}
}

/// Write `value` as `0x` and eight hex digits.
pub fn write_hex(console: &mut impl Console, value: u32) {
    let mut text = *b"0x00000000";
    for (index, digit) in text[2..].iter_mut().enumerate() {
        let nibble = (value >> (28 - 4 * index)) & 0xF;
        *digit = b"0123456789abcdef"[nibble as usize];
    }
    console.write(&text);
}

/// Write `v<major>.<minor>.<revision>+<build>`.
pub fn write_version(console: &mut impl Console, version: &ImageVersion) {
    console.write(b"v");
    write_decimal(console, u32::from(version.major));
    console.write(b".");
    write_decimal(console, u32::from(version.minor));
    console.write(b".");
    write_decimal(console, u32::from(version.revision));
    console.write(b"+");
    write_decimal(console, version.build);
}

/// Write `value` in decimal.
pub fn write_decimal(console: &mut impl Console, mut value: u32) {
    let mut text = [0u8; 10];
    let mut start = text.len();
    loop {
        start -= 1;
        text[start] = b'0' + (value % 10) as u8;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    console.write(&text[start..]);
}

/// Short name of an image validation failure.
pub fn image_error_name(error: ImageError) -> &'static [u8] {
    match error {
        ImageError::Flash => b"flash",
        ImageError::NoImage => b"no image",
        ImageError::Malformed => b"malformed",
        ImageError::Unsupported => b"unsupported",
        ImageError::HashMismatch => b"hash mismatch",
        ImageError::BadSignature => b"bad signature",
    }
}

/// Turns boot progress and outcomes into console lines, each starting with
/// the loader's `prefix` (for example `b"bl702-boot: "`).
#[derive(Debug)]
pub struct Reporter {
    prefix: &'static [u8],
    copying: bool,
}

impl Reporter {
    pub const fn new(prefix: &'static [u8]) -> Self {
        Self {
            prefix,
            copying: false,
        }
    }

    /// Write the line prefix.
    pub fn prefix(&self, console: &mut impl Console) {
        console.write(self.prefix);
    }

    /// Write `prefix`, `text`.
    pub fn line(&self, console: &mut impl Console, text: &[u8]) {
        console.write(self.prefix);
        console.write(text);
    }

    /// Boot progress: one line when an install starts and a dot per 64 KiB
    /// copied.
    pub fn event(&mut self, console: &mut impl Console, event: BootEvent) {
        match event {
            BootEvent::Erasing => self.line(console, b"installing update "),
            BootEvent::Copying { done, .. } => {
                self.copying = true;
                if done.is_multiple_of(64 * 1024) {
                    console.write(b".");
                }
            }
            BootEvent::Validating | BootEvent::Dropping => self.end_line(console),
            #[cfg(feature = "swap-move")]
            BootEvent::Reverting => self.line(console, b"reverting update "),
        }
    }

    /// Outcome of the upgrade-request step.
    pub fn update(&mut self, console: &mut impl Console, update: Update) {
        self.end_line(console);
        match update {
            Update::None => {}
            Update::Installed => self.line(console, b"update installed\r\n"),
            Update::Downgrade => self.line(console, b"update rejected: downgrade\r\n"),
            #[cfg(feature = "swap-move")]
            Update::Reverted => self.line(console, b"unconfirmed update reverted\r\n"),
            Update::Rejected(error) => {
                self.line(console, b"update rejected: ");
                console.write(image_error_name(error));
                console.write(b"\r\n");
            }
        }
    }

    /// Trial state of the image about to start: a line while it is on
    /// trial or left unconfirmed, nothing otherwise.
    pub fn trial(&mut self, console: &mut impl Console, trial: Trial) {
        match trial {
            Trial::Testing { boot } => {
                self.line(console, b"trial boot ");
                write_decimal(console, u32::from(boot));
                console.write(b"\r\n");
            }
            Trial::Unconfirmed => self.line(console, b"image unconfirmed\r\n"),
            Trial::Permanent | Trial::Confirmed => {}
        }
    }

    /// Terminate a progress line, if one is open.
    pub fn end_line(&mut self, console: &mut impl Console) {
        if core::mem::take(&mut self.copying) {
            console.write(b"\r\n");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Text(Vec<u8>);

    impl Console for Text {
        fn write(&mut self, bytes: &[u8]) {
            self.0.extend_from_slice(bytes);
        }
    }

    #[test]
    fn number_formatting() {
        let mut text = Text(Vec::new());
        write_hex(&mut text, 0x2300_E100);
        text.write(b" ");
        write_version(&mut text, &ImageVersion::new(0, 1, 2, 0));
        text.write(b" ");
        write_decimal(&mut text, u32::MAX);
        assert_eq!(text.0, b"0x2300e100 v0.1.2+0 4294967295");
    }

    #[test]
    fn progress_and_outcome_lines() {
        let mut text = Text(Vec::new());
        let mut reporter = Reporter::new(b"x-boot: ");
        let total = 0x30000;
        reporter.event(&mut text, BootEvent::Validating);
        reporter.event(&mut text, BootEvent::Erasing);
        let mut done = 0;
        while done < total {
            reporter.event(&mut text, BootEvent::Copying { done, total });
            done += 256;
        }
        reporter.event(&mut text, BootEvent::Validating);
        reporter.event(&mut text, BootEvent::Dropping);
        reporter.update(&mut text, Update::Installed);
        reporter.update(&mut text, Update::Downgrade);
        reporter.update(&mut text, Update::None);
        reporter.update(&mut text, Update::Rejected(ImageError::HashMismatch));
        assert_eq!(
            text.0,
            b"x-boot: installing update ...\r\nx-boot: update installed\r\n\
              x-boot: update rejected: downgrade\r\n\
              x-boot: update rejected: hash mismatch\r\n"
        );
    }
}
