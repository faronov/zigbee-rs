use std::env;
use std::fs;
use std::path::Path;

const MAGIC: u32 = 0x3659_4850;
const RUN_ADDRESS: u32 = 0x1fff_1838;
const FLASH_WRITE_ADDRESS: u32 = 0x0001_0000;
const XIP_BASE: u32 = 0x1100_0000;
const XIP_MASK: u32 = 0x001f_ffff;
const HEADER_SIZE: usize = 0x100;
const MAX_SEGMENTS: usize = 15;

#[derive(Debug, Clone, PartialEq, Eq)]
struct Segment {
    address: u32,
    data: Vec<u8>,
}

fn main() {
    if let Err(error) = run(env::args().skip(1)) {
        eprintln!("phy62x2-image: {error}");
        std::process::exit(1);
    }
}

fn run(args: impl Iterator<Item = String>) -> Result<(), String> {
    let args: Vec<_> = args.collect();
    if args.len() != 2 {
        return Err("usage: phy62x2-image <firmware.hex> <firmware.bin>".into());
    }

    let input = fs::read_to_string(&args[0])
        .map_err(|error| format!("failed to read {}: {error}", args[0]))?;
    let segments = normalize_segments(parse_hex(&input)?)?;
    let image = build_image(&segments)?;
    write_image(&args[1], &image)
}

fn write_image(path: &str, image: &[u8]) -> Result<(), String> {
    if let Some(parent) = Path::new(path).parent()
        && !parent.as_os_str().is_empty()
    {
        fs::create_dir_all(parent)
            .map_err(|error| format!("failed to create {}: {error}", parent.display()))?;
    }
    fs::write(path, image).map_err(|error| format!("failed to write {path}: {error}"))
}

fn parse_hex(input: &str) -> Result<Vec<Segment>, String> {
    let mut upper_address = 0u32;
    let mut segments = Vec::<Segment>::new();

    for (line_index, raw_line) in input.lines().enumerate() {
        let line_number = line_index + 1;
        let line = raw_line.trim();
        if line.is_empty() {
            continue;
        }
        let bytes = decode_hex_record(line)
            .map_err(|error| format!("Intel HEX line {line_number}: {error}"))?;
        let count = bytes[0] as usize;
        let offset = u16::from_be_bytes([bytes[1], bytes[2]]) as u32;
        let record_type = bytes[3];
        let data = &bytes[4..4 + count];

        match record_type {
            0x00 => {
                let address = upper_address
                    .checked_add(offset)
                    .ok_or_else(|| format!("Intel HEX line {line_number}: address overflow"))?;
                if let Some(last) = segments.last_mut()
                    && last.address + last.data.len() as u32 == address
                {
                    last.data.extend_from_slice(data);
                } else {
                    segments.push(Segment {
                        address,
                        data: data.to_vec(),
                    });
                }
            }
            0x01 => break,
            0x04 if data.len() == 2 => {
                upper_address = u16::from_be_bytes([data[0], data[1]]) as u32 * 0x1_0000;
            }
            0x05 => {}
            other => {
                return Err(format!(
                    "Intel HEX line {line_number}: unsupported record type 0x{other:02x}"
                ));
            }
        }
    }

    if segments.is_empty() {
        return Err("Intel HEX file contains no loadable data".into());
    }
    Ok(segments)
}

fn decode_hex_record(line: &str) -> Result<Vec<u8>, String> {
    let payload = line
        .strip_prefix(':')
        .ok_or_else(|| "record does not start with ':'".to_string())?;
    if payload.len() < 10 || payload.len() % 2 != 0 {
        return Err("invalid record length".into());
    }

    let mut bytes = Vec::with_capacity(payload.len() / 2);
    for index in (0..payload.len()).step_by(2) {
        let byte = u8::from_str_radix(&payload[index..index + 2], 16)
            .map_err(|_| "record contains non-hexadecimal data".to_string())?;
        bytes.push(byte);
    }

    let count = bytes[0] as usize;
    if bytes.len() != count + 5 {
        return Err("byte count does not match record length".into());
    }
    if bytes.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte)) != 0 {
        return Err("checksum mismatch".into());
    }
    Ok(bytes)
}

fn normalize_segments(mut segments: Vec<Segment>) -> Result<Vec<Segment>, String> {
    segments.sort_by_key(|segment| segment.address);
    let mut normalized = Vec::<Segment>::new();

    for segment in segments {
        classify_address(segment.address)?;
        if let Some(last) = normalized.last_mut() {
            let last_class = classify_address(last.address)?;
            let class = classify_address(segment.address)?;
            let last_end = last
                .address
                .checked_add(last.data.len() as u32)
                .ok_or_else(|| "segment address overflow".to_string())?;
            if segment.address < last_end {
                return Err(format!(
                    "overlapping segments at 0x{:08x} and 0x{:08x}",
                    last.address, segment.address
                ));
            }
            if last_class == class && segment.address == last_end {
                last.data.extend_from_slice(&segment.data);
                continue;
            }
        }
        normalized.push(segment);
    }

    if normalized.len() > MAX_SEGMENTS {
        return Err(format!(
            "{} loadable segments exceed the PHY6 limit of {MAX_SEGMENTS}",
            normalized.len()
        ));
    }
    Ok(normalized)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AddressClass {
    Xip,
    Sram,
}

fn classify_address(address: u32) -> Result<AddressClass, String> {
    if address & !XIP_MASK == XIP_BASE {
        return Ok(AddressClass::Xip);
    }
    if address & 0x1fff_0000 == 0x1fff_0000 {
        return Ok(AddressClass::Sram);
    }
    Err(format!("unsupported load address 0x{address:08x}"))
}

fn build_image(segments: &[Segment]) -> Result<Vec<u8>, String> {
    if segments.is_empty() || segments.len() > MAX_SEGMENTS {
        return Err("invalid PHY6 segment count".into());
    }
    if segments
        .iter()
        .all(|segment| segment.address != RUN_ADDRESS)
    {
        return Err(format!(
            "no SRAM run-descriptor segment starts at 0x{RUN_ADDRESS:08x}"
        ));
    }

    let first_xip = FLASH_WRITE_ADDRESS + HEADER_SIZE as u32;
    let mut xip_end = first_xip;
    let mut has_xip = false;
    let mut total_sram = 0u32;
    for segment in segments {
        match classify_address(segment.address)? {
            AddressClass::Xip => {
                let flash_address = segment.address & XIP_MASK;
                if !has_xip && flash_address != first_xip {
                    return Err(format!(
                        "first XIP segment 0x{:08x} maps to 0x{flash_address:06x}, expected 0x{first_xip:06x}",
                        segment.address
                    ));
                }
                if flash_address < xip_end {
                    return Err(format!(
                        "XIP segment 0x{:08x} maps to 0x{flash_address:06x}, before previous end 0x{xip_end:06x}",
                        segment.address
                    ));
                }
                xip_end = align4(
                    flash_address
                        .checked_add(segment.data.len() as u32)
                        .ok_or_else(|| "XIP segment size overflow".to_string())?,
                );
                has_xip = true;
            }
            AddressClass::Sram => {
                total_sram = total_sram
                    .checked_add(align4(segment.data.len() as u32))
                    .ok_or_else(|| "SRAM segment size overflow".to_string())?;
            }
        }
    }

    let mut next_sram_flash = first_xip;
    if next_sram_flash + total_sram >= first_xip {
        next_sram_flash = xip_end;
    }

    let mut placements = Vec::with_capacity(segments.len());
    for segment in segments {
        let flash_address = match classify_address(segment.address)? {
            AddressClass::Xip => segment.address & XIP_MASK,
            AddressClass::Sram => {
                let address = next_sram_flash;
                next_sram_flash = next_sram_flash
                    .checked_add(align4(segment.data.len() as u32))
                    .ok_or_else(|| "PHY6 image size overflow".to_string())?;
                address
            }
        };
        placements.push(flash_address);
    }

    let mut image = vec![0xff; HEADER_SIZE];
    put_u32(&mut image, 0, MAGIC);
    put_u32(&mut image, 4, segments.len() as u32);
    put_u32(&mut image, 8, RUN_ADDRESS);

    for (index, (segment, flash_address)) in segments.iter().zip(placements.iter()).enumerate() {
        let table_offset = 16 + index * 16;
        put_u32(&mut image, table_offset, *flash_address);
        put_u32(&mut image, table_offset + 4, segment.data.len() as u32);
        put_u32(&mut image, table_offset + 8, segment.address);
        put_u32(&mut image, table_offset + 12, !crc32(&segment.data));

        let output_offset = flash_address
            .checked_sub(FLASH_WRITE_ADDRESS)
            .ok_or_else(|| "segment is below the image write address".to_string())?
            as usize;
        if output_offset < HEADER_SIZE {
            return Err("segment overlaps the PHY6 header".into());
        }
        image.resize(output_offset, 0xff);
        image.extend_from_slice(&segment.data);
        while !image.len().is_multiple_of(4) {
            image.push(0xff);
        }
    }

    while !image.len().is_multiple_of(16) {
        image.push(0xff);
    }
    let image_size = image.len() as u32;
    put_u32(&mut image, 12, image_size);
    let file_crc = !crc32(&image);
    image.extend_from_slice(&file_crc.to_le_bytes());
    Ok(image)
}

fn align4(value: u32) -> u32 {
    (value + 3) & !3
}

fn put_u32(output: &mut [u8], offset: usize, value: u32) {
    output[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xffff_ffffu32;
    for byte in data {
        crc ^= *byte as u32;
        for _ in 0..8 {
            let mask = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xedb8_8320 & mask);
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex_record(address: u16, record_type: u8, data: &[u8]) -> String {
        let mut bytes = Vec::with_capacity(data.len() + 5);
        bytes.push(data.len() as u8);
        bytes.extend_from_slice(&address.to_be_bytes());
        bytes.push(record_type);
        bytes.extend_from_slice(data);
        let checksum = 0u8.wrapping_sub(bytes.iter().fold(0u8, |sum, b| sum.wrapping_add(*b)));
        bytes.push(checksum);
        let mut output = String::from(":");
        for byte in bytes {
            output.push_str(&format!("{byte:02X}"));
        }
        output
    }

    #[test]
    fn parses_extended_linear_addresses() {
        let input = [
            hex_record(0, 4, &[0x11, 0x01]),
            hex_record(0x0100, 0, &[1, 2, 3, 4]),
            hex_record(0, 4, &[0x1f, 0xff]),
            hex_record(0x1838, 0, &[5, 6, 7, 8]),
            hex_record(0, 1, &[]),
        ]
        .join("\n");

        assert_eq!(
            parse_hex(&input).unwrap(),
            vec![
                Segment {
                    address: 0x1101_0100,
                    data: vec![1, 2, 3, 4],
                },
                Segment {
                    address: 0x1fff_1838,
                    data: vec![5, 6, 7, 8],
                },
            ]
        );
    }

    #[test]
    fn creates_phy6_header_and_trailing_crc() {
        let segments = vec![
            Segment {
                address: 0x1101_0100,
                data: vec![1, 2, 3],
            },
            Segment {
                address: 0x1fff_0000,
                data: vec![0; 0x400],
            },
            Segment {
                address: RUN_ADDRESS,
                data: vec![0; 8],
            },
        ];
        let image = build_image(&segments).unwrap();

        assert_eq!(u32::from_le_bytes(image[0..4].try_into().unwrap()), MAGIC);
        assert_eq!(u32::from_le_bytes(image[4..8].try_into().unwrap()), 3);
        assert_eq!(
            u32::from_le_bytes(image[8..12].try_into().unwrap()),
            RUN_ADDRESS
        );
        let size = u32::from_le_bytes(image[12..16].try_into().unwrap()) as usize;
        assert_eq!(size + 4, image.len());
        let stored_crc = u32::from_le_bytes(image[size..size + 4].try_into().unwrap());
        assert_eq!(stored_crc, !crc32(&image[..size]));
    }

    #[test]
    fn preserves_alignment_gaps_between_xip_segments() {
        let segments = vec![
            Segment {
                address: 0x1101_0100,
                data: vec![1, 2, 3, 4],
            },
            Segment {
                address: 0x1101_0108,
                data: vec![5, 6, 7, 8],
            },
            Segment {
                address: RUN_ADDRESS,
                data: vec![0; 8],
            },
        ];

        let image = build_image(&segments).unwrap();
        assert_eq!(&image[HEADER_SIZE..HEADER_SIZE + 4], &[1, 2, 3, 4]);
        assert_eq!(&image[HEADER_SIZE + 4..HEADER_SIZE + 8], &[0xFF; 4]);
        assert_eq!(&image[HEADER_SIZE + 8..HEADER_SIZE + 12], &[5, 6, 7, 8]);
    }

    #[test]
    fn keeps_discontiguous_sram_regions_separate() {
        let segments = normalize_segments(vec![
            Segment {
                address: 0x1fff_0000,
                data: vec![0; 0x400],
            },
            Segment {
                address: RUN_ADDRESS,
                data: vec![1; 8],
            },
        ])
        .unwrap();

        assert_eq!(segments.len(), 2);
        assert_eq!(segments[0].address, 0x1fff_0000);
        assert_eq!(segments[1].address, RUN_ADDRESS);
    }

    #[test]
    fn rejects_overlapping_segments() {
        let error = normalize_segments(vec![
            Segment {
                address: 0x1101_0100,
                data: vec![0; 8],
            },
            Segment {
                address: 0x1101_0104,
                data: vec![0; 8],
            },
        ])
        .unwrap_err();

        assert!(error.contains("overlapping segments"));
    }

    #[test]
    fn rejects_bad_hex_checksum() {
        let error = parse_hex(":00000001FE\n").unwrap_err();
        assert!(error.contains("checksum mismatch"));
    }
}
