//! Per-die BL702 RF calibration.

use super::{
    RfError, capture_samples, configure_pucr, read_bt_lo, read_lo_acal, read_rccal, read_rx_offset,
    restore_sample_ram, rmw, sample, save_sample_ram, switch_channel, wait_for_set,
    with_saved_state, write_bt_lo, write_lo_acal, write_rccal, write_rx_offset, write_zb_lo,
};
use crate::bl702::registers::{read32, write32};

pub(super) fn run_full_calibration<D>(delay_us: &mut D) -> Result<(), RfError>
where
    D: FnMut(u32),
{
    unsafe {
        core::ptr::write_volatile(0x0280_0003 as *mut u8, 0);
    }

    let saved_samples = save_sample_ram();
    let result = (|| {
        run_acal(delay_us)?;
        run_kcal(delay_us)?;
        run_roscal(delay_us)?;
        run_rccal(delay_us)
    })();
    restore_sample_ram(&saved_samples);
    result
}

fn run_acal<D>(delay_us: &mut D) -> Result<(), RfError>
where
    D: FnMut(u32),
{
    if read32(0x4000_100c) & 0x2 == 0 {
        return Ok(());
    }

    rmw(0x4000_1014, 0xffff_fff3, 0x4);
    with_saved_state(|| {
        rmw(0x4000_1008, 0xefef_fffe, 0);
        configure_pucr(3);
        rmw(0x4000_1010, u32::MAX, 0x1);
        rmw(0x4000_1238, 0xffc0_ff8f, 0x0020_0030);

        let value = read32(0x4000_1268);
        write32(0x4000_1268, value);

        calibrate_acal_point(delay_us, 2410.0, 0, 0)?;
        calibrate_acal_point(delay_us, 2430.0, 8, 6)?;
        calibrate_acal_point(delay_us, 2450.0, 16, 12)?;
        calibrate_acal_point(delay_us, 2470.0, 24, 18)?;
        Ok(())
    })?;
    rmw(0x4000_1014, u32::MAX, 0x0c);
    Ok(())
}

fn calibrate_acal_point<D>(
    delay_us: &mut D,
    frequency_mhz: f64,
    live_shift: u32,
    cache_shift: u32,
) -> Result<(), RfError>
where
    D: FnMut(u32),
{
    switch_channel(delay_us, frequency_mhz);
    let code = u32::from(acal_core(delay_us)?);

    let live_mask = !(0x3f << live_shift);
    rmw(0x4000_1400, live_mask, code << live_shift);

    let cached = read_lo_acal();
    let cache_mask = !(0x3f << cache_shift);
    write_lo_acal((cached & cache_mask) | ((code & 0x3f) << cache_shift));
    Ok(())
}

fn acal_core<D>(delay_us: &mut D) -> Result<u16, RfError>
where
    D: FnMut(u32),
{
    rmw(0x4000_1238, 0xffc0_ffff, 0x0020_0000);
    let mut code = 32i16;

    for bit in (0..=4).rev() {
        let step = 1i16 << bit;
        if read32(0x4000_1238) & (1 << 31) != 0 {
            code -= step;
        } else {
            code += step;
        }
        if !(0..=63).contains(&code) {
            return Err(RfError::InvalidMeasurement);
        }
        rmw(0x4000_1238, 0xffc0_ffff, (code as u32) << 16);
        delay_us(1);
    }

    if read32(0x4000_1238) & (1 << 31) == 0 && code <= 62 {
        code += 1;
    }
    Ok(code as u16)
}

fn run_kcal<D>(delay_us: &mut D) -> Result<(), RfError>
where
    D: FnMut(u32),
{
    if read32(0x4000_100c) & 0x4 == 0 {
        return Ok(());
    }

    rmw(0x4000_1014, 0xffff_ffcf, 0x10);
    with_saved_state(|| {
        switch_channel(delay_us, 2402.0);
        let first = kcal_core(delay_us)?;
        write_lo_ratio(true, 0, first);

        switch_channel(delay_us, 2480.0);
        let last = kcal_core(delay_us)?;
        write_lo_ratio(true, 39, last);

        let first = i32::from(read_lo_ratio(true, 0));
        let last = i32::from(read_lo_ratio(true, 39));
        let coefficient = f64::from(last - first) / 39.0;

        for channel in 0..40 {
            let interpolated = (channel as f64 * coefficient + f64::from(first)) as u32 & 0x3ff;
            write_lo_ratio(true, channel, interpolated as u16);
            write32(0x4000_1404 + channel as u32 * 4, interpolated);
        }

        for channel in 0..8 {
            let ratio = read32(0x4000_1404 + (2 + 5 * channel) as u32 * 4) & 0x3ff;
            write32(0x4000_14a4 + channel as u32 * 4, ratio);
            write_lo_ratio(false, channel, ratio as u16);
        }
        Ok(())
    })?;
    rmw(0x4000_1014, u32::MAX, 0x30);
    Ok(())
}

fn kcal_core<D>(delay_us: &mut D) -> Result<u16, RfError>
where
    D: FnMut(u32),
{
    rmw(0x4000_127c, 0xffff_7fff, 0);
    rmw(0x4000_1244, 0xffff_0000, 0xdac0);
    rmw(0x4000_1010, u32::MAX, 1 << 4);
    rmw(0x4000_1238, u32::MAX, 0x0c);
    rmw(0x4000_1100, 0xfbff_ffff, 0);
    delay_us(1);

    rmw(0x4000_1244, u32::MAX, 1 << 16);
    wait_for_set(0x4000_1248, 1 << 16)?;
    let first = read32(0x4000_1248);
    rmw(0x4000_1244, 0xfffe_ffff, 0);

    rmw(0x4000_1238, 0xffff_fff3, 0);
    rmw(0x4000_1100, u32::MAX, 1 << 26);
    delay_us(1);

    rmw(0x4000_1244, u32::MAX, 1 << 16);
    wait_for_set(0x4000_1248, 1 << 16)?;
    let second = read32(0x4000_1248);
    rmw(0x4000_1244, 0xfffe_ffff, 0);

    let delta = (second as u16 as i32) - (first as u16 as i32);
    let mut scaled = f64::from(delta);
    scaled *= 0.25;
    scaled *= 4.0;
    scaled *= 32.0;
    scaled /= 56_000.0;
    let delta_flo = scaled as f32;
    if !delta_flo.is_finite() || delta_flo == 0.0 {
        return Err(RfError::InvalidMeasurement);
    }

    let value = (0.5f32 / delta_flo) * 1024.0;
    if !value.is_finite() || !(0.0..=1023.0).contains(&value) {
        return Err(RfError::InvalidMeasurement);
    }
    let kcal = value as u16;
    rmw(0x4000_1244, 0xc00f_ffff, u32::from(kcal) << 20);
    Ok(kcal)
}

fn read_lo_ratio(bt: bool, index: usize) -> u16 {
    let raw = if bt {
        read_bt_lo(index)
    } else {
        super::read_zb_lo(index)
    };
    (raw >> 1) & 0x03ff
}

fn write_lo_ratio(bt: bool, index: usize, ratio: u16) {
    let raw = if bt {
        read_bt_lo(index)
    } else {
        super::read_zb_lo(index)
    };
    let value = (raw & 0xf801) | ((ratio & 0x03ff) << 1);
    if bt {
        write_bt_lo(index, value);
    } else {
        write_zb_lo(index, value);
    }
}

fn run_roscal<D>(delay_us: &mut D) -> Result<(), RfError>
where
    D: FnMut(u32),
{
    if read32(0x4000_100c) & 0x8 == 0 {
        return Ok(());
    }

    rmw(0x4000_1014, 0xffff_ff3f, 0x40);
    let any_success = with_saved_state(|| {
        rmw(0x4000_150c, u32::MAX, 0x3000_0000);
        switch_channel(delay_us, 2496.0);
        rmw(0x4000_0050, u32::MAX, 0x400);
        rmw(0x4000_1004, 0x5fff_ffff, 0);

        rmw(0x4000_1008, 0xffff_fcfe, 0);
        write32(0x4000_1100, read32(0x4000_1114) ^ 0x0020_0000);

        rmw(0x4000_1510, u32::MAX, 0x0140_0000);
        rmw(0x4000_1510, u32::MAX, 0x0280_0000);
        write32(0x4000_152c, 0x1000_1100);
        rmw(0x4000_1528, 0xffff_ffcb, 0x10);

        let mut any_success = false;
        for gain in 6..16 {
            rmw(0x4000_1004, 0xe0ff_ffff, (gain as u32 & 0x1f) << 24);
            for bandwidth in 0..2 {
                rmw(0x4000_1004, 0xbfff_ffff, (bandwidth as u32) << 30);

                let success = roscal_core(delay_us)?;
                if success {
                    any_success = true;
                    rmw(0x4000_1014, 0xffff_ff3f, 0x80);
                }

                let result = read32(0x4000_120c);
                let code_i = (result >> 24) & 0x3f;
                let code_q = (result >> 16) & 0x3f;
                store_roscal_result(gain, bandwidth, code_i, code_q);
            }
        }

        let source = read_rx_offset(6);
        for gain in 0..6 {
            let value = merge_roscal_fields(read_rx_offset(gain), source);
            write_rx_offset(gain, value);
            let address = 0x4000_1300 + gain as u32 * 4;
            write32(address, merge_roscal_fields(read32(address), source));
        }
        Ok(any_success)
    })?;

    if !any_success {
        return Err(RfError::InvalidMeasurement);
    }
    rmw(0x4000_1014, u32::MAX, 0xc0);
    Ok(())
}

fn roscal_core<D>(delay_us: &mut D) -> Result<bool, RfError>
where
    D: FnMut(u32),
{
    let mut code_i = 32i32;
    let mut code_q = 32i32;
    let mut sequence_i = 0u32;
    let mut sequence_q = 0u32;

    for _ in 0..127 {
        let (quadrant, average_i, average_q) = ros_result(delay_us)?;
        if (-2..=2).contains(&average_i) && (-2..=2).contains(&average_q) {
            write_rosdac(code_i, code_q)?;
            return Ok(true);
        }
        if code_i == 0 || code_i == 63 || code_q == 0 || code_q == 63 {
            return Ok(false);
        }

        match quadrant {
            3 => {
                code_i -= 1;
                code_q -= 1;
                sequence_i = ((sequence_i << 1) | 1) & 0xf;
                sequence_q = ((sequence_q << 1) | 1) & 0xf;
            }
            -3 => {
                code_i += 1;
                code_q += 1;
                sequence_i = (sequence_i << 1) & 0xf;
                sequence_q = (sequence_q << 1) & 0xf;
            }
            -1 => {
                code_i += 1;
                code_q -= 1;
                sequence_i = (sequence_i << 1) & 0xf;
                sequence_q = ((sequence_q << 1) | 1) & 0xf;
            }
            1 => {
                code_i -= 1;
                code_q += 1;
                sequence_i = ((sequence_i << 1) | 1) & 0xf;
                sequence_q = (sequence_q << 1) & 0xf;
            }
            _ => return Err(RfError::InvalidMeasurement),
        }

        write_rosdac(code_i, code_q)?;
        delay_us(5);
        if matches!(sequence_i, 5 | 10) && matches!(sequence_q, 5 | 10) {
            write_rosdac(code_i, code_q)?;
            return Ok(true);
        }
    }
    Ok(false)
}

fn ros_result<D>(delay_us: &mut D) -> Result<(i32, i32, i32), RfError>
where
    D: FnMut(u32),
{
    rmw(0x4000_1528, u32::MAX, 0x2);
    wait_for_set(0x4000_1528, 0x1)?;
    rmw(0x4000_1528, 0xffff_fffd, 0);
    delay_us(5);

    let mut sum_i = 0i32;
    let mut sum_q = 0i32;
    for index in 0..256 {
        let value = sample(index);
        sum_i = sum_i.wrapping_add((value as i32) >> 16);
        sum_q = sum_q.wrapping_add(i32::from(value as u16 as i16));
    }

    let average_i = sum_i / 256;
    let average_q = sum_q / 256;
    let quadrant = if sum_i >= 0 {
        if sum_q < 0 { 1 } else { 3 }
    } else if sum_q < 0 {
        -3
    } else {
        -1
    };
    Ok((quadrant, average_i, average_q))
}

fn write_rosdac(code_i: i32, code_q: i32) -> Result<(), RfError> {
    if !(0..=63).contains(&code_i) || !(0..=63).contains(&code_q) {
        return Err(RfError::InvalidMeasurement);
    }
    let mut value = read32(0x4000_120c);
    value = (value & 0xc0ff_ffff) | ((code_i as u32) << 24);
    value = (value & 0xffc0_ffff) | ((code_q as u32) << 16);
    write32(0x4000_120c, value);
    Ok(())
}

fn store_roscal_result(gain: usize, bandwidth: usize, code_i: u32, code_q: u32) {
    let (mask_i, shift_i, mask_q, shift_q) = if bandwidth == 0 {
        (0x0000_0fc0, 6, 0x0000_003f, 0)
    } else {
        (0x00fc_0000, 18, 0x0003_f000, 12)
    };

    let cached = read_rx_offset(gain);
    let cached = (cached & !mask_i) | ((code_i << shift_i) & mask_i);
    let cached = (cached & !mask_q) | ((code_q << shift_q) & mask_q);
    write_rx_offset(gain, cached);

    let address = 0x4000_1300 + gain as u32 * 4;
    let live = read32(address);
    let live = (live & !mask_i) | ((code_i << shift_i) & mask_i);
    let live = (live & !mask_q) | ((code_q << shift_q) & mask_q);
    write32(address, live);
}

fn merge_roscal_fields(destination: u32, source: u32) -> u32 {
    const MASK: u32 = 0x00ff_ffff;
    (destination & !MASK) | (source & MASK)
}

fn run_rccal<D>(delay_us: &mut D) -> Result<(), RfError>
where
    D: FnMut(u32),
{
    if read32(0x4000_100c) & 0x10 == 0 {
        return Ok(());
    }

    rmw(0x4000_1014, 0xffff_fcff, 0x100);
    with_saved_state(|| {
        rmw(0x4000_150c, u32::MAX, 0x3000_0000);
        rmw(0x4000_0050, u32::MAX, 0x400);
        rmw(0x4000_1008, 0xfffe_ffee, 0x300);
        write32(0x4000_1100, read32(0x4000_1114) ^ 0x0030_0000);
        rmw(0x4000_1120, 0xffff_feff, 0);
        rmw(0x4000_1224, u32::MAX, 0x000b_0000);
        rmw(0x4000_1010, u32::MAX, 0x100);
        rmw(0x4000_1510, u32::MAX, 0x0140_0000);
        rmw(0x4000_1510, u32::MAX, 0x0280_0000);
        let _ = read32(0x4000_152c);
        write32(0x4000_152c, 0x1000_1100);
        rmw(0x4000_1528, 0xffff_ffcb, 0x10);
        rmw(0x4000_1514, u32::MAX, 0x1000_0000);
        rmw(0x4000_1120, 0xffff_ff07, 0x68);
        delay_us(10);

        rmw(0x4000_1514, 0x7c00_ffff, 0x0004_0000);
        rmw(0x4000_1514, u32::MAX, 1 << 31);
        delay_us(10);
        capture_samples()?;
        rmw(0x4000_1514, 0x7fff_ffff, 0);

        let baseline = rccal_get_vpp(0);
        if baseline == 0 {
            return Err(RfError::InvalidMeasurement);
        }
        rmw(0x4000_1120, 0xffff_ff07, 0xd0);
        let reference = baseline as f32 * 0.895;
        delay_us(10);

        rmw(0x4000_1514, 0xfc00_ffff, 0x001c_0000);
        let mut code = 0u32;
        let mut step = 32u32;
        for _ in 0..6 {
            let trial = code + step;
            write_rccal_trial(trial)?;
            run_rccal_measurement(delay_us)?;
            if rccal_get_vpp(0) as f32 >= reference {
                code = trial;
            }
            step >>= 1;
        }

        write_rccal_trial(code)?;
        let mut sequence = 0u32;
        for _ in 0..63 {
            write_rccal_trial(code)?;
            run_rccal_measurement(delay_us)?;
            sequence = (sequence << 1) & 0xf;

            let vpp = rccal_get_vpp(0) as f32;
            if reference < vpp {
                if code == 63 {
                    return Err(RfError::InvalidMeasurement);
                }
                sequence = (sequence | 1) & 0xf;
                code += 1;
                if sequence == 5 {
                    break;
                }
            } else {
                if code == 0 {
                    return Err(RfError::InvalidMeasurement);
                }
                code -= 1;
                if sequence == 10 {
                    break;
                }
            }
        }

        code &= 0x3f;
        let packed = (code << 24) | (code << 16) | (code << 8) | code;
        write32(0x4000_1210, packed);
        write32(0x4000_1218, packed);
        write32(0x4000_121c, packed);

        let mut cached = read_rccal();
        cached = (cached & !0x0000_003f) | code;
        cached = (cached & !0x0000_0fc0) | (code << 6);
        cached = (cached & !0x0003_f000) | (code << 12);
        cached = (cached & !0x00fc_0000) | (code << 18);
        write_rccal(cached);
        Ok(())
    })?;

    rmw(0x4000_1014, u32::MAX, 0x300);
    Ok(())
}

fn run_rccal_measurement<D>(delay_us: &mut D) -> Result<(), RfError>
where
    D: FnMut(u32),
{
    rmw(0x4000_1514, 0x7fff_ffff, 0);
    rmw(0x4000_1514, u32::MAX, 1 << 31);
    delay_us(10);
    capture_samples()?;
    rmw(0x4000_1514, 0x7fff_ffff, 0);
    Ok(())
}

fn write_rccal_trial(code: u32) -> Result<(), RfError> {
    if code > 63 {
        return Err(RfError::InvalidMeasurement);
    }
    let mut value = read32(0x4000_1210);
    value = (value & 0xc0ff_ffff) | (code << 24);
    value = (value & 0xffff_c0ff) | (code << 8);
    write32(0x4000_1210, value);
    Ok(())
}

fn rccal_get_vpp(iq: u32) -> u32 {
    let mut minimum = 0i16;
    let mut maximum = 0i16;
    for index in 0..256 {
        let value = sample(index);
        let data = if iq == 0 {
            (value >> 16) as u16 as i16
        } else {
            value as u16 as i16
        };
        if index == 0 {
            minimum = data;
            maximum = data;
        } else {
            minimum = minimum.min(data);
            maximum = maximum.max(data);
        }
    }
    (i32::from(maximum) - i32::from(minimum)) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn merges_only_roscal_fields() {
        assert_eq!(merge_roscal_fields(0xaa00_0000, 0x0055_aa33), 0xaa55_aa33);
    }

    #[test]
    fn lo_ratio_preserves_dither_and_upper_bits() {
        let raw = 0xf801u16;
        let ratio = 0x02a5u16;
        assert_eq!((raw & 0xf801) | (ratio << 1), 0xfd4b);
    }
}
