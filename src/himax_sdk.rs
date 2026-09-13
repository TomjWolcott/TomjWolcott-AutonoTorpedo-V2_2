// himax_sdk.rs
// Grove AI (Himax) i2ccomm SD-log — direct port from himax_sdk.c
// embassy-stm32, STM32G473RCT6

use embassy_stm32::i2c::{self, I2c};
use embassy_time::{Instant, Timer};
use embassy_stm32::mode::Async;

use crate::comms_println;

/// 7-bit address (embassy I2C wants 7-bit, unlike HAL which took the 8-bit shifted form)
pub const GROVE_I2C_ADDR: u8 = 0x62;
pub const I2C_FEATURE_RECORDER: u8 = 0x80;
pub const I2C_CMD_RECORD_START: u8 = 0x01;

fn crc16_ccitt(buf: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in buf {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

async fn grove_start_recording(
    i2c: &mut I2c<'static, Async, i2c::Master>,
    threshold: i32,
) -> Result<(), embassy_stm32::i2c::Error> {
    // Packet layout (Himax i2ccomm):
    //   [0] Feature
    //   [1] Command
    //   [2] Payload length LSB
    //   [3] Payload length MSB
    //   [4..4+N-1] Payload (optional)
    //   [N] CRC16 LSB
    //   [N+1] CRC16 MSB
    let mut pkt = [0u8; 8];
    let mut payload_len: u16 = 0;

    pkt[0] = I2C_FEATURE_RECORDER;
    pkt[1] = I2C_CMD_RECORD_START;

    if (0..=100).contains(&threshold) {
        payload_len = 1;
        pkt[4] = threshold as u8;
    }

    pkt[2] = (payload_len & 0xFF) as u8;
    pkt[3] = ((payload_len >> 8) & 0xFF) as u8;

    let mut total = 4 + payload_len as usize;

    let crc = crc16_ccitt(&pkt[..total]);
    pkt[total] = (crc & 0xFF) as u8;
    pkt[total + 1] = ((crc >> 8) & 0xFF) as u8;
    total += 2;

    i2c.write(GROVE_I2C_ADDR, &pkt[..total]).await
}

pub async fn init_for_himax(
    i2c: &mut I2c<'static, Async, i2c::Master>
) -> u16 {
    comms_println!("========================================");
    comms_println!("  STM32 + Grove AI (i2ccomm SD-log)");
    comms_println!("========================================");
    comms_println!("Target i2ccomm slave addr: 0x{:02X} (7-bit)", GROVE_I2C_ADDR);

    comms_println!("Waiting 5s for Grove AI to boot...");
    // Timer::after_millis(5000).await;

    comms_println!("--- I2C bus scan on I2C ---");
    let mut found_count = 0;
    let mut found_0x62 = false;
    for addr in 0x03u8..=0x77u8 {
        if i2c.blocking_read(addr, &mut []).is_ok() {
            comms_println!(
                "  0x{:02X}  ACK{}",
                addr,
                if addr == GROVE_I2C_ADDR { "  <-- i2ccomm target" } else { "" }
            );
            found_count += 1;
            if addr == GROVE_I2C_ADDR {
                found_0x62 = true;
            }
        }
    }
    comms_println!("  {} device(s) found", found_count);

    if !found_0x62 {
        comms_println!("WARNING: 0x62 not found in scan! Retrying every 2s...");
        for retry in 0..10 {
            if found_0x62 { break; }
            Timer::after_millis(2000).await;
            if i2c.write(GROVE_I2C_ADDR, &[]).await.is_ok() {
                comms_println!("  0x62 appeared on retry {}", retry + 1);
                found_0x62 = true;
            } else {
                comms_println!("  retry {}: 0x62 still NACK", retry + 1);
            }
        }
        if !found_0x62 {
            comms_println!("FAIL: 0x62 never responded.");
            comms_println!("  Check: SDA/SCL wires go to Grove connector (not camera I2C)");
            comms_println!("  Check: GND is shared between STM32 and Grove AI");
            comms_println!("  Note: 0x28 = OV5647 camera, NOT the i2ccomm slave");
        }
        return 0;
    }

    1
}

pub async fn start_recording_for_himax(
    i2c: &mut I2c<'static, Async, i2c::Master>,
) -> u16 {
    comms_println!("Sending start-recording (threshold=50%%)...");
    match grove_start_recording(i2c, 50).await {
        Ok(()) => {
            comms_println!("I2C TX OK — check Grove AI UART for [I2C_CMD] message");
            1
        }
        Err(e) => {
            comms_println!("I2C TX FAILED (err={:?})", e);
            0
        }
    }
}