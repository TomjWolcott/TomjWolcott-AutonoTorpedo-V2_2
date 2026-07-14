// himax_sdk.rs
// Grove AI (Himax) i2ccomm SD-log — direct port from himax_sdk.c
// embassy-stm32, STM32G473RCT6

use core::fmt::Write as _;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::usart::Uart;
use embassy_time::{Instant, Timer};
use embassy_stm32::mode::Async;

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

async fn grove_start_recording<I2C: embassy_stm32::i2c::Instance>(
    i2c: &mut I2c<'_, Async, i2c::Master>,
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

pub async fn uart_log(
    uart: &mut Uart<'_, Async>,
    args: core::fmt::Arguments<'_>,
) {
    let mut buf: heapless::String<160> = heapless::String::new();
    let _ = write!(buf, "[{:010} ms] ", Instant::now().as_millis());
    let _ = core::fmt::write(&mut buf, args);
    let _ = buf.push_str("\r\n");
    let _ = uart.write(buf.as_bytes()).await;
}

#[macro_export]
macro_rules! uart_log {
    ($uart:expr, $($arg:tt)*) => {
        $crate::himax_sdk::uart_log($uart, format_args!($($arg)*))
    };
}

pub async fn init_for_himax<I2C, U>(
    i2c: &mut I2c<'_, I2C>,
    uart: &mut Uart<'_, U>,
) -> u16
where
    I2C: embassy_stm32::i2c::Instance,
    U: embassy_stm32::usart::BasicInstance,
{
    uart_log!(uart, "========================================").await;
    uart_log!(uart, "  STM32 + Grove AI (i2ccomm SD-log)").await;
    uart_log!(uart, "========================================").await;
    uart_log!(uart, "Target i2ccomm slave addr: 0x{:02X} (7-bit)", GROVE_I2C_ADDR).await;

    uart_log!(uart, "Waiting 5s for Grove AI to boot...").await;
    // Timer::after_millis(5000).await;

    uart_log!(uart, "--- I2C bus scan on I2C1 (PB7=SDA, PB8=SCL) ---").await;
    let mut found_count = 0;
    let mut found_0x62 = false;
    for addr in 0x03u8..=0x77u8 {
        if i2c.blocking_read(addr, &mut []).is_ok() {
            uart_log!(
                uart,
                "  0x{:02X}  ACK{}",
                addr,
                if addr == GROVE_I2C_ADDR { "  <-- i2ccomm target" } else { "" }
            )
            .await;
            found_count += 1;
            if addr == GROVE_I2C_ADDR {
                found_0x62 = true;
            }
        }
    }
    uart_log!(uart, "  {} device(s) found", found_count).await;

    if !found_0x62 {
        uart_log!(uart, "WARNING: 0x62 not found in scan! Retrying every 2s...").await;
        for retry in 0..10 {
            if found_0x62 {
                break;
            }
            Timer::after_millis(2000).await;
            if i2c.write(GROVE_I2C_ADDR, &[]).await.is_ok() {
                uart_log!(uart, "  0x62 appeared on retry {}", retry + 1).await;
                found_0x62 = true;
            } else {
                uart_log!(uart, "  retry {}: 0x62 still NACK", retry + 1).await;
            }
        }
        if !found_0x62 {
            uart_log!(uart, "FAIL: 0x62 never responded.").await;
            uart_log!(uart, "  Check: SDA/SCL wires go to Grove connector (not camera I2C)").await;
            uart_log!(uart, "  Check: GND is shared between STM32 and Grove AI").await;
            uart_log!(uart, "  Note: 0x28 = OV5647 camera, NOT the i2ccomm slave").await;
        }
        return 0;
    }

    1
}

pub async fn start_recording_for_himax<I2C, U>(
    i2c: &mut I2c<'_, I2C>,
    uart: &mut Uart<'_, U>,
) -> u16
where
    I2C: embassy_stm32::i2c::Instance,
    U: embassy_stm32::usart::BasicInstance,
{
    uart_log!(uart, "Sending start-recording (threshold=50%%)...").await;
    match grove_start_recording(i2c, 50).await {
        Ok(()) => {
            uart_log!(uart, "I2C TX OK — check Grove AI UART for [I2C_CMD] message").await;
            1
        }
        Err(e) => {
            uart_log!(uart, "I2C TX FAILED (err={:?})", e).await;
            0
        }
    }
}