#![no_std]
#![no_main]

mod fmt;
mod motor_controller;

use core::cell::{OnceCell, RefCell};
use core::fmt::Pointer;
use core::iter::Once;
use defmt::{unwrap, Format, Formatter};
use defmt::export::display;
use embassy_embedded_hal::SetConfig;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Flex, Level, Output, OutputType, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::{bind_interrupts, dma, i2c, interrupt, spi, Config, Peripherals};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::adc;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Delay, Duration, Ticker, Timer};
use embedded_hal_bus::util::AtomicCell;
use embedded_hal_bus::spi as bus_spi;
use fmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannels};
use embassy_stm32::rcc::{Hse, HseMode, Sysclk};
use embassy_stm32::timer;
use embassy_stm32::timer::GeneralInstance4Channel;
use embassy_stm32::timer::simple_pwm::SimplePwmChannel;
use embedded_graphics::Drawable;
use embedded_graphics::geometry::{AngleUnit, Point};
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Primitive;
use embedded_graphics::primitives::{Arc, Circle, PrimitiveStyle};
use embedded_graphics::text::{Baseline, Text};
use icm426xx::fifo::FifoPacket4;
use icm426xx::{OutputDataRate, Timestamp};
// use mmc5983_rs::Mmc5983;
use ssd1306::{I2CDisplayInterface, Ssd1306, Ssd1306Async};
use ssd1306::prelude::*;
use static_cell::StaticCell;
use mmc5983_rs::Mmc5983;
use motor_controller::MotorControllersPeri;
use ms5837::OverSamplingRatio;

bind_interrupts!(struct Irqs {
        I2C2_EV => i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C2>;
        I2C2_ER => i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C2>;
        DMA1_CHANNEL1 => dma::InterruptHandler<DMA1_CH1>;
        DMA1_CHANNEL2 => dma::InterruptHandler<DMA1_CH2>;

        I2C3_EV => i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C3>;
        I2C3_ER => i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C3>;
        DMA1_CHANNEL3 => dma::InterruptHandler<DMA1_CH3>;
        DMA1_CHANNEL4 => dma::InterruptHandler<DMA1_CH4>;

        I2C4_EV => i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C4>;
        I2C4_ER => i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C4>;
        DMA1_CHANNEL5 => dma::InterruptHandler<DMA1_CH5>;
        DMA1_CHANNEL6 => dma::InterruptHandler<DMA1_CH6>;

        DMA1_CHANNEL7 => dma::InterruptHandler<DMA1_CH7>;
        DMA1_CHANNEL8 => dma::InterruptHandler<DMA1_CH8>;

        DMA2_CHANNEL1 => dma::InterruptHandler<DMA2_CH1>;
    });

type SharedSpiBus = bus_spi::AtomicDevice<'static, Spi<'static, Async, spi::mode::Master>, Output<'static>, Delay>;

static IMU_MAG_SPI: StaticCell<Mutex<NoopRawMutex, Spi<'static, Async, spi::mode::Master>>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut stm_config = Config::default();
    {
        stm_config.rcc.hse = Some(Hse { freq: Hertz(16_000_000), mode: HseMode::Oscillator });
        stm_config.rcc.sys = Sysclk::HSE;
    };

    let p = embassy_stm32::init(stm_config);
    // BLINKY
    let led1 = Output::new(p.PB7, Level::High, Speed::Low);
    let led2 = Output::new(p.PB5, Level::High, Speed::Low);

    spawner.spawn(blink(led1, 500, 100).unwrap());
    spawner.spawn(blink(led2, 400, 600).unwrap());

    // PWM
    let (
        pin_t1ch3, pin_t1ch4,
        pin_t2ch3, pin_t2ch4,
        pin_t3ch1, pin_t3ch2,
        pin_t3ch3, pin_t3ch4
    ) = (
        PwmPin::new(p.PC2, OutputType::PushPull), PwmPin::new(p.PC3, OutputType::PushPull),
        PwmPin::new(p.PA2, OutputType::PushPull), PwmPin::new(p.PA3, OutputType::PushPull),
        PwmPin::new(p.PA6, OutputType::PushPull), PwmPin::new(p.PA7, OutputType::PushPull),
        PwmPin::new(p.PB0, OutputType::PushPull), PwmPin::new(p.PB1, OutputType::PushPull),
    );

    let motor_pwm_freq = Hertz(10_000);

    let motor_controllers_peri = MotorControllersPeri::new(
        [Flex::new(p.PC1), Flex::new(p.PA1), Flex::new(p.PA5), Flex::new(p.PC5)],
        [0, 1, 2, 3],
        SimplePwm::new(p.TIM1, None, None, Some(pin_t1ch3), Some(pin_t1ch4), motor_pwm_freq, Default::default()),
        SimplePwm::new(p.TIM2, None, None, Some(pin_t2ch3), Some(pin_t2ch4), motor_pwm_freq, Default::default()),
        SimplePwm::new(p.TIM3, Some(pin_t3ch1), Some(pin_t3ch2), Some(pin_t3ch3), Some(pin_t3ch4), motor_pwm_freq, Default::default())
    );
    
    spawner.spawn(motor_controller_test(motor_controllers_peri).unwrap());

    // I2C
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = Hertz(400_000);

    let mut ssd1306_en = Output::new(p.PB12, Level::High, Speed::Low);

    let mut i2c2 = I2c::new(
        p.I2C2, p.PA9, p.PA8,
        p.DMA1_CH1, p.DMA1_CH2, Irqs, i2c_config,
    );

    let mut i2c3 = I2c::new(
        p.I2C3, p.PC8, p.PC9,
        p.DMA1_CH3, p.DMA1_CH4, Irqs, i2c_config,
    );

    let mut i2c4 = I2c::new(
        p.I2C4, p.PC6, p.PC7,
        p.DMA1_CH5, p.DMA1_CH6, Irqs, i2c_config,
    );

    // i2c_scan("i2c2", &mut i2c2).await;
    // i2c_scan("i2c3", &mut i2c3).await;
    // i2c_scan("i2c4", &mut i2c4).await;

    spawner.spawn(ssd1306_test(i2c3, ssd1306_en).unwrap());
    spawner.spawn(ms5837_test(i2c2).unwrap());

    // SPI
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(1_000_000);

    let bus = IMU_MAG_SPI.init(Mutex::new(Spi::new(
        p.SPI2, p.PB13, p.PB15, p.PB14,
        p.DMA1_CH7, p.DMA1_CH8, Irqs, spi_config
    )));

    let mut icm42688p_cs = Output::new(p.PA11, Level::High, Speed::High);
    let mut mmc5983ma_cs = Output::new(p.PA12, Level::High, Speed::High);

    let icm42688p_spidev = SpiDevice::new(bus, icm42688p_cs);
    let mmc5983_spidev = SpiDevice::new(bus, mmc5983ma_cs);

    spawner.spawn(icm42688p_test(icm42688p_spidev).unwrap());
    spawner.spawn(mmc5983_test(mmc5983_spidev).unwrap());

    // adc

}

#[embassy_executor::task]
async fn motor_controller_test(
    motor_controller: MotorControllersPeri
) {
    info!("Hi!!");
}

// #[embassy_executor::task]
// async fn adc_dma_test(
//     adc: peripherals::ADC1,
//     dma: peripherals::DMA1_CH1,
//     mut pin: impl AdcChannel<peripherals::ADC1>,
// ) {
//     let mut adc = Adc::new(adc, Irqs);
//     adc.set_resolution(Resolution::BITS12);
//
//     // Ring buffer DMA, continuously sampling one channel
//     let mut ring_buf: [u16; 256] = [0; 256];
//     let mut ringbuffered_adc: RingBufferedAdc<peripherals::ADC1> =
//         adc.into_ring_buffered(dma, &mut ring_buf);
//
//     ringbuffered_adc.start().unwrap();
//
//     let mut samples = [0u16; 64];
//     loop {
//         match ringbuffered_adc.read(&mut samples).await {
//             Ok(_) => {
//                 // convert first sample to volts (assuming 3.3V ref, 12-bit)
//                 let voltage = samples[0] as f32 * 3.3 / 4095.0;
//                 info!("adc: {} V (raw {})", voltage, samples[0]);
//             }
//             Err(e) => {
//                 info!("adc dma overrun: {:?}", e);
//             }
//         }
//     }
// }

#[embassy_executor::task(pool_size = 2)]
async fn blink(mut led: Output<'static>, wait1: u64, wait2: u64) {
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(wait1)).await;
        led.set_low();
        Timer::after(Duration::from_millis(wait2)).await;
    }
}

async fn i2c_scan(name: &'static str, i2c: &mut I2c<'static, Async, i2c::Master>) {
    info!("[{}]: I2C scan", name);

    for addr in 0x08u8..0x78u8 {
        let mut buf = [0u8; 1];
        match i2c.blocking_read(addr, &mut buf) {
            Ok(_) => info!("[{}]: Found device at 0x{:02X}", name, addr),
            Err(i2c::Error::Timeout) => {}
            Err(_) => {} // NACK, no device
        }
    }

    info!("[{}]: Scan complete", name);
}

#[embassy_executor::task]
async fn icm42688p_test(spidev: SpiDevice<'static, NoopRawMutex, Spi<'static, Async, spi::mode::Master>, Output<'static>>) {
    use icm426xx::{OutputDataRate, fifo::FifoPacket4};
    use embassy_time::Delay;
    use bytemuck::{cast_slice, from_bytes};

    // Adjust these to match your configured full-scale range
    let accel_lsb = 16.0f32 / (1 << 19) as f32; // g per LSB, e.g. ±16g
    let gyro_lsb = 2000.0f32 / (1 << 19) as f32; // dps per LSB, e.g. ±2000dps

    let mut icm = icm426xx::ICM42688::new(spidev);
    let mut icm_config = icm426xx::Config::default();
    icm_config.rate = OutputDataRate::Hz50;
    let mut icm = icm.initialize(Delay, icm_config).await.unwrap();

    let mut bank = icm.ll().bank::<{ icm426xx::register_bank::BANK0 }>();
    let who_am_i = bank.who_am_i().async_read().await.unwrap();
    info!("icm42688p, whoami: 0x{:X}", who_am_i.value());
    let mut ticker = Ticker::every(Duration::from_hz(2));

    let mut raw_words = [0u32; 128];
    loop {
        break;
        if let Ok(num_words) = icm.read_fifo(&mut raw_words).await {
            // raw_words.iter_mut().for_each(|word| *word = word.swap_bytes());

            let raw_bytes: &[u8] = cast_slice(&raw_words[1.min(num_words)..num_words]);
            let packet_size = core::mem::size_of::<FifoPacket4>();
            info!("num_words: {} -------", num_words);

            for (i, chunk) in raw_bytes.chunks_exact(packet_size).enumerate() {
                let pkt: &FifoPacket4 = from_bytes(chunk);

                let ax_g = pkt.accel_data_x() as f32 * accel_lsb;
                let ay_g = pkt.accel_data_y() as f32 * accel_lsb;
                let az_g = pkt.accel_data_z() as f32 * accel_lsb;

                let gx_dps = pkt.gyro_data_x() as f32 * gyro_lsb;
                let gy_dps = pkt.gyro_data_y() as f32 * gyro_lsb;
                let gz_dps = pkt.gyro_data_z() as f32 * gyro_lsb;

                let temp = pkt.temperature_raw() as f32 / 132.48 + 25.0;

                // info!(
                //     "[{:02}]: [0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X}]", i,
                //     chunk[0], chunk[1], chunk[2], chunk[3], chunk[4], chunk[5], chunk[6], chunk[7], chunk[8], chunk[9], chunk[10], chunk[11], chunk[12], chunk[13], chunk[14], chunk[15], chunk[16], chunk[17], chunk[18], chunk[19],
                // );

                // info!(
                //     "[{}]: \n  header: 0x{:X} \n  accel raw: x={} y={} z={}, \n  gyro raw: x={} y={} z={}\n  temp: {}\n  time: {}",
                //     i, pkt.fifo_header,
                //     pkt.accel_data_x(), pkt.accel_data_y(), pkt.accel_data_z(),
                //     pkt.gyro_data_x(), pkt.gyro_data_y(), pkt.gyro_data_z(),
                //     pkt.temperature_raw(),
                //     pkt.timestamp()
                // );
                info!(
                    "[{}]: \n  header: 0x{:X} \n  accel (g): x={} y={} z={}, \n  gyro (deg/s): x={} y={} z={}\n  temp: {} C\n  time delta: {}ms",
                    i, pkt.fifo_header,
                    ax_g, ay_g, az_g,
                    gx_dps, gy_dps, gz_dps,
                    temp,
                    match pkt.timestamp() { Timestamp::OdrTimestamp(x) => x as f32 * 0.016, _ => -1.0 }
                );
            }
        }
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn mmc5983_test(spidev:  SpiDevice<'static, NoopRawMutex, Spi<'static, Async, spi::mode::Master>, Output<'static>>) {
    let mut mag = Mmc5983::new_with_spi(spidev);

    let res = mag.init().await;

    info!("MMC init succeeded? {}", res.is_ok());

    match mag.product_id().await {
        Ok(id) => info!("MMC Success, id=0x{:X}, correct? = {}", id.raw(), id.is_correct()),
        Err(_e) => info!("MMC Fail")
    }
}

#[embassy_executor::task]
async fn ssd1306_test(
    i2c: I2c<'static, Async, i2c::Master>,
    mut ssd1306_rst: Output<'static>
) {
    ssd1306_rst.set_high();
    let i2c_disp = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306Async::new(
        i2c_disp,
        DisplaySize128x64,
        DisplayRotation::Rotate0
    ).into_buffered_graphics_mode();
    let res = display.init().await;

    info!("SSD1306 success?? {}", res.is_ok());

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello Rust!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().await.unwrap();

    let mut i = 0;
    let loc = Point::new(50, 30);
    let stroke = 3;
    let radius = 15;
    let mut colors = [BinaryColor::Off, BinaryColor::On];

    loop {
        Timer::after(Duration::from_millis(50)).await;
        i = (i + 1) % 20;

        if i == 0 {
            colors = [colors[1], colors[0]];
        }

        Circle::new(loc, 2 * radius)
            .into_styled(PrimitiveStyle::with_stroke(colors[0], stroke))
            .draw(&mut display).unwrap();

        Arc::new(loc, 2 * radius, 0.0.deg(), (360.0 * (i as f32) / 20.0 + 5.0).deg())
            .into_styled(PrimitiveStyle::with_stroke(colors[1], stroke))
            .draw(&mut display).unwrap();

        let res = display.flush().await;

        if res.is_err() {
            info!("SSD1306 ERR");
        }
    }
}

const MBAR_PER_ATM: f32 = 1013.25;

#[embassy_executor::task]
async fn ms5837_test(i2c: I2c<'static, Async, i2c::Master>) {
    let sensor = ms5837::new(i2c, Delay);
    let mut sensor = match sensor.init().await {
        Ok(s) => {
            info!("MS5837 Success!");
            s
        },
        Err(_e) => return, // handle/log init error as needed
    };

    let mut ticker = Ticker::every(Duration::from_hz(10));
    loop {
        break;
        match sensor.read_temperature_and_pressure(OverSamplingRatio::R4096).await {
            Ok(tp) => {
                let pressure_atm = tp.pressure / MBAR_PER_ATM;
                // use pressure_atm
                info!("ATM pressure = {}", pressure_atm)
            }
            Err(_e) => {
                // handle/log error
            }
        }
        ticker.next().await;
    }
}