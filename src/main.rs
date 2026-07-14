#![no_std]
#![no_main]

mod fmt;
mod motor_tasks;
mod sensor_tasks;
mod adc_tasks;
mod comms;
mod himax_sdk;

use core::cell::{OnceCell, RefCell};
use core::fmt::Pointer;
use core::iter::Once;
use core::num;
use cortex_m::interrupt::CriticalSection;
use defmt::{unwrap, Format, Formatter};
use defmt::export::display;
use embassy_embedded_hal::SetConfig;
use embassy_sync::channel::{Channel, Sender};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Flex, Level, Output, OutputType, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::{bind_interrupts, dma, i2c, interrupt, spi, Config, Peri, Peripherals};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::adc;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Delay, Duration, Ticker, Timer};
use embedded_hal_bus::util::AtomicCell;
use embedded_hal_bus::spi as bus_spi;
use fmt::info;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::adc::{Adc, AdcChannel, AdcConfig, AnyAdcChannel, RingBufferedAdc, Rovsm, SampleTime, Temperature, Trovs, VrefInt};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannels};
use embassy_stm32::rcc::{mux, Hse, HseMode, Sysclk};
use embassy_stm32::timer;
use embassy_stm32::timer::GeneralInstance4Channel;
use embassy_stm32::timer::simple_pwm::SimplePwmChannel;
use embassy_sync::watch::Watch;
use embedded_graphics::Drawable;
use embedded_graphics::geometry::{AngleUnit, Point, Size};
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Primitive;
use embedded_graphics::primitives::{Arc, Circle, Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use heapless::{String, Vec};
use icm426xx::fifo::FifoPacket4;
use icm426xx::{OutputDataRate, Timestamp};
// use mmc5983_rs::Mmc5983;
use ssd1306::{I2CDisplayInterface, Ssd1306, Ssd1306Async};
use ssd1306::prelude::*;
use static_cell::StaticCell;
use mmc5983_rs::Mmc5983;
use motor_tasks::MotorControllersPeri;
use ms5837::OverSamplingRatio;
use crate::motor_tasks::{motor_controller_test, GainselState};
use core::fmt::Write;
use glam::{Vec2, Vec3};
use adc_tasks::{AdcController, ADC_RESULTS};
use sensor_tasks::{IMU_MAG_RESULTS, PRESSURE_RESULTS};
use crate::adc_tasks::adc_test;
use crate::sensor_tasks::{icm42688p_mmc5983_sender, ms5837_test};
use embassy_stm32::usart::{Config as UartConfig, Uart, UartRx, UartTx};
use embassy_stm32::usart;

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
        DMA2_CHANNEL2 => dma::InterruptHandler<DMA2_CH2>;

        DMA2_CHANNEL3 => dma::InterruptHandler<DMA2_CH3>;
        DMA2_CHANNEL4 => dma::InterruptHandler<DMA2_CH4>;
        UART4 => usart::InterruptHandler<embassy_stm32::peripherals::UART4>;

        DMA2_CHANNEL5 => dma::InterruptHandler<DMA2_CH5>;
        DMA2_CHANNEL6 => dma::InterruptHandler<DMA2_CH6>;
        USART1 => usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
    });

static IMU_MAG_SPI: StaticCell<Mutex<NoopRawMutex, Spi<'static, Async, spi::mode::Master>>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut stm_config = Config::default();
    {
        stm_config.rcc.hse = Some(Hse { freq: Hertz(16_000_000), mode: HseMode::Oscillator });
        stm_config.rcc.sys = Sysclk::HSE;
        stm_config.rcc.mux.adc12sel = mux::Adcsel::SYS;
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
    // i2c_config.timeout = Duration::from_millis(5);

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

    // Output::new(p.PC12, Level::High, Speed::Low).set_high();
    // Output::new(p.PD2, Level::High, Speed::Low).set_high();

    // i2c_scan("i2c2", &mut i2c2).await;
    // i2c_scan("i2c3", &mut i2c3).await;
    // i2c_scan("i2c4", &mut i2c4).await;

    spawner.spawn(ssd1306_test(i2c3, ssd1306_en).unwrap());
    spawner.spawn(ms5837_test(i2c2).unwrap());

    // SPI
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(100_000);
    // spi_config.

    let bus = IMU_MAG_SPI.init(Mutex::new(Spi::new(
        p.SPI2, p.PB13, p.PB15, p.PB14,
        p.DMA1_CH7, p.DMA1_CH8, Irqs, spi_config
    )));

    let mut icm42688p_cs = Output::new(p.PA11, Level::High, Speed::High);
    let mut mmc5983ma_cs = Output::new(p.PA12, Level::High, Speed::Low);

    let icm42688p_spidev = SpiDevice::new(bus, icm42688p_cs);
    let mmc5983_spidev = SpiDevice::new(bus, mmc5983ma_cs);

    spawner.spawn(icm42688p_mmc5983_sender(icm42688p_spidev, mmc5983_spidev).unwrap());

    // adc
    let mut adc1_config = AdcConfig::default();
    adc1_config.oversampling_ratio = Some(0x07); // x128
    adc1_config.oversampling_shift = Some(0x07); // 7 bit shift
    adc1_config.oversampling_mode = Some((Rovsm::RESUMED, Trovs::AUTOMATIC, true));
    let adc1 = Adc::new(p.ADC1, adc1_config);

    let mut adc2_config = AdcConfig::default();
    adc2_config.oversampling_ratio = Some(0x07); // x128
    adc2_config.oversampling_shift = Some(0x07); // 7 bit shift
    adc2_config.oversampling_mode = Some((Rovsm::RESUMED, Trovs::AUTOMATIC, true));
    let adc2 = Adc::new(p.ADC2, adc2_config);

    let adc_controller = AdcController {
        vref: adc1.enable_vrefint().degrade_adc(),
        temp: adc1.enable_temperature().degrade_adc(),
        m0_ipropi: p.PC0.degrade_adc(),
        m1_ipropi: p.PA0.degrade_adc(),
        m2_ipropi: p.PA4.degrade_adc(),
        m3_ipropi: p.PC4.degrade_adc(),
        batt_voltage: p.PB2.degrade_adc(),
        adc1,
        adc2,
        dma_adc1: p.DMA2_CH1,
        dma_adc2: p.DMA2_CH2,
    };

    spawner.spawn(adc_test(adc_controller).unwrap());

    // uart
    let mut uart4_config = UartConfig::default();
    uart4_config.baudrate = 115200;

    let (uart4_tx, uart4_rx) = Uart::new(
        p.UART4,
        p.PC11,
        p.PC10,
        p.DMA2_CH3,
        p.DMA2_CH4,
        Irqs,
        uart4_config
    ).unwrap().split();

    spawner.spawn(uart_comms_rx(uart4_rx).unwrap());
    spawner.spawn(uart_comms_tx(uart4_tx).unwrap());

    // let mut uart1_config = UartConfig::default();
    // uart1_config.baudrate = 115200;

    // let (uart1_tx, uart1_rx) = Uart::new(
    //     p.USART1,
    //     p.PA10,
    //     p.PB6,
    //     p.DMA2_CH5,
    //     p.DMA2_CH6,
    //     Irqs,
    //     uart1_config
    // ).unwrap().split();
}

#[embassy_executor::task]
async fn uart_comms_rx(mut rx: UartRx<'static, Async>) {
    let mut buffer = [0u8; 1000];

    loop {
        let num_bytes = match rx.read_until_idle(&mut buffer).await {
            Ok(num_bytes) => num_bytes,
            Err(e) => {
                info!("Uart4_rx error: {:?}", e);
                continue;
            }
        };

        info!("Received: {:?}", &buffer[..num_bytes]);
    }

}

const UART_TX_MSG_SIZE: usize = 500; // max bytes per message, tune as needed
const UART_TX_QUEUE_DEPTH: usize = 10; // max queued messages

pub const UART_HEADER: [u8; 4] = [0x11, 0x0F, 0xFF, 0x00];
const UART_HEADER_LEN: usize = 6; // 4 magic bytes + len byte + id byte

pub type UartTxMsg = Vec<u8, UART_TX_MSG_SIZE>;

#[macro_export]
macro_rules! comms_println {
    ($($arg:tt)*) => {{
        let mut payload: heapless::String<{ $crate::UART_TX_MSG_SIZE - $crate::UART_HEADER_LEN }> =
            heapless::String::new();

        if core::fmt::Write::write_fmt(&mut payload, format_args!($($arg)*)).is_err() {
            defmt::warn!("uart_println!: message truncated, formatting overflowed buffer");
        }

        let total_len = $crate::UART_HEADER_LEN + payload.len();
        if total_len > 255 || total_len > $crate::UART_TX_MSG_SIZE {
            defmt::warn!("uart_println!: message too long ({} bytes), dropped", total_len);
        } else {
            let mut msg: $crate::UartTxMsg = heapless::Vec::new();
            let _ = msg.extend_from_slice(&$crate::UART_HEADER);
            let _ = msg.push(total_len as u8);
            let _ = msg.push(5u8);
            let _ = msg.extend_from_slice(payload.as_bytes());

            match $crate::uart_tx_sender().try_send(msg) {
                Ok(_) => {}
                Err(_) => defmt::warn!("uart_println!: tx queue full, dropped"),
            }
        }
    }};
}

static UART_TX_CHANNEL: Channel<CriticalSectionRawMutex, UartTxMsg, UART_TX_QUEUE_DEPTH> =
    Channel::new();

/// Call from any task to get a handle for queuing outgoing UART bytes.
pub fn uart_comms_tx_sender() -> Sender<'static, CriticalSectionRawMutex, UartTxMsg, UART_TX_QUEUE_DEPTH> {
    UART_TX_CHANNEL.sender()
}

#[embassy_executor::task]
async fn uart_comms_tx(mut tx: UartTx<'static, Async>) {
    let receiver = UART_TX_CHANNEL.receiver();

    loop {
        let msg = receiver.receive().await;
        if let Err(e) = tx.write(&msg).await {
            info!("Uart4_tx error: {:?}", e);
        }
    }
}

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
async fn ssd1306_test(
    i2c: I2c<'static, Async, i2c::Master>,
    mut ssd1306_rst: Output<'static>
) {
    ssd1306_rst.set_high();
    let mut adc_reader = ADC_RESULTS.receiver().unwrap();
    let mut pressure_reader = PRESSURE_RESULTS.receiver().unwrap();
    let mut imu_reader = IMU_MAG_RESULTS.receiver().unwrap();
    let i2c_disp = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306Async::new(
        i2c_disp,
        DisplaySize128x64,
        DisplayRotation::Rotate0
    ).into_buffered_graphics_mode();
    if let Err(e) = display.init().await {
        info!("SSD1306 Init error");
        return;
    }
    let mut buf: String<32> = String::new();

    info!("SSD1306 Init Success!!");

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
        // display.clear_buffer();
        i = (i + 1) % 20;

        if i == 0 {
            colors = [colors[1], colors[0]];
        }

        if let Some(adc_results) = adc_reader.try_get() {
            buf.clear();
            write!(buf, "Bat:{:.3}, Tmp:{:.2}", adc_results.batt_v, adc_results.temp_c).unwrap();
            Rectangle::new(Point::new(0, 10), Size::new(buf.len() as u32 * 6, 10))
                .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
                .draw(&mut display).unwrap();
            Text::with_baseline(buf.as_str(), Point::new(0, 10), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();
        }

        if let Some(pressure) = pressure_reader.try_get() {
            let x = (pressure.pressure_atm - pressure.pressure_atm_normal) * 2000.0;
            let width = 50;
            // info!("HI, x={}", x);

            Rectangle::new(Point::new(0, 20), Size::new(width, 10))
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
                .draw(&mut display).unwrap();
            Rectangle::new(Point::new(0, 20), Size::new(width, 10))
                .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
                .draw(&mut display).unwrap();

            Rectangle::new(Point::new(0, 20), Size::new((width as f32 / 2.0 + x).clamp(0.0, width as f32) as u32, 10))
                .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
                .draw(&mut display).unwrap();
        }

        if let Some(imu) = imu_reader.try_get() {
            let center = Point::new(110, 45);
            let radius = 15;
            let tl = center - Point::new(radius, radius);

            Circle::new(center - Point::new(radius+5, radius+5), (2*radius+10) as u32)
                .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
                .draw(&mut display).unwrap();

            Circle::new(tl, (2*radius) as u32)
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
                .draw(&mut display).unwrap();

            let v_acc = Point::new(
                (-imu.acc.y*radius as f32) as i32, 
                (-imu.acc.x*radius as f32) as i32
            );
            let v_mag = Point::new(
                (-imu.mag.y/imu.mag_strength*radius as f32) as i32,
                (-imu.mag.x/imu.mag_strength*radius as f32) as i32
            );

            Line::new(center, center + v_acc)
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                .draw(&mut display).unwrap();

            Line::new(center, center + v_mag)
                .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
                .draw(&mut display).unwrap();
        }

        let res = display.flush().await;

        if res.is_err() {
            info!("SSD1306 ERR");
        }
    }
}

