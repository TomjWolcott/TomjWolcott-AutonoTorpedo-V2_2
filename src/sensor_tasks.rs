use defmt::info;
use embassy_sync::watch::Watch;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::spi::Spi;
use embassy_stm32::mode::Async;
use embassy_stm32::{i2c, spi};
use embassy_stm32::gpio::Output;
use embassy_stm32::i2c::I2c;
use embassy_time::{Delay, Duration, Ticker};
use glam::Vec3;
use mmc5983_rs::Mmc5983;
use ms5837::OverSamplingRatio;

pub static PRESSURE_RESULTS: Watch<CriticalSectionRawMutex, PressureResults, 4> = Watch::new();
pub static IMU_MAG_RESULTS: Watch<CriticalSectionRawMutex, ImuMagResults, 4> = Watch::new();

#[derive(Default, Clone, Copy)]
pub struct PressureResults {
    pub pressure_atm: f32,
    pub sensor_temp: f32,
}

#[derive(Default, Clone, Copy)]
pub struct ImuMagResults {
    pub acc: Vec3,
    pub gyr: Vec3,
    pub mag: Vec3,
    pub imu_temp: f32,
    pub mag_temp: f32
}

#[embassy_executor::task]
pub async fn icm42688p_mmc5983_sender(
    icm_spidev: SpiDevice<'static, NoopRawMutex, Spi<'static, Async, spi::mode::Master>, Output<'static>>,
    mmc_spidev: SpiDevice<'static, NoopRawMutex, Spi<'static, Async, spi::mode::Master>, Output<'static>>,
) {
    use icm426xx::{OutputDataRate, fifo::FifoPacket4};
    use embassy_time::{Delay, Duration, Ticker};
    use bytemuck::{cast_slice, from_bytes};
    use glam::Vec3;

    let sender = IMU_MAG_RESULTS.sender();
    let mut ticker = Ticker::every(Duration::from_hz(100));

    // Adjust these to match your configured full-scale range
    let accel_lsb = 16.0f32 / (1 << 19) as f32; // g per LSB, e.g. ±16g
    let gyro_lsb = 2000.0f32 / (1 << 19) as f32; // dps per LSB, e.g. ±2000dps
    let mut raw_words = [0u32; 128];

    // ICM42688P
    let icm = icm426xx::ICM42688::new(icm_spidev);
    let mut icm_config = icm426xx::Config::default();
    icm_config.rate = OutputDataRate::Hz500;
    let mut icm_opt = match icm.initialize(Delay, icm_config).await {
        Ok(mut icm) => {
            info!("ICM Init Success!!");
            let mut bank = icm.ll().bank::<{ icm426xx::register_bank::BANK0 }>();
            match bank.who_am_i().async_read().await {
                Ok(who_am_i) => info!("ICM, whoami: 0x{:X}", who_am_i.value()),
                Err(e) => info!("ICM whoami, error: {:?}", e),
            };

            Some(icm)
        },
        Err(e) => {info!("ICM42688P Init error: {:?}", e); None}
    };

    // MMC5983MA
    let mut mmc = Mmc5983::new_with_spi(mmc_spidev);
    let mut mmc_success = match mmc.init().await {
        Ok(_) => {info!("MMC Init Success!!"); true},
        Err(_e) => {info!("MMC Init error"); false},
    };

    mmc_success &= match mmc.product_id().await {
        Ok(product_id) => {
            info!("ICM, product_id: 0x{:X}, is_correct?: {}", product_id.raw(), product_id.is_correct());
            product_id.is_correct()
        },
        Err(_e) => {
            info!("ICM, product_id error");
            false
        },
    };

    let mut icm = if icm_opt.is_none() || !mmc_success {
        info!("Init errors with ICM and/or MMC, exiting");

        return;
    } else {
        icm_opt.unwrap()
    };

    loop {
        ticker.next().await;

        // MMC
        let mag = Vec3::new(0.0, 0.0, 0.0);
        let mag_temp = 0.0;

        // ICM
        if let Ok(num_words) = icm.read_fifo(&mut raw_words).await {
            let raw_bytes: &[u8] = cast_slice(&raw_words[1.min(num_words)..num_words]);
            let packet_size = core::mem::size_of::<FifoPacket4>();

            if let Some((i, chunk)) = raw_bytes.chunks_exact(packet_size).enumerate().last() {
                let pkt: &FifoPacket4 = from_bytes(chunk);

                let acc = Vec3::new(
                    pkt.accel_data_x() as f32 * accel_lsb,
                    pkt.accel_data_y() as f32 * accel_lsb,
                    pkt.accel_data_z() as f32 * accel_lsb,
                );

                let gyr = Vec3::new(
                    pkt.gyro_data_x() as f32 * gyro_lsb,
                    pkt.gyro_data_y() as f32 * gyro_lsb,
                    pkt.gyro_data_z() as f32 * gyro_lsb
                );

                let imu_temp = pkt.temperature_raw() as f32 / 132.48 + 25.0;
                // let dt = match pkt.timestamp() { Timestamp::OdrTimestamp(x) => x as f32 * 0.000016, _ => -1.0 };

                sender.send(ImuMagResults {
                    acc, gyr, mag,
                    imu_temp,
                    mag_temp
                });

                /*
                info!(
                    "[{:02}]: [0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X},0x{:02X}]", i,
                    chunk[0], chunk[1], chunk[2], chunk[3], chunk[4], chunk[5], chunk[6], chunk[7], chunk[8], chunk[9], chunk[10], chunk[11], chunk[12], chunk[13], chunk[14], chunk[15], chunk[16], chunk[17], chunk[18], chunk[19],
                );

                info!(
                    "[{}]: \n  header: 0x{:X} \n  accel raw: x={} y={} z={}, \n  gyro raw: x={} y={} z={}\n  temp: {}\n  time: {}",
                    i, pkt.fifo_header,
                    pkt.accel_data_x(), pkt.accel_data_y(), pkt.accel_data_z(),
                    pkt.gyro_data_x(), pkt.gyro_data_y(), pkt.gyro_data_z(),
                    pkt.temperature_raw(),
                    pkt.timestamp()
                );

                info!(
                    "[{}]: \n  header: 0x{:X} \n  accel (g): x={} y={} z={}, \n  gyro (deg/s): x={} y={} z={}\n  temp: {} C\n  time delta: {}ms",
                    i, pkt.fifo_header,
                    ax_g, ay_g, az_g,
                    gx_dps, gy_dps, gz_dps,
                    temp,
                    match pkt.timestamp() { Timestamp::OdrTimestamp(x) => x as f32 * 0.016, _ => -1.0 }
                );

                 */
            }
        }
    }
}

const MBAR_PER_ATM: f32 = 1013.25;

#[embassy_executor::task]
pub async fn ms5837_test(i2c: I2c<'static, Async, i2c::Master>) {
    let sender = PRESSURE_RESULTS.sender();

    let sensor = ms5837::new(i2c, Delay);
    let mut sensor = match sensor.init().await {
        Ok(s) => {
            info!("MS5837 Init Success!!");
            s
        },
        Err(e) => {
            info!("Init error with MS5837, error");
            return;
        }, // handle/log init error as needed
    };

    let mut ticker = Ticker::every(Duration::from_hz(10));
    loop {
        match sensor.read_temperature_and_pressure(OverSamplingRatio::R4096).await {
            Ok(tp) => {
                // use pressure_atm
                sender.send(PressureResults {
                    pressure_atm: tp.pressure / MBAR_PER_ATM,
                    sensor_temp: tp.temperature,
                });
            }
            Err(_e) => {
                // handle/log error
            }
        }
        ticker.next().await;
    }
}