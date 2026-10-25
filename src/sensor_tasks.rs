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
use glam::{Vec3, Quat};
use mmc5983_rs::Mmc5983;
use ms5837::OverSamplingRatio;

use crate::comms::Messagable;

pub static PRESSURE_RESULTS: Watch<CriticalSectionRawMutex, PressureResults, 4> = Watch::new();
pub static IMU_MAG_RESULTS: Watch<CriticalSectionRawMutex, ImuMagResults, 4> = Watch::new();

#[derive(Default, Clone, Copy)]
pub struct PressureResults {
    pub pressure_atm: f32,
    pub pressure_atm_normal: f32,
    pub sensor_temp: f32,
}

#[derive(Default, Clone, Copy)]
pub struct ImuMagResults {
    pub acc: Vec3,
    pub gyr: Vec3,
    pub mag: Vec3,
    pub mag_strength: f32,
    pub imu_temp: f32,
    pub mag_temp: f32,
    pub ori: Quat
}

/* c++ versions

void AK09940A_Output::into_message(std::vector<uint8_t> &data) {
	for (int i = 0; i < 3; i++) {
		uint32_t data_val = (uint32_t)static_cast<int32_t>(mag[i]);
		data.push_back((uint8_t)(data_val >> 24));
		data.push_back((uint8_t)((data_val >> 16) & 0xFF));
		data.push_back((uint8_t)((data_val >> 8) & 0xFF));
		data.push_back((uint8_t)(data_val & 0xFF));
	}
}
    void ICM42688_Data::into_message(std::vector<uint8_t> &data) {
	uint8_t msg_data[12];

	for (int i = 0; i < 3; i++) {
		uint16_t acc_data = (uint16_t)static_cast<int16_t>(acc[i] * 4096.0);
		uint16_t gyr_data = (uint16_t)static_cast<int16_t>(gyro[i] * 4096.0);
		msg_data[0 + 2*i] = (uint8_t)(acc_data >> 8);
		msg_data[1 + 2*i] = (uint8_t)(acc_data & 0xFF);
		msg_data[6 + 2*i] = (uint8_t)(gyr_data >> 8);
		msg_data[7 + 2*i] = (uint8_t)(gyr_data & 0xFF);
	}
	
	data.insert(data.end(), msg_data, msg_data + 12);
}
*/

impl Messagable for ImuMagResults {
    fn insert_msg_bytes(&self, dst: &mut [u8]) -> usize {
        for i in 0..3 {
            let mag_data = (100000.0 * self.mag[i]) as i32;

            dst[4*i..4*(i+1)].copy_from_slice(&mag_data.to_be_bytes());
        }

        for i in 0..3 {
            let acc_data = (self.acc[i] * 4096.0) as i16;
            let gyr_data = (self.gyr[i] * 4096.0) as i16;

            dst[12 + 2*i] = (acc_data >> 8) as u8;
            dst[13 + 2*i] = (acc_data & 0xFF) as u8;
            dst[18 + 2*i] = (gyr_data >> 8) as u8;
            dst[19 + 2*i] = (gyr_data & 0xFF) as u8;
        }
        24 // Return the number of bytes written    
    }

    fn try_from_msg_bytes(_src: &[u8]) -> Option<Self> {
        None // TODO: Implement this if needed
    }

}


fn quat_from_acc_mag(mut acc: Vec3, mut mag: Vec3) -> Quat {
	acc = acc.normalize();
    mag = mag.normalize();
	let mut q1 = Quat::IDENTITY;
	let mut q2 = Quat::IDENTITY;

	if acc.z.abs() < 0.9999 {
	    let acc_to_z_angle = libm::acosf(acc.z);
		let z_to_acc_axis = Vec3::new(-acc.y, acc.x, 0.0).normalize();
		q1 = Quat::from_axis_angle(z_to_acc_axis, acc_to_z_angle);
		mag = q1 * mag;
	}

	if mag.z.abs() < 0.9999 {
		let mag_to_x_angle = libm::atan2f(mag.y, mag.x);
		q2 = Quat::from_axis_angle(Vec3::Z, -mag_to_x_angle);
	}

	return q2 * q1;
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
    let mut ticker = Ticker::every(Duration::from_hz(800));

    // Adjust these to match your configured full-scale range
    let accel_lsb = 16.0f32 / (1 << 19) as f32; // g per LSB, e.g. ±16g
    let gyro_lsb = 2000.0f32 / (1 << 19) as f32; // dps per LSB, e.g. ±2000dps
    let mut raw_words = [0u32; 128];

    // // ICM42688P
    let icm = icm426xx::ICM42688::new(icm_spidev);
    let mut icm_config = icm426xx::Config::default();
    icm_config.rate = OutputDataRate::Hz2000;
    let icm_opt = match icm.initialize(Delay, icm_config).await {
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
        Err(e) => {info!("MMC Init error, error: {:?}", e); false},
    };

    mmc_success &= match mmc.product_id().await {
        Ok(product_id) => {
            info!("MMC, product_id: 0x{:X}, is_correct?: {}", product_id.raw(), product_id.is_correct());
            product_id.is_correct()
        },
        Err(_e) => {
            info!("MMC, product_id error");
            false
        },
    };

    let mut icm = if icm_opt.is_none() || !mmc_success {
        info!("Init errors with ICM and/or MMC, exiting");

        return;
    } else {
        icm_opt.unwrap()
    };

    let offset = match mmc.calibrate_offset(&mut Delay).await {
        Ok(mag_offset) => Vec3::new(
            mag_offset.x_gauss(),
            mag_offset.y_gauss(),
            mag_offset.z_gauss(),
        ),
        Err(e) => {
            info!("MMC error calibrating offset: {:?}", e);
            Vec3::ZERO
        }
    };


    let mag_strength = match mmc.magnetic_field().await {
        Ok(field) => Vec3::new(
            field.x_gauss(), 
            field.y_gauss(), 
            field.z_gauss()
        ).length(),
        Err(e) => {
            info!("MMC error reading magnetic field for strength: {:?}", e);
            1.0
        }
    };

    info!("MMC, offset: {:?}", offset.to_array());

    match mmc.set_bandwidth(mmc5983_rs::BandwidthMode::Hz800).await {
        Ok(_) => {},
        Err(e) => {
            info!("MMC error setting bandwidth to __: {:?}", e)
        }
    }

    let mut imu_mag_res = ImuMagResults::default();
    imu_mag_res.mag_strength = mag_strength;
    let mut now = embassy_time::Instant::now();
    let tuning_parameter = 0.1;

    const N: usize = 10;
    let mut prev_mag = [Vec3::ZERO; N];
    let mut mags = [Vec3::ZERO; N];
    let mut i = 0;

    loop {
        ticker.next().await;

        // MMC
        match mmc.magnetic_field().await {
            Ok(field) => {
                let mag = Vec3::new(
                    field.x_gauss() - offset.x,
                    field.y_gauss() - offset.y,
                    field.z_gauss() - offset.z,
                );

                prev_mag[i] = mag;
                i = (i+1) % N;

                mags = prev_mag.clone();
                mags = sort_arr(mags, |v1, v2| v1.x > v2.x);
                imu_mag_res.mag = mags[N/2];

                imu_mag_res.mag_strength = imu_mag_res.mag.length();

                // info!("Magnetic field: ({},{},{}) Gauss, mag: {}", imu_mag_res.mag.x, imu_mag_res.mag.y, imu_mag_res.mag.z, imu_mag_res.mag.length());
            }
            Err(e) => {
                info!("MMC error reading magnetic field: {:?}", e);
            }
        };

        // match mmc.temperature().await {
        //     Ok(mag_temp) => {
        //         imu_mag_res.mag_temp = mag_temp.degrees_celsius();
        //     }
        //     Err(e) => {
        //         info!("MMC error reading temperature: {:?}", e);
        //     }
        // };

        // ICM
        if let Ok(num_words) = icm.read_fifo(&mut raw_words).await {
            let raw_bytes: &[u8] = cast_slice(&raw_words[1.min(num_words)..num_words]);
            let packet_size = core::mem::size_of::<FifoPacket4>();

            if let Some(chunk) = raw_bytes.chunks_exact(packet_size).last() {
                let pkt: &FifoPacket4 = from_bytes(chunk);

                imu_mag_res.acc = Vec3::new(
                    pkt.accel_data_x() as f32 * accel_lsb,
                    pkt.accel_data_y() as f32 * accel_lsb,
                    pkt.accel_data_z() as f32 * accel_lsb,
                );
                imu_mag_res.gyr = Vec3::new(
                    pkt.gyro_data_x() as f32 * gyro_lsb,
                    pkt.gyro_data_y() as f32 * gyro_lsb,
                    pkt.gyro_data_z() as f32 * gyro_lsb
                );
                imu_mag_res.imu_temp = pkt.temperature_raw() as f32 / 132.48 + 25.0;
            }
        }

	    let ori_from_acc_mag = quat_from_acc_mag(imu_mag_res.acc, imu_mag_res.mag);
        let dt_s = now.elapsed().as_micros() as f32 / 1e6;

        if dt_s > 0.0 {
            imu_mag_res.ori = ori_from_acc_mag;
        } else {
            let gyro_mag = imu_mag_res.gyr.length();
            let gyro_angle_rad = dt_s * gyro_mag * 3.1415926535 / 180.0;
            let gyro_axis = imu_mag_res.gyr / gyro_mag;

            let mut ori_from_gyro = imu_mag_res.ori;
            ori_from_gyro *= Quat::from_axis_angle(gyro_axis, -gyro_angle_rad);

            imu_mag_res.ori = ori_from_acc_mag.slerp(ori_from_gyro, tuning_parameter);

    //		vel += a * dt_s;
    //		pos += vel * dt_s;
        }

        // info!("SENDING IMU: \n  [{}, {}, {}]\n  [{}, {}, {}]\n  [{}, {}, {}]", 
        //     imu_mag_res.acc.x, imu_mag_res.acc.y, imu_mag_res.acc.z,
        //     imu_mag_res.gyr.x, imu_mag_res.gyr.y, imu_mag_res.gyr.z,
        //     imu_mag_res.mag.x, imu_mag_res.mag.y, imu_mag_res.mag.z
        // );
        sender.send(imu_mag_res.clone());
	    now = embassy_time::Instant::now();
    }
}

fn sort_arr<const N: usize>(mut vs: [Vec3; N], f_gt: impl Fn(&Vec3, &Vec3) -> bool) -> [Vec3; N] {
    for i in 0..N {
        for j in 1..(N-i) {
            if f_gt(&vs[j-1], &vs[j]) {
                let v_temp = vs[j-1];
                vs[j-1] = vs[j];
                vs[j] = v_temp;
            }
        }
    }

    vs
}

const MBAR_PER_ATM: f32 = 1013.25;


impl Messagable for PressureResults {
    fn insert_msg_bytes(&self, dst: &mut [u8]) -> usize {
        let floats: [f32; 3] = [
            10.36 * (self.pressure_atm - 1.0),// depth (m)
            self.sensor_temp,// temperature (C)
            MBAR_PER_ATM * self.pressure_atm// pressure (mbar)
        ];

        for i in 0..3 {
            let uint32_data = floats[i].to_bits();

            dst[4*i] = (uint32_data >> 24) as u8;
            dst[1 + 4*i] = ((uint32_data >> 16) & 0xFF) as u8;
            dst[2 + 4*i] = ((uint32_data >> 8) & 0xFF) as u8;
            dst[3 + 4*i] = (uint32_data & 0xFF) as u8;
        }
        12 // Return the number of bytes written
    }

    fn try_from_msg_bytes(_src: &[u8]) -> Option<Self> {
        None // TODO: Implement this if needed
    }
}

#[embassy_executor::task]
pub async fn ms5837_test(i2c: I2c<'static, Async, i2c::Master>) {
    let sender = PRESSURE_RESULTS.sender();
    let mut pressure_atm_normal = 1.0;

    let sensor = ms5837::new(i2c, Delay);
    let mut sensor = match sensor.init().await {
        Ok(s) => {
            info!("MS5837 Init Success!!");
            s
        },
        Err(e) => {
            info!("Init error with MS5837, error: {:?}", e);
            return;
        }, // handle/log init error as needed
    };

    let mut ticker = Ticker::every(Duration::from_hz(10));
    
    let pressure_atm_normal = match sensor.read_temperature_and_pressure(OverSamplingRatio::R4096).await {
        Ok(tp) => {
            tp.pressure / MBAR_PER_ATM
        }
        Err(e) => {
            1.0
        }
    };
    loop {
        match sensor.read_temperature_and_pressure(OverSamplingRatio::R4096).await {
            Ok(tp) => {
                // use pressure_atm
                sender.send(PressureResults {
                    pressure_atm: tp.pressure / MBAR_PER_ATM,
                    pressure_atm_normal,
                    sensor_temp: tp.temperature,
                });
                // info!("MS5837, pressure: {}", tp.pressure / MBAR_PER_ATM);
            }
            Err(e) => {
                info!("read temp & pressure error with MS5837, error: {:?}", e);
                // handle/log error
            }
        }
        ticker.next().await;
    }
}