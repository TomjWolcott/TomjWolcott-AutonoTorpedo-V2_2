use core::{time::Duration};

use crate::{adc_tasks::AdcResults, localization::LocalizationOutput, sensor_tasks::{ImuMagResults, PressureResults}};

pub use super::*;
use embassy_stm32::time::Hertz;
use num_enum::{IntoPrimitive, TryFromPrimitive};

pub mod rx;
pub mod tx;

pub const FIRMWARE_VERSION: (usize, usize) = (0, 1);
pub const MSG_HEADER: [u8; 4] = [0x11, 0x0F, 0xFF, 0x00];
pub const MSG_HEADER_LEN: usize = 6; // 4 magic bytes + len byte + id byte

pub const MSG_SIZE_MAX: usize = 256;
pub const TEXT_MSG_SIZE_MAX: usize = MSG_SIZE_MAX - MSG_HEADER_LEN;

pub trait Messagable: Sized {
    fn insert_msg_bytes(&self, dst: &mut [u8]) -> usize;

    fn try_from_msg_bytes(src: &[u8]) -> Option<Self>;
}

#[derive(Clone)]
pub struct Message([u8; MSG_SIZE_MAX]);

#[derive(Debug)]
pub enum NewMessageError {
    WrongMagicNumbers,
    MismatchedSize(usize, usize),
    NotLongEnough(usize)
}

impl Message {
    pub fn try_new(bytes: &[u8]) -> Result<Self, NewMessageError> {
            if bytes.len() < MSG_HEADER_LEN {
                return Err(NewMessageError::NotLongEnough(bytes.len()))
            }

            if &bytes[..4] != &MSG_HEADER {
                return Err(NewMessageError::WrongMagicNumbers);
            }

            let num_bytes = bytes[4] as usize;

            if num_bytes > bytes.len() {
                return Err(NewMessageError::MismatchedSize(num_bytes, bytes.len()));
            }

            let bytes = &bytes[..num_bytes];

            Ok(Self::new_raw(bytes))
    }

    pub fn new_raw(bytes: &[u8]) -> Self {
        let mut dst = [0u8; 256];
        dst[..bytes.len()].copy_from_slice(bytes);

        Message(dst)
    }

    // ----------

    pub fn length(&self) -> usize {
        self.0[4] as usize
    }

    pub fn ty(&self) -> MessageType {
        MessageType::try_from(self.0[5]).unwrap_or(MessageType::InvalidMessage)
    }

    pub fn buffer(&self) -> &[u8] {
        &self.0[..self.length()]
    }

    // ----------

    pub async fn send(&self) {
        match tx::UART_TX_CHANNEL.sender().try_send(self.clone()) {
            Ok(_) => {}
            Err(_) => defmt::warn!("Message::send: tx queue full, dropped"),
        }
    }

}

impl core::fmt::Display for Message {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Message {{ ty: {:?}, len: {}, buffer: {:?} }}", self.ty(), self.length(), &self.0[..self.length()])
    }
}

#[derive(Debug, Eq, PartialEq, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
pub enum MessageType {           // Device | Controller  || Implemented?
    Ping = 0,                    //    x   |     x       || Yes
    Action = 1,                  //        |     x       || Not fully
    SendData = 2,                //    x   |             || Yes
    PingWithMs = 3,              //    x   |             || Yes
    SendConfig = 4,              //    x   |     x       || Not fully
    Text = 5,                    //    x   |             || Yes
    CalibrationData = 6,         //    x   |             || No
    Echo = 7,                    //    x   |     x       || No
    ActionQueue = 8,             //    x   |     x       || Not fully
    SendCurrentActionQueue = 9,  //        |     x       || Not fully
    InvalidMessage = 255
}

// Actions
pub enum ActionMessage {
    Noop,
    CalibrationSettings(CalibrationSettings),
    SetMotorSpeeds(MotorInputs),
    RequestSendConfig,
    CalibrationMessage(CalibrationMessage),
}

/*
CalibrationSettings ActionMsg::asCalibrationSettings() {
	CalibrationSettings settings = {};

	settings.type = (CalibrationType)(data[7]);
	settings.startSignal = (CalibrationStartSignal)(data[8]);
	settings.waitMsAfterUnplug = (uint16_t)(data[9] << 8) | (uint16_t)(data[10]);
	settings.dataCollectRateHz = (uint16_t)(data[11] << 8) | (uint16_t)(data[12]);
	settings.dataCollectTimeMs = (uint16_t)(data[13] << 8) | (uint16_t)(data[14]);

	return settings;
}

CalibrationMsgType ActionMsg::asCalibrationMsg() {
	return (CalibrationMsgType)(data[7]);
}

MotorSpeeds ActionMsg::asMotorSpeeds() {
	return MotorSpeeds(
		static_cast<float>(data[7]) / 127.0 - 1.0,
		static_cast<float>(data[8]) / 127.0 - 1.0,
		static_cast<float>(data[9]) / 127.0 - 1.0,
		static_cast<float>(data[10]) / 127.0 - 1.0
	);
} */

impl TryFrom<Message> for ActionMessage {
    type Error = ();

    fn try_from(msg: Message) -> Result<Self, Self::Error> {
        match msg.ty() {
            MessageType::Action => match msg.0[6] {
                0 => Ok(ActionMessage::Noop),
                1 => Ok(ActionMessage::CalibrationSettings(CalibrationSettings {
                    ty: match msg.0[7] {
                        0 => CalibrationType::Accelerometer,
                        1 => CalibrationType::Magnetometer,
                        2 => CalibrationType::Gyrometer,
                        _ => return Err(()),
                    },
                    start_signal: match msg.0[8] {
                        0 => CalibrationStartSignal::Now,
                        1 => CalibrationStartSignal::OnUnplug,
                        _ => return Err(()),
                    },
                    wait_ms_after_unplug: Duration::from_millis(((msg.0[9] as u16) << 8 | (msg.0[10] as u16)) as u64),
                    data_collect_rate: Hertz(((msg.0[11] as u16) << 8 | (msg.0[12] as u16)) as u32),
                    data_collect_time: Duration::from_millis(((msg.0[13] as u16) << 8 | (msg.0[14] as u16)) as u64),
                })),
                2 => Ok(ActionMessage::SetMotorSpeeds(MotorInputs::new(
                    (msg.0[7] as f32) / 127.0 - 1.0,
                    (msg.0[8] as f32) / 127.0 - 1.0,
                    (msg.0[9] as f32) / 127.0 - 1.0,
                    (msg.0[10] as f32) / 127.0 - 1.0,
                ))),
                3 => Ok(ActionMessage::RequestSendConfig),
                4 => Ok(ActionMessage::CalibrationMessage(match msg.0[7] {
                    0 => CalibrationMessage::Start,
                    1 => CalibrationMessage::Stop,
                    2 => CalibrationMessage::Change,
                    _ => return Err(()),
                })),
                _ => Err(()),
            }
            _ => Err(()),
        }
    }
}

enum ControlLoopEdittingType {
    Start,
    Stop,
    Change,
}

pub enum CalibrationMessage {
    Start,
    Stop,
    Change,
}

enum CalibrationType {
    Accelerometer,
    Magnetometer,
    Gyrometer,
}

enum CalibrationStartSignal {
    Now,
    OnUnplug,
}

pub struct CalibrationSettings {
    ty: CalibrationType,
    start_signal: CalibrationStartSignal,
    wait_ms_after_unplug: Duration,
    data_collect_rate: Hertz,
    data_collect_time: Duration,
}

// impl ActionMessage {
//     fn ty(&self) -> Option<ActionType> {

//     }
// }

pub struct Ping;

impl From<Ping> for Message {
    fn from(_value: Ping) -> Self {
        let mut dst = [0u8; 256];

        dst[..4].copy_from_slice(&MSG_HEADER);
        dst[4] = 6u8;
        dst[5] = MessageType::Ping.into();

        Message(dst)
    }
}

pub struct PingWithMs(usize);

impl PingWithMs {
    pub fn new() -> Self {
        Self(embassy_time::Instant::now().as_millis() as usize)
    }
}

impl From<PingWithMs> for Message {
    fn from(value: PingWithMs) -> Self {
        let mut dst = [0u8; 256];

        dst[..4].copy_from_slice(&MSG_HEADER);
        dst[4] = 10u8;
        dst[5] = MessageType::PingWithMs.into();
        dst[6] = ((value.0 >> 24) & 0xFF) as u8;
        dst[7] = ((value.0 >> 16) & 0xFF) as u8;
        dst[8] = ((value.0 >> 08) & 0xFF) as u8;
        dst[9] = ((value.0 >> 00) & 0xFF) as u8;

        Message(dst)
    }
}

pub struct OtherData {
    timestamp_us: u64,
    rate_hz: u16,
    firmware_version: (usize, usize)
}

impl OtherData {
    pub fn get() -> Self {
        OtherData {
            timestamp_us: embassy_time::Instant::now().as_micros() as u64,
            rate_hz: 20,
            firmware_version: FIRMWARE_VERSION
        }
    }
}

impl Messagable for OtherData {
    fn insert_msg_bytes(&self, dst: &mut [u8]) -> usize {
        dst[0..8].copy_from_slice(&self.timestamp_us.to_be_bytes());

        dst[8] = (self.rate_hz >> 8) as u8;
        dst[9] = (self.rate_hz & 0xFF) as u8;

        dst[10] = self.firmware_version.0 as u8;
        dst[11] = self.firmware_version.1 as u8;

        dst[12] = 0; // free_heap >> 24
        dst[13] = 0; // free_heap >> 16
        dst[14] = 0; // free_heap >> 8
        dst[15] = 0; // free_heap

        16
    }

    fn try_from_msg_bytes(src: &[u8]) -> Option<Self> {
        None // TODO: Implement this if needed
    }
}

pub struct DataChunk {
    pub adc_data: Option<AdcResults>,
    pub imu_mag_output: Option<ImuMagResults>,
    pub pressure_output: Option<PressureResults>,
    pub other: Option<OtherData>,
    pub localization_output: Option<LocalizationOutput>
}

impl DataChunk { 
    pub const ADC_DATA: u8   = 0b00000001;
    pub const MAG: u8        = 0b00000010;
    pub const ACC_GYRO: u8   = 0b00000100;
    pub const PRESSURE: u8   = 0b00001000;
    pub const LOCALIZED: u8  = 0b00010000;
    pub const MOTOR_DATA: u8 = 0b00100000;
    pub const OTHER_INFO: u8 = 0b01000000;
}

impl From<DataChunk> for Message {
    fn from(value: DataChunk) -> Self {
        let mut dst = [0u8; 256];
        dst[..4].copy_from_slice(&MSG_HEADER);
        dst[5] = MessageType::SendData.into();

        let mut index = 7;
        let mut data_sent: u8 = 0b00000000;

        if let Some(adc_data) = value.adc_data {
            data_sent |= DataChunk::ADC_DATA;

            // info!("Before: index: {}, \n  dst[0..32]: {:?}", index, &dst[0..32]);
            index += adc_data.insert_msg_bytes(&mut dst[index..]);
            // info!(" After: index: {}, \n  dst[0..32]: {:?}\n\n", index, &dst[0..32]);
        }

        if let Some(imu_mag_output) = value.imu_mag_output {
            data_sent |= DataChunk::MAG | DataChunk::ACC_GYRO;

            index += imu_mag_output.insert_msg_bytes(&mut dst[index..]);
        }

        if let Some(pressure_output) = value.pressure_output {
            data_sent |= DataChunk::PRESSURE;

            index += pressure_output.insert_msg_bytes(&mut dst[index..]);
        }

        if let Some(localization_output) = value.localization_output {
            data_sent |= DataChunk::LOCALIZED;

            index += localization_output.insert_msg_bytes(&mut dst[index..]);
        }

        if let Some(adc_data) = value.adc_data {
            data_sent |= DataChunk::MOTOR_DATA;

            index += adc_data.motor_stats().insert_msg_bytes(&mut dst[index..]);
        }

        if let Some(other) = value.other {
            data_sent |= DataChunk::OTHER_INFO;

            index += other.insert_msg_bytes(&mut dst[index..]);
        }

        dst[4] = index as u8;
        dst[6] = data_sent;

        Message(dst)
    }
}

// Config

pub struct PidParams {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub min: f32,
    pub max: f32,
    pub imin: f32,
    pub imax: f32,
}

const NUM_PID_SETTINGS: usize = 2;

pub struct ATConfig {
    // Sensors
    pub mag_bias: Vec3,
    pub mag_scale: Vec3,

    pub acc_bias: Vec3,
    pub acc_scale: Vec3,
    pub gyr_bias: Vec3,

    pub surface_pressure_mbar: f32,
    // Motors
    pub max_duty_cycle: f32,
    pub duty_scaler: f32,

    // PID
    pub pid: [PidParams; NUM_PID_SETTINGS],
    pub ori_filter_beta: f32,
}

// TBD Config to MSG

// Send


// Calibration data

pub struct CalibrationData {
    pub acc_bias: [f32; 3],
    pub acc_scale: [f32; 3],
    pub mag_bias: [f32; 3],
    pub mag_scale: [f32; 3],
    pub gyr_bias: [f32; 3]
}

pub struct TextMessage(pub heapless::String<TEXT_MSG_SIZE_MAX>);

impl From<TextMessage> for Message {
    fn from(value: TextMessage) -> Self {
        let mut dst = [0u8; 256];

        dst[..4].copy_from_slice(&MSG_HEADER);
        dst[4] = 6 + value.0.len() as u8;
        dst[5] = MessageType::Text.into();
        dst[6..6+value.0.len()].copy_from_slice(&value.0.as_bytes()[..250.min(value.0.len())]);

        Message(dst)
    }
}