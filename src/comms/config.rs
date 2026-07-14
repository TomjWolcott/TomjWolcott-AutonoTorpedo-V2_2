use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize)]
pub struct MagCalibration {
    bias: [f32; 3],
    scale: [f32; 3],
}

#[derive(Deserialize, Serialize)]
pub struct ImuCalibration {
    acc_bias: [f32; 3],
    acc_scale: [f32; 3],
    gyr_bias: [f32; 3],
}

#[derive(Deserialize, Serialize)]
pub struct PressureCalibration {
    surface_pressure_atm: f32
}

#[derive(Deserialize, Serialize)]
pub struct Calibrations {
    mag: MagCalibration,
    imu: ImuCalibration,
    pressure: PressureCalibration
}

#[derive(Deserialize, Serialize)]
pub struct ATConfig {
    calibrations: Calibrations,
    max_current_draw: f32,
    motor_pwm_duty_limit: f32,
}