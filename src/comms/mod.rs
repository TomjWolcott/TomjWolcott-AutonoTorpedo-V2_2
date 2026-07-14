use serde::{Deserialize, Serialize};

mod config;


#[derive(Clone, Serialize, Deserialize)]
pub enum Message {
    Ping,
    Action(Action),
    SendData(SensorData),
    
}

#[derive(Clone, Serialize, Deserialize)]
pub struct SensorData {
    mag: [f32; 3],
    acc: [f32; 3]
}

#[derive(Clone, Serialize, Deserialize)]
pub enum Action {
    Noop = 0,
    SetMotorSpeeds = 2,
}