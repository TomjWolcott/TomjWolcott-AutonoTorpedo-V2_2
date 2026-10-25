use cortex_m::prelude::_embedded_hal_Pwm;
use defmt::{info, Format};
use embassy_stm32::gpio::{Flex, Level, Speed};
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::simple_pwm::{SimplePwm, SimplePwmChannels};
use embassy_stm32::peripherals::{TIM1, TIM2, TIM3};
use embassy_sync::watch::Watch;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};

use crate::adc_tasks::ADC_RESULTS;
use crate::comms::Messagable;

pub static MOTOR_INPUTS: Watch<CriticalSectionRawMutex, MotorInputs, 2> = Watch::new();

#[derive(Default, Clone, Copy, Format, Debug)]
pub struct MotorInputs {
    pub speeds: [f32; 4],
    pub gainsels: [GainselState; 4]
}

impl MotorInputs {
    pub const OFF: Self = MotorInputs { speeds: [0.0; 4], gainsels: [GainselState::HighCurrent; 4] };

    pub fn new(m_tl: f32, m_tr: f32, m_bl: f32, m_br: f32) -> Self {
        let mut speeds = [0.0; 4];

        speeds[MotorId::TL.num()] = m_tl;
        speeds[MotorId::TR.num()] = m_tr;
        speeds[MotorId::BL.num()] = m_bl;
        speeds[MotorId::BR.num()] = m_br;
        MotorInputs { speeds, gainsels: [GainselState::HighCurrent; 4] }
    }
}

#[derive(Default, Clone, Copy, Format, Debug)]
pub enum GainselState {
    #[default]
    HighCurrent = 0,
    MedCurrent = 1,
    LowCurrent = 2,
}

impl GainselState {
    pub fn num(&self) -> usize {
        match self {
            GainselState::LowCurrent => 2,
            GainselState::MedCurrent => 1,
            GainselState::HighCurrent => 0,
        }
    }
}

// TODO: Add more complex characterization here: Speed vs. input curve etc.
#[derive(Clone, Copy, Format)]
pub struct MotorCharacterization {
    minimum_speed: f32, // Range: [0.0, 1.0] before multiplier
    multiplier: f32, // Range: (-Inf, Inf)
    index: MotorIndex
}

#[derive(Clone, Copy, Format)]
pub enum MotorId {
    TL = 0, TR = 1, BL = 2, BR = 3
}

impl MotorId {
    pub fn num(self) -> usize {
        match self {
            MotorId::TL => 0,
            MotorId::TR => 1,
            MotorId::BL => 2,
            MotorId::BR => 3
        }
    }
}

const MOTOR_CHARACTERIZATIONS: [MotorCharacterization; 4] = [
    MotorCharacterization { minimum_speed: 0.6, multiplier: -1.0, index: MotorIndex::M3 }, // TL
    MotorCharacterization { minimum_speed: 0.6, multiplier: 1.0, index: MotorIndex::M1 }, // TR
    MotorCharacterization { minimum_speed: 0.6, multiplier: 1.0, index: MotorIndex::M2 }, // BL
    MotorCharacterization { minimum_speed: 0.6, multiplier: -1.0, index: MotorIndex::M0 },  // BR
];

fn motor(id: MotorId) -> MotorCharacterization {
    MOTOR_CHARACTERIZATIONS[id.num()]
}

impl Into<MotorIndex> for MotorId {
    fn into(self) -> MotorIndex {
        motor(self).index
    }
}

#[derive(Clone, Copy, Format)]
pub enum MotorIndex {
    M0 = 0, M1 = 1, M2 = 2, M3 = 3
}

impl MotorIndex {
    fn num(self) -> usize {
        match self {
            MotorIndex::M0 => 0,
            MotorIndex::M1 => 1,
            MotorIndex::M2 => 2,
            MotorIndex::M3 => 3
        }
    }

    fn from_num(n: usize) -> Option<Self> {
        match n {
            0 => Some(MotorIndex::M0),
            1 => Some(MotorIndex::M1),
            2 => Some(MotorIndex::M2),
            3 => Some(MotorIndex::M3),
            _ => None
        }
    }
}

impl MotorIndex {
    const VARIANTS: [Self; 4] = [Self::M0, Self::M1, Self::M2, Self::M3];
}

pub struct MotorControllersPeri {
    gainsels: [Flex<'static>; 4],
    gainsel_states: [GainselState; 4],
    prev_speeds: [f32; 4],
    ipropi_indexes: [u32; 4],
    m0_channels: SimplePwmChannels<'static, TIM1>, // CH3 = m0_forward, CH4 = m0_reverse
    m1_channels: SimplePwmChannels<'static, TIM2>, // CH3 = m1_forward, CH4 = m1_reverse
    m2_m3_channels: SimplePwmChannels<'static, TIM3>, // CH1 = m2_forward, CH2 = m2_reverse, CH3 = m3_forward, CH4 = m4_reverse
}

impl MotorControllersPeri {
    pub fn new(
        gainsels: [Flex<'static>; 4],
        ipropi_indexes: [u32; 4],
        mut t1: SimplePwm<'static, TIM1>,
        mut t2: SimplePwm<'static, TIM2>,
        mut t3: SimplePwm<'static, TIM3>,
    ) -> Self {
        t1.enable(Channel::Ch3);
        t1.enable(Channel::Ch4);

        t2.enable(Channel::Ch3);
        t2.enable(Channel::Ch4);

        t3.enable(Channel::Ch1);
        t3.enable(Channel::Ch2);
        t3.enable(Channel::Ch3);
        t3.enable(Channel::Ch4);

        let mut peri = Self {
            gainsels,
            gainsel_states: [GainselState::HighCurrent; 4],
            ipropi_indexes,
            prev_speeds: [0.0; 4],
            m0_channels: t1.split(),
            m1_channels: t2.split(),
            m2_m3_channels: t3.split(),
        };

        peri.turn_off();
        peri.set_all_gainsel(GainselState::HighCurrent);

        peri
    }

    pub fn turn_off(&mut self) {
        self.m0_channels.ch3.set_duty_cycle_fully_off();
        self.m0_channels.ch4.set_duty_cycle_fully_off();
        self.m1_channels.ch3.set_duty_cycle_fully_off();
        self.m1_channels.ch4.set_duty_cycle_fully_off();
        self.m2_m3_channels.ch1.set_duty_cycle_fully_off();
        self.m2_m3_channels.ch2.set_duty_cycle_fully_off();
        self.m2_m3_channels.ch3.set_duty_cycle_fully_off();
        self.m2_m3_channels.ch4.set_duty_cycle_fully_off();
    }

    pub async fn set_motor_speed(&mut self, motor_index: MotorIndex, speed: f32) {
        let prev_speed = self.prev_speeds[motor_index.num()];
        let is_forward = speed > 0.0;
        if speed.abs() > 0.0 { info!("Setting motor {:?} speed to {}, prev: [{:?}]", motor_index, speed, self.prev_speeds);} 

        // if (prev_speed.abs() <= 0.6 || (speed * prev_speed) < 0.0) && speed.abs() > 0.0 {
        //     info!("HIT: prev_speed: {}, speed: {}", prev_speed, speed);
        //     self.set_motor_speed_raw(motor_index, 100, is_forward);
        //     Timer::after(Duration::from_millis(2)).await;  // TODO: There are sometimes race conditions here!!!
        // }

        self.prev_speeds[motor_index.num()] = speed;
        let speed_u8 = ((100.0 * speed.abs()) as u8) % 101;

        self.set_motor_speed_raw(motor_index, speed_u8, is_forward);
    }

    fn set_motor_speed_raw(&mut self, motor_index: MotorIndex, speed_u8: u8, is_forward: bool) {
        match motor_index {
            MotorIndex::M0 => {
                if is_forward {
                    self.m0_channels.ch3.set_duty_cycle_percent(speed_u8);
                    self.m0_channels.ch4.set_duty_cycle_fully_off();
                } else {
                    self.m0_channels.ch3.set_duty_cycle_fully_off();
                    self.m0_channels.ch4.set_duty_cycle_percent(speed_u8);
                }
            },
            MotorIndex::M1 => {
                if is_forward {
                    self.m1_channels.ch3.set_duty_cycle_percent(speed_u8);
                    self.m1_channels.ch4.set_duty_cycle_fully_off();
                } else {
                    self.m1_channels.ch3.set_duty_cycle_fully_off();
                    self.m1_channels.ch4.set_duty_cycle_percent(speed_u8);
                }
            },
            MotorIndex::M2 => {
                if is_forward {
                    self.m2_m3_channels.ch1.set_duty_cycle_percent(speed_u8);
                    self.m2_m3_channels.ch2.set_duty_cycle_fully_off();
                } else {
                    self.m2_m3_channels.ch1.set_duty_cycle_fully_off();
                    self.m2_m3_channels.ch2.set_duty_cycle_percent(speed_u8);
                }
            },
            MotorIndex::M3 => {
                if is_forward {
                    self.m2_m3_channels.ch3.set_duty_cycle_percent(speed_u8);
                    self.m2_m3_channels.ch4.set_duty_cycle_fully_off();
                } else {
                    self.m2_m3_channels.ch3.set_duty_cycle_fully_off();
                    self.m2_m3_channels.ch4.set_duty_cycle_percent(speed_u8);
                }
            }
        }
    }

    pub fn set_gainsel(&mut self, motor_index: MotorIndex, _gainsel_state: GainselState) {
        //   TODO: Acutally let gainsel change, this is to prevent the situation where
        // the stated gainsel does not match what the motor driver sees causing motor 
        // current to be calculated incorrectly.
        let gainsel_state = GainselState::HighCurrent;
        match gainsel_state {
            GainselState::LowCurrent => {
                self.gainsels[motor_index.num()].set_level(Level::High);
                self.gainsels[motor_index.num()].set_as_output(Speed::Low);
            },
            GainselState::MedCurrent => {
                self.gainsels[motor_index.num()].set_level(Level::Low);
                self.gainsels[motor_index.num()].set_as_analog();
            },
            GainselState::HighCurrent => {
                self.gainsels[motor_index.num()].set_level(Level::Low);
                self.gainsels[motor_index.num()].set_as_output(Speed::Low);
            }
        }

        self.gainsel_states[motor_index.num()] = gainsel_state;
    }

    pub fn set_all_gainsel(&mut self, gainsel_state: GainselState) {
        self.set_gainsel(MotorIndex::M0, gainsel_state);
        self.set_gainsel(MotorIndex::M1, gainsel_state);
        self.set_gainsel(MotorIndex::M2, gainsel_state);
        self.set_gainsel(MotorIndex::M3, gainsel_state);
    }

    pub async fn set_from_inputs(&mut self, inputs: &MotorInputs) {
        for i in 0..4 {
            self.set_motor_speed(MotorIndex::from_num(i).unwrap(), inputs.speeds[i]).await;
            // self.set_gainsel(MotorIndex::from_num(i).unwrap(), inputs.gainsels[i]);
        }
    }
}

#[embassy_executor::task]
pub async fn motor_controller_test(
    mut motor_controller: MotorControllersPeri
) {
    let mut adc_reader = ADC_RESULTS.receiver().unwrap();
    let mut rx = MOTOR_INPUTS.receiver().unwrap();

    loop {
        let motor_inputs = rx.changed().await;

        motor_controller.set_from_inputs(&motor_inputs).await;

        // if let Some(adc_results) = adc_reader.try_get() {
        //     info!("adc_results: {:?}", adc_results);
        // }

        // info!("motor_inputs: {:?}", motor_inputs);
    }
}

#[derive(Clone, Copy, Format, Default)]
pub struct MotorStats {
    pub motor_a: [f32; 4],
    pub motor_v: [f32; 4],
    pub motor_ins: [f32; 4],
}

impl Messagable for MotorStats {
    fn insert_msg_bytes(&self, dst: &mut [u8]) -> usize {
        let mut motor_a_data: [u32; 4] = [0; 4];
        let mut motor_v_data: [u32; 4] = [0; 4];
        let mut motor_ins_data: [u32; 4] = [0; 4];

        for i in 0..4 {
            motor_a_data[i] = (65536.0 * self.motor_a[i] / 8.0) as u32;
            motor_v_data[i] = (65536.0 * self.motor_v[i] / 8.0) as u32;
            motor_ins_data[i] = (65536.0 * (self.motor_ins[i] + 1.0) / 2.0) as u32;
        }

        dst[0..8].copy_from_slice(&[
            ((motor_a_data[0] >> 8) & 0xFF) as u8,
            (motor_a_data[0] & 0xFF) as u8,
            ((motor_a_data[1] >> 8) & 0xFF) as u8,
            (motor_a_data[1] & 0xFF) as u8,
            ((motor_a_data[2] >> 8) & 0xFF) as u8,
            (motor_a_data[2] & 0xFF) as u8,
            ((motor_a_data[3] >> 8) & 0xFF) as u8,
            (motor_a_data[3] & 0xFF) as u8
        ]);

        dst[8..16].copy_from_slice(&[
            ((motor_v_data[0] >> 8) & 0xFF) as u8,
            (motor_v_data[0] & 0xFF) as u8,
            ((motor_v_data[1] >> 8) & 0xFF) as u8,
            (motor_v_data[1] & 0xFF) as u8,
            ((motor_v_data[2] >> 8) & 0xFF) as u8,
            (motor_v_data[2] & 0xFF) as u8,
            ((motor_v_data[3] >> 8) & 0xFF) as u8,
            (motor_v_data[3] & 0xFF) as u8
        ]);

        dst[16..24].copy_from_slice(&[
            ((motor_ins_data[0] >> 8) & 0xFF) as u8,
            (motor_ins_data[0] & 0xFF) as u8,
            ((motor_ins_data[1] >> 8) & 0xFF) as u8,
            (motor_ins_data[1] & 0xFF) as u8,
            ((motor_ins_data[2] >> 8) & 0xFF) as u8,
            (motor_ins_data[2] & 0xFF) as u8,
            ((motor_ins_data[3] >> 8) & 0xFF) as u8,
            (motor_ins_data[3] & 0xFF) as u8
        ]);

        24
    }

    fn try_from_msg_bytes(_src: &[u8]) -> Option<Self> {
        None // TODO: Implement this if needed
    }

}