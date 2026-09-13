use cortex_m::prelude::_embedded_hal_Pwm;
use defmt::{info, Format};
use embassy_stm32::gpio::{Flex, Level, Speed};
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::simple_pwm::{SimplePwm, SimplePwmChannels};
use embassy_stm32::peripherals::{TIM1, TIM2, TIM3};
use embassy_sync::watch::Watch;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use serde::{Deserialize, Serialize};

use crate::adc_tasks::ADC_RESULTS;

pub static MOTOR_INPUTS: Watch<CriticalSectionRawMutex, MotorInputs, 4> = Watch::new();

#[derive(Default, Clone, Copy, Format, Serialize, Deserialize)]
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

#[derive(Default, Clone, Copy, Format, Serialize, Deserialize)]
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

const MOTOR_MAP: [MotorIndex; 4] = [
    MotorIndex::M0,
    MotorIndex::M1,
    MotorIndex::M2,
    MotorIndex::M3
];

impl Into<MotorIndex> for MotorId {
    fn into(self) -> MotorIndex {
        MOTOR_MAP[self as usize]
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
}

pub struct MotorControllersPeri {
    gainsels: [Flex<'static>; 4],
    gainsel_states: [GainselState; 4],
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

    pub fn set_motor_speed(&mut self, motor_index: MotorIndex, speed: f32) {
        let speed_u8 = ((100.0 * speed.abs()) as u8) % 101;

        info!("Setting motor {:?} speed to {} ({}%)", motor_index, speed, speed_u8);
        match motor_index {
            MotorIndex::M0 => {
                if speed > 0.0 {
                    self.m0_channels.ch3.set_duty_cycle_percent(speed_u8);
                    self.m0_channels.ch4.set_duty_cycle_fully_off();
                } else {
                    self.m0_channels.ch3.set_duty_cycle_fully_off();
                    self.m0_channels.ch4.set_duty_cycle_percent(speed_u8);
                }
            },
            MotorIndex::M1 => {
                if speed > 0.0 {
                    self.m1_channels.ch3.set_duty_cycle_percent(speed_u8);
                    self.m1_channels.ch4.set_duty_cycle_fully_off();
                } else {
                    self.m1_channels.ch3.set_duty_cycle_fully_off();
                    self.m1_channels.ch4.set_duty_cycle_percent(speed_u8);
                }
            },
            MotorIndex::M2 => {
                if speed > 0.0 {
                    self.m2_m3_channels.ch1.set_duty_cycle_percent(speed_u8);
                    self.m2_m3_channels.ch2.set_duty_cycle_fully_off();
                } else {
                    self.m2_m3_channels.ch1.set_duty_cycle_fully_off();
                    self.m2_m3_channels.ch2.set_duty_cycle_percent(speed_u8);
                }
            },
            MotorIndex::M3 => {
                if speed > 0.0 {
                    self.m2_m3_channels.ch3.set_duty_cycle_percent(speed_u8);
                    self.m2_m3_channels.ch4.set_duty_cycle_fully_off();
                } else {
                    self.m2_m3_channels.ch3.set_duty_cycle_fully_off();
                    self.m2_m3_channels.ch4.set_duty_cycle_percent(speed_u8);
                }
            }
        }
    }

    pub fn set_gainsel(&mut self, motor_index: MotorIndex, gainsel_state: GainselState) {
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

    pub fn set_from_inputs(&mut self, inputs: &MotorInputs) {
        for i in 0..4 {
            self.set_motor_speed(MOTOR_MAP[i], inputs.speeds[i]);
            self.set_gainsel(MOTOR_MAP[i], inputs.gainsels[i]);
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

        motor_controller.set_from_inputs(&motor_inputs);

        // if let Some(adc_results) = adc_reader.try_get() {
        //     info!("adc_results: {:?}", adc_results);
        // }

        // info!("motor_inputs: {:?}", motor_inputs);
    }
}