use embassy_stm32::gpio::{Flex, Level, Speed};
use embassy_stm32::timer::simple_pwm::{SimplePwm, SimplePwmChannels};
use embassy_stm32::peripherals::{TIM1, TIM2, TIM3};

#[derive(Default, Clone, Copy)]
pub enum GainselState {
    #[default]
    HighCurrent,
    MedCurrent,
    LowCurrent,
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

#[derive(Clone, Copy)]
pub enum MotorId {
    TL = 0, TR = 1, BL = 2, BR = 3
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

#[derive(Clone, Copy)]
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
        t1: SimplePwm<'static, TIM1>,
        t2: SimplePwm<'static, TIM2>,
        t3: SimplePwm<'static, TIM3>,
    ) -> Self {
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
        let speed_u8 = (100.0 * speed.abs()) as u8;

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
                self.gainsels[motor_index.num()].set_level(Level::Low);
                self.gainsels[motor_index.num()].set_as_output(Speed::Low);
            },
            GainselState::MedCurrent => {
                self.gainsels[motor_index.num()].set_level(Level::Low);
                self.gainsels[motor_index.num()].set_as_analog();
            },
            GainselState::HighCurrent => {
                self.gainsels[motor_index.num()].set_level(Level::High);
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
}