use embassy_sync::watch::Watch;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_stm32::adc::{Adc, AnyAdcChannel, SampleTime};
use embassy_stm32::peripherals::{ADC1, ADC2, DMA2_CH1, DMA2_CH2};
use embassy_stm32::Peri;
use embassy_time::{Duration, Ticker};
use crate::Irqs;
use crate::comms::Messagable;
use crate::motor_tasks::{MOTOR_INPUTS, MotorInputs, MotorStats};
use defmt::{info, Format};

pub static ADC_RESULTS: Watch<CriticalSectionRawMutex, AdcResults, 4> = Watch::new();

#[derive(Default, Clone, Copy, Format)]
pub struct AdcResults {
    pub vref_v: f32,
    pub temp_c: f32,
    pub ipropis_v: [f32; 4],
    pub motor_stats: MotorStats,
    pub batt_v: f32,
    pub estimated_batt_v: f32,
}

const TEMP30_CAL_ADDR:  *const u16 = 0x1FFF_75A8 as *const u16;
const TEMP130_CAL_ADDR: *const u16 = 0x1FFF_75CA as *const u16;
const VREFINT_CAL_ADDR: *const u16 = 0x1FFF_75AA as *const u16;
const DRV8213_IPROPI_R: f32 = 860.0;
// Ohms
const DRV8213_REF_V: f32 = 0.501;
const DRV8213_GAINS: [f32; 3] = [205.0, 1050.0, 4900.0];

// Voltage drop = -0.526 * I^2 + 0.053 * I
const BATT_ESR: f32 = 0.4607;

impl AdcResults {
    pub fn compute(data: [u16; 7], motor_inputs: MotorInputs) -> Self {
        let vrefint_v = Self::calc_vrefint_v(data[0]);
        let batt_v = 3.0 * Self::calc_voltage_v(data[6], vrefint_v);
        let mut ipropis_v = [0.0; 4];
        let mut motor_a = [0.0; 4];
        let mut motor_v = [0.0; 4];
        let mut motor_w = [0.0; 4];
        let mut total_current = 0.0;

        for i in 0..4 {
            ipropis_v[i] = Self::calc_voltage_v(data[2+i], vrefint_v);
            motor_v[i] = batt_v * motor_inputs.speeds[i].abs();
            motor_a[i] = 1000000.0 * ipropis_v[i] / (DRV8213_IPROPI_R * DRV8213_GAINS[motor_inputs.gainsels[i].num()]);
            motor_w[i] = motor_v[i] * motor_a[i];

            total_current += motor_a[i];
        }

        let batt_v = 3.0 * Self::calc_voltage_v(data[6], vrefint_v);
        let voltage_drop = BATT_ESR * total_current;

        Self {
            vref_v: vrefint_v,
            temp_c: Self::calc_temp_c(data[1], vrefint_v),
            ipropis_v,
            motor_stats: MotorStats { motor_a, motor_v, motor_ins: motor_inputs.speeds },
            batt_v,
            estimated_batt_v: batt_v - voltage_drop,    
        }
    }

    fn calc_temp_c(raw_temp: u16, vrefint_v: f32) -> f32 {
        let temp30 = unsafe { core::ptr::read_volatile(TEMP30_CAL_ADDR) } as f32;
        let temp130 = unsafe { core::ptr::read_volatile(TEMP130_CAL_ADDR) } as f32;

        if temp130 != temp30 {
            let scaled = (raw_temp as f32 * vrefint_v) / 3.0;
            ((scaled - temp30) * (130.0 - 30.0)) / (temp130 - temp30) + 30.0
        } else {
            f32::NAN // sentinel for invalid calibration
        }
    }

    fn calc_vrefint_v(raw_vrefint: u16) -> f32 {
        let vrefint_cal = unsafe { core::ptr::read_volatile(VREFINT_CAL_ADDR) } as f32;
        (vrefint_cal * 3.0) / raw_vrefint as f32
    }

    fn calc_voltage_v(raw_data: u16, vrefint_v: f32) -> f32 {
        (raw_data as f32 * vrefint_v) / 4095.0
    }

    pub fn motor_stats(&self) -> MotorStats {
        self.motor_stats.clone()
    }
}

impl Messagable for AdcResults {
    fn insert_msg_bytes(&self, dst: &mut [u8]) -> usize {
        let vref_mv = (self.vref_v * 1000.0) as u16;
        let self_temp_c = (self.temp_c) as u16;
        let batt_mv = (self.batt_v * 1000.0) as u16;
        let est_batt_mv = (self.estimated_batt_v * 1000.0) as u16;
        let mut ipropis_mv = [0u16; 4];

        for i in 0..4 { ipropis_mv[i] = (self.ipropis_v[i] * 1000.0) as u16 }

        dst[0..16].copy_from_slice(&[
            (vref_mv >> 8) as u8,
            (vref_mv & 0xFF) as u8,
            (self_temp_c >> 8) as u8,
            (self_temp_c & 0xFF) as u8,
            (batt_mv >> 8) as u8,
            (batt_mv & 0xFF) as u8,
            (est_batt_mv >> 8) as u8,
            (est_batt_mv & 0xFF) as u8,
            (ipropis_mv[0] >> 8) as u8,
            (ipropis_mv[0] & 0xFF) as u8,
            (ipropis_mv[1] >> 8) as u8,
            (ipropis_mv[1] & 0xFF) as u8,
            (ipropis_mv[2] >> 8) as u8,
            (ipropis_mv[2] & 0xFF) as u8,
            (ipropis_mv[3] >> 8) as u8,
            (ipropis_mv[3] & 0xFF) as u8,
        ]);
        16 // Return the number of bytes written
    }

    fn try_from_msg_bytes(_src: &[u8]) -> Option<Self> {
        None // TODO: Implement this if needed
    }
}

pub struct AdcController {
    pub vref: AnyAdcChannel<'static, ADC1>,
    pub temp: AnyAdcChannel<'static, ADC1>,
    pub m0_ipropi: AnyAdcChannel<'static, ADC1>,
    pub m1_ipropi: AnyAdcChannel<'static, ADC1>,
    pub m2_ipropi: AnyAdcChannel<'static, ADC2>,
    pub m3_ipropi: AnyAdcChannel<'static, ADC2>,
    pub batt_voltage: AnyAdcChannel<'static, ADC2>,
    pub adc1: Adc<'static, ADC1>,
    pub adc2: Adc<'static, ADC2>,
    pub dma_adc1: Peri<'static, DMA2_CH1>,
    pub dma_adc2: Peri<'static, DMA2_CH2>,
}

impl AdcController {
    async fn read_all(&mut self) -> [u16; 7] {
        let mut buf = [0u16; 7];
        let (buf1, buf2) = buf.split_at_mut(4);

        self.adc1.read(self.dma_adc1.reborrow(), Irqs, [
            (&mut self.vref, SampleTime::CYCLES640_5),
            (&mut self.temp, SampleTime::CYCLES640_5),
            (&mut self.m0_ipropi, SampleTime::CYCLES640_5),
            (&mut self.m1_ipropi, SampleTime::CYCLES640_5)
        ].into_iter(), buf1.try_into().unwrap()).await;

        self.adc2.read(self.dma_adc2.reborrow(), Irqs, [
            (&mut self.m2_ipropi, SampleTime::CYCLES640_5),
            (&mut self.m3_ipropi, SampleTime::CYCLES640_5),
            (&mut self.batt_voltage, SampleTime::CYCLES640_5),
        ].into_iter(), buf2.try_into().unwrap()).await;

        buf
    }
}

#[embassy_executor::task]
pub async fn adc_test(
    mut adc_controller: AdcController
) {
    let sender = ADC_RESULTS.sender();
    let mut ticker = Ticker::every(Duration::from_hz(5));
    let mut motor_inputs = MotorInputs::default();
    let mut motor_rx = MOTOR_INPUTS.receiver().unwrap();

    loop {
        if let Some(new_motor_inputs) = motor_rx.try_get() {
            motor_inputs = new_motor_inputs;
        }

        let data = adc_controller.read_all().await;
        let adc_results = AdcResults::compute(data, motor_inputs);

        sender.send(adc_results);

        ticker.next().await;
    }
}