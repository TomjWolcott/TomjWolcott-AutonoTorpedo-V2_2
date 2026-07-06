#![no_std]

use embedded_hal_async::{delay::DelayNs, i2c::I2c};

#[cfg(test)]
mod c_implementation {
    extern "C" {
        // C implementation described in the data sheet.
        // This is test only to validate the rust implementation.
        pub fn crc4(buffer: *mut u16) -> u8;
    }
}

/// Generates a 4bit cyclic redundancy check as described in the datasheet.
///
/// The 4bit crc is stored in the 4 LSBs of the result.
fn crc4(buffer: &[u16]) -> u8 {
    let mut n_remainder: u16 = 0;
    for byte in buffer
        .iter()
        .chain([0u16].iter())
        .flat_map(|word| word.to_be_bytes())
    {
        n_remainder ^= byte as u16;
        for _ in 0..8 {
            if n_remainder & 0x8000 != 0 {
                n_remainder = (n_remainder << 1) ^ 0x3000;
            } else {
                n_remainder = n_remainder << 1;
            }
        }
    }
    n_remainder = (n_remainder >> 12) & 0x000F;
    (n_remainder ^ 0x00) as u8
}

/// A catch all error for this driver
#[derive(Debug, PartialEq)]
pub enum SensorError<E> {
    PromCrcMismatch { got: u8, expected: u8 },
    I2cError(E),
}

const I2C_ADDRESS: u8 = 0x76;
pub(crate) mod sealed {
    pub trait Sealed {}
}

pub trait State: sealed::Sealed {}

/// Create an uninitialised driver object
///
/// # Example
///
/// ```
/// // NOTE: Use real i2c instance for your app.
/// use embedded_hal_mock::eh1::i2c::Mock as I2cMock;
/// use ms5837::mock_utils::SleepNop;
/// // NOTE: You should implement the DelayNs trait for this driver to work
/// // correctly.
/// let i2c = I2cMock::new(&[]);
/// let pressure_sensor = ms5837::new(i2c, SleepNop);
/// ```
pub fn new<I2C: I2c, D: DelayNs>(i2c: I2C, sleep: D) -> Uninitialised<I2C, D> {
    Uninitialised::<I2C, D> { i2c, sleep }
}

/// The oversampling ratio to use internal to the ADC. This is analogous to taking
/// n samples and then takeing the average.
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum OverSamplingRatio {
    R256 = 0x0,
    R512 = 0x2,
    R1024 = 0x4,
    R2048 = 0x6,
    R4096 = 0x8,
}

impl OverSamplingRatio {
    fn conversion_time_us(&self) -> u32 {
        use OverSamplingRatio::*;
        match *self {
            R256 => 600,
            R512 => 1170,
            R1024 => 2280,
            R2048 => 4540,
            R4096 => 9040,
        }
    }
}

/// The factory calibration data as fetched from the PROM.
#[derive(PartialEq, Debug)]
pub struct FactoryCalibrationData {
    /// Pressure sensitivity
    pressure_sensitivity: u16,
    pressure_offset: u16,
    temperature_coefficient_of_pressure_sensitivty: u16,
    temperature_coefficient_of_pressure_offset: u16,
    reference_temperature: u16,
    temperature_coefficient_of_temperature: u16,
}

/// An I2C command to send to the pressure sensor.
enum Command {
    Reset,
    ConvertD1(OverSamplingRatio),
    ConvertD2(OverSamplingRatio),
    AdcRead,
    PromRead(u8),
}

/// Convert the command into a single byte that can be sent over i2c.
impl From<Command> for u8 {
    fn from(val: Command) -> u8 {
        use Command::*;
        match val {
            Reset => 0x1E,
            ConvertD1(osr) => 0x40u8 | osr as u8,
            ConvertD2(osr) => 0x50u8 | osr as u8,
            AdcRead => 0x00,
            PromRead(address) => 0xA0u8 | (address << 1),
        }
    }
}

/// An uninitialised ms5837 object.
pub struct Uninitialised<I2C: I2c, D: DelayNs> {
    i2c: I2C,
    sleep: D,
}

impl<I2C: I2c, D: DelayNs> State for Uninitialised<I2C, D> {}
impl<I2C: I2c, D: DelayNs> sealed::Sealed for Uninitialised<I2C, D> {}

impl<I2C: I2c, D: DelayNs> Uninitialised<I2C, D> {
    /// Reset the ms5837 internal state machine.
    async fn reset(&mut self) -> Result<(), SensorError<I2C::Error>> {
        self.i2c
            .write(I2C_ADDRESS, &[Command::Reset.into()])
            .await
            .map_err(SensorError::I2cError)
    }

    /// Read the contents of the PROM.
    async fn read_prom(
        &mut self,
        prom_buffer: &mut [u16; 7],
    ) -> Result<(), SensorError<I2C::Error>> {
        let mut prom_address: u8 = 0;
        for entry in prom_buffer.iter_mut() {
            let mut buffer = [0, 0];
            self.i2c
                .write_read(
                    I2C_ADDRESS,
                    &[Command::PromRead(prom_address).into()],
                    &mut buffer,
                )
                .await
                .map_err(SensorError::I2cError)?;
            *entry = u16::from_be_bytes(buffer);
            prom_address += 1;
        }
        Ok(())
    }

    /// Reads and parses the PROM contents into factory calibration data.
    async fn read_calibration_data(
        &mut self,
    ) -> Result<FactoryCalibrationData, SensorError<I2C::Error>> {
        let mut prom = [0u16; 7];
        self.read_prom(&mut prom).await?;
        let expected_crc4 = ((0xF000 & prom[0]) >> 12) as u8;
        prom[0] = prom[0] & 0x0FFF;
        let got_crc4 = crc4(&prom[..]);
        if expected_crc4 != got_crc4 {
            return Err(SensorError::PromCrcMismatch {
                expected: expected_crc4,
                got: got_crc4,
            });
        }
        let prom = &prom[1..];
        Ok(FactoryCalibrationData {
            pressure_sensitivity: prom[0],
            pressure_offset: prom[1],
            temperature_coefficient_of_pressure_sensitivty: prom[2],
            temperature_coefficient_of_pressure_offset: prom[3],
            reference_temperature: prom[4],
            temperature_coefficient_of_temperature: prom[5],
        })
    }

    /// Releases the i2c handle consuming the driver object.
    ///
    /// # Example
    ///
    /// ```
    /// // NOTE: Use real i2c instance for your app.
    /// use embedded_hal_mock::eh1::i2c::Mock as I2cMock;
    /// // Dummy sleep implementation.
    /// use ms5837::mock_utils::SleepNop;
    /// let i2c = I2cMock::new(&[]);
    /// let pressure_sensor = ms5837::new(i2c, SleepNop);
    /// let i2c = pressure_sensor.release();
    /// ```
    pub fn release(self) -> (I2C, D) {
        (self.i2c, self.sleep)
    }

    /// Initialised the pressure sensor.
    ///
    /// # Errors
    /// Initialisation can fail if;
    /// - There was a problem communicating over i2c.
    /// - There was a crc mismatch when reading factory calibration data off the
    ///   PROM.
    ///
    /// NOTE: on failure the driver will release the i2c handle along with returning
    /// the error.
    ///
    /// # Example
    ///
    /// ```rust
    /// // NOTE: Use real i2c instance for your app.
    /// # use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    /// # let i2c = I2cMock::new(&[I2cTransaction::write(0x76, vec![0x1E]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA0], vec![0x6F, 0xA6]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA2], vec![0x8E, 0x00]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA4], vec![0x4F, 0x68]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA6], vec![0x57, 0x52]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA8], vec![0x66, 0x22]),
    /// #     I2cTransaction::write_read(0x76, vec![0xAA], vec![0x66, 0x22]),
    /// #     I2cTransaction::write_read(0x76, vec![0xAC], vec![0x66, 0x22])
    /// # ]);
    /// use ms5837::mock_utils::SleepNop;
    /// # futures::executor::block_on(async {
    /// let pressure_sensor = ms5837::new(i2c, SleepNop);
    /// let pressure_sensor = pressure_sensor.init().await;
    /// # });
    /// ```
    pub async fn init(mut self) -> Result<Initialised<I2C, D>, SensorError<I2C::Error>> {
        self.reset().await?;

        let calibration_data = self.read_calibration_data().await?;

        Ok(Initialised {
            i2c: self.i2c,
            calibration_data,
            sleep: self.sleep,
        })
    }
}

/// An initialised ms5837 object.
pub struct Initialised<I2C: I2c, D: DelayNs> {
    i2c: I2C,
    calibration_data: FactoryCalibrationData,
    sleep: D,
}

impl<I2C: I2c, D: DelayNs> State for Initialised<I2C, D> {}
impl<I2C: I2c, D: DelayNs> sealed::Sealed for Initialised<I2C, D> {}

/// A group of temperature and pressure samples. These are grouped as pressure
/// normalisation requires sampling the current temperature.
#[derive(Debug)]
pub struct TemperaturePressure {
    pub temperature: f32,
    pub pressure: f32,
}

impl<I2C: I2c, D: DelayNs> Initialised<I2C, D> {
    /// Release the i2c handle consuming the driver.
    pub fn release(self) -> (I2C, D) {
        (self.i2c, self.sleep)
    }

    // Starts conversion and reads raw temperature from the sensor.
    async fn read_raw_temperature(
        &mut self,
        over_sampling_ratio: OverSamplingRatio,
    ) -> Result<u32, SensorError<I2C::Error>> {
        let mut raw_temperature_buffer = [0u8; 4];
        self.i2c
            .write(
                I2C_ADDRESS,
                &[Command::ConvertD2(over_sampling_ratio).into()],
            )
            .await
            .map_err(SensorError::I2cError)?;
        self.sleep
            .delay_us(over_sampling_ratio.conversion_time_us())
            .await;
        self.i2c
            .write_read(
                I2C_ADDRESS,
                &[Command::AdcRead.into()],
                // ADC is 24bit but we are storing in u32.
                &mut raw_temperature_buffer[1..],
            )
            .await
            .map_err(SensorError::I2cError)?;
        Ok(u32::from_be_bytes(raw_temperature_buffer))
    }

    // Starts conversion and reads raw pressure from the sensor.
    async fn read_raw_pressure(
        &mut self,
        over_sampling_ratio: OverSamplingRatio,
    ) -> Result<u32, SensorError<I2C::Error>> {
        let mut raw_pressure_buffer = [0u8; 4];
        self.i2c
            .write(
                I2C_ADDRESS,
                &[Command::ConvertD1(over_sampling_ratio).into()],
            )
            .await
            .map_err(SensorError::I2cError)?;
        self.sleep
            .delay_us(over_sampling_ratio.conversion_time_us())
            .await;
        self.i2c
            .write_read(
                I2C_ADDRESS,
                &[Command::AdcRead.into()],
                // ADC is 24bit but we are storing in u32.
                &mut raw_pressure_buffer[1..],
            )
            .await
            .map_err(SensorError::I2cError)?;
        Ok(u32::from_be_bytes(raw_pressure_buffer))
    }

    // Normalises the raw temperature into degrees C.
    fn normalise_temperature(&self, temperature: u32) -> f32 {
        let temperature = temperature as i64;
        let d_temperature: i64 =
            temperature - (self.calibration_data.reference_temperature as i64) * 2i64.pow(8);

        ((2000
            + d_temperature * (self.calibration_data.temperature_coefficient_of_temperature as i64)
            / 2i64.pow(23)) as f32)
            / 10.0
    }

    // Normalises raw temperature and pressure readings and converts it into a
    // pair of pressure and temperature readings in mbar and deg C respectively.
    fn normalise_raw_measurement(&self, temperature: u32, pressure: u32) -> TemperaturePressure {
        let adc_temperature = temperature as i64;
        let adc_pressure = pressure as i64;
        let FactoryCalibrationData {
            pressure_sensitivity,
            pressure_offset,
            temperature_coefficient_of_pressure_sensitivty,
            temperature_coefficient_of_pressure_offset,
            reference_temperature,
            temperature_coefficient_of_temperature,
        } = self.calibration_data;
        let (
            pressure_sensitivity,
            pressure_offset,
            temperature_coefficient_of_pressure_sensitivty,
            temperature_coefficient_of_pressure_offset,
            reference_temperature,
            temperature_coefficient_of_temperature,
        ): (i64, i64, i64, i64, i64, i64) = (
            pressure_sensitivity.into(),
            pressure_offset.into(),
            temperature_coefficient_of_pressure_sensitivty.into(),
            temperature_coefficient_of_pressure_offset.into(),
            reference_temperature.into(),
            temperature_coefficient_of_temperature.into(),
        );

        let dt = adc_temperature - (reference_temperature << 8);

        // Actual temperature = 2000 + dT * temperature_sensitivity
        let temperature = 2000 + (dt * (temperature_coefficient_of_temperature >> 23));
        let t2;
        let mut offset2;
        let mut sensitivity2;

        // Second order temperature compensation
        if temperature < 2000 {
            t2 = (3 * dt.pow(2)) >> 33;
            offset2 = 3 * (temperature - 2000).pow(2) / 2;
            sensitivity2 = 5 * (temperature - 2000).pow(2) / 8;

            if temperature < -1500 {
                offset2 += 7 * (temperature + 1500).pow(2);
                sensitivity2 += 4 * (temperature + 1500).pow(2);
            }
        } else {
            t2 = (2 * dt.pow(2)) >> 37;
            offset2 = (temperature - 2000).pow(2) >> 4;
            sensitivity2 = 0;
        }

        // OFF = OFF_T1 + TCO * dT
        let mut offset =
            (pressure_offset << 16) + ((temperature_coefficient_of_pressure_offset * dt) >> 7);
        offset -= offset2;

        // Sensitivity at actual temperature = SENS_T1 + TCS * dT
        let mut sensitivty = (pressure_sensitivity << 15)
            + ((temperature_coefficient_of_pressure_sensitivty * dt) >> 8);
        sensitivty -= sensitivity2;

        // Temperature compensated pressure = D1 * SENS - OFF
        let pressure = (((adc_pressure * sensitivty) >> 21) - offset) >> 13;
        let temperature = temperature - t2;

        TemperaturePressure {
            pressure: (pressure as f32) / 10.0,
            temperature: (temperature as f32) / 100.0,
        }
    }

    /// Reads the temperature and pressure samples from the sensor.
    ///
    /// # Errors
    /// This may return an error if there is a problem with i2c communication.
    ///
    /// # Example
    ///
    /// ```rust
    /// # use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    /// # let i2c = I2cMock::new(&[I2cTransaction::write(0x76, vec![0x1E]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA0], vec![0x6F, 0xA6]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA2], vec![0x8E, 0x00]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA4], vec![0x4F, 0x68]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA6], vec![0x57, 0x52]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA8], vec![0x66, 0x22]),
    /// #     I2cTransaction::write_read(0x76, vec![0xAA], vec![0x66, 0x22]),
    /// #     I2cTransaction::write_read(0x76, vec![0xAC], vec![0x66, 0x22]),
    /// #     I2cTransaction::write(0x76, vec![0b0101_1000]),
    /// #     I2cTransaction::write_read(0x76, vec![0x00], vec![0x67,0xFE,0xB6]),
    /// #     I2cTransaction::write(0x76, vec![0b0100_1000] ),
    /// #     I2cTransaction::write_read(0x76, vec![0x00], vec![0x4B,0xA7,0xE3]),
    /// # ]);
    /// use ms5837::{OverSamplingRatio, mock_utils::SleepNop};
    /// # futures::executor::block_on(async {
    /// let pressure_sensor = ms5837::new(i2c, SleepNop);
    /// let mut pressure_sensor = pressure_sensor.init().await.unwrap();
    /// println!(
    ///     "{:?}",
    ///     pressure_sensor
    ///         .read_temperature_and_pressure(OverSamplingRatio::R4096)
    ///         .await
    ///         .unwrap()
    /// );
    /// # });
    /// ```
    pub async fn read_temperature_and_pressure(
        &mut self,
        over_sampling_ratio: OverSamplingRatio,
    ) -> Result<TemperaturePressure, SensorError<I2C::Error>> {
        // Based on figures 9 and 10 from the datasheet.
        let temperature = self.read_raw_temperature(over_sampling_ratio).await?;
        let pressure = self.read_raw_pressure(over_sampling_ratio).await?;

        Ok(self.normalise_raw_measurement(temperature, pressure))
    }

    /// Reads the temperature from the sensor.
    ///
    /// # Errors
    /// This may return an error if there is a problem with i2c communication.
    ///
    /// # Example
    ///
    /// ```rust
    /// # use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    /// # let i2c = I2cMock::new(&[I2cTransaction::write(0x76, vec![0x1E]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA0], vec![0x6F, 0xA6]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA2], vec![0x8E, 0x00]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA4], vec![0x4F, 0x68]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA6], vec![0x57, 0x52]),
    /// #     I2cTransaction::write_read(0x76, vec![0xA8], vec![0x66, 0x22]),
    /// #     I2cTransaction::write_read(0x76, vec![0xAA], vec![0x66, 0x22]),
    /// #     I2cTransaction::write_read(0x76, vec![0xAC], vec![0x66, 0x22]),
    /// #     I2cTransaction::write(0x76, vec![0b0101_1000] ),
    /// #     I2cTransaction::write_read(0x76, vec![0x00], vec![0x67,0xFE,0xB6]),
    /// # ]);
    /// use ms5837::{OverSamplingRatio, mock_utils::SleepNop};
    /// # futures::executor::block_on(async {
    /// let pressure_sensor = ms5837::new(i2c, SleepNop);
    /// let mut pressure_sensor = pressure_sensor.init().await.unwrap();
    /// println!(
    ///     "{:?}",
    ///     pressure_sensor
    ///         .read_temperature(OverSamplingRatio::R4096)
    ///         .await
    ///         .unwrap()
    /// );
    /// # });
    /// ```
    pub async fn read_temperature(
        &mut self,
        over_sampling_ratio: OverSamplingRatio,
    ) -> Result<f32, SensorError<I2C::Error>> {
        let raw_temperature = self.read_raw_temperature(over_sampling_ratio).await?;
        Ok(self.normalise_temperature(raw_temperature))
    }
}