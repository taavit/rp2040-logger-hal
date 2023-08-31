pub mod configuration;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

use configuration::{AccelerationFullScale, Configuration};

static ADDRESS: u8 = 0x1D;

#[allow(dead_code)]
pub enum Register {
    TempOutL = 0x05,
    TempOutH = 0x06,
    StatusM = 0x07,

    OutXLM = 0x08,
    OutXHM = 0x09,
    OutYLM = 0x0A,
    OutYHM = 0x0B,
    OutZLM = 0x0C,
    OutZHM = 0x0D,

    WhoAmI = 0x0F,

    Ctrl0 = 0x1F,
    Ctrl1 = 0x20, //Acceleration register
    Ctrl2 = 0x21,
    Ctrl3 = 0x22,
    Ctrl4 = 0x23,
    Ctrl5 = 0x24, // Temperature registers
    Ctrl6 = 0x25, // Magnetometer resolution
    Ctrl7 = 0x26,

    OutXLA = 0x28,
    OutXHA = 0x29,
    OutYLA = 0x2A,
    OutYHA = 0x2B,
    OutZLA = 0x2C,
    OutZHA = 0x2D,
}

pub struct LSM303D<I2C> {
    i2c: I2C,
    acc_multiplier: f64,
    mag_divider: f64,
    address: u8,
}

#[derive(Default, Debug, Copy, Clone)]
pub struct AccelerometerMeasurements {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Default, Debug, Copy, Clone)]
pub struct MagnetometerMeasurements {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Default, Debug, Clone, Copy)]
pub struct Measurements {
    pub temperature: f64,
    pub magnetometer: MagnetometerMeasurements,
    pub accelerometer: AccelerometerMeasurements,
}

impl<I2C> LSM303D<I2C>
where
    I2C: WriteRead + Write + Read,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            mag_divider: 1.0,
            acc_multiplier: 1.0,
            address: ADDRESS,
        }
    }

    pub fn configure(&mut self, configuration: Configuration) -> Result<(), ()> {
        self.acc_multiplier = match configuration.accelerometer.scale {
            AccelerationFullScale::Acc2G => 20.0,
            AccelerationFullScale::Acc16G => 160.0,
            AccelerationFullScale::Acc4G => 40.0,
            AccelerationFullScale::Acc6G => 60.0,
            AccelerationFullScale::Acc8G => 80.0,
        };
        self.i2c
            .write(
                self.address,
                &[Register::Ctrl1 as u8, configuration.as_ctrl1()],
            )
            .map_err(|_| ())?;
        self.i2c
            .write(
                self.address,
                &[Register::Ctrl2 as u8, configuration.as_ctrl2()],
            )
            .map_err(|_| ())?;
        self.i2c
            .write(
                self.address,
                &[Register::Ctrl5 as u8, configuration.as_ctrl5()],
            )
            .map_err(|_| ())?;
        self.i2c
            .write(
                self.address,
                &[Register::Ctrl6 as u8, configuration.as_ctrl6()],
            )
            .map_err(|_| ())?;
        self.i2c
            .write(
                self.address,
                &[Register::Ctrl7 as u8, configuration.as_ctrl7()],
            )
            .map_err(|_| ())?;

        Ok(())
    }

    #[allow(dead_code)]
    pub fn check_connection(&mut self) -> Result<bool, ()> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.address, &[Register::WhoAmI as u8], &mut buffer)
            .map_err(|_| ())?;

        Ok(buffer[0] == 0b01001001)
    }

    pub fn read_measurements(&mut self) -> Result<Measurements, ()> {
        let mut buffer = [0u8; 2];

        self.i2c
            .write_read(
                self.address,
                &[Register::TempOutL as u8 | 0x80],
                &mut buffer,
            )
            .map_err(|_| ())?;

        let temperature = i16::from_le_bytes(buffer);

        self.i2c
            .write_read(self.address, &[Register::OutXLA as u8 | 0x80], &mut buffer)
            .map_err(|_| ())?;
        let acc_x = (f64::from(i16::from_le_bytes(buffer)) * self.acc_multiplier) / 32678.0;
        self.i2c
            .write_read(self.address, &[Register::OutYLA as u8 | 0x80], &mut buffer)
            .map_err(|_| ())?;
        let acc_y = (f64::from(i16::from_le_bytes(buffer)) * self.acc_multiplier) / 32678.0;
        self.i2c
            .write_read(self.address, &[Register::OutZLA as u8 | 0x80], &mut buffer)
            .map_err(|_| ())?;
        let acc_z = (f64::from(i16::from_le_bytes(buffer)) * self.acc_multiplier) / 32678.0;

        self.i2c
            .write_read(self.address, &[Register::OutXLM as u8 | 0x80], &mut buffer)
            .map_err(|_| ())?;
        let mag_x = (f64::from(i16::from_le_bytes(buffer)) * self.mag_divider) / 32678.0;
        self.i2c
            .write_read(self.address, &[Register::OutYLM as u8 | 0x80], &mut buffer)
            .map_err(|_| ())?;
        let mag_y = (f64::from(i16::from_le_bytes(buffer)) * self.mag_divider) / 32678.0;
        self.i2c
            .write_read(self.address, &[Register::OutZLM as u8 | 0x80], &mut buffer)
            .map_err(|_| ())?;
        let mag_z = (f64::from(i16::from_le_bytes(buffer)) * self.mag_divider) / 32678.0;

        Ok(Measurements {
            temperature: temperature.into(),
            accelerometer: AccelerometerMeasurements {
                x: acc_x,
                y: acc_y,
                z: acc_z,
            },
            magnetometer: MagnetometerMeasurements {
                x: mag_x,
                y: mag_y,
                z: mag_z,
            },
        })
    }
}
