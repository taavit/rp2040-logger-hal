#[allow(dead_code)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum AccelerationDataRate {
    PowerOff = 0b0000_0000,
    Hz3_125 = 0b0001_0000,
    Hz6_25 = 0b0010_0000,
    Hz12_5 = 0b0011_0000,

    Hz25 = 0b0100_0000,
    Hz50 = 0b0101_0000,
    Hz100 = 0b0110_0000,
    Hz200 = 0b0111_0000,
    Hz400 = 0b1000_0000,
    Hz800 = 0b1001_0000,
    Hz1600 = 0b1010_0000,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum AccelerationFullScale {
    Acc2G = 0b0000_0000,
    Acc4G = 0b0000_1000,
    Acc6G = 0b0001_0000,
    Acc8G = 0b0001_1000,
    Acc16G = 0b0010_0000,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum MagnetometerDataRate {
    Hz3_125 = 0b0000_0000,
    Hz6_25 = 0b0000_0100,
    Hz12_5 = 0b0000_1000,
    Hz25 = 0b0000_1100,
    Hz50 = 0b0001_0000,
    Hz100 = 0b0001_0100,
}

#[allow(dead_code)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum MagnetometerResolution {
    Low = 0b0000_0000,
    High = 0b0110_0000,
}

/// Magnetic field in Gauss
#[allow(dead_code)]
#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum MagnetometerFullScale {
    Mag2 = 0b0000_0000,
    Mag4 = 0b0010_0000,
    Mag8 = 0b0100_0000,
    Mag12 = 0b0110_0000,
}

#[allow(dead_code)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum MagneticSensorMode {
    ContinuousConversion = 0b0000_0000,
    SingleConversion = 0b0000_0001,
    PowerDown = 0b0000_0010,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct AccelerometerConfiguration {
    pub axis_x: bool,
    pub axis_y: bool,
    pub axis_z: bool,

    pub data_rate: AccelerationDataRate,
    pub scale: AccelerationFullScale,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct MagnetometerConfiguration {
    pub data_rate: MagnetometerDataRate,
    pub scale: MagnetometerFullScale,
    pub mode: MagneticSensorMode,
    pub resolution: MagnetometerResolution,
}

pub struct InternalTemperatureConfiguration {
    pub active: bool,
}

pub struct Configuration {
    pub magnetometer: MagnetometerConfiguration,
    pub accelerometer: AccelerometerConfiguration,
    pub temperature: InternalTemperatureConfiguration,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            accelerometer: AccelerometerConfiguration {
                axis_x: true,
                axis_y: true,
                axis_z: true,
                data_rate: AccelerationDataRate::PowerOff,
                scale: AccelerationFullScale::Acc2G,
            },
            magnetometer: MagnetometerConfiguration {
                data_rate: MagnetometerDataRate::Hz25,
                mode: MagneticSensorMode::PowerDown,
                scale: MagnetometerFullScale::Mag2,
                resolution: MagnetometerResolution::Low,
            },
            temperature: InternalTemperatureConfiguration { active: false },
        }
    }
}

impl Configuration {
    pub fn configure_accelerometer(
        mut self,
        axis_x: bool,
        axis_y: bool,
        axis_z: bool,
        data_rate: AccelerationDataRate,
        scale: AccelerationFullScale,
    ) -> Self {
        self.accelerometer = AccelerometerConfiguration {
            axis_x,
            axis_y,
            axis_z,
            data_rate,
            scale,
        };

        self
    }

    pub fn configure_magnetometer(
        mut self,
        data_rate: MagnetometerDataRate,
        mode: MagneticSensorMode,
        scale: MagnetometerFullScale,
        resolution: MagnetometerResolution,
    ) -> Self {
        self.magnetometer = MagnetometerConfiguration {
            data_rate,
            mode,
            scale,
            resolution,
        };

        self
    }
    pub fn configure_temperature(mut self, active: bool) -> Self {
        self.temperature = InternalTemperatureConfiguration { active };

        self
    }
    pub fn as_ctrl1(&self) -> u8 {
        let mut ctrl1 = 0;
        ctrl1 |= if self.accelerometer.axis_x {
            0b0000_0001
        } else {
            0b0000_0000
        };
        ctrl1 |= if self.accelerometer.axis_y {
            0b0000_0010
        } else {
            0b0000_0000
        };
        ctrl1 |= if self.accelerometer.axis_z {
            0b0000_0100
        } else {
            0b0000_0000
        };
        ctrl1 |= self.accelerometer.data_rate as u8;

        ctrl1
    }
    pub fn as_ctrl2(&self) -> u8 {
        let mut ctrl2 = 0;
        ctrl2 |= self.accelerometer.scale as u8;

        ctrl2
    }
    pub fn as_ctrl5(&self) -> u8 {
        let mut ctrl5 = 0;

        ctrl5 |= if self.temperature.active {
            0b1000_0000
        } else {
            0b0000_0000
        };
        ctrl5 |= self.magnetometer.resolution as u8;
        ctrl5 |= self.magnetometer.data_rate as u8;

        ctrl5
    }
    pub fn as_ctrl6(&self) -> u8 {
        let mut ctrl6 = 0;

        ctrl6 |= self.magnetometer.scale as u8;

        ctrl6
    }
    pub fn as_ctrl7(&self) -> u8 {
        let mut ctrl7 = 0;
        ctrl7 |= self.magnetometer.mode as u8;

        ctrl7
    }
}
