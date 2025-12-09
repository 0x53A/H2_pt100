#![no_std]

use serde::{Serialize, Deserialize};


#[derive(Serialize, Deserialize)]
pub struct SamplingConfig {
    pub resolution_bits: u8,
    pub sample_time_cycles: u16,
    pub oversampling: u16,
    pub n_measurements: u16,
}

#[derive(Serialize, Deserialize)]
pub struct CalibrationConfig {
    pub cal_1_high_resistor_ohms: f32,
    pub cal_1_low_resistor_ohms: f32,
    pub cal_2_high_resistor_ohms: f32,
    pub cal_2_low_resistor_ohms: f32,
}

#[derive(Serialize, Deserialize)]
pub struct Config {
    pub sampling: SamplingConfig,
    pub calibration: CalibrationConfig,

    pub pt100_1_series_resistor_ohms: f32,
    pub pt100_2_series_resistor_ohms: f32,
    // pub pt100_3_series_resistor_ohms: f32,
}

#[derive(Serialize, Deserialize)]
pub struct RawMeasurements {
    pub cal1: MedianStats,
    pub cal2: MedianStats,

    pub pt100_1: MedianStats,
    pub pt100_2: MedianStats,
    // pub pt100_3: MedianStats,
}

#[derive(Serialize, Deserialize)]
pub struct MedianStats {
    /// 20th percentile value
    pub p20: u16,
    /// Median (50th percentile)
    pub median: u16,
    /// 80th percentile value
    pub p80: u16,
}

#[derive(Serialize, Deserialize)]
pub struct Temperatures {
    pub adc_cal: AdcCalibration,
    pub pt100_1: CalculatedValues,
    pub pt100_2: CalculatedValues,
    // pub pt100_3: f32,
}

#[derive(Serialize, Deserialize)]
pub struct CalculatedValues {
    pub r_pt: f32,
    pub temperature: f32,
}

/// ADC calibration data calculated from two reference voltage dividers
#[derive(Serialize, Deserialize)]
pub struct AdcCalibration {
    /// The offset of the ADC, that is, the value that would be measured at 0V
    pub offset: u16,
    /// The value of VCC, with the offset already compensated for.
    /// That is, if you connected VCC to the ADC, you would measure 'offset + vcc'.
    pub vcc: u16,
}

#[derive(Serialize, Deserialize)]
pub struct UartOutput {
    pub boot_id: u32,
    pub config: Config,
    pub raw: RawMeasurements,
    pub calculated: Temperatures,
}