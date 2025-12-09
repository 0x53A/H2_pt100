#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::adc::vals::{Rovsm, Trovs};
use embassy_stm32::adc::{Adc, Resolution, SampleTime};
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::opamp::{OpAmp, OpAmpSpeed};
use embassy_stm32::rng::Rng;
use embassy_stm32::usart::UartTx;
use embassy_time::{Delay, Duration, Ticker};
use embedded_hal::delay::DelayNs as _;
use heapless::String;
use heapless::Vec;
use serde::Serialize;
use {defmt_rtt as _, panic_probe as _};


use common::*;

bind_interrupts!(struct Irqs {
    RNG => embassy_stm32::rng::InterruptHandler<embassy_stm32::peripherals::RNG>;
});


#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("Boot");

    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::mux::*;
        use embassy_stm32::rcc::*;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL85,
            divp: None,
            divq: Some(PllQDiv::DIV4),
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.mux.adc12sel = Adcsel::SYS;
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
    }

    let p = embassy_stm32::init(config);

    info!("Clocks initialized");

    // --------------------------------------------------
    // Init peripherals
    // --------------------------------------------------

    let mut rng = Rng::new(p.RNG, Irqs);
    let id = rng.next_u32();

    // UART
    let mut usart = UartTx::new(
        p.USART2,
        p.PA2,
        p.DMA2_CH6,
        embassy_stm32::usart::Config::default(),
    )
    .unwrap();

    // status LED
    let mut led_status = Output::new(p.PB8, Level::High, Speed::Low);

    // let opamp = OpAmp::new(p.OPAMP1, OpAmpSpeed::Normal);
    // opamp.buffer_int(p.);
    // opamp.pga_int(in_pin, out_pin, gain)

    // GPIO out
    // Note: currently the GPIO is always on,
    //   but if measurement frequency is reduced,
    //   it could be switched off between measurements to reduce pt100 self-heating
    let mut gpio_out = Output::new(p.PB7, Level::Low, Speed::Low);

    // --------------------------------------------------
    // ADC Pin Configuration for PT100 sensors
    // --------------------------------------------------

    // PT100 sensor ADC pins
    let mut pt100_1_pin = p.PA0;
    let mut pt100_2_pin = p.PA7;
    // let mut pt100_3_pin = p.PA3;

    // Calibration resistor divider pins
    let mut adc_calibration_pin_1 = p.PA4;
    let mut adc_calibration_pin_2 = p.PA1;

    // Initialize ADC
    // let mut adc1 = Adc::new(p.ADC1);
    // adc1.set_resolution(Resolution::BITS12);
    // adc1.set_sample_time(SampleTime::CYCLES640_5);
    // adc1.set_oversampling_ratio(0b011); // Note: this is directly written to cfgr2::OVSR, 0b111 = 256x, 0b011 = 16x
    // // adc1.set_oversampling_shift(4);
    // adc1.enable_regular_oversampling_mode(Rovsm::CONTINUED, Trovs::AUTOMATIC, true);

    // Initialize ADC2
    let mut adc2 = Adc::new(p.ADC2);
    adc2.set_resolution(Resolution::BITS12);
    adc2.set_sample_time(SampleTime::CYCLES640_5);
    adc2.set_oversampling_ratio(0b111); // Note: this is directly written to cfgr2::OVSR, 0b111 = 256x, 0b011 = 16x
    adc2.set_oversampling_shift(4);
    adc2.enable_regular_oversampling_mode(Rovsm::RESUMED, Trovs::AUTOMATIC, true);

    // ------------------------

    info!("Peripherals initialized!");

    let mut delay: Delay = Delay;
    delay.delay_ms(100);

    info!("Starting measurement loop");

    // let mut ticker = Ticker::every(Duration::from_millis(100));

    loop {
        const N_MEASUREMENTS: usize = 20;

        led_status.toggle();

        gpio_out.set_high();
        delay.delay_ms(1);

        let mut values_calibration_1: Vec<u16, N_MEASUREMENTS> = Vec::new();
        let mut values_calibration_2: Vec<u16, N_MEASUREMENTS> = Vec::new();

        let mut values_pt100_1: Vec<u16, N_MEASUREMENTS> = Vec::new();
        let mut values_pt100_2: Vec<u16, N_MEASUREMENTS> = Vec::new();
        // let mut values_pt100_3: Vec<u16, N_MEASUREMENTS> = Vec::new();

        for _ in 0..N_MEASUREMENTS {
            // --------------------------------------------------
            // Read calibration values
            // --------------------------------------------------

            let value = adc2.blocking_read(&mut adc_calibration_pin_1);
            let _ = values_calibration_1.push(value);

            let value = adc2.blocking_read(&mut adc_calibration_pin_2);
            let _ = values_calibration_2.push(value);

            // --------------------------------------------------
            // Read PT100 sensor values
            // --------------------------------------------------

            let value = adc2.blocking_read(&mut pt100_1_pin);
            let _ = values_pt100_1.push(value);

            let value = adc2.blocking_read(&mut pt100_2_pin);
            let _ = values_pt100_2.push(value);

            //     let value = adc2.blocking_read(&mut pt100_3_pin);
            //     let _ = values_pt100_3.push(value);

            // delay.delay_ns(10);
        }

        // gpio_out.set_low();

        // --------------------------------------------------
        // Calculate median values
        // --------------------------------------------------
        let stats_calibration_1 = calculate_median(&mut values_calibration_1);
        let stats_calibration_2 = calculate_median(&mut values_calibration_2);
        let stats_pt100_1 = calculate_median(&mut values_pt100_1);
        let stats_pt100_2 = calculate_median(&mut values_pt100_2);
        // let stats_pt100_3 = calculate_median(&mut values_pt100_3);

        let adc_calibration =
            calculate_adc_calibration(stats_calibration_1.median, stats_calibration_2.median);

        // Debug: print raw ADC measurements
        // info!(
        //     "Raw ADC - Cal1: {} [{}-{}], Cal2: {} [{}-{}], PT1: {} [{}-{}], PT2: {} [{}-{}], PT3: {} [{}-{}]",
        //     stats_calibration_1.median, stats_calibration_1.p20, stats_calibration_1.p80,
        //     stats_calibration_2.median, stats_calibration_2.p20, stats_calibration_2.p80,
        //     stats_pt100_1.median, stats_pt100_1.p20, stats_pt100_1.p80,
        //     stats_pt100_2.median, stats_pt100_2.p20, stats_pt100_2.p80,
        //     stats_pt100_3.median, stats_pt100_3.p20, stats_pt100_3.p80
        // );
        // info!(
        //     "ADC Calibration - offset: {}, vcc: {}",
        //     adc_calibration.offset,
        //     adc_calibration.vcc
        // );

        // --------------------------------------------------
        // Convert ADC values to temperature
        // --------------------------------------------------

        // resistors have been measured, let's add 1 ohm to compensate for lead resistance
        let temp_pt100_1 = adc_to_temp(&adc_calibration, stats_pt100_1.median, 998.0 + 1.0);
        let temp_pt100_2 = adc_to_temp(&adc_calibration, stats_pt100_2.median, 999.4 + 1.0);
        // let temp_pt100_3 = adc_to_temp(&adc_calibration, stats_pt100_3.median, 998.2 + 1.0);

        // --------------------------------------------------
        // Output temperature values
        // --------------------------------------------------

        // Log via defmt (RTT) - convert to fixed point for display
        let t1_int = (temp_pt100_1.temperature * 100.0) as i32;
        let t2_int = (temp_pt100_2.temperature * 100.0) as i32;
        info!(
            "Temperatures: {}.{:02}({:02}Ω), {}.{:02}({:02}Ω)",
            t1_int / 100,
            (t1_int % 100).abs(),
            temp_pt100_1.r_pt,
            t2_int / 100,
            (t2_int % 100).abs(),
            temp_pt100_2.r_pt,
        );

        // Output via UART as JSON
        let output = UartOutput {
            boot_id: id,
            config: Config {
                sampling: SamplingConfig {
                    resolution_bits: 12,
                    sample_time_cycles: 640,
                    oversampling: 256,
                    n_measurements: N_MEASUREMENTS as u16,
                },
                calibration: CalibrationConfig {
                    cal_1_high_resistor_ohms: 100_000.0,
                    cal_1_low_resistor_ohms: 10_050.0,
                    cal_2_high_resistor_ohms: 100_000.0,
                    cal_2_low_resistor_ohms: 4_700.0,
                },
                pt100_1_series_resistor_ohms: 998.0 + 1.0,
                pt100_2_series_resistor_ohms: 999.4 + 1.0,
            },
            raw: RawMeasurements {
                cal1: stats_calibration_1,
                cal2: stats_calibration_2,
                pt100_1: stats_pt100_1,
                pt100_2: stats_pt100_2,
                // pt100_3: stats_pt100_3,
            },
            calculated: Temperatures {
                adc_cal: adc_calibration,
                pt100_1: temp_pt100_1,
                pt100_2: temp_pt100_2,
                // pt100_3: temp_pt100_3,
            },
        };

        let mut uart_buf: [u8; 2048] = [0; 2048];
        match serde_json_core::to_slice(&output, &mut uart_buf) {
            Ok(json_str) => {
                let _ = usart.blocking_write(&uart_buf[..json_str]);
                let _ = usart.blocking_write(b"\r\n");
            }
            Err(_) => {
                let _ = usart.blocking_write(b"Serialization Error\r\n");
            }
        }

        // ticker.next().await;
    }
}

fn calculate_median<const N: usize>(values: &mut Vec<u16, N>) -> MedianStats {
    let len = values.len();
    if len == 0 {
        return MedianStats {
            p20: 0,
            median: 0,
            p80: 0,
        };
    }

    // Insertion sort
    for i in 1..len {
        let key = values[i];
        let mut j = i;
        while j > 0 && values[j - 1] > key {
            values[j] = values[j - 1];
            j -= 1;
        }
        values[j] = key;
    }

    MedianStats {
        p20: values[len * 20 / 100],
        median: values[len / 2],
        p80: values[len * 80 / 100],
    }
}

/// Format temperatures as CSV string with 2 decimal places
fn _format_temperatures(
    s: &mut String<64>,
    t1: f32,
    t2: f32,
    t3: f32,
) -> Result<(), core::fmt::Error> {
    use core::fmt::Write as FmtWrite;

    // Convert to fixed-point (2 decimal places)
    let t1_int = (t1 * 100.0) as i32;
    let t2_int = (t2 * 100.0) as i32;
    let t3_int = (t3 * 100.0) as i32;

    core::write!(
        s,
        "{}.{:02},{}.{:02},{}.{:02}\r\n",
        t1_int / 100,
        (t1_int % 100).abs(),
        t2_int / 100,
        (t2_int % 100).abs(),
        t3_int / 100,
        (t3_int % 100).abs()
    )
}

// --------------------------------------------------
// ADC Calibration
// --------------------------------------------------

fn calculate_adc_calibration(value_1: u16, value_2: u16) -> AdcCalibration {
    // We have two resistor dividers (see schematic).

    // I've measured the actual resistances the following:
    // With VCC = 5.004 V
    // First divider has 0.457 V
    // Second divider has 0.225 V

    const R1: f32 = 100_000.0; // 100k
    const R2: f32 = 10_050.0; // 10k

    const R3: f32 = 100_000.0; // 100k
    const R4: f32 = 4_700.0; // 4.7k

    // Calculate the voltage division ratios
    let ratio_1 = R2 / (R1 + R2); // ~0.0913 or 9.13% of VCC
    let ratio_2 = R4 / (R3 + R4); // ~0.0450 or 4.50% of VCC

    // Solve the system of equations:
    // value_1 = offset + ratio_1 * vcc
    // value_2 = offset + ratio_2 * vcc
    //
    // Subtract to eliminate offset:
    // value_1 - value_2 = (ratio_1 - ratio_2) * vcc
    //
    // Solve for vcc:
    let vcc_raw = (value_1 as f32 - value_2 as f32) / (ratio_1 - ratio_2);

    // Now solve for offset:
    // offset = value_1 - ratio_1 * vcc
    let offset = (value_1 as f32 - ratio_1 * vcc_raw).clamp(0.0, u16::MAX as f32) as u16;

    let vcc = vcc_raw.clamp(0.0, u16::MAX as f32) as u16;

    AdcCalibration { offset, vcc }
}

// --------------------------------------------------
// Temperature Conversion
// --------------------------------------------------

fn adc_to_temp(
    adc_calibration: &AdcCalibration,
    adc_value: u16,
    r_series: f32,
) -> CalculatedValues {
    let adc_corrected = adc_value.saturating_sub(adc_calibration.offset) as f32;
    let percent_vcc = adc_corrected / adc_calibration.vcc as f32;

    // Calculate the resistance of the platinum probe
    let r_pt = (percent_vcc * r_series) / (1.0 - percent_vcc);

    // PT100/PT1000: R(T) = R0 * (1 + α * T)
    // PT100:  R0 = 100Ω at 0°C
    // PT1000: R0 = 1000Ω at 0°C
    // α ≈ 0.00385 1/°C (same for both)
    const ALPHA: f32 = 0.00385;

    // Auto-detect sensor type based on resistance
    // PT100 at 150°C ≈ 158Ω, PT1000 at 0°C = 1000Ω
    // Threshold of 300Ω safely distinguishes between them
    let r0: f32 = if r_pt < 300.0 { 100.0 } else { 1000.0 };

    let temperature = (r_pt - r0) / (r0 * ALPHA);

    CalculatedValues { r_pt, temperature }
}

/// alternatively, if you measure two temperature values with an external thermometer,
/// you can use a simple two-point mapping.
/// (old values from esp version)
fn _map_temp(adc_value: u16) -> f32 {
    // at 100C, the ADC was outputting 3445
    let out_max = 100.0;
    let in_max = 3445.0;

    // at 32C, the ADC was outputting 3160
    let out_min = 32.0;
    let in_min = 3160.0;

    let input: f32 = adc_value as f32;

    (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}
