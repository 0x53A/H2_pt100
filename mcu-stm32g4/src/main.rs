#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::adc::vals::{Rovsm, Trovs};
use embassy_stm32::adc::{Adc, AdcConfig, Dual, Resolution, SampleTime};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::opamp::{OpAmp, OpAmpGain, OpAmpSpeed};
use embassy_stm32::rng::Rng;
use embassy_stm32::usart::UartTx;
use embassy_stm32::{bind_interrupts, opamp};
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

const R_SERIES_1: f32 = 3183.6;
const R_SERIES_2: f32 = 3203.3;

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

    let mut p = embassy_stm32::init(config);

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

    // GPIO out
    // Note: currently the GPIO is always on,
    //   but if measurement frequency is reduced,
    //   it could be switched off between measurements to reduce pt100 self-heating

    // let mut gpio_out_1 = Output::new(p.PA4, Level::High, Speed::Low);
    // gpio_out_1.set_high();

    // let mut gpio_out_2 = Output::new(p.PA15, Level::High, Speed::Low);
    // gpio_out_2.set_high();

    // loop {

    // }

    // --------------------------------------------------
    // ADC Pin Configuration for PT100 sensors
    // --------------------------------------------------

    // PT100 sensor ADC pins
    let mut pt100_pin_1 = p.PA7;
    let mut pt100_pin_2 = p.PB0;

    // let mut opamp1 = OpAmp::new(p.OPAMP1, OpAmpSpeed::Normal);
    // opamp1.calibrate();

    let mut opamp2 = OpAmp::new(p.OPAMP2, OpAmpSpeed::Normal);
    opamp2.calibrate();

    // // Initialize ADC
    // let mut adc1 = Adc::new(
    //     p.ADC1,
    //     AdcConfig {
    //         dual_mode: None,
    //         oversampling_mode: Some((Rovsm::CONTINUED, Trovs::AUTOMATIC, true)),
    //         oversampling_ratio: Some(0b111), // 256x
    //         oversampling_shift: Some(4),
    //         resolution: Some(Resolution::BITS12),
    //     },
    // );

    // Initialize ADC2
    let mut adc2 = Adc::new(
        p.ADC2,
        AdcConfig {
            dual_mode: None,
            oversampling_mode: Some((Rovsm::CONTINUED, Trovs::AUTOMATIC, true)),
            oversampling_ratio: Some(0b111), // 256x
            oversampling_shift: Some(4),     //Some(4),
            resolution: Some(Resolution::BITS12),
        },
    );

    // ------------------------

    info!("Peripherals initialized!");

    let mut delay: Delay = Delay;
    delay.delay_ms(100);

    info!("Starting measurement loop");

    // let mut ticker = Ticker::every(Duration::from_millis(100));

    loop {
        const N_MEASUREMENTS: usize = 30;

        led_status.toggle();

        let mut values_pt100_1: Vec<u16, N_MEASUREMENTS> = Vec::new();
        let mut values_pt100_2: Vec<u16, N_MEASUREMENTS> = Vec::new();

        for _ in 0..N_MEASUREMENTS {
            // --------------------------------------------------
            // Read PT100 sensor values
            // --------------------------------------------------

            {
                let mut pt100_1 = opamp2.pga_int(pt100_pin_1.reborrow(), OpAmpGain::Mul16);
                let value = adc2.blocking_read(&mut pt100_1, SampleTime::CYCLES640_5);
                let _ = values_pt100_1.push(value);
            }

            {
                let mut pt100_2 = opamp2.pga_int(pt100_pin_2.reborrow(), OpAmpGain::Mul16);
                let value = adc2.blocking_read(&mut pt100_2, SampleTime::CYCLES640_5);
                let _ = values_pt100_2.push(value);
            }
        }

        // --------------------------------------------------
        // Calculate median values
        // --------------------------------------------------

        let stats_pt100_1 = calculate_median(&mut values_pt100_1);
        let stats_pt100_2 = calculate_median(&mut values_pt100_2);

        // --------------------------------------------------
        // Convert ADC values to temperature
        // --------------------------------------------------

        let temp_pt100_1 = adc_to_temp(stats_pt100_1.median, R_SERIES_1, 16);
        let temp_pt100_2 = adc_to_temp(stats_pt100_2.median, R_SERIES_2, 16);

        // --------------------------------------------------
        // Output values
        // --------------------------------------------------

        info!(
            "1: {} (+{} -{}) | 2: {} (+{} -{})",
            stats_pt100_1.median,
            stats_pt100_1.p80 - stats_pt100_1.median,
            stats_pt100_1.median - stats_pt100_1.p20,
            stats_pt100_2.median,
            stats_pt100_2.p80 - stats_pt100_2.median,
            stats_pt100_2.median - stats_pt100_2.p20,
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
                    amplification: 16,
                },
                pt100_1_series_resistor_ohms: R_SERIES_1,
                pt100_2_series_resistor_ohms: R_SERIES_2,
            },
            raw: RawMeasurements {
                pt100_1: stats_pt100_1,
                pt100_2: stats_pt100_2,
            },
            calculated: Temperatures {
                pt100_1: temp_pt100_1,
                pt100_2: temp_pt100_2,
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

// --------------------------------------------------
// Temperature Conversion
// --------------------------------------------------

fn adc_to_temp(adc_value: u16, r_series: f32, opamp_gain: u16) -> CalculatedValues {
    let de_amplified_adc_value = adc_value as f32 / u16::MAX as f32;
    let percent_vcc = de_amplified_adc_value / opamp_gain as f32;
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
