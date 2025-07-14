#![no_std]
#![no_main]
#![feature(never_type)]

use core::fmt::Write;

use embedded_graphics::{
    mono_font::MonoTextStyle,
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
use esp_backtrace as _;
use esp_hal::{
    analog::adc::Attenuation,
    delay::Delay,
    gpio::{DriveStrength, Level, Output, OutputConfig},
    i2c::{self, master::I2c},
    main,
    peripherals::{GPIO17, GPIO18, GPIO21, I2C0},
    time::Rate,
};

use embedded_graphics::mono_font::ascii::FONT_10X20;

use anyhow::Result;
use heapless::String;
use ssd1306::mode::DisplayConfig;

esp_bootloader_esp_idf::esp_app_desc!();

use esp_alloc as _;

// --

type MyDisplay = ssd1306::Ssd1306<
    ssd1306::prelude::I2CInterface<I2c<'static, esp_hal::Blocking>>,
    ssd1306::prelude::DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsMode<ssd1306::prelude::DisplaySize128x64>,
>;

#[main]
fn main() -> ! {
    // esp_alloc::heap_allocator!(size: 32 * 1024);

    match _main() {
        Err(_e) => loop {},
    }
}

fn _main() -> Result<!> {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = esp_hal::delay::Delay::new();

    let mut display = init_display(
        peripherals.GPIO21,
        peripherals.I2C0,
        peripherals.GPIO17,
        peripherals.GPIO18,
        &delay,
    )?;

    let gpio_out_pin = peripherals.GPIO45;

    let pt100_1_pin = peripherals.GPIO4;
    let pt100_2_pin = peripherals.GPIO5;
    let pt100_3_pin = peripherals.GPIO6;

    let adc_calibration_pin_1 = peripherals.GPIO2;
    let adc_calibration_pin_2 = peripherals.GPIO3;

    let mut gpio_out = Output::new(gpio_out_pin, Level::Low, OutputConfig::default().with_drive_strength(DriveStrength::_40mA));
    
    let mut adc_config = esp_hal::analog::adc::AdcConfig::default();
    
    let mut pt100_1_adc_pin = adc_config.enable_pin(pt100_1_pin, Attenuation::_0dB);
    let mut pt100_2_adc_pin = adc_config.enable_pin(pt100_2_pin, Attenuation::_0dB);
    let mut pt100_3_adc_pin = adc_config.enable_pin(pt100_3_pin, Attenuation::_0dB);

    let mut adc_calibration_pin_1 = adc_config.enable_pin(adc_calibration_pin_1, Attenuation::_0dB);
    let mut adc_calibration_pin_2 = adc_config.enable_pin(adc_calibration_pin_2, Attenuation::_0dB);
    
    
    let mut adc = esp_hal::analog::adc::Adc::new(peripherals.ADC1, adc_config);
    
    let mut str: String<1000> = heapless::String::new();
    write!(&mut str, "Started Loop")?;

    set_status(&mut display, &str)?;

    loop {
        gpio_out.set_high();
        delay.delay_millis(1);

        // calibrate

        let mut values_calibration_1: heapless::Vec<u16, 100> = heapless::Vec::new();
        let mut values_calibration_2: heapless::Vec<u16, 100> = heapless::Vec::new();
        for _i in 0..100 {
            let value = adc.read_blocking(&mut adc_calibration_pin_1);
            values_calibration_1.push(value).unwrap();
        }

        for _i in 0..100 {
            let value = adc.read_blocking(&mut adc_calibration_pin_2);
            values_calibration_2.push(value).unwrap();
        }

        let mut values_pt100_1: heapless::Vec<u16, 100> = heapless::Vec::new();
        let mut values_pt100_2: heapless::Vec<u16, 100> = heapless::Vec::new();
        let mut values_pt100_3: heapless::Vec<u16, 100> = heapless::Vec::new();
        for _i in 0..100 {
            let value = adc.read_blocking(&mut pt100_1_adc_pin);
            values_pt100_1.push(value).unwrap();
        }
        for _i in 0..100 {
            let value = adc.read_blocking(&mut pt100_2_adc_pin);
            values_pt100_2.push(value).unwrap();
        }
        for _i in 0..100 {
            let value = adc.read_blocking(&mut pt100_3_adc_pin);
            values_pt100_3.push(value).unwrap();
        }

        gpio_out.set_low();

        values_calibration_1.sort();
        values_calibration_2.sort();

        values_pt100_1.sort();
        values_pt100_2.sort();
        values_pt100_3.sort();

        let median_value_calibration_1 = values_calibration_1[values_calibration_1.len() / 2];
        let median_value_calibration_2 = values_calibration_2[values_calibration_2.len() / 2];
        
        let adc_calibration = calculate_adc_calibration(median_value_calibration_1, median_value_calibration_2);

        let median_value_pt100_1 = values_pt100_1[values_pt100_1.len() / 2];
        let median_value_pt100_2 = values_pt100_2[values_pt100_2.len() / 2];
        let median_value_pt100_3 = values_pt100_3[values_pt100_3.len() / 2];

        let temp_pt100_1 = adc_to_temp(&adc_calibration, median_value_pt100_1, 998.0);

        let temp_pt100_2 = adc_to_temp(&adc_calibration, median_value_pt100_2, 998.8);

        let temp_pt100_3 = adc_to_temp(&adc_calibration, median_value_pt100_3, 999.0);

        // write to oled display
        display_sensor_values(&mut display, temp_pt100_1, temp_pt100_2, temp_pt100_3)?;

        // output to USB serial
        let mut str: String<1000> = heapless::String::new();
        write!(&mut str, "{temp_pt100_1:0.2},{temp_pt100_2:0.2},{temp_pt100_3:0.2}")?;
        esp_println::println!("{str}");

        delay.delay_millis(20);
    }
}

struct ADC_Calibration {
    /// the offset of the ADC, that is, the value that would be measured at 0V
    offset: u16,
    /// the value of VCC, with the offset already compensated for. That is, if you connected VCC to the ADC, you would measure 'offset + vcc'.
    vcc: u16,
}

fn calculate_adc_calibration(value_1: u16, value_2: u16) -> ADC_Calibration {

    // we have two resistor dividers (see schematic).

    // I've measured the actual resistances the following:
    // With VCC = 5.004 V
    // First divider has 0.457 V
    // Second divider has 0.225 V

    const R1 : f32 = 100_000.0; // 100k
    const R2 : f32 = 10_050.0; // 10k

    const R3 : f32 = 100_000.0; // 100k
    const R4 : f32 = 4_710.0; // 4.7k

    // Calculate the voltage division ratios
    let ratio_1 = R2 / (R1 + R2); // ~0.0913 or 9.13% of VCC
    let ratio_2 = R4 / (R3 + R4); // ~0.0450 or 4.50% of VCC

    // Now we need to solve the system of equations:
    // value_1 = offset + ratio_1 * vcc
    // value_2 = offset + ratio_2 * vcc
    
    // Subtract to eliminate offset:
    // value_1 - value_2 = (ratio_1 - ratio_2) * vcc
    
    // Solve for vcc:
    let vcc_raw = (value_1 as f32 - value_2 as f32) / (ratio_1 - ratio_2);
    
    // Now solve for offset:
    // offset = value_1 - ratio_1 * vcc
    // or equivalently: offset = value_2 - ratio_2 * vcc
    let offset = (value_1 as f32 - ratio_1 * vcc_raw).clamp(0.0, u16::max_value() as f32) as u16;
    
    let vcc = vcc_raw.clamp(0.0, u16::max_value() as f32) as u16;
    
    ADC_Calibration {
        offset,
        vcc,
    }
}

fn adc_to_temp(adc_calibration: &ADC_Calibration, adc_value: u16, r_series: f32) -> f32 {

    let adc_corrected = adc_value.saturating_sub(adc_calibration.offset) as f32;
    let percent_vcc = adc_corrected / adc_calibration.vcc as f32;
    
    // the calculated resistance of the pt100
    let r_pt100 = (percent_vcc * r_series) / (1.0 - percent_vcc);

    // PT100: R(T) = R0 * (1 + α * T)
    // R0 = 100Ω bei 0°C, α ≈ 0.00385 1/°C
    const R0: f32 = 100.0;
    const ALPHA: f32 = 0.00385;

    let temperature = (r_pt100 - R0) / (R0 * ALPHA);

    temperature
}

/// alternatively, if you measure two temperature values with an external thermometer,
/// you can use a simple two-point mapping.
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

fn display_sensor_values(display: &mut MyDisplay, temp_pt100_1: f32, temp_pt100_2: f32, temp_pt100_3: f32) -> Result<()> {
    let bg = BinaryColor::Off;
    let fg = BinaryColor::On;

    display.clear(bg).unwrap();

    Rectangle::new(display.bounding_box().top_left, display.bounding_box().size)
        .into_styled(
            PrimitiveStyleBuilder::new()
                .fill_color(bg)
                .stroke_color(fg)
                .stroke_width(1)
                .build(),
        )
        .draw(display)
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    let mut line1: String<32> = String::new();
    write!(&mut line1, "H-Mat.: {temp_pt100_1:.2} C").unwrap();

    let mut line2: String<32> = String::new();
    write!(&mut line2, "In: {temp_pt100_2:.2} C").unwrap();

    let mut line3: String<32> = String::new();
    write!(&mut line3, "Out: {temp_pt100_3:.2} C").unwrap();

    let text_style = MonoTextStyle::new(&embedded_graphics::mono_font::ascii::FONT_9X15, fg);

    Text::new(&line1, Point::new(5, 18), text_style)
        .draw(display)
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    Text::new(&line2, Point::new(5, 37), text_style)
        .draw(display)
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    Text::new(&line3, Point::new(5, 54), text_style)
        .draw(display)
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    display
        .flush()
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    Ok(())
}

fn init_display(
    rst: GPIO21<'static>,
    i2c: I2C0<'static>,
    sda: GPIO17<'static>,
    scl: GPIO18<'static>,
    delay: &Delay,
) -> Result<MyDisplay> {
    // esp_println::dbg!("About to initialize the Heltec SSD1306 I2C LED driver");

    let i2c_config = i2c::master::Config::default().with_frequency(Rate::from_khz(400));

    let i2c = I2c::new(i2c, i2c_config)?.with_scl(scl).with_sda(sda);

    let di = ssd1306::I2CDisplayInterface::new(i2c);

    let mut reset = Output::new(rst, Level::High, OutputConfig::default());

    // high for 1 ms
    delay.delay_millis(1 as u32);

    reset.set_low();
    delay.delay_millis(10 as u32);

    reset.set_high();

    // Note:
    //   PinDriver of IDF had a Drop implementation that resets the pin, which would turn off the display
    //   This seems to no longer be neccessary when using esp-hal.
    // mem::forget(reset);

    let mut display: ssd1306::Ssd1306<
        ssd1306::prelude::I2CInterface<I2c<'_, esp_hal::Blocking>>,
        ssd1306::prelude::DisplaySize128x64,
        ssd1306::mode::BufferedGraphicsMode<ssd1306::prelude::DisplaySize128x64>,
    > = ssd1306::Ssd1306::new(
        di,
        ssd1306::size::DisplaySize128x64,
        ssd1306::rotation::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    display
        .init()
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    write_text(
        &mut display,
        "Hello Rust!",
        BinaryColor::Off,
        BinaryColor::On,
        BinaryColor::Off,
        BinaryColor::On,
    )
    .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    display
        .flush()
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    Ok(display)
}

fn write_text<D>(
    display: &mut D,
    text: &str,
    bg: D::Color,
    fg: D::Color,
    fill: D::Color,
    stroke: D::Color,
) -> Result<(), D::Error>
where
    D: DrawTarget + Dimensions,
{
    display.clear(bg)?;

    Rectangle::new(display.bounding_box().top_left, display.bounding_box().size)
        .into_styled(
            PrimitiveStyleBuilder::new()
                .fill_color(fill)
                .stroke_color(stroke)
                .stroke_width(1)
                .build(),
        )
        .draw(display)?;

    Text::new(
        &text,
        Point::new(10, (display.bounding_box().size.height - 10) as i32 / 2),
        MonoTextStyle::new(&FONT_10X20, fg),
    )
    .draw(display)?;

    Ok(())
}

fn set_status(display: &mut MyDisplay, text: &str) -> Result<()> {
    write_text(
        display,
        &text,
        BinaryColor::Off,
        BinaryColor::On,
        BinaryColor::Off,
        BinaryColor::On,
    )
    .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    display
        .flush()
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    return Ok(());
}
