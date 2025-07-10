#![no_std]
#![no_main]
#![feature(never_type)]

use core::fmt::Write;

use embedded_can::{self, *};
use embedded_graphics::{
    mono_font::MonoTextStyle,
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{AdcCalBasic, AdcCalCurve, Attenuation},
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    i2c::{self, master::I2c},
    main,
    peripherals::{GPIO17, GPIO18, GPIO21, I2C0, GPIO10, GPIO3},
    time::Rate,
    twai::{self, filter::SingleStandardFilter, *},
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
        Ok(_) => loop {},
        Err(_e) => loop {},
    }
}

fn _main() -> Result<!> {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = esp_hal::delay::Delay::new();

    let mut display = init_display(
        peripherals.I2C0,
        peripherals.GPIO10,
        peripherals.GPIO3,
        &delay,
    )?;

    let mut str: String<1000> = heapless::String::new();
    write!(&mut str, "Hello World")?;
    set_status(&mut display, &str)?;

    loop { }

    let gpio_out_pin = peripherals.GPIO4;
    let adc_in_pin = peripherals.GPIO3;

    let mut gpio_out = Output::new(gpio_out_pin, Level::Low, OutputConfig::default());
    let mut adc_config = esp_hal::analog::adc::AdcConfig::default();
    let mut adc_pin =
        adc_config.enable_pin(adc_in_pin, Attenuation::_0dB);
    let mut adc = esp_hal::analog::adc::Adc::new(peripherals.ADC1, adc_config);



    loop {
        gpio_out.set_high();
        delay.delay_millis(1);

        let mut values: heapless::Vec<u16, 100> = heapless::Vec::new();
        for _i in 0..100 {
            // let value = adc.read_blocking(&mut adc_pin);
            // values.push(value).unwrap();
        }

        // gpio_out.set_low();

        values.sort();
        let median_value = values[values.len() / 2];

        let mut str: String<1000> = heapless::String::new();
        let v = adc_to_volt(median_value);
        let temp = volt_to_temp(v);

        // let mut str: String<1000> = heapless::String::new();
        // write!(&mut str, "ADC: {median_value}")?;
        // set_status(&mut display, &str)?;

        // delay.delay_millis(200);

        write_sensor_values(&mut display, median_value, v, temp)?;

        // delay.delay_millis(200);
        delay.delay_millis(20);
    }
}

fn adc_to_volt(adc_value: u16) -> f32 {
    const offset: u16 = 1715;
    const volt_per_unit: f32 = 0.000227931488801054;

    (adc_value.saturating_sub(offset) as f32) * volt_per_unit
}

fn volt_to_temp(adc_voltage: f32) -> f32 {
    let V_ADC_MAX = 0.933379447;
    let i_ADC_MAX = 4095;

    let R_SERIES = 998.0; // 1kΩ Vorwiderstand
    let VCC = 3.211; // Versorgungsspannung

    let r_pt100 = (adc_voltage * R_SERIES) / (VCC - adc_voltage);

    // PT100 Widerstand zu Temperatur (vereinfachte lineare Näherung)
    // PT100: R(T) = R0 * (1 + α * T)
    // R0 = 100Ω bei 0°C, α ≈ 0.00385 /°C
    const R0: f32 = 100.0;
    const ALPHA: f32 = 0.00385;

    let temperature = (r_pt100 - R0) / (R0 * ALPHA);

    temperature
}

fn map_temp(input: u16) -> f32 {
    let in_max = 3445.0;
    let in_min = 3160.0;

    let out_max = 100.0;
    let out_min = 32.0;

    let input: f32 = input as f32;

    // (input - _32c) as f32 * (100.0 - 32.0) / (_100c - _32c) as f32 + 32.0

    (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

// Funktion zur Umwandlung des ADC-Werts in Grad Celsius
fn adc_to_celsius(adc_value: u16) -> f32 {
    const ADC_MAX: f32 = ((1 << 13) - 1) as f32; // 8191
    const ADC_VOLTAGE_MAX: f32 = 0.845;
    const VCC: f32 = 3.211;
    const R_SERIES: f32 = 998.0; // 1kΩ Vorwiderstand

    // ADC Wert in Spannung umwandeln
    let adc_voltage = (adc_value as f32 / ADC_MAX) * ADC_VOLTAGE_MAX;

    // Spannungsteiler: V_adc = VCC * R_pt100 / (R_series + R_pt100)
    // Umformen nach R_pt100: R_pt100 = (adc_voltage * R_series) / (VCC - adc_voltage)
    let r_pt100 = (adc_voltage * R_SERIES) / (VCC - adc_voltage);

    // let temp = (adc_value - 0.09090909090909*ADC_MAX) * (200 - 0) / (0.14954*ADC_MAX - 0.09090909090909*ADC_MAX) + 0;

    // PT100 Widerstand zu Temperatur (vereinfachte lineare Näherung)
    // PT100: R(T) = R0 * (1 + α * T)
    // R0 = 100Ω bei 0°C, α ≈ 0.00385 /°C
    const R0: f32 = 100.0;
    const ALPHA: f32 = 0.00385;

    let temperature = (r_pt100 - R0) / (R0 * ALPHA);

    temperature
}

fn write_sensor_values(display: &mut MyDisplay, adc: u16, u: f32, temp: f32) -> Result<()> {
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

    // let mut line1: String<32> = String::new();
    // write!(&mut line1, "ADC: {}", adc).unwrap();

    // let mut line2: String<32> = String::new();
    // write!(&mut line2, "U: {:.2}V | T: {:.2}C", u, temp).unwrap();
    let mut line1: String<32> = String::new();
    write!(&mut line1, "U: {u:.2} V").unwrap();

    let mut line2: String<32> = String::new();
    write!(&mut line2, "T: {temp:.2} C").unwrap();

    let text_style = MonoTextStyle::new(&embedded_graphics::mono_font::ascii::FONT_10X20, fg);

    Text::new(&line1, Point::new(5, 22), text_style)
        .draw(display)
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    Text::new(&line2, Point::new(5, 47), text_style)
        .draw(display)
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    display
        .flush()
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    Ok(())
}

fn init_display(
    i2c: I2C0<'static>,
    sda: GPIO10<'static>,
    scl: GPIO3<'static>,
    delay: &Delay,
) -> Result<MyDisplay> {
    esp_println::dbg!("About to initialize the Heltec SSD1306 I2C LED driver");

    let i2c_config = i2c::master::Config::default().with_frequency(Rate::from_khz(400));

    let i2c = I2c::new(i2c, i2c_config)?.with_scl(scl).with_sda(sda);

    let di = ssd1306::I2CDisplayInterface::new(i2c);

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
