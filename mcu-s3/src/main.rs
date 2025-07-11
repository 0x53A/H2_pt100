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
    gpio::{Level, Output, OutputConfig},
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

    let gpio_out_pin = peripherals.GPIO4;
    let adc_in_pin = peripherals.GPIO3;

    let mut gpio_out = Output::new(gpio_out_pin, Level::Low, OutputConfig::default());
    let mut adc_config = esp_hal::analog::adc::AdcConfig::default();
    let mut adc_pin = adc_config.enable_pin(adc_in_pin, Attenuation::_0dB);
    let mut adc = esp_hal::analog::adc::Adc::new(peripherals.ADC1, adc_config);

    let mut str: String<1000> = heapless::String::new();
    write!(&mut str, "Started Loop")?;
    set_status(&mut display, &str)?;

    loop {
        gpio_out.set_high();
        delay.delay_millis(1);

        let mut values: heapless::Vec<u16, 100> = heapless::Vec::new();
        for _i in 0..100 {
            let value = adc.read_blocking(&mut adc_pin);
            values.push(value).unwrap();
        }

        gpio_out.set_low();

        values.sort();
        let median_value = values[values.len() / 2];

        let v = adc_to_volt(median_value);
        let temp = volt_to_temp(v);

        // write to oled display
        display_sensor_values(&mut display, median_value, v, temp)?;

        // output to USB serial
        let mut str: String<1000> = heapless::String::new();
        write!(&mut str, "{temp:0.2}")?;
        esp_println::println!("{str}");

        delay.delay_millis(20);
    }
}

fn adc_to_volt(adc_value: u16) -> f32 {
    const OFFSET: u16 = 1715;
    const VOLT_PER_UNIT: f32 = 0.000227931488801054;

    (adc_value.saturating_sub(OFFSET) as f32) * VOLT_PER_UNIT
}

fn volt_to_temp(adc_voltage: f32) -> f32 {
    const R_SERIES: f32 = 998.0; // 1kΩ Vorwiderstand
    const VCC: f32 = 3.211; // output voltage of the GPIO pin

    // the calculated resistance of the pt100 based on the measured voltage
    let r_pt100 = (adc_voltage * R_SERIES) / (VCC - adc_voltage);

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

fn display_sensor_values(display: &mut MyDisplay, _adc: u16, u: f32, temp: f32) -> Result<()> {
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
    rst: GPIO21<'static>,
    i2c: I2C0<'static>,
    sda: GPIO17<'static>,
    scl: GPIO18<'static>,
    delay: &Delay,
) -> Result<MyDisplay> {
    esp_println::dbg!("About to initialize the Heltec SSD1306 I2C LED driver");

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
