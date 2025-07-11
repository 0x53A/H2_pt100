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
