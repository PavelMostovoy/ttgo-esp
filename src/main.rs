use display_interface_spi::SPIInterfaceNoCS;
use esp_idf_svc::hal::{delay, gpio, prelude::*, spi};
use mipidsi;
use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyle};
use embedded_graphics::pixelcolor::*;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::*;
use std::time::{Duration, Instant};
use std::thread::sleep;
use esp_idf_svc::sys::time;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    // setup pins for ttgo display
    let pins = peripherals.pins;
    let backlight: gpio::Gpio4 = pins.gpio4;
    let dc: gpio::Gpio16 = pins.gpio16;
    let rst: gpio::Gpio23 = pins.gpio23;
    let spi: spi::SPI2 = peripherals.spi2;
    let sclk: gpio::Gpio18 = pins.gpio18;
    let sdo: gpio::Gpio19 = pins.gpio19;
    let cs: gpio::Gpio5 = pins.gpio5;

    let mut backlight = gpio::PinDriver::output(backlight).unwrap();
    backlight.set_high().unwrap();

    let di = SPIInterfaceNoCS::new(
        spi::SpiDeviceDriver::new_single(
            spi,
            sclk,
            sdo,
            Option::<gpio::Gpio21>::None,
            Some(cs),
            &spi::SpiDriverConfig::new().dma(spi::Dma::Disabled),
            &spi::SpiConfig::new().baudrate(26.MHz().into()),
        ).unwrap(),
        gpio::PinDriver::output(dc).unwrap(),
    );
    // Inverted color fixed by using st7789_pico1
    let mut display = mipidsi::Builder::st7789_pico1(di)
        .init(&mut delay::Ets, Some(gpio::PinDriver::output(rst).unwrap())).unwrap();

    display.set_orientation(mipidsi::options::Orientation::Portrait(false)).unwrap();
    let _ = display.clear(Rgb565::BLACK);

    // The TTGO board's screen does not start at offset 0x0, and the physical size is 135x240, instead of 240x320
    // let top_left = Point::new(0, 0);
    // let size = Size::new(135, 240);
    // let mut cropped_display =  display.cropped(&Rectangle::new(top_left, size));

    led_draw(&mut display, "test ").unwrap();
    sleep(Duration::new(1, 0));
    let now = Instant::now();
    for n in 1..10 {
        let _ = display.clear(Rgb565::BLACK);
        sleep(Duration::new(1, 0));
        led_draw(&mut display, format!("{}", now.elapsed().as_secs()).as_str()).unwrap();
        sleep(Duration::new(1, 0));
        let _ = display.clear(Rgb565::RED);
        sleep(Duration::new(1, 0));
    }




}

fn led_draw<D>(display: &mut D, data: &str) -> Result<(), D::Error>
    where
        D: DrawTarget,
        D::Color: RgbColor,
{
    display.clear(RgbColor::BLACK)?;

    Rectangle::new(display.bounding_box().top_left, display.bounding_box().size)
        .into_styled(
            PrimitiveStyleBuilder::new()
                .fill_color(RgbColor::BLUE)
                .stroke_color(RgbColor::YELLOW)
                .stroke_width(10)
                .build(),
        ).draw(display)?;


    Text::new(data,Point::new(10, (display.bounding_box().size.height - 10) as i32 / 2),
              MonoTextStyle::new(&FONT_10X20, RgbColor::RED)).draw(display)?;

    Ok(())
}
