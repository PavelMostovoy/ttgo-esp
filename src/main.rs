mod wifi_module;

use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};
use std::thread::sleep;

use display_interface_spi::SPIInterfaceNoCS;

use esp_idf_svc::hal::{delay, gpio, prelude::*, spi};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use mipidsi;

use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyle};
use embedded_graphics::pixelcolor::*;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::*;
// use esp_idf_svc::nvs::{EspDefaultNvsPartition};

use rand::Rng;


static FLAG: AtomicBool = AtomicBool::new(false);
static FLAG_LIGHT: AtomicBool = AtomicBool::new(false);


fn gpio_int_callback() {
    // Assert FLAG indicating a press button happened
    FLAG.store(true, Ordering::Relaxed);
}

fn backlight_flag() {
    FLAG_LIGHT.store(true, Ordering::Relaxed);
}


fn main() {
    // CPU clock



    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    // wifi setup
    let sysloop = EspSystemEventLoop::take().unwrap();
    let mut wifi = wifi_module::wifi(peripherals.modem, sysloop.clone());


    // setup pins for ttgo display
    let pins = peripherals.pins;
    let backlight: gpio::Gpio4 = pins.gpio4;
    let dc: gpio::Gpio16 = pins.gpio16;
    let rst: gpio::Gpio23 = pins.gpio23;
    let spi: spi::SPI2 = peripherals.spi2;
    let sclk: gpio::Gpio18 = pins.gpio18;
    let sdo: gpio::Gpio19 = pins.gpio19;
    let cs: gpio::Gpio5 = pins.gpio5;

    // Setup button_left
    // Configure button_left pin as input
    let mut button_left = gpio::PinDriver::input(pins.gpio0).unwrap();
    let mut button_right = gpio::PinDriver::input(pins.gpio35).unwrap();
    // Configure button_left pin with internal pull up
    button_left.set_pull(gpio::Pull::Up).unwrap();
    // Configure button_left pin to detect interrupts on a positive edge
    button_left.set_interrupt_type(gpio::InterruptType::NegEdge).unwrap();
    button_right.set_interrupt_type(gpio::InterruptType::AnyEdge).unwrap();
    // Attach the ISR to the button_left interrupt
    unsafe { button_left.subscribe(gpio_int_callback).unwrap() }
    unsafe { button_right.subscribe(backlight_flag).unwrap() }
    // Enable interrupts
    button_left.enable_interrupt().unwrap();
    button_right.enable_interrupt().unwrap();


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
    let mut temp_value = now.elapsed().as_millis();
    let mut count = 0_u32;
    let mut light_status: bool = true;
    loop {
        let color: [u8; 3] = [rand::random::<u8>(), rand::random::<u8>(), rand::random::<u8>()];
        let _ = rectangle_simple(&mut display, &color);
        sleep(Duration::new(0, 1_000_000_000));
        // simple_text_clear(&mut display, format!("{}", &temp_value).as_str());
        // sleep(Duration::new(1, 0));

        //better use log but print also works
        // println!("Second button status {}", FLAG_LIGHT.load(Ordering::Relaxed));
        if FLAG_LIGHT.load(Ordering::Relaxed) {
            button_right.enable_interrupt().unwrap();
            FLAG_LIGHT.store(false, Ordering::Relaxed);
            light_status = !light_status;
            if light_status {
                backlight.set_high().unwrap();
            } else { backlight.set_low().unwrap() }
        }


        if FLAG.load(Ordering::Relaxed) {
            let _ = display.clear(Rgb565::BLACK);
            // Reset global flag
            FLAG.store(false, Ordering::Relaxed);
            // re enabling interrupt
            button_left.enable_interrupt().unwrap();
            // updating count
            count = count.wrapping_add(1);
            if count > 5 {
                break;
            }
        }
    }
    temp_value = now.elapsed().as_millis();
    simple_text_clear(&mut display, format!("{}", &temp_value).as_str());
}

fn rectangle_simple<D>(display: &mut D, color: &[u8; 3]) -> Result<(), D::Error>
    where D: DrawTarget<Color=Rgb565>
{
    let size = Size::new(rand::thread_rng().gen_range(2..20), rand::thread_rng().gen_range(2..20));
    let start_point = Point::new(rand::thread_rng().gen_range(0..125) as i32, rand::thread_rng().gen_range(0..230) as i32);
    Rectangle::new(start_point, size)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::new(color[0], color[1], color[2])))
        .draw(display)?;
    Ok(())
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


    Text::new(data, Point::new(10, (display.bounding_box().size.height - 10) as i32 / 2),
              MonoTextStyle::new(&FONT_10X20, RgbColor::RED)).draw(display)?;

    Ok(())
}

fn simple_text<D>(display: &mut D, data: &str) -> Result<(), D::Error>
    where D: DrawTarget<Color=Rgb565>
{
    Text::new(data, Point::new(10, (display.bounding_box().size.height - 10) as i32 / 2),
              MonoTextStyle::new(&FONT_10X20, RgbColor::BLACK)).draw(display)?;
    Ok(())
}

fn simple_text_clear<D>(display: &mut D, data: &str) -> Result<(), D::Error>
    where D: DrawTarget<Color=Rgb565>
{
    let size = Size::new(45, 20);
    let start_point_text = Point::new(1, 15);
    let start_point = Point::new(0, 0);
    Rectangle::new(start_point, size)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN))
        .draw(display)?;
    Text::new(data, start_point_text,
              MonoTextStyle::new(&FONT_10X20, RgbColor::RED)).draw(display)?;
    Ok(())
}
