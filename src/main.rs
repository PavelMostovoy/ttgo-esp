use std::thread::sleep;
use esp_idf_svc::hal::delay::BLOCK;
use esp_idf_svc::hal::gpio;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::uart::*;



fn main() {
    esp_idf_svc::sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let tx = peripherals.pins.gpio33;
    let rx = peripherals.pins.gpio37;

    let config = config::Config::new().baudrate(Hertz(9600));
    let uart = UartDriver::new(
        peripherals.uart1,
        tx,
        rx,
        Option::<gpio::Gpio0>::None,
        Option::<gpio::Gpio1>::None,
        &config,
    )
        .unwrap();

    loop {
        let mut buf = [0_u8; 1];
        uart.read(&mut buf, BLOCK).unwrap();
        let mut message: Vec<String> = vec![];
        if let Ok(char) = std::str::from_utf8(&buf) {
            if char == "$" {


                loop {

                    let mut buf = [0_u8; 1];
                    uart.read(&mut buf, BLOCK).unwrap();

                    if let Ok(char) = std::str::from_utf8(&buf) {
                        if char == "*" {break}

                         message.push(String::from(char));

                    };


                }

            }

        };
        println!("{:?}", message);
        sleep(core::time::Duration::from_secs(1));
    }
}