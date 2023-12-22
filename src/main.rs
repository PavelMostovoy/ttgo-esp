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
        let mut message = String::new();
        if let Ok(char) = std::str::from_utf8(&buf) {
            if char == "\n" {
                let mut counter:u8 = 0;
                loop {
                    if counter >=4{break};
                    let mut buf = [0_u8; 1];
                    uart.read(&mut buf, BLOCK).unwrap();
                    if let Ok(string) = std::str::from_utf8(&buf) {
                        if string == "*" { counter +=1 };
                        let char = string.chars().next().unwrap();

                        message.push(char);

                    };
                    if counter > 0 {counter +=1;}
                }
            }
        };
        println!("{}", message);
        // sleep(core::time::Duration::from_secs(1));
    }
}