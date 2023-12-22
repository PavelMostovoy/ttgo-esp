mod wifi_module;

use std::time::{Duration};
use std::thread::sleep;


use esp_idf_svc::hal::{prelude::*};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::http::client::{Configuration, EspHttpConnection};

use embedded_svc::http::client::{Client};
use embedded_svc::http::Method;
use embedded_svc::io::Read;


fn main() {


    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();


    // wifi setup
    let sysloop = EspSystemEventLoop::take().unwrap();
    let wifi = wifi_module::wifi(peripherals.modem, sysloop.clone());


    let url = "https://www.google.com/";


    loop {
        // HTTP Configuration
        // Create HTTPS Connection Handle
        let mut connection = EspHttpConnection::new(&Configuration {
            use_global_ca_store: true,
            crt_bundle_attach: Some(esp_idf_svc::sys::esp_crt_bundle_attach),
            ..Default::default()
        }).unwrap();
        // Create HTTPS Client
        let mut client = Client::wrap(connection);
        let request = client.request(Method::Get, &url, &[]).unwrap();
        // let request = client.get(&url).unwrap();
        // Submit Request and Store Response

        let response = request.submit().unwrap();
        let status = response.status();

        log::info!("status {}", &status);


        match status {
            200..=299 => {
                let mut buf = [0_u8; 256];
                let mut offset = 0;
                let mut total = 0;
                let mut reader = response;
                loop {
                    if let Ok(size) = Read::read(&mut reader, &mut buf[offset..]) {
                        log::info!("Buf size {}", &size);
                        if size == 0 {
                            break;
                        }
                        total += size;
                        let size_plus_offset = size + offset;
                        match std::str::from_utf8(&buf[..size_plus_offset]) {
                            Ok(text) => {
                                log::info!("{}", text);
                                offset = 0;
                            }
                            Err(error) => {
                                let valid_up_to = error.valid_up_to();
                                unsafe {
                                    log::info!("{}", std::str::from_utf8_unchecked(&buf[..valid_up_to]));
                                }
                                buf.copy_within(valid_up_to.., 0);
                                offset = size_plus_offset - valid_up_to;
                            }
                        }
                    }
                }}
            _ => log::info!("Incorrect response")

        }
        sleep(Duration::new(2, 0));
    }
}
