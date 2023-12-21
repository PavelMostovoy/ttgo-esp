use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripheral;
use esp_idf_svc::{ipv4, ping};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{AccessPointConfiguration, BlockingWifi, ClientConfiguration, Configuration, EspWifi};


// Wifi setup temporary with predefined credentials
const SSID: &str = env!("WIFI_SSID");
const PASS: &str = env!("WIFI_PASS");


pub fn  wifi(
    modem: impl peripheral::Peripheral<P=esp_idf_svc::hal::modem::Modem> + 'static,
    sysloop: EspSystemEventLoop) -> Box<EspWifi<'static>> {
    let nvs = EspDefaultNvsPartition::take().unwrap();
    let mut esp_wifi = EspWifi::new(modem, sysloop.clone(), Some(nvs)).unwrap();

    let mut wifi = BlockingWifi::wrap(&mut esp_wifi, sysloop).unwrap();

    wifi.set_configuration(&Configuration::Client(ClientConfiguration::default())).unwrap();

    log::info!("Starting wifi...");

    wifi.start().unwrap();

    log::info!("Scanning...");

    let ap_infos = wifi.scan().unwrap();

    let ours = ap_infos.into_iter().find(|a| a.ssid == SSID);

    let channel = if let Some(ours) = ours {
        log::info!(
                "Found configured access point {} on channel {}",
                SSID, ours.channel
            );
        Some(ours.channel)
    } else {
        log::info!(
                "Configured access point {} not found during scanning, will go with unknown channel",
                SSID
            );
        None
    };

    wifi.set_configuration(&Configuration::Mixed(
        ClientConfiguration {
            ssid: SSID.into(),
            password: PASS.into(),
            channel,
            ..Default::default()
        },
        // access point configuration
        AccessPointConfiguration {
            ssid: "aptest".into(),
            channel: channel.unwrap_or(1),
            ..Default::default()
        }
    )).unwrap();

    log::info!("Connecting wifi...");

    wifi.connect().unwrap();

    log::info!("Waiting for DHCP lease...");

    wifi.wait_netif_up().unwrap();

    let ip_info = wifi.wifi().sta_netif().get_ip_info().unwrap();

    log::info!("Wifi DHCP info: {:?}", ip_info);

    // ping(ip_info.subnet.gateway);

    Box::new(esp_wifi)
}

fn ping(ip: ipv4::Ipv4Addr) {
    log::info!("About to do some pings for {:?}", ip);

    let ping_summary = ping::EspPing::default().ping(ip, &Default::default()).unwrap();
    if ping_summary.transmitted != ping_summary.received {
        log::debug!("Pinging IP {} resulted in timeouts", ip);
    }

    log::info!("Pinging done");
}