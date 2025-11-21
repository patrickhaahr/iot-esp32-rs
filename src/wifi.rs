//! WiFi module for ESP32
//!
//! Provides WiFi initialization and connection management using esp-wifi.

use esp_hal::delay::Delay;
use esp_wifi::wifi::{AuthMethod, ClientConfiguration, Configuration, WifiController, WifiDevice};
use log::info;

/// WiFi credentials configuration
pub struct WifiCredentials<'a> {
    pub ssid: &'a str,
    pub password: &'a str,
}

/// Connect to WiFi
///
/// Returns the WifiController and the interface device.
pub fn connect<'a, 'd>(
    controller: &'a mut WifiController<'d>,
    _interfaces: &'a mut WifiDevice<'d>,
    credentials: &WifiCredentials,
) -> Result<(), &'static str> {
    info!("Setting up WiFi...");

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: credentials.ssid.try_into().unwrap(),
        password: credentials.password.try_into().unwrap(),
        auth_method: AuthMethod::WPA2Personal,
        ..Default::default()
    });

    controller
        .set_configuration(&client_config)
        .map_err(|_| "Failed to set config")?;

    controller.start().map_err(|_| "Failed to start WiFi")?;
    info!("WiFi started, connecting to '{}'...", credentials.ssid);

    controller.connect().map_err(|_| "Failed to connect")?;

    let delay = Delay::new();

    // Wait for connection
    loop {
        match controller.is_connected() {
            Ok(true) => {
                info!("WiFi connected successfully!");
                break;
            }
            Ok(false) => {
                info!("Waiting for WiFi connection...");
                delay.delay_millis(500);
            }
            Err(e) => {
                info!("WiFi error: {:?}", e);
                delay.delay_millis(1000);
            }
        }
    }

    Ok(())
}
