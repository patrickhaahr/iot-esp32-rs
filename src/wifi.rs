//! WiFi module for ESP32
//!
//! Provides WiFi initialization and connection management.

use esp_hal::delay::Delay;
use esp_hal::peripherals::WIFI;
use esp_radio::wifi::{self, AuthMethod, ClientConfig, Config, ModeConfig, WifiController};
use esp_radio::Controller;
use log::info;

/// WiFi credentials configuration
pub struct WifiCredentials<'a> {
    pub ssid: &'a str,
    pub password: &'a str,
}

/// Initialize and connect to WiFi
///
/// Returns the WifiController on success, allowing the caller to manage the connection.
pub fn connect<'d>(
    radio_controller: &'d Controller<'d>,
    wifi_peripheral: WIFI<'d>,
    credentials: &WifiCredentials,
) -> Result<WifiController<'d>, wifi::WifiError> {
    info!("Setting up WiFi...");

    // Create WiFi controller
    let (mut wifi_controller, _interfaces) = wifi::new(
        radio_controller,
        wifi_peripheral,
        Config::default(),
    )?;

    // Configure station mode with credentials
    let client_config = ClientConfig::default()
        .with_ssid(credentials.ssid.into())
        .with_password(credentials.password.into())
        .with_auth_method(AuthMethod::Wpa2Personal);

    wifi_controller.set_config(&ModeConfig::Client(client_config))?;

    // Start WiFi
    wifi_controller.start()?;
    info!("WiFi started, connecting to '{}'...", credentials.ssid);

    // Initiate connection
    wifi_controller.connect()?;

    let delay = Delay::new();

    // Wait for connection
    loop {
        match wifi_controller.is_connected() {
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

    Ok(wifi_controller)
}
