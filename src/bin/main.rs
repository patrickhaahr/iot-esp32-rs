#![no_std]
#![no_main]

extern crate alloc;

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Output, OutputConfig, Pull, RtcPin};
use esp_hal::main;
use esp_hal::rng::Rng;
use esp_hal::rtc_cntl::{Rtc, SocResetReason, reset_reason, wakeup_cause};
use esp_hal::system::Cpu;
use esp_hal::timer::timg::TimerGroup;
use log::{info, LevelFilter};

use esp_wifi::init;

use my_esp_project::button::ButtonState;
use my_esp_project::deep_sleep::{
    self, SLEEP_DELAY_MS, clear_ext1_wakeup_status, decode_wake_gpios, read_ext1_wakeup_status,
};
use my_esp_project::led::{LED_DISPLAY_MS, LedActivityState};
use my_esp_project::mqtt;
use my_esp_project::ntp;
use my_esp_project::wifi::{self, WifiCredentials};

/// Polling interval (milliseconds)
const POLL_INTERVAL_MS: u32 = 10;

/// Button names for logging
const BUTTON_NAMES: [&str; 4] = [
    "Button 1 (Very Happy/GREEN)",
    "Button 2 (Happy/YELLOW)",
    "Button 3 (Neutral/BLUE)",
    "Button 4 (Sad/RED)",
];

esp_bootloader_esp_idf::esp_app_desc!();

// WiFi credentials loaded from .env file at compile time
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

/// Turn off LED by index
fn turn_off_led(
    idx: usize,
    green: &mut Output,
    yellow: &mut Output,
    blue: &mut Output,
    red: &mut Output,
) {
    match idx {
        0 => green.set_low(),
        1 => yellow.set_low(),
        2 => blue.set_low(),
        3 => red.set_low(),
        _ => {}
    }
}

/// Turn on LED by index
fn turn_on_led(
    idx: usize,
    green: &mut Output,
    yellow: &mut Output,
    blue: &mut Output,
    red: &mut Output,
) {
    match idx {
        0 => green.set_high(),
        1 => yellow.set_high(),
        2 => blue.set_high(),
        3 => red.set_high(),
        _ => {}
    }
}

/// Run LED startup test sequence
fn run_led_test(
    green: &mut Output,
    yellow: &mut Output,
    blue: &mut Output,
    red: &mut Output,
    delay: &Delay,
) {
    info!("Power-on reset detected - running LED startup test...");
    green.set_high();
    delay.delay_millis(200);
    green.set_low();
    yellow.set_high();
    delay.delay_millis(200);
    yellow.set_low();
    blue.set_high();
    delay.delay_millis(200);
    blue.set_low();
    red.set_high();
    delay.delay_millis(200);
    red.set_low();
    info!("LED test complete! Ready for buttons.");
}

#[main]
fn main() -> ! {
    // CRITICAL: Read EXT1 wakeup status IMMEDIATELY before ANY initialization
    let ext1_status_on_wake = read_ext1_wakeup_status();

    // Initialize heap allocator (required for WiFi)
    esp_alloc::heap_allocator!(size: 120 * 1024);

    esp_println::logger::init_logger_from_env();
    log::set_max_level(LevelFilter::Info);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Initialize RTC for deep sleep control
    let mut rtc = Rtc::new(peripherals.LPWR);

    // Check reset and wake reason
    let reset_rsn = reset_reason(Cpu::ProCpu);
    let wake_rsn = wakeup_cause();

    info!("=== ESP32 Smiley Feedback Panel with WiFi ===");
    info!("Reset reason: {:?}", reset_rsn);
    info!("Wake reason: {:?}", wake_rsn);

    // Initialize timer for WiFi
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let rng = Rng::new(peripherals.RNG);

    // Initialize ESP-WiFi
    let init = init(timg0.timer0, rng).unwrap();

    let (mut wifi_controller, interfaces) = esp_wifi::wifi::new(&init, peripherals.WIFI).unwrap();
    let mut wifi_device = interfaces.sta;

    // Connect to WiFi
    let credentials = WifiCredentials {
        ssid: WIFI_SSID,
        password: WIFI_PASSWORD,
    };

    wifi::connect(&mut wifi_controller, &mut wifi_device, &credentials)
        .expect("Failed to connect to WiFi");

    info!("WiFi connection established!");

    // Create smoltcp interface for networking
    use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
    use smoltcp::time::Instant;
    use smoltcp::wire::EthernetAddress;

    let mac_addr = wifi_device.mac_address();
    let hardware_addr = EthernetAddress(mac_addr);
    let config = Config::new(hardware_addr.into());
    let mut iface = Interface::new(config, &mut wifi_device, Instant::from_millis(0));

    let mut socket_set_entries: [SocketStorage; 4] = Default::default();
    let mut sockets = SocketSet::new(&mut socket_set_entries[..]);

    // Setup network interface with DHCP (do this ONCE)
    info!("Setting up network interface with DHCP...");
    match ntp::setup_network_interface(&mut iface, &mut wifi_device, &mut sockets) {
        Ok(ip) => info!("Network configured with IP: {}", ip),
        Err(e) => {
            info!("DHCP failed: {}", e);
            // Continue anyway - WiFi is connected
        }
    }

    // Initialize TLS
    // We create one instance of Tls
    // Initialize SHA and RSA peripherals if needed by Tls
    // Note: Add 'mut' keyword when uncommenting set_debug() below
    let tls = esp_mbedtls::Tls::new().unwrap();
    // let mut tls = esp_mbedtls::Tls::new().unwrap();
    // tls.set_debug(4);

    // Attempt NTP time synchronization
    info!("Attempting NTP time synchronization...");
    let current_time = match ntp::sync_time_with_device(&mut iface, &mut wifi_device, &mut sockets)
    {
        Ok(time) => {
            let (h, m, s) = time.to_copenhagen_hms(false); // Use CET (winter time)
            info!(
                "NTP sync successful! Copenhagen time: {:02}:{:02}:{:02}",
                h, m, s
            );
            Some(time)
        }
        Err(e) => {
            info!("NTP sync failed: {}", e);
            info!(
                "NTP server: {} ({})",
                ntp::DENMARK_NTP_SERVER,
                ntp::DENMARK_NTP_SERVER_IP
            );
            None
        }
    };

    // Configure LEDs (all start OFF)
    let led_config = OutputConfig::default();

    // LED Mappings
    let mut green_led = Output::new(peripherals.GPIO18, esp_hal::gpio::Level::Low, led_config); // Very Happy
    let mut yellow_led = Output::new(peripherals.GPIO5, esp_hal::gpio::Level::Low, led_config); // Happy
    let mut blue_led = Output::new(peripherals.GPIO4, esp_hal::gpio::Level::Low, led_config); // Neutral
    let mut red_led = Output::new(peripherals.GPIO15, esp_hal::gpio::Level::Low, led_config); // Sad

    // Configure Buttons (all active HIGH with internal pull-downs)
    // NOTE: GPIO 25, 26, 27, 32 are RTC-capable pins
    let input_config = InputConfig::default().with_pull(Pull::Down);

    // Store raw GPIO pins for EXT1 wakeup (need to reborrow later)
    let mut pin25 = peripherals.GPIO25;
    let mut pin26 = peripherals.GPIO26;
    let mut pin27 = peripherals.GPIO27;
    let mut pin32 = peripherals.GPIO32;

    // Initialize Input drivers
    let button1 = Input::new(pin32.reborrow(), input_config); // Very Happy
    let button2 = Input::new(pin27.reborrow(), input_config); // Happy
    let button3 = Input::new(pin26.reborrow(), input_config); // Neutral
    let button4 = Input::new(pin25.reborrow(), input_config); // Sad

    let delay = Delay::new();

    // Check if we woke from deep sleep
    let woke_from_deep_sleep = matches!(reset_rsn, Some(SocResetReason::CoreDeepSleep));

    if woke_from_deep_sleep {
        info!("Woke from deep sleep!");

        // Decode which GPIO woke us from the status register bits
        let wake_bits = decode_wake_gpios(ext1_status_on_wake);

        info!(
            "EXT1 GPIO status: GPIO25={}, GPIO26={}, GPIO27={}, GPIO32={}",
            wake_bits.gpio25, wake_bits.gpio26, wake_bits.gpio27, wake_bits.gpio32
        );

        // Clear EXT1 wakeup status IMMEDIATELY after wake to prevent re-triggering
        clear_ext1_wakeup_status(ext1_status_on_wake);

        // Determine which button triggered the wake
        let wake_button = if wake_bits.gpio32 != 0 {
            Some((0, &mut green_led, "Button 1 (Very Happy/GREEN)"))
        } else if wake_bits.gpio27 != 0 {
            Some((1, &mut yellow_led, "Button 2 (Happy/YELLOW)"))
        } else if wake_bits.gpio26 != 0 {
            Some((2, &mut blue_led, "Button 3 (Neutral/BLUE)"))
        } else if wake_bits.gpio25 != 0 {
            Some((3, &mut red_led, "Button 4 (Sad/RED)"))
        } else if button1.is_high() {
            // Fallback to checking live state if register was empty
            Some((0, &mut green_led, "Button 1 (Very Happy/GREEN)"))
        } else if button2.is_high() {
            // Fallback to checking live state if register was empty
            Some((1, &mut yellow_led, "Button 2 (Happy/YELLOW)"))
        } else if button3.is_high() {
            // Fallback to checking live state if register was empty
            Some((2, &mut blue_led, "Button 3 (Neutral/BLUE)"))
        } else if button4.is_high() {
            // Fallback to checking live state if register was empty
            Some((3, &mut red_led, "Button 4 (Sad/RED)"))
        } else {
            info!("Wake from timer or unknown source");
            None
        };

        if let Some((button_idx, led, button_name)) = wake_button {
            // Turn on LED immediately
            led.set_high();
            info!("{} PRESSED", button_name);

            // Publish MQTT feedback if we have NTP time
            if let Some(time) = current_time {
                let button_number = (button_idx + 1) as u8; // Convert 0-3 to 1-4
                info!(
                    "MQTT: Publishing feedback for button {} at timestamp {}",
                    button_number, time.unix_timestamp
                );

                match mqtt::publish_button_feedback(
                    &mut iface,
                    &mut wifi_device,
                    &mut sockets,
                    button_number,
                    time.unix_timestamp,
                    "esp32-smiley-001",
                    tls.reference(),
                ) {
                    Ok(_) => info!("MQTT: Feedback published successfully"),
                    Err(e) => info!("MQTT: Failed to publish feedback: {}", e),
                }
            } else {
                info!("MQTT: Skipping publish (no NTP time available)");
            }

            // Display LED for 500ms
            delay.delay_millis(LED_DISPLAY_MS);

            // Turn off LED
            led.set_low();
            info!("LED display complete");
        }

        // Fall through to main loop for 10-second activity window
    } else {
        // Power-on reset - run LED startup test
        run_led_test(
            &mut green_led,
            &mut yellow_led,
            &mut blue_led,
            &mut red_led,
            &delay,
        );
    }

    info!("Buttons (RTC pins): GPIO25, GPIO26, GPIO27, GPIO32");
    info!("Active HIGH Logic (Press = High)");

    // Initialize state trackers
    let mut button_states = [
        ButtonState::new(),
        ButtonState::new(),
        ButtonState::new(),
        ButtonState::new(),
    ];
    let mut led_activity = LedActivityState::new();
    // Mark activity at startup so device will sleep after 10 seconds if no buttons pressed
    led_activity.mark_activity(0);
    let mut elapsed_ms: u32 = 0;

    // Main loop
    loop {
        // Read all button states (Active High)
        let button_readings = [
            button1.is_high(),
            button2.is_high(),
            button3.is_high(),
            button4.is_high(),
        ];

        // Process all button inputs (no lockout - user can spam buttons)
        for (idx, &is_high) in button_readings.iter().enumerate() {
            if let Some(true) = button_states[idx].update(is_high, elapsed_ms) {
                info!("{} PRESSED", BUTTON_NAMES[idx]);

                // Publish MQTT feedback if we have NTP time
                if let Some(time) = current_time {
                    let button_number = (idx + 1) as u8; // Convert 0-3 to 1-4
                    info!(
                        "MQTT: Publishing feedback for button {} at timestamp {}",
                        button_number, time.unix_timestamp
                    );

                    // We need to re-borrow components from the main scope for each call
                    // Since we are in a loop, we can't just move them.
                    // But the publish_button_feedback function takes mutable references, which is fine
                    // as long as we don't hold onto them across iterations in a way the borrow checker dislikes.
                    // The issue previously was reusing `iface` etc which were borrowed mutably in previous iteration?
                    // No, `publish_button_feedback` finishes, so borrows should end.
                    //
                    // The previous error was:
                    // `iface` was mutably borrowed here in the previous iteration of the loop
                    //
                    // This happens if the lifetime 'a of the mutable borrow is inferred to be longer than the loop body.
                    //
                    // publish_button_feedback<'a, 'd>(..., sockets: &'a mut SocketSet<'a>, ...)
                    // The signature requires `sockets` to live for 'a, AND it borrows `sockets` for 'a.
                    // This effectively locks `sockets` for its entire lifetime.
                    // We need to fix the signature in mqtt.rs to allow re-borrowing.

                    match mqtt::publish_button_feedback(
                        &mut iface,
                        &mut wifi_device,
                        &mut sockets,
                        button_number,
                        time.unix_timestamp,
                        "esp32-smiley-001",
                        tls.reference(),
                    ) {
                        Ok(_) => info!("MQTT: Feedback published successfully"),
                        Err(e) => info!("MQTT: Failed to publish feedback: {}", e),
                    }
                } else {
                    info!("MQTT: Skipping publish (no NTP time available)");
                }

                // Turn off previous LED if different button pressed
                if led_activity.is_led_active() && led_activity.get_led() != idx {
                    turn_off_led(
                        led_activity.get_led(),
                        &mut green_led,
                        &mut yellow_led,
                        &mut blue_led,
                        &mut red_led,
                    );
                }

                // Turn on new LED
                turn_on_led(
                    idx,
                    &mut green_led,
                    &mut yellow_led,
                    &mut blue_led,
                    &mut red_led,
                );

                // Activate/reset LED timer
                led_activity.activate(idx, elapsed_ms);
            }
        }

        // Check if LED display period has expired (turn off LED after 500ms)
        if led_activity.check_led_expired(elapsed_ms) {
            let led_idx = led_activity.get_led();
            info!("LED display complete - turning off LED");
            turn_off_led(
                led_idx,
                &mut green_led,
                &mut yellow_led,
                &mut blue_led,
                &mut red_led,
            );
            led_activity.deactivate_led();
        }

        // Check if sleep delay has expired (wait 10 seconds after last press before sleep)
        if led_activity.check_sleep_expired(elapsed_ms, SLEEP_DELAY_MS) {
            // Drop Inputs before configuring sleep to release GPIO pins
            #[allow(clippy::drop_non_drop)]
            {
                core::mem::drop(button1);
                core::mem::drop(button2);
                core::mem::drop(button3);
                core::mem::drop(button4);
            }

            let mut wakeup_pins: [&mut dyn RtcPin; 4] =
                [&mut pin25, &mut pin26, &mut pin27, &mut pin32];

            deep_sleep::enter_deep_sleep(&mut rtc, &mut wakeup_pins, 300, &delay);
        }

        delay.delay_millis(POLL_INTERVAL_MS);
        elapsed_ms = elapsed_ms.wrapping_add(POLL_INTERVAL_MS);
    }
}
