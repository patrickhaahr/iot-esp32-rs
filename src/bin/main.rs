#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Output, OutputConfig, Pull};
use esp_hal::main;
use log::info;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    info!("=== ESP32 4-Button 4-LED Hardware Test ===");
    info!("Starting hardware test...");

    // Configure LEDs (all start OFF)
    let led_config = OutputConfig::default();

    // LED1: GREEN (Very Happy) - matches LED1_GPIO config
    let mut green_led = Output::new(peripherals.GPIO15, esp_hal::gpio::Level::Low, led_config);

    // LED2: YELLOW (Happy) - matches LED2_GPIO config
    let mut yellow_led = Output::new(peripherals.GPIO4, esp_hal::gpio::Level::Low, led_config);

    // LED3: BLUE (Neutral) - matches LED3_GPIO config
    let mut blue_led = Output::new(peripherals.GPIO5, esp_hal::gpio::Level::Low, led_config);

    // LED4: RED (Sad) - matches LED4_GPIO config
    let mut red_led = Output::new(peripherals.GPIO18, esp_hal::gpio::Level::Low, led_config);

    // Configure Buttons (all active LOW with internal pull-ups)
    let input_config = InputConfig::default().with_pull(Pull::Up);

    // BTN1: Very Happy - matches BTN1_GPIO config
    let button1 = Input::new(peripherals.GPIO19, input_config);

    // BTN2: Happy - matches BTN2_GPIO config
    let button2 = Input::new(peripherals.GPIO21, input_config);

    // BTN3: Neutral - matches BTN3_GPIO config
    let button3 = Input::new(peripherals.GPIO22, input_config);

    // BTN4: Sad - matches BTN4_GPIO config
    let button4 = Input::new(peripherals.GPIO23, input_config);

    let delay = Delay::new();

    // Startup LED test - flash all LEDs
    info!("Running LED startup test...");
    green_led.set_high();
    delay.delay_millis(200);
    green_led.set_low();
    yellow_led.set_high();
    delay.delay_millis(200);
    yellow_led.set_low();
    blue_led.set_high();
    delay.delay_millis(200);
    blue_led.set_low();
    red_led.set_high();
    delay.delay_millis(200);
    red_led.set_low();
    info!("LED test complete!");

    info!("Ready for button presses. Press any button to test.");
    info!("Buttons are active LOW (pressed = LOW, released = HIGH)");

    // Main loop - simple button to LED mapping
    loop {
        // Button 1 -> GREEN LED
        if button1.is_low() {
            green_led.set_high();
            info!("Button 1 (Very Happy) PRESSED - GREEN LED ON");
        } else {
            green_led.set_low();
        }

        // Button 2 -> YELLOW LED
        if button2.is_low() {
            yellow_led.set_high();
            info!("Button 2 (Happy) PRESSED - YELLOW LED ON");
        } else {
            yellow_led.set_low();
        }

        // Button 3 -> BLUE LED
        if button3.is_low() {
            blue_led.set_high();
            info!("Button 3 (Neutral) PRESSED - BLUE LED ON");
        } else {
            blue_led.set_low();
        }

        // Button 4 -> RED LED
        if button4.is_low() {
            red_led.set_high();
            info!("Button 4 (Sad) PRESSED - RED LED ON");
        } else {
            red_led.set_low();
        }

        // Small delay to prevent excessive serial output
        delay.delay_millis(100);
    }
}
