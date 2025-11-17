#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Output, OutputConfig, Pull};
use esp_hal::main;
use log::info;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // LED on GPIO4
    let led_config = OutputConfig::default();
    let mut led = Output::new(peripherals.GPIO4, esp_hal::gpio::Level::Low, led_config);

    // Button for GPIO32 with pull-up
    let button_config = InputConfig::default().with_pull(Pull::Up);
    let button = Input::new(peripherals.GPIO32, button_config);

    info!("LED and button initialized");

    let mut last_state = button.is_high();

    loop {
        let current_state = button.is_high();

        if current_state != last_state {
            if current_state {
                led.set_high();
                info!("Button press: LED on");
            } else {
                led.set_low();
                info!("Button release: LED off");
            }
            last_state = current_state;
        }
    }
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples/src/bin
}
