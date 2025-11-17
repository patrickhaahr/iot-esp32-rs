#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embedded_hal::delay::DelayNs;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Output, OutputConfig, Pull};
//use esp_hal::rtc_cntl::{Rtc, sleep::{TimerWakeup, WakeupLevel},};
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

    // RED LED on GPIO4 - start ON
    let led_config = OutputConfig::default();
    let mut red_led = Output::new(peripherals.GPIO4, esp_hal::gpio::Level::High, led_config);

    // BLUE LED on GPIO5 - start OFF
    let led_config = OutputConfig::default();
    let mut blue_led = Output::new(peripherals.GPIO5, esp_hal::gpio::Level::Low, led_config);

    // Button for GPIO32 with pull-up
    let button_config = InputConfig::default().with_pull(Pull::Up);
    let button = Input::new(peripherals.GPIO32, button_config);

    // Debouce delay
    let mut delay = Delay::new();

    // Deep sleep
    //let mut rtc = Rtc::new(peripherals.LPWR);

    info!("LEDs and button initialized - Red LED ON by default");

    let mut last_state = button.is_high();
    let mut red_led_active = true;

    loop {
        let current_state = button.is_high();

        if current_state != last_state {
            if !current_state {
                // Button pressed (going from high to low)
                if red_led_active {
                    red_led.set_low();
                    blue_led.set_high();
                    red_led_active = false;
                    info!("Button pressed: Switched to BLUE LED");
                } else {
                    red_led.set_high();
                    blue_led.set_low();
                    red_led_active = true;
                    info!("Button pressed: Switched to RED LED");
                }
                delay.delay_ms(50);
            }
            last_state = current_state;
        }
        delay.delay_ms(10); // Small delay to prevent busy waiting
    }
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples/src/bin
}
