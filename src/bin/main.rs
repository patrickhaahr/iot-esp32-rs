#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::time::Duration;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Output, OutputConfig};
use esp_hal::main;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::rtc_cntl::sleep::{Ext0WakeupSource, TimerWakeupSource, WakeupLevel};
use esp_hal::rtc_cntl::{reset_reason, wakeup_cause};
use esp_hal::system::SleepSource;
use log::info;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // RED LED on GPIO4 - start OFF
    let led_config = OutputConfig::default();
    let mut red_led = Output::new(peripherals.GPIO4, esp_hal::gpio::Level::Low, led_config);

    // BLUE LED on GPIO5 - start OFF
    let led_config = OutputConfig::default();
    let mut blue_led = Output::new(peripherals.GPIO5, esp_hal::gpio::Level::Low, led_config);

    // Button on GPIO32 with pull-up (active low)
    let button = peripherals.GPIO32;

    // Debounce delay
    let delay = Delay::new();

    // Deep sleep
    let mut rtc = Rtc::new(peripherals.LPWR);

    let reason = reset_reason(esp_hal::system::Cpu::ProCpu);
    let wake_reason = wakeup_cause();

    info!("Reset reason: {:?}", reason);
    info!("Wakeup reason: {:?}", wake_reason);

    // Handle wakeup reason - turn on appropriate LED
    match wake_reason {
        SleepSource::Timer => {
            blue_led.set_high();
            red_led.set_low();
            info!("Timer wakeup - BLUE LED ON");
            // Keep LED on for 10 seconds before going back to sleep
            delay.delay_millis(10000);
        }
        SleepSource::Ext0 => {
            blue_led.set_low();
            red_led.set_high();
            info!("EXT0 button wakeup - RED LED ON");
            // Keep LED on for 10 seconds before going back to sleep
            delay.delay_millis(10000);
        }
        _ => {
            // First boot or other wakeup source
            info!("Initial boot or unknown wakeup source");
            // Small delay before first sleep
            delay.delay_millis(1000);
        }
    }

    // Create wakeup sources
    // Timer wakeup after 10 seconds
    let timer = TimerWakeupSource::new(Duration::from_secs(10));

    // Button wakeup on GPIO32
    // Note: Using WakeupLevel::High - will wake when pin goes HIGH
    // If you have a pull-down resistor and button connects to VCC, this wakes on press
    // If you have a pull-up resistor and button connects to GND, this wakes on release
    let ext0 = Ext0WakeupSource::new(button, WakeupLevel::High);

    // Enter deep sleep with both wakeup sources
    info!("Entering deep sleep...");
    rtc.sleep_deep(&[&timer, &ext0])

    /*
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
    */
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples/src/bin
}
