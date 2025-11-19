#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;

use core::time::Duration;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, InputConfig, Output, OutputConfig, Pull, RtcPin};
use esp_hal::main;
use esp_hal::rtc_cntl::sleep::{Ext1WakeupSource, TimerWakeupSource, WakeupLevel};
use esp_hal::rtc_cntl::{reset_reason, wakeup_cause, Rtc, SocResetReason};
use esp_hal::system::Cpu;
use esp_hal::timer::timg::TimerGroup;
use log::info;
use my_esp_project::ntp;
use my_esp_project::wifi::{self, WifiCredentials};

/// Debounce timing constant (milliseconds)
const DEBOUNCE_MS: u32 = 50;

/// LED display duration after button press (milliseconds)
const LED_DISPLAY_MS: u32 = 500;

/// Minimum time to wait before entering deep sleep (milliseconds)
const SLEEP_DELAY_MS: u32 = 10000;

/// Polling interval (milliseconds)
const POLL_INTERVAL_MS: u32 = 10;

/// Button debounce state tracker
#[derive(Clone, Copy)]
struct ButtonState {
    /// Last stable state (true = HIGH/pressed, false = LOW/released)
    /// Note: With Active HIGH logic: HIGH = Pressed, LOW = Released
    last_stable: bool,
    /// Timestamp (in ms) when current raw state first appeared
    state_start_ms: u32,
    /// Current raw state being observed
    current_raw: bool,
}

impl ButtonState {
    fn new() -> Self {
        Self {
            last_stable: false, // Buttons start LOW (pulled down, not pressed)
            state_start_ms: 0,
            current_raw: false,
        }
    }

    /// Update button state with new reading. Returns Some(pressed) if a debounced event occurred.
    /// pressed=true means button was pressed (LOW->HIGH), pressed=false means button was released
    fn update(&mut self, is_high: bool, current_ms: u32) -> Option<bool> {
        let raw_state = is_high; 

        if raw_state != self.current_raw {
            // State changed - start tracking new state
            self.current_raw = raw_state;
            self.state_start_ms = current_ms;
            None
        } else if raw_state != self.last_stable {
            // State is stable and different from last confirmed state
            let elapsed = current_ms.wrapping_sub(self.state_start_ms);
            if elapsed >= DEBOUNCE_MS {
                // Debounce period passed - confirm state change
                self.last_stable = raw_state;
                // Return event: true if button was pressed (LOW->HIGH), false if released
                Some(raw_state)
            } else {
                None
            }
        } else {
            None
        }
    }
}

/// LED activity state tracker
struct LedActivityState {
    /// Is any LED currently on?
    led_active: bool,
    /// Which LED is on (0-3)
    led_index: usize,
    /// When the LED was last activated (ms timestamp)
    last_press_ms: u32,
    /// Has any button been pressed (tracks sleep delay)
    had_activity: bool,
}

impl LedActivityState {
    fn new() -> Self {
        Self {
            led_active: false,
            led_index: 0,
            last_press_ms: 0,
            had_activity: false,
        }
    }

    /// Activate LED for given button (resets timer if already active)
    fn activate(&mut self, led_index: usize, current_ms: u32) {
        self.led_active = true;
        self.led_index = led_index;
        self.last_press_ms = current_ms;
        self.had_activity = true;
    }

    /// Check if LED display period has expired. Returns true if LED should turn off.
    fn check_led_expired(&self, current_ms: u32) -> bool {
        if self.led_active {
            let elapsed = current_ms.wrapping_sub(self.last_press_ms);
            return elapsed >= LED_DISPLAY_MS;
        }
        false
    }

    /// Check if sleep delay has expired. Returns true if device should sleep.
    fn check_sleep_expired(&self, current_ms: u32) -> bool {
        if self.had_activity && !self.led_active {
            let elapsed = current_ms.wrapping_sub(self.last_press_ms);
            return elapsed >= SLEEP_DELAY_MS;
        }
        false
    }

    /// Turn off LED but keep tracking for sleep delay
    fn deactivate_led(&mut self) {
        self.led_active = false;
    }

    /// Is any LED currently active?
    fn is_led_active(&self) -> bool {
        self.led_active
    }

    /// Get active LED index
    fn get_led(&self) -> usize {
        self.led_index
    }

    /// Mark activity for sleep tracking (used at startup)
    fn mark_activity(&mut self, current_ms: u32) {
        self.had_activity = true;
        self.last_press_ms = current_ms;
    }
}

esp_bootloader_esp_idf::esp_app_desc!();

// WiFi credentials loaded from .env file at compile time
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

#[main]
fn main() -> ! {
    // CRITICAL: Read EXT1 wakeup status IMMEDIATELY before ANY initialization
    // ESP32 TRM Section 30.3.5: RTC_CNTL_EXT_WAKEUP1_STATUS register (0x3FF4_80D0)
    let ext1_status_on_wake = unsafe {
        const RTC_CNTL_EXT_WAKEUP1_STATUS_REG: *const u32 = 0x3ff4_80D0 as *const u32;
        core::ptr::read_volatile(RTC_CNTL_EXT_WAKEUP1_STATUS_REG)
    };

    // Initialize heap allocator (required for WiFi)
    esp_alloc::heap_allocator!(size: 72 * 1024);

    esp_println::logger::init_logger_from_env();

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

    // Initialize timer for RTOS scheduler
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    // Start the RTOS scheduler (required for WiFi)
    esp_rtos::start(timg0.timer0);

    // Initialize esp-radio
    let radio_controller = esp_radio::init().unwrap();

    // Connect to WiFi
    let credentials = WifiCredentials {
        ssid: WIFI_SSID,
        password: WIFI_PASSWORD,
    };

    let (_wifi_controller, mut interfaces) = wifi::connect(&radio_controller, peripherals.WIFI, &credentials)
        .expect("Failed to connect to WiFi");

    info!("WiFi connection established!");

    // Attempt NTP time synchronization
    info!("Attempting NTP time synchronization...");
    match ntp::sync_time_with_device(&mut interfaces.sta) {
        Ok(time) => {
            let (h, m, s) = time.to_copenhagen_hms(false); // Use CET (winter time)
            info!(
                "NTP sync successful! Copenhagen time: {:02}:{:02}:{:02}",
                h, m, s
            );
        }
        Err(e) => {
            info!("NTP sync failed: {}", e);
            info!("NTP server: {} ({})", ntp::DENMARK_NTP_SERVER, ntp::DENMARK_NTP_SERVER_IP);
        }
    }

    // Configure LEDs (all start OFF)
    let led_config = OutputConfig::default();

    // LED Mappings
    let mut green_led = Output::new(peripherals.GPIO18, esp_hal::gpio::Level::Low, led_config);  // Very Happy
    let mut yellow_led = Output::new(peripherals.GPIO5, esp_hal::gpio::Level::Low, led_config); // Happy
    let mut blue_led = Output::new(peripherals.GPIO4, esp_hal::gpio::Level::Low, led_config);   // Neutral
    let mut red_led = Output::new(peripherals.GPIO15, esp_hal::gpio::Level::Low, led_config);   // Sad

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
        // - GPIO25 = RTC_GPIO 6  (Bit 6)
        // - GPIO26 = RTC_GPIO 7  (Bit 7)
        // - GPIO27 = RTC_GPIO 17 (Bit 17)
        // - GPIO32 = RTC_GPIO 9  (Bit 9)
        let gpio25_bit = (ext1_status_on_wake >> 6) & 1;
        let gpio26_bit = (ext1_status_on_wake >> 7) & 1;
        let gpio27_bit = (ext1_status_on_wake >> 17) & 1;
        let gpio32_bit = (ext1_status_on_wake >> 9) & 1;

        info!("EXT1 GPIO status: GPIO25={}, GPIO26={}, GPIO27={}, GPIO32={}",
              gpio25_bit, gpio26_bit, gpio27_bit, gpio32_bit);

        // Clear EXT1 wakeup status IMMEDIATELY after wake to prevent re-triggering
        unsafe {
            const RTC_CNTL_EXT_WAKEUP1_STATUS_REG: *mut u32 = 0x3ff4_80D0 as *mut u32;
            const STATUS_CLR_BIT: u32 = 1 << 18;
            let new_value = ext1_status_on_wake | STATUS_CLR_BIT;
            core::ptr::write_volatile(RTC_CNTL_EXT_WAKEUP1_STATUS_REG, new_value);
        }

        // Determine which button triggered the wake
        let wake_button = if gpio32_bit != 0 {
            Some((0, &mut green_led, "Button 1 (Very Happy/GREEN)"))
        } else if gpio27_bit != 0 {
            Some((1, &mut yellow_led, "Button 2 (Happy/YELLOW)"))
        } else if gpio26_bit != 0 {
            Some((2, &mut blue_led, "Button 3 (Neutral/BLUE)"))
        } else if gpio25_bit != 0 {
            Some((3, &mut red_led, "Button 4 (Sad/RED)"))
        } else if button1.is_high() {
            // Fallback to checking live state if register was empty
            Some((0, &mut green_led, "Button 1 (Very Happy/GREEN)"))
        } else if button2.is_high() {
            Some((1, &mut yellow_led, "Button 2 (Happy/YELLOW)"))
        } else if button3.is_high() {
            Some((2, &mut blue_led, "Button 3 (Neutral/BLUE)"))
        } else if button4.is_high() {
            Some((3, &mut red_led, "Button 4 (Sad/RED)"))
        } else {
            info!("Wake from timer or unknown source");
            None
        };

        if let Some((_button_idx, led, button_name)) = wake_button {
            // Turn on LED immediately
            led.set_high();
            info!("{} PRESSED", button_name);

            // Display LED for 500ms
            delay.delay_millis(LED_DISPLAY_MS);

            // Turn off LED
            led.set_low();
            info!("LED display complete");
        }

        // Fall through to main loop for 10-second activity window
    } else {
        // Power-on reset - run LED startup test
        info!("Power-on reset detected - running LED startup test...");
        green_led.set_high(); delay.delay_millis(200); green_led.set_low();
        yellow_led.set_high(); delay.delay_millis(200); yellow_led.set_low();
        blue_led.set_high(); delay.delay_millis(200); blue_led.set_low();
        red_led.set_high(); delay.delay_millis(200); red_led.set_low();
        info!("LED test complete! Ready for buttons.");
    }

    info!("Buttons (RTC pins): GPIO25, GPIO26, GPIO27, GPIO32");
    info!("Active HIGH Logic (Press = High)");

    // Initialize state trackers
    let mut button_states = [
        ButtonState::new(), ButtonState::new(), ButtonState::new(), ButtonState::new()
    ];
    let mut led_activity = LedActivityState::new();
    // Mark activity at startup so device will sleep after 10 seconds if no buttons pressed
    led_activity.mark_activity(0);
    let mut elapsed_ms: u32 = 0;

    const BUTTON_NAMES: [&str; 4] = [
        "Button 1 (Very Happy/GREEN)", "Button 2 (Happy/YELLOW)", 
        "Button 3 (Neutral/BLUE)", "Button 4 (Sad/RED)"
    ];

    // Main loop
    loop {
        // Read all button states (Active High)
        let button_readings = [
            button1.is_high(), button2.is_high(), button3.is_high(), button4.is_high()
        ];

        // Process all button inputs (no lockout - user can spam buttons)
        for (idx, &is_high) in button_readings.iter().enumerate() {
            if let Some(pressed) = button_states[idx].update(is_high, elapsed_ms) {
                if pressed {
                    info!("{} PRESSED", BUTTON_NAMES[idx]);
                    
                    // Turn off previous LED if different button pressed
                    if led_activity.is_led_active() && led_activity.get_led() != idx {
                        match led_activity.get_led() {
                            0 => green_led.set_low(),
                            1 => yellow_led.set_low(),
                            2 => blue_led.set_low(),
                            3 => red_led.set_low(),
                            _ => {}
                        }
                    }
                    
                    // Turn on new LED
                    match idx {
                        0 => green_led.set_high(),
                        1 => yellow_led.set_high(),
                        2 => blue_led.set_high(),
                        3 => red_led.set_high(),
                        _ => {}
                    }

                    // Activate/reset LED timer
                    led_activity.activate(idx, elapsed_ms);
                }
            }
        }

        // Check if LED display period has expired (turn off LED after 500ms)
        if led_activity.check_led_expired(elapsed_ms) {
            let led_idx = led_activity.get_led();
            info!("LED display complete - turning off LED");
            match led_idx {
                0 => green_led.set_low(),
                1 => yellow_led.set_low(),
                2 => blue_led.set_low(),
                3 => red_led.set_low(),
                _ => {}
            }
            led_activity.deactivate_led();
        }

        // Check if sleep delay has expired (wait 10 seconds after last press before sleep)
        if led_activity.check_sleep_expired(elapsed_ms) {
            // Prepare for deep sleep
            info!("Entering deep sleep...");
            delay.delay_millis(100);

            // Drop Inputs & Configure Sleep
            core::mem::drop(button1); core::mem::drop(button2);
            core::mem::drop(button3); core::mem::drop(button4);

            let mut wakeup_pins: [&mut dyn RtcPin; 4] = [
                &mut pin25, &mut pin26, &mut pin27, &mut pin32,
            ];
            let ext1 = Ext1WakeupSource::new(&mut wakeup_pins, WakeupLevel::High);

            // Enable RTC Pull-Downs
            {
                let rtc_io = esp_hal::peripherals::RTC_IO::regs();
                rtc_io.touch_pad6().modify(|_, w| w.rue().clear_bit().rde().set_bit());
                rtc_io.touch_pad7().modify(|_, w| w.rue().clear_bit().rde().set_bit());
                rtc_io.xtal_32k_pad().modify(|_, w| {
                    w.x32p_rue().clear_bit().x32p_rde().set_bit().x32p_fun_ie().set_bit()
                });
            }

            let timer = TimerWakeupSource::new(Duration::from_secs(300));
            rtc.sleep_deep(&[&ext1, &timer]);
        }

        delay.delay_millis(POLL_INTERVAL_MS);
        elapsed_ms = elapsed_ms.wrapping_add(POLL_INTERVAL_MS);
    }
}
