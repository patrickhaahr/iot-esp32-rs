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
use esp_hal::gpio::{Input, InputConfig, Output, OutputConfig, Pull, RtcPin};
use esp_hal::main;
use esp_hal::rtc_cntl::sleep::{Ext1WakeupSource, TimerWakeupSource, WakeupLevel};
use esp_hal::rtc_cntl::{reset_reason, wakeup_cause, Rtc, SocResetReason};
use esp_hal::system::Cpu;
use log::info;

/// Debounce timing constant (milliseconds)
const DEBOUNCE_MS: u32 = 50;

/// Lockout period after button press (milliseconds)
const LOCKOUT_MS: u32 = 7000;

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

/// Lockout state tracker
struct LockoutState {
    /// Is lockout currently active?
    active: bool,
    /// Which button triggered lockout (0-3)
    button_index: usize,
    /// When lockout ends (ms timestamp)
    end_ms: u32,
}

impl LockoutState {
    fn new() -> Self {
        Self {
            active: false,
            button_index: 0,
            end_ms: 0,
        }
    }

    /// Start lockout period for given button
    fn start(&mut self, button_index: usize, current_ms: u32) {
        self.active = true;
        self.button_index = button_index;
        self.end_ms = current_ms.wrapping_add(LOCKOUT_MS);
    }

    /// Check if lockout has expired. Returns true if lockout just ended.
    fn check_expired(&mut self, current_ms: u32) -> bool {
        if self.active {
            // Check if lockout period has elapsed (handle wrap-around)
            let elapsed = current_ms.wrapping_sub(self.end_ms.wrapping_sub(LOCKOUT_MS));
            if elapsed >= LOCKOUT_MS {
                self.active = false;
                return true;
            }
        }
        false
    }

    /// Is lockout currently active?
    fn is_active(&self) -> bool {
        self.active
    }

    /// Get active button index
    fn get_button(&self) -> usize {
        self.button_index
    }
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    // CRITICAL: Read EXT1 wakeup status IMMEDIATELY before ANY initialization
    // ESP32 TRM Section 30.3.5: RTC_CNTL_EXT_WAKEUP1_STATUS register (0x3FF4_80D0)
    let ext1_status_on_wake = unsafe {
        const RTC_CNTL_EXT_WAKEUP1_STATUS_REG: *const u32 = 0x3ff4_80D0 as *const u32;
        core::ptr::read_volatile(RTC_CNTL_EXT_WAKEUP1_STATUS_REG)
    };

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Initialize RTC for deep sleep control
    let mut rtc = Rtc::new(peripherals.LPWR);

    // Check reset and wake reason
    let reset_rsn = reset_reason(Cpu::ProCpu);
    let wake_rsn = wakeup_cause();

    info!("=== ESP32 Smiley Feedback Panel ===");
    info!("Reset reason: {:?}", reset_rsn);
    info!("Wake reason: {:?}", wake_rsn);

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
            info!("{} PRESSED (debounced)", button_name);
            info!("Lockout started for {}ms - ignoring all inputs", LOCKOUT_MS);

            // Wait for lockout period (7 seconds)
            delay.delay_millis(LOCKOUT_MS);

            // Turn off LED
            led.set_low();
            info!("Lockout ended - turning off LED");
        }

        // Return to deep sleep immediately
        info!("Entering deep sleep (target: <10µA current draw)...");
        delay.delay_millis(100); // Allow log output to complete

        // Drop Input drivers to release GPIO pins for EXT1 wakeup configuration
        core::mem::drop(button1);
        core::mem::drop(button2);
        core::mem::drop(button3);
        core::mem::drop(button4);

        // Configure EXT1 wakeup source with all 4 RTC buttons (Active High)
        let mut wakeup_pins: [&mut dyn RtcPin; 4] = [
            &mut pin25, &mut pin26, &mut pin27, &mut pin32,
        ];
        // WakeupLevel::High supports "OR" logic natively on ESP32 (Any High)
        let ext1 = Ext1WakeupSource::new(&mut wakeup_pins, WakeupLevel::High);

        // Configure RTC Pull-Downs for Deep Sleep (matches external 10k pull-downs)
        {
            let rtc_io = esp_hal::peripherals::RTC_IO::regs();

            // GPIO25 (TOUCH_PAD6)
            rtc_io.touch_pad6().modify(|_, w| w.rue().clear_bit().rde().set_bit());

            // GPIO26 (TOUCH_PAD7)
            rtc_io.touch_pad7().modify(|_, w| w.rue().clear_bit().rde().set_bit());

            // GPIO27 (RTC_GPIO17) - No internal pull-down available on some revisions, 
            // but we have external pull-down so it's fine.

            // GPIO32 (XTAL_32K_P)
            rtc_io.xtal_32k_pad().modify(|_, w| {
                w.x32p_rue().clear_bit()     // Disable pull-up
                 .x32p_rde().set_bit()       // Enable pull-down
                 .x32p_fun_ie().set_bit()    // Enable input function
            });
        }

        // Backup timer wakeup (5 minutes)
        let timer = TimerWakeupSource::new(Duration::from_secs(300));

        // Go to sleep
        rtc.sleep_deep(&[&ext1, &timer]);
    }

    // Power-on reset - run LED startup test
    info!("Power-on reset detected - running LED startup test...");
    green_led.set_high(); delay.delay_millis(200); green_led.set_low();
    yellow_led.set_high(); delay.delay_millis(200); yellow_led.set_low();
    blue_led.set_high(); delay.delay_millis(200); blue_led.set_low();
    red_led.set_high(); delay.delay_millis(200); red_led.set_low();
    info!("LED test complete! Ready for buttons.");

    info!("Buttons (RTC pins): GPIO25, GPIO26, GPIO27, GPIO32");
    info!("Active HIGH Logic (Press = High)");

    // Initialize state trackers
    let mut button_states = [
        ButtonState::new(), ButtonState::new(), ButtonState::new(), ButtonState::new()
    ];
    let mut lockout = LockoutState::new();
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

        // Check lockout expiration
        if lockout.check_expired(elapsed_ms) {
            let button_idx = lockout.get_button();
            info!("Lockout ended - turning off LED");
            match button_idx {
                0 => green_led.set_low(),
                1 => yellow_led.set_low(),
                2 => blue_led.set_low(),
                3 => red_led.set_low(),
                _ => {}
            }

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

        // Process buttons if not locked out
        if !lockout.is_active() {
            for (idx, &is_high) in button_readings.iter().enumerate() {
                if let Some(pressed) = button_states[idx].update(is_high, elapsed_ms) {
                    if pressed {
                        info!("{} PRESSED", BUTTON_NAMES[idx]);
                        
                        // Turn on LED
                        match idx {
                            0 => green_led.set_high(),
                            1 => yellow_led.set_high(),
                            2 => blue_led.set_high(),
                            3 => red_led.set_high(),
                            _ => {}
                        }

                        // Start lockout
                        lockout.start(idx, elapsed_ms);
                        info!("Lockout started ({}ms)", LOCKOUT_MS);
                    }
                }
            }
        } else {
             // Keep LED on during lockout (ensure state is held)
             // (Already handled by not turning it off until lockout check_expired)
        }

        delay.delay_millis(POLL_INTERVAL_MS);
        elapsed_ms = elapsed_ms.wrapping_add(POLL_INTERVAL_MS);
    }
}
