//! LED activity state management

/// LED display duration after button press (milliseconds)
pub const LED_DISPLAY_MS: u32 = 500;

/// LED activity state tracker
pub struct LedActivityState {
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
    pub fn new() -> Self {
        Self {
            led_active: false,
            led_index: 0,
            last_press_ms: 0,
            had_activity: false,
        }
    }

    /// Activate LED for given button (resets timer if already active)
    pub fn activate(&mut self, led_index: usize, current_ms: u32) {
        self.led_active = true;
        self.led_index = led_index;
        self.last_press_ms = current_ms;
        self.had_activity = true;
    }

    /// Check if LED display period has expired. Returns true if LED should turn off.
    pub fn check_led_expired(&self, current_ms: u32) -> bool {
        if self.led_active {
            let elapsed = current_ms.wrapping_sub(self.last_press_ms);
            return elapsed >= LED_DISPLAY_MS;
        }
        false
    }

    /// Check if sleep delay has expired. Returns true if device should sleep.
    pub fn check_sleep_expired(&self, current_ms: u32, sleep_delay_ms: u32) -> bool {
        if self.had_activity && !self.led_active {
            let elapsed = current_ms.wrapping_sub(self.last_press_ms);
            return elapsed >= sleep_delay_ms;
        }
        false
    }

    /// Turn off LED but keep tracking for sleep delay
    pub fn deactivate_led(&mut self) {
        self.led_active = false;
    }

    /// Is any LED currently active?
    pub fn is_led_active(&self) -> bool {
        self.led_active
    }

    /// Get active LED index
    pub fn get_led(&self) -> usize {
        self.led_index
    }

    /// Mark activity for sleep tracking (used at startup)
    pub fn mark_activity(&mut self, current_ms: u32) {
        self.had_activity = true;
        self.last_press_ms = current_ms;
    }
}

impl Default for LedActivityState {
    fn default() -> Self {
        Self::new()
    }
}
