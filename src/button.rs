//! Button debounce state management

/// Debounce timing constant (milliseconds)
pub const DEBOUNCE_MS: u32 = 50;

/// Button debounce state tracker
#[derive(Clone, Copy)]
pub struct ButtonState {
    /// Last stable state (true = HIGH/pressed, false = LOW/released)
    /// Note: With Active HIGH logic: HIGH = Pressed, LOW = Released
    last_stable: bool,
    /// Timestamp (in ms) when current raw state first appeared
    state_start_ms: u32,
    /// Current raw state being observed
    current_raw: bool,
}

impl ButtonState {
    pub fn new() -> Self {
        Self {
            last_stable: false, // Buttons start LOW (pulled down, not pressed)
            state_start_ms: 0,
            current_raw: false,
        }
    }

    /// Update button state with new reading. Returns Some(pressed) if a debounced event occurred.
    /// pressed=true means button was pressed (LOW->HIGH), pressed=false means button was released
    pub fn update(&mut self, is_high: bool, current_ms: u32) -> Option<bool> {
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

impl Default for ButtonState {
    fn default() -> Self {
        Self::new()
    }
}
