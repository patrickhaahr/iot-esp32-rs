//! Deep sleep management for ESP32

use core::time::Duration;

use esp_hal::delay::Delay;
use esp_hal::gpio::RtcPin;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::rtc_cntl::sleep::{Ext1WakeupSource, TimerWakeupSource, WakeupLevel};
use log::info;

/// Minimum time to wait before entering deep sleep (milliseconds)
pub const SLEEP_DELAY_MS: u32 = 10000; // 10 seconds

/// RTC_CNTL_EXT_WAKEUP1_STATUS register address
/// ESP32 TRM Section 30.3.5
const RTC_CNTL_EXT_WAKEUP1_STATUS_REG: u32 = 0x3FF4_80D0;

/// Bit positions for GPIO wake status in EXT1 register
pub struct GpioWakeBits {
    pub gpio25: u32, // RTC_GPIO 6  (Bit 6)
    pub gpio26: u32, // RTC_GPIO 7  (Bit 7)
    pub gpio27: u32, // RTC_GPIO 17 (Bit 17)
    pub gpio32: u32, // RTC_GPIO 9  (Bit 9)
}

/// Read EXT1 wakeup status register
/// MUST be called IMMEDIATELY before ANY initialization
pub fn read_ext1_wakeup_status() -> u32 {
    unsafe {
        let reg = RTC_CNTL_EXT_WAKEUP1_STATUS_REG as *const u32;
        core::ptr::read_volatile(reg)
    }
}

/// Decode which GPIOs triggered the wake from the status register
pub fn decode_wake_gpios(status: u32) -> GpioWakeBits {
    GpioWakeBits {
        gpio25: (status >> 6) & 1,
        gpio26: (status >> 7) & 1,
        gpio27: (status >> 17) & 1,
        gpio32: (status >> 9) & 1,
    }
}

/// Clear EXT1 wakeup status to prevent re-triggering
pub fn clear_ext1_wakeup_status(status: u32) {
    unsafe {
        let reg = RTC_CNTL_EXT_WAKEUP1_STATUS_REG as *mut u32;
        const STATUS_CLR_BIT: u32 = 1 << 18;
        let new_value = status | STATUS_CLR_BIT;
        core::ptr::write_volatile(reg, new_value);
    }
}

/// Configure RTC pull-downs for wake pins
fn configure_rtc_pulldowns() {
    let rtc_io = esp_hal::peripherals::RTC_IO::regs();
    rtc_io
        .touch_pad6()
        .modify(|_, w| w.rue().clear_bit().rde().set_bit());
    rtc_io
        .touch_pad7()
        .modify(|_, w| w.rue().clear_bit().rde().set_bit());
    rtc_io.xtal_32k_pad().modify(|_, w| {
        w.x32p_rue()
            .clear_bit()
            .x32p_rde()
            .set_bit()
            .x32p_fun_ie()
            .set_bit()
    });
}

/// Enter deep sleep with EXT1 button wakeup and timer backup
///
/// # Arguments
/// * `rtc` - RTC peripheral for sleep control
/// * `wakeup_pins` - Array of RTC-capable GPIO pins for wake
/// * `timer_secs` - Backup timer wakeup in seconds
/// * `delay` - Delay peripheral for short pause before sleep
pub fn enter_deep_sleep(
    rtc: &mut Rtc,
    wakeup_pins: &mut [&mut dyn RtcPin],
    timer_secs: u64,
    delay: &Delay,
) -> ! {
    info!("Entering deep sleep...");
    delay.delay_millis(100);

    let ext1 = Ext1WakeupSource::new(wakeup_pins, WakeupLevel::High);
    configure_rtc_pulldowns();

    let timer = TimerWakeupSource::new(Duration::from_secs(timer_secs));
    rtc.sleep_deep(&[&ext1, &timer]);
}
