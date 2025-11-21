//! NTP module for time synchronization
//!
//! Provides Simple Network Time Protocol (SNTP) client functionality
//! to synchronize time from NTP servers using smoltcp.

extern crate alloc;
use alloc::vec;
use core::net::{IpAddr, Ipv4Addr, SocketAddr};

use esp_hal::delay::Delay;
use esp_wifi::wifi::WifiDevice;
use log::info;
use smoltcp::iface::{Interface, SocketSet};
use smoltcp::socket::dhcpv4;
use smoltcp::socket::udp;
use smoltcp::time::Instant;
use smoltcp::wire::{IpCidr, IpEndpoint, Ipv4Address};
use sntpc::NtpTimestampGenerator;

/// Copenhagen timezone offset in seconds (UTC+1 for CET)
pub const COPENHAGEN_OFFSET_SECS: i64 = 3600;

/// Copenhagen summer time offset in seconds (UTC+2 for CEST)
pub const COPENHAGEN_SUMMER_OFFSET_SECS: i64 = 7200;

/// Denmark NTP server address
pub const DENMARK_NTP_SERVER: &str = "dk.pool.ntp.org";

/// Denmark NTP server IP (one of the pool servers)
pub const DENMARK_NTP_SERVER_IP: Ipv4Addr = Ipv4Addr::new(162, 159, 200, 1);

/// NTP port
pub const NTP_PORT: u16 = 123;

/// Timestamp generator for NTP operations
#[derive(Copy, Clone)]
pub struct EspTimestampGenerator {
    counter: u64,
}

impl EspTimestampGenerator {
    pub const fn new() -> Self {
        Self { counter: 0 }
    }
}

impl Default for EspTimestampGenerator {
    fn default() -> Self {
        Self::new()
    }
}

impl NtpTimestampGenerator for EspTimestampGenerator {
    fn init(&mut self) {
        self.counter = 0;
    }

    fn timestamp_sec(&self) -> u64 {
        self.counter
    }

    fn timestamp_subsec_micros(&self) -> u32 {
        0
    }
}

/// NTP time result
#[derive(Debug, Clone, Copy)]
pub struct NtpTime {
    /// Seconds since Unix epoch (1970-01-01 00:00:00 UTC)
    pub unix_timestamp: u64,
    /// Microseconds part
    pub microseconds: u32,
}

impl NtpTime {
    /// Convert to Copenhagen local time
    pub fn to_copenhagen_time(&self, is_summer_time: bool) -> u64 {
        let offset = if is_summer_time {
            COPENHAGEN_SUMMER_OFFSET_SECS
        } else {
            COPENHAGEN_OFFSET_SECS
        };
        self.unix_timestamp.wrapping_add(offset as u64)
    }

    /// Get hours, minutes, seconds from Unix timestamp
    pub fn to_hms(&self) -> (u8, u8, u8) {
        let total_seconds = self.unix_timestamp % 86400;
        let hours = (total_seconds / 3600) as u8;
        let minutes = ((total_seconds % 3600) / 60) as u8;
        let seconds = (total_seconds % 60) as u8;
        (hours, minutes, seconds)
    }

    /// Get hours, minutes, seconds in Copenhagen timezone
    pub fn to_copenhagen_hms(&self, is_summer_time: bool) -> (u8, u8, u8) {
        let copenhagen_ts = self.to_copenhagen_time(is_summer_time);
        let total_seconds = copenhagen_ts % 86400;
        let hours = (total_seconds / 3600) as u8;
        let minutes = ((total_seconds % 3600) / 60) as u8;
        let seconds = (total_seconds % 60) as u8;
        (hours, minutes, seconds)
    }
}

/// Get NTP server socket address
pub fn get_ntp_server_addr() -> SocketAddr {
    SocketAddr::new(IpAddr::V4(DENMARK_NTP_SERVER_IP), NTP_PORT)
}

/// Setup DHCP and configure network interface
///
/// This function must be called ONCE after WiFi connection to configure the interface with DHCP.
/// Returns the configured IP address.
pub fn setup_network_interface(
    iface: &mut Interface,
    device: &mut WifiDevice<'_>,
    sockets: &mut SocketSet,
) -> Result<Ipv4Address, &'static str> {
    info!("Network: Starting DHCP configuration...");
    let delay = Delay::new();

    // Create DHCP socket
    let dhcp_socket = dhcpv4::Socket::new();
    let dhcp_handle = sockets.add(dhcp_socket);

    // Wait for DHCP to get an IP address
    let mut got_ip = false;
    let mut our_ip = Ipv4Address::UNSPECIFIED;
    let mut timeout_counter = 0u32;

    while !got_ip && timeout_counter < 100 {
        let timestamp = Instant::from_millis(timeout_counter as i64 * 100);
        iface.poll(timestamp, device, sockets);

        let dhcp_socket = sockets.get_mut::<dhcpv4::Socket>(dhcp_handle);
        if let Some(dhcpv4::Event::Configured(config)) = dhcp_socket.poll() {
            info!("Network: DHCP configured: {:?}", config.address);
            iface.update_ip_addrs(|addrs| {
                if let Some(addr) = addrs.iter_mut().next() {
                    *addr = IpCidr::Ipv4(config.address);
                } else {
                    // Try to add the address
                    let _ = addrs.push(IpCidr::Ipv4(config.address));
                }
            });

            if let Some(router) = config.router {
                iface.routes_mut().add_default_ipv4_route(router).ok();
            }

            our_ip = config.address.address();
            got_ip = true;
        }

        delay.delay_millis(100);
        timeout_counter += 1;
    }

    sockets.remove(dhcp_handle);

    if !got_ip {
        return Err("DHCP timeout - failed to obtain IP address");
    }

    info!("Network: Got IP address: {}", our_ip);
    Ok(our_ip)
}

/// Synchronize time with NTP server using smoltcp
///
/// This function assumes the interface is already configured with an IP address via DHCP.
pub fn sync_time_with_device(
    iface: &mut Interface,
    device: &mut WifiDevice<'_>,
    sockets: &mut SocketSet,
) -> Result<NtpTime, &'static str> {
    info!(
        "NTP: Starting time synchronization with {}",
        DENMARK_NTP_SERVER
    );

    let delay = Delay::new();

    // Get our IP address from the interface
    let our_ip = iface
        .ip_addrs()
        .iter()
        .find_map(|addr| match addr {
            IpCidr::Ipv4(ipv4) => Some(ipv4.address()),
            #[allow(unreachable_patterns)]
            _ => None,
        })
        .ok_or("No IPv4 address configured on interface")?;

    info!("NTP: Using IP address: {}", our_ip);

    // Create UDP socket with owned buffers
    let udp_rx_buffer = udp::PacketBuffer::new(vec![udp::PacketMetadata::EMPTY; 4], vec![0u8; 256]);
    let udp_tx_buffer = udp::PacketBuffer::new(vec![udp::PacketMetadata::EMPTY; 4], vec![0u8; 256]);

    let udp_socket = udp::Socket::new(udp_rx_buffer, udp_tx_buffer);
    let udp_handle = sockets.add(udp_socket);

    // Send NTP request
    let server_addr = Ipv4Address::new(
        DENMARK_NTP_SERVER_IP.octets()[0],
        DENMARK_NTP_SERVER_IP.octets()[1],
        DENMARK_NTP_SERVER_IP.octets()[2],
        DENMARK_NTP_SERVER_IP.octets()[3],
    );
    let server_endpoint = IpEndpoint::new(server_addr.into(), NTP_PORT);

    // Bind UDP socket to local port
    {
        let udp_socket = sockets.get_mut::<udp::Socket>(udp_handle);
        udp_socket
            .bind(IpEndpoint::new(our_ip.into(), 12345))
            .map_err(|_| "Failed to bind UDP socket")?;

        // Create NTP request packet (simplified SNTP v4 request)
        let mut ntp_request = [0u8; 48];
        // LI = 0 (no warning), VN = 4 (SNTPv4), Mode = 3 (client)
        ntp_request[0] = 0b00_100_011; // LI=0, VN=4, Mode=3

        // Send NTP request
        info!("NTP: Sending request to {}:{}", server_addr, NTP_PORT);
        udp_socket
            .send_slice(&ntp_request, server_endpoint)
            .map_err(|_| "Failed to send NTP request")?;
    }

    // Poll to actually send the packet
    let mut poll_counter = 0u32;
    let timestamp = Instant::from_millis(poll_counter as i64 * 50);
    iface.poll(timestamp, device, sockets);
    poll_counter += 1;

    // Wait for response
    info!("NTP: Waiting for response...");
    let mut ntp_response = [0u8; 48];
    let mut got_response = false;

    while !got_response && poll_counter < 100 {
        let timestamp = Instant::from_millis(poll_counter as i64 * 50);
        iface.poll(timestamp, device, sockets);

        let udp_socket = sockets.get_mut::<udp::Socket>(udp_handle);
        if udp_socket.can_recv() {
            if let Ok((size, _endpoint)) = udp_socket.recv_slice(&mut ntp_response) {
                if size >= 48 {
                    got_response = true;
                    info!("NTP: Received {} bytes", size);
                }
            }
        }

        if !got_response {
            delay.delay_millis(50);
            poll_counter += 1;
        }
    }

    sockets.remove(udp_handle);

    if !got_response {
        return Err("NTP response timeout");
    }

    // Parse NTP response - extract transmit timestamp (bytes 40-47)
    // NTP timestamp is seconds since 1900-01-01 00:00:00
    let ntp_seconds = u32::from_be_bytes([
        ntp_response[40],
        ntp_response[41],
        ntp_response[42],
        ntp_response[43],
    ]);
    let ntp_fraction = u32::from_be_bytes([
        ntp_response[44],
        ntp_response[45],
        ntp_response[46],
        ntp_response[47],
    ]);

    // Convert NTP timestamp (since 1900) to Unix timestamp (since 1970)
    // Difference is 70 years = 2208988800 seconds
    const NTP_TO_UNIX_OFFSET: u64 = 2208988800;
    let unix_timestamp = (ntp_seconds as u64).saturating_sub(NTP_TO_UNIX_OFFSET);

    // Convert fraction to microseconds
    let microseconds = ((ntp_fraction as u64 * 1_000_000) >> 32) as u32;

    info!("NTP: Sync successful! Unix timestamp: {}", unix_timestamp);

    Ok(NtpTime {
        unix_timestamp,
        microseconds,
    })
}

/// Check if current date is in Copenhagen summer time (CEST)
pub fn is_summer_time_approx(month: u8) -> bool {
    (4..=9).contains(&month)
}

/// Format Unix timestamp to human-readable datetime string
/// Returns format: "YYYY-MM-DD HH:MM:SS TZ"
/// Example: "2025-11-21 14:32:20 CET"
pub fn format_datetime(unix_timestamp: u64, is_summer_time: bool) -> heapless::String<32> {
    use heapless::String;

    // Convert to Copenhagen timezone
    let offset_secs = if is_summer_time {
        COPENHAGEN_SUMMER_OFFSET_SECS
    } else {
        COPENHAGEN_OFFSET_SECS
    };
    let local_ts = unix_timestamp.wrapping_add(offset_secs as u64);

    // Calculate date components
    // Days since Unix epoch (1970-01-01)
    let days_since_epoch = local_ts / 86400;

    // Simple algorithm to get year, month, day from days since epoch
    // Starting from 1970-01-01
    let mut year = 1970u32;
    let mut remaining_days = days_since_epoch as u32;

    // Calculate year
    loop {
        let days_in_year = if is_leap_year(year) { 366 } else { 365 };
        if remaining_days >= days_in_year {
            remaining_days -= days_in_year;
            year += 1;
        } else {
            break;
        }
    }

    // Calculate month and day
    let (month, day) = days_to_month_day(remaining_days, is_leap_year(year));

    // Calculate time components
    let time_of_day = local_ts % 86400;
    let hours = (time_of_day / 3600) as u8;
    let minutes = ((time_of_day % 3600) / 60) as u8;
    let seconds = (time_of_day % 60) as u8;

    // Timezone string
    let tz = if is_summer_time { "CEST" } else { "CET" };

    // Format: "YYYY-MM-DD HH:MM:SS TZ"
    let mut result = String::new();
    use core::fmt::Write;
    let _ = write!(
        &mut result,
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02} {}",
        year, month, day, hours, minutes, seconds, tz
    );

    result
}

/// Check if a year is a leap year
fn is_leap_year(year: u32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
}

/// Convert day of year to month and day
fn days_to_month_day(day_of_year: u32, is_leap: bool) -> (u8, u8) {
    let days_in_months = if is_leap {
        [31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    } else {
        [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
    };

    let mut remaining = day_of_year;
    for (month_idx, &days) in days_in_months.iter().enumerate() {
        if remaining < days {
            return ((month_idx + 1) as u8, (remaining + 1) as u8);
        }
        remaining -= days;
    }

    // Should not reach here, but return last day of year as fallback
    (12, 31)
}
