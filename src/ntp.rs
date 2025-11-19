//! NTP module for time synchronization
//!
//! Provides Simple Network Time Protocol (SNTP) client functionality
//! to synchronize time from NTP servers using smoltcp.

use core::net::{IpAddr, Ipv4Addr, SocketAddr};

use esp_hal::delay::Delay;
use esp_radio::wifi::WifiDevice;
use log::info;
use smoltcp::iface::{Interface, SocketSet};
use smoltcp::socket::dhcpv4;
use smoltcp::socket::udp;
use smoltcp::time::Instant;
use smoltcp::wire::{IpCidr, IpEndpoint, Ipv4Address};
use sntpc::{NtpContext, NtpTimestampGenerator};

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
) -> Result<Ipv4Address, &'static str> {
    info!("Network: Starting DHCP configuration...");
    let delay = Delay::new();

    // Create DHCP socket
    let dhcp_socket = dhcpv4::Socket::new();

    // Socket set
    let mut sockets_storage: [_; 1] = Default::default();
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let dhcp_handle = sockets.add(dhcp_socket);

    // Wait for DHCP to get an IP address
    let mut got_ip = false;
    let mut our_ip = Ipv4Address::UNSPECIFIED;
    let mut timeout_counter = 0u32;

    while !got_ip && timeout_counter < 100 {
        let timestamp = Instant::from_millis(timeout_counter as i64 * 100);
        iface.poll(timestamp, device, &mut sockets);

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
) -> Result<NtpTime, &'static str> {
    info!("NTP: Starting time synchronization with {}", DENMARK_NTP_SERVER);

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

    // Create socket storage
    let mut udp_rx_buffer = [0u8; 256];
    let mut udp_tx_buffer = [0u8; 256];
    let mut udp_rx_meta = [udp::PacketMetadata::EMPTY; 4];
    let mut udp_tx_meta = [udp::PacketMetadata::EMPTY; 4];

    // Create UDP socket
    let udp_socket = udp::Socket::new(
        udp::PacketBuffer::new(&mut udp_rx_meta[..], &mut udp_rx_buffer[..]),
        udp::PacketBuffer::new(&mut udp_tx_meta[..], &mut udp_tx_buffer[..]),
    );

    // Socket set
    let mut sockets_storage: [_; 1] = Default::default();
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
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

    // Poll to actually send the packet
    let mut poll_counter = 0u32;
    let timestamp = Instant::from_millis(poll_counter as i64 * 50);
    iface.poll(timestamp, device, &mut sockets);
    poll_counter += 1;

    // Wait for response
    info!("NTP: Waiting for response...");
    let mut ntp_response = [0u8; 48];
    let mut got_response = false;

    while !got_response && poll_counter < 100 {
        let timestamp = Instant::from_millis(poll_counter as i64 * 50);
        iface.poll(timestamp, device, &mut sockets);

        let udp_socket = sockets.get_mut::<udp::Socket>(udp_handle);
        if udp_socket.can_recv()
            && let Ok((size, _endpoint)) = udp_socket.recv_slice(&mut ntp_response)
            && size >= 48
        {
            got_response = true;
            info!("NTP: Received {} bytes", size);
        }

        if !got_response {
            delay.delay_millis(50);
            poll_counter += 1;
        }
    }

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

    info!(
        "NTP: Sync successful! Unix timestamp: {}",
        unix_timestamp
    );

    Ok(NtpTime {
        unix_timestamp,
        microseconds,
    })
}

/// Synchronize time with NTP server (legacy placeholder)
pub fn sync_time() -> Result<NtpTime, &'static str> {
    info!("NTP: Starting time synchronization with {}", DENMARK_NTP_SERVER);

    let server_addr = get_ntp_server_addr();
    let _context = NtpContext::new(EspTimestampGenerator::new());

    info!("NTP: Server address: {}:{}", server_addr.ip(), server_addr.port());

    Err("Use sync_time_with_device() instead - requires WifiDevice")
}

/// Check if current date is in Copenhagen summer time (CEST)
pub fn is_summer_time_approx(month: u8) -> bool {
    (4..=9).contains(&month)
}
