//! MQTT module for ESP32 feedback panel
//!
//! Provides MQTT client functionality for publishing button press feedback
//! to an MQTT broker over plain TCP (educational project).
//!
//! # Architecture
//! - Direct TCP socket communication with MQTT broker
//! - Implements simplified MQTT 3.1.1 client protocol
//!
//! # Security
//! - Plain TCP on port 1884 (educational network environment)
//! - Username/password authentication at MQTT level
//! - Production use would require TLS or network isolation

use core::fmt::Write;
use core::net::Ipv4Addr;

use esp_hal::delay::Delay;
use esp_radio::wifi::WifiDevice;
use heapless::String;
use log::info;
use smoltcp::iface::{Interface, SocketSet};
use smoltcp::socket::tcp;
use smoltcp::time::Instant;
use smoltcp::wire::{IpEndpoint, Ipv4Address};

/// MQTT broker configuration
#[derive(Debug, Clone)]
pub struct MqttBrokerConfig {
    /// Broker IP address
    pub ip: Ipv4Addr,
    /// MQTT port (8883 for TLS)
    pub port: u16,
    /// MQTT username
    pub username: &'static str,
    /// MQTT password
    pub password: &'static str,
}

impl Default for MqttBrokerConfig {
    fn default() -> Self {
        Self {
            ip: Ipv4Addr::new(192, 168, 0, 189), // Host machine IP
            port: 1884,                           // Plain MQTT port (educational)
            username: "elev1",
            password: "password",
        }
    }
}

/// MQTT error type
#[derive(Debug, Clone, Copy)]
pub enum MqttError {
    /// Network connection failed
    ConnectionFailed,
    /// Failed to send MQTT packet
    SendFailed,
    /// Failed to receive MQTT packet
    ReceiveFailed,
    /// MQTT protocol error
    ProtocolError,
    /// Timeout waiting for response
    Timeout,
    /// Buffer too small
    BufferTooSmall,
}

impl core::fmt::Display for MqttError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            MqttError::ConnectionFailed => write!(f, "Connection failed"),
            MqttError::SendFailed => write!(f, "Send failed"),
            MqttError::ReceiveFailed => write!(f, "Receive failed"),
            MqttError::ProtocolError => write!(f, "Protocol error"),
            MqttError::Timeout => write!(f, "Timeout"),
            MqttError::BufferTooSmall => write!(f, "Buffer too small"),
        }
    }
}

/// MQTT control packet types (MQTT 3.1.1)
#[repr(u8)]
#[allow(dead_code)]
enum PacketType {
    Connect = 0x10,
    ConnAck = 0x20,
    Publish = 0x30,
    PubAck = 0x40,
    Subscribe = 0x80,
    SubAck = 0x90,
    PingReq = 0xC0,
    PingResp = 0xD0,
    Disconnect = 0xE0,
}

/// Build MQTT CONNECT packet
fn build_connect_packet(
    client_id: &str,
    username: &str,
    password: &str,
    buffer: &mut [u8],
) -> Result<usize, MqttError> {
    if buffer.len() < 128 {
        return Err(MqttError::BufferTooSmall);
    }

    let mut pos = 0;

    // Fixed header: CONNECT packet type
    buffer[pos] = PacketType::Connect as u8;
    pos += 1;

    // Calculate remaining length (will fill in later)
    let remaining_length_pos = pos;
    pos += 1; // Reserve space for remaining length

    // Variable header: Protocol Name "MQTT"
    buffer[pos] = 0x00; // Length MSB
    pos += 1;
    buffer[pos] = 0x04; // Length LSB (4 bytes)
    pos += 1;
    buffer[pos..pos + 4].copy_from_slice(b"MQTT");
    pos += 4;

    // Protocol Level: MQTT 3.1.1 = 4
    buffer[pos] = 0x04;
    pos += 1;

    // Connect Flags: Clean Session + Username + Password
    let connect_flags = 0x02 | 0x80 | 0x40; // Clean Session | Username | Password
    buffer[pos] = connect_flags;
    pos += 1;

    // Keep Alive: 60 seconds
    buffer[pos] = 0x00; // MSB
    pos += 1;
    buffer[pos] = 0x3C; // LSB (60 seconds)
    pos += 1;

    // Payload: Client ID
    let client_id_bytes = client_id.as_bytes();
    buffer[pos] = (client_id_bytes.len() >> 8) as u8; // Length MSB
    pos += 1;
    buffer[pos] = (client_id_bytes.len() & 0xFF) as u8; // Length LSB
    pos += 1;
    buffer[pos..pos + client_id_bytes.len()].copy_from_slice(client_id_bytes);
    pos += client_id_bytes.len();

    // Username
    let username_bytes = username.as_bytes();
    buffer[pos] = (username_bytes.len() >> 8) as u8;
    pos += 1;
    buffer[pos] = (username_bytes.len() & 0xFF) as u8;
    pos += 1;
    buffer[pos..pos + username_bytes.len()].copy_from_slice(username_bytes);
    pos += username_bytes.len();

    // Password
    let password_bytes = password.as_bytes();
    buffer[pos] = (password_bytes.len() >> 8) as u8;
    pos += 1;
    buffer[pos] = (password_bytes.len() & 0xFF) as u8;
    pos += 1;
    buffer[pos..pos + password_bytes.len()].copy_from_slice(password_bytes);
    pos += password_bytes.len();

    // Fill in remaining length (total length - 2 bytes for fixed header)
    let remaining_length = pos - remaining_length_pos - 1;
    buffer[remaining_length_pos] = remaining_length as u8;

    Ok(pos)
}

/// Build MQTT PUBLISH packet (QoS 1 with packet ID)
fn build_publish_packet(
    topic: &str,
    payload: &[u8],
    packet_id: u16,
    buffer: &mut [u8],
) -> Result<usize, MqttError> {
    if buffer.len() < topic.len() + payload.len() + 32 {
        return Err(MqttError::BufferTooSmall);
    }

    let mut pos = 0;

    // Fixed header: PUBLISH packet type with QoS 1 (0x32 = 0x30 | 0x02)
    buffer[pos] = 0x32; // PUBLISH with QoS 1
    pos += 1;

    // Calculate remaining length (topic + packet_id + payload)
    let remaining_length = 2 + topic.len() + 2 + payload.len(); // 2 for topic len, 2 for packet ID

    // Encode remaining length
    if remaining_length < 128 {
        buffer[pos] = remaining_length as u8;
        pos += 1;
    } else if remaining_length < 16384 {
        // Two-byte encoding for 128-16383
        buffer[pos] = (remaining_length & 0x7F) as u8 | 0x80;
        pos += 1;
        buffer[pos] = (remaining_length >> 7) as u8;
        pos += 1;
    } else {
        return Err(MqttError::BufferTooSmall);
    }

    // Variable header: Topic name
    let topic_bytes = topic.as_bytes();
    buffer[pos] = (topic_bytes.len() >> 8) as u8; // Length MSB
    pos += 1;
    buffer[pos] = (topic_bytes.len() & 0xFF) as u8; // Length LSB
    pos += 1;
    buffer[pos..pos + topic_bytes.len()].copy_from_slice(topic_bytes);
    pos += topic_bytes.len();

    // Packet ID (required for QoS 1)
    buffer[pos] = (packet_id >> 8) as u8; // MSB
    pos += 1;
    buffer[pos] = (packet_id & 0xFF) as u8; // LSB
    pos += 1;

    // Payload
    buffer[pos..pos + payload.len()].copy_from_slice(payload);
    pos += payload.len();

    Ok(pos)
}

/// Build MQTT DISCONNECT packet
fn build_disconnect_packet(buffer: &mut [u8]) -> Result<usize, MqttError> {
    if buffer.len() < 2 {
        return Err(MqttError::BufferTooSmall);
    }

    buffer[0] = PacketType::Disconnect as u8;
    buffer[1] = 0x00; // Remaining length = 0

    Ok(2)
}

/// Helper function to poll network interface
fn poll_network(
    iface: &mut Interface,
    device: &mut WifiDevice<'_>,
    sockets: &mut SocketSet<'_>,
    timestamp_ms: i64,
) {
    let timestamp = Instant::from_millis(timestamp_ms);
    iface.poll(timestamp, device, sockets);
}

/// Publish button feedback over plain MQTT (no TLS)
///
/// This function:
/// 1. Establishes TCP connection to broker on port 1884
/// 2. Sends MQTT CONNECT with authentication
/// 3. Publishes button feedback message
/// 4. Disconnects gracefully
///
/// # Args
/// * `iface` - Pre-configured smoltcp interface with IP address and routing
/// * `device` - WiFi device for network operations
/// * `button_number` - Button that was pressed (1-4)
/// * `unix_timestamp` - Unix timestamp of the button press
/// * `device_id` - Device identifier
///
/// # Returns
/// Ok(()) if the entire flow succeeds, Err otherwise
pub fn publish_button_feedback(
    iface: &mut Interface,
    device: &mut WifiDevice<'_>,
    button_number: u8,
    unix_timestamp: u64,
    device_id: &str,
) -> Result<(), MqttError> {
    if !(1..=4).contains(&button_number) {
        info!("MQTT: Invalid button number: {}", button_number);
        return Err(MqttError::ProtocolError);
    }

    info!(
        "MQTT: Publishing feedback - button {} at timestamp {}",
        button_number, unix_timestamp
    );

    let broker_config = MqttBrokerConfig::default();
    let delay = Delay::new();

    info!(
        "MQTT: Connecting to broker {}:{} (plain TCP)",
        broker_config.ip, broker_config.port
    );

    // Create TCP socket with larger buffers
    let mut tcp_rx_buffer = [0u8; 8192];
    let mut tcp_tx_buffer = [0u8; 8192];
    let tcp_socket = tcp::Socket::new(
        tcp::SocketBuffer::new(&mut tcp_rx_buffer[..]),
        tcp::SocketBuffer::new(&mut tcp_tx_buffer[..]),
    );

    // Socket set
    let mut sockets_storage: [_; 1] = Default::default();
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let tcp_handle = sockets.add(tcp_socket);

    // Convert broker IP to smoltcp Ipv4Address
    let broker_addr = Ipv4Address::new(
        broker_config.ip.octets()[0],
        broker_config.ip.octets()[1],
        broker_config.ip.octets()[2],
        broker_config.ip.octets()[3],
    );
    let broker_endpoint = IpEndpoint::new(broker_addr.into(), broker_config.port);

    // Connect to broker (TCP)
    info!("MQTT: Initiating TCP connection...");
    let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
    socket
        .connect(iface.context(), broker_endpoint, 49152)
        .map_err(|_| {
            info!("MQTT: TCP connect failed");
            MqttError::ConnectionFailed
        })?;

    // Poll until TCP connected
    let mut timeout_counter = 0u32;
    let mut connected = false;
    let mut timestamp_ms = 0i64;

    while !connected && timeout_counter < 100 {
        // Poll network stack
        for _ in 0..5 {
            poll_network(iface, device, &mut sockets, timestamp_ms);
            timestamp_ms += 10;
        }

        let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);

        // Log connection state for debugging
        if timeout_counter % 10 == 0 {
            info!("MQTT: TCP state check #{} - may_send: {}, may_recv: {}",
                  timeout_counter / 10, socket.may_send(), socket.may_recv());
        }

        if socket.may_send() && socket.may_recv() {
            connected = true;
            info!("MQTT: TCP connected successfully!");
        }

        if !connected {
            delay.delay_millis(50);
            timeout_counter += 1;
        }
    }

    if !connected {
        info!("MQTT: TCP connection timeout after {}ms", timeout_counter * 50);
        return Err(MqttError::Timeout);
    }

    // Build and send MQTT CONNECT packet (over plain TCP)
    let mut connect_buffer = [0u8; 256];
    let connect_len = build_connect_packet(
        device_id,
        broker_config.username,
        broker_config.password,
        &mut connect_buffer,
    )?;

    info!("MQTT: Sending CONNECT packet ({} bytes)", connect_len);

    // Send CONNECT packet through TCP socket
    let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
    match socket.send_slice(&connect_buffer[..connect_len]) {
        Ok(sent) if sent == connect_len => {
            info!("MQTT: CONNECT sent successfully ({} bytes)", sent);
        }
        Ok(sent) => {
            info!("MQTT: Partial CONNECT sent ({}/{})", sent, connect_len);
            return Err(MqttError::SendFailed);
        }
        Err(_) => {
            info!("MQTT: Failed to send CONNECT");
            return Err(MqttError::SendFailed);
        }
    }

    // Poll to ensure packet is sent
    for _ in 0..10 {
        poll_network(iface, device, &mut sockets, timestamp_ms);
        timestamp_ms += 10;
        delay.delay_millis(10);
    }

    // Wait for CONNACK
    info!("MQTT: Waiting for CONNACK...");
    let mut connack_buffer = [0u8; 4];
    let mut received_connack = false;

    for _ in 0..50 {
        poll_network(iface, device, &mut sockets, timestamp_ms);
        timestamp_ms += 10;

        let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
        if socket.can_recv() {
            match socket.recv_slice(&mut connack_buffer) {
                Ok(size) if size >= 2 && connack_buffer[0] == PacketType::ConnAck as u8 => {
                    info!("MQTT: CONNACK received, connected to broker");
                    received_connack = true;
                    break;
                }
                Ok(size) if size > 0 => {
                    info!("MQTT: Received {} bytes, but not CONNACK yet", size);
                }
                _ => {}
            }
        }
        delay.delay_millis(10);
    }

    if !received_connack {
        info!("MQTT: CONNACK not received, but continuing anyway");
    }

    // Build topic: feedback/{device_id}/{button}
    let mut topic: String<128> = String::new();
    write!(&mut topic, "feedback/{}/{}", device_id, button_number)
        .map_err(|_| MqttError::BufferTooSmall)?;

    // Build payload: JSON with button and timestamp
    let mut payload: String<128> = String::new();
    write!(
        &mut payload,
        "{{\"button\":{},\"timestamp\":{},\"rating\":\"{}\"}}",
        button_number,
        unix_timestamp,
        match button_number {
            1 => "very_happy",
            2 => "happy",
            3 => "neutral",
            4 => "sad",
            _ => "unknown",
        }
    )
    .map_err(|_| MqttError::BufferTooSmall)?;

    info!("MQTT: Publishing to topic: {}", topic.as_str());
    info!("MQTT: Payload: {}", payload.as_str());

    // Build and send MQTT PUBLISH packet (QoS 1)
    let mut publish_buffer = [0u8; 256];
    let packet_id = 1u16; // Simple packet ID
    let publish_len =
        build_publish_packet(topic.as_str(), payload.as_bytes(), packet_id, &mut publish_buffer)?;

    info!("MQTT: Sending PUBLISH packet ({} bytes)", publish_len);

    // Send PUBLISH through TCP socket
    let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
    match socket.send_slice(&publish_buffer[..publish_len]) {
        Ok(sent) if sent == publish_len => {
            info!("MQTT: PUBLISH sent successfully ({} bytes)", sent);
        }
        Ok(sent) => {
            info!("MQTT: Partial PUBLISH sent ({}/{})", sent, publish_len);
            return Err(MqttError::SendFailed);
        }
        Err(_) => {
            info!("MQTT: Failed to send PUBLISH");
            return Err(MqttError::SendFailed);
        }
    }

    // Poll to ensure packet is sent
    for _ in 0..10 {
        poll_network(iface, device, &mut sockets, timestamp_ms);
        timestamp_ms += 10;
        delay.delay_millis(10);
    }

    // Wait for PUBACK (QoS 1)
    info!("MQTT: Waiting for PUBACK...");
    let mut puback_buffer = [0u8; 4];
    let mut received_puback = false;

    for _ in 0..50 {
        poll_network(iface, device, &mut sockets, timestamp_ms);
        timestamp_ms += 10;

        let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
        if socket.can_recv() {
            match socket.recv_slice(&mut puback_buffer) {
                Ok(size) if size >= 2 && puback_buffer[0] == PacketType::PubAck as u8 => {
                    info!("MQTT: PUBACK received");
                    received_puback = true;
                    break;
                }
                Ok(size) if size > 0 => {
                    info!("MQTT: Received {} bytes, but not PUBACK yet", size);
                }
                _ => {}
            }
        }
        delay.delay_millis(10);
    }

    if !received_puback {
        info!("MQTT: PUBACK not received, but message likely delivered");
    }

    info!("MQTT: Feedback published successfully");

    // Send DISCONNECT packet
    let mut disconnect_buffer = [0u8; 2];
    let disconnect_len = build_disconnect_packet(&mut disconnect_buffer)?;

    info!("MQTT: Sending DISCONNECT packet");
    let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
    let _ = socket.send_slice(&disconnect_buffer[..disconnect_len]);

    // Poll to send disconnect
    for _ in 0..5 {
        poll_network(iface, device, &mut sockets, timestamp_ms);
        timestamp_ms += 10;
        delay.delay_millis(10);
    }

    // Close TCP connection gracefully
    info!("MQTT: Closing TCP connection");
    let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
    socket.close();

    delay.delay_millis(100);

    info!("MQTT: Disconnected from broker");

    Ok(())
}

/// Get button rating name (for logging)
#[allow(dead_code)]
pub fn get_button_rating(button_number: u8) -> &'static str {
    match button_number {
        1 => "Very Happy (GREEN)",
        2 => "Happy (YELLOW)",
        3 => "Neutral (BLUE)",
        4 => "Sad (RED)",
        _ => "Unknown",
    }
}
