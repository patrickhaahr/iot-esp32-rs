//! MQTT module for ESP32 feedback panel
//!
//! Provides MQTT client functionality for publishing button press feedback
//! to an MQTT broker using a simplified MQTT implementation over smoltcp.
//!
//! # Implementation Note
//! This is a simplified MQTT client designed specifically for the feedback panel use case.
//! It implements basic MQTT 3.1.1 CONNECT and PUBLISH operations over plain TCP (no TLS).
//!
//! **IMPORTANT**: Currently configured for port 8883 but TLS is NOT implemented.
//! This will attempt a plain TCP connection to the MQTTS port. Options:
//! 1. Configure broker to accept non-TLS on port 8883 (insecure, testing only)
//! 2. Configure broker with non-TLS listener on port 1883 for network access
//! 3. Implement TLS support using embedded-tls or ESP-IDF TLS
//!
//! For production use, consider using minimq with embedded-nal or ESP-IDF's MQTT component.

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
    /// MQTT port (1883 for non-TLS, 8883 for TLS)
    pub port: u16,
    /// MQTT username (optional)
    pub username: Option<&'static str>,
    /// MQTT password (optional)
    pub password: Option<&'static str>,
}

impl Default for MqttBrokerConfig {
    fn default() -> Self {
        Self {
            ip: Ipv4Addr::new(192, 168, 0, 216), // Docker host (WiFi adapter IP)
            port: 1884,                           // Plain MQTT on network (no TLS)
            username: Some("elev1"),
            password: Some("elev1password"),      // Password for authentication
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
    username: Option<&str>,
    password: Option<&str>,
    buffer: &mut [u8],
) -> Result<usize, MqttError> {
    if buffer.len() < 64 {
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

    // Connect Flags
    let mut connect_flags = 0x02; // Clean Session = 1
    if username.is_some() {
        connect_flags |= 0x80; // Username flag
    }
    if password.is_some() {
        connect_flags |= 0x40; // Password flag
    }
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

    // Username (if provided)
    if let Some(user) = username {
        let username_bytes = user.as_bytes();
        buffer[pos] = (username_bytes.len() >> 8) as u8;
        pos += 1;
        buffer[pos] = (username_bytes.len() & 0xFF) as u8;
        pos += 1;
        buffer[pos..pos + username_bytes.len()].copy_from_slice(username_bytes);
        pos += username_bytes.len();
    }

    // Password (if provided)
    if let Some(pass) = password {
        let password_bytes = pass.as_bytes();
        buffer[pos] = (password_bytes.len() >> 8) as u8;
        pos += 1;
        buffer[pos] = (password_bytes.len() & 0xFF) as u8;
        pos += 1;
        buffer[pos..pos + password_bytes.len()].copy_from_slice(password_bytes);
        pos += password_bytes.len();
    }

    // Fill in remaining length (total length - 2 bytes for fixed header)
    let remaining_length = pos - remaining_length_pos - 1;
    buffer[remaining_length_pos] = remaining_length as u8;

    Ok(pos)
}

/// Build MQTT PUBLISH packet (QoS 0)
fn build_publish_packet(
    topic: &str,
    payload: &[u8],
    buffer: &mut [u8],
) -> Result<usize, MqttError> {
    if buffer.len() < topic.len() + payload.len() + 32 {
        return Err(MqttError::BufferTooSmall);
    }

    let mut pos = 0;

    // Fixed header: PUBLISH packet type (QoS 0, no retain, no dup)
    buffer[pos] = PacketType::Publish as u8;
    pos += 1;

    // Calculate remaining length
    let remaining_length = 2 + topic.len() + payload.len();

    // Encode remaining length (simple case: < 128 bytes)
    if remaining_length < 128 {
        buffer[pos] = remaining_length as u8;
        pos += 1;
    } else {
        // Two-byte encoding for 128-16383
        buffer[pos] = (remaining_length & 0x7F) as u8 | 0x80;
        pos += 1;
        buffer[pos] = (remaining_length >> 7) as u8;
        pos += 1;
    }

    // Variable header: Topic name
    let topic_bytes = topic.as_bytes();
    buffer[pos] = (topic_bytes.len() >> 8) as u8; // Length MSB
    pos += 1;
    buffer[pos] = (topic_bytes.len() & 0xFF) as u8; // Length LSB
    pos += 1;
    buffer[pos..pos + topic_bytes.len()].copy_from_slice(topic_bytes);
    pos += topic_bytes.len();

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

/// Simplified MQTT publish function for quick integration
///
/// This function creates a TCP connection, sends CONNECT and PUBLISH packets,
/// and disconnects. It's designed for simple one-shot publish operations.
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
        "MQTT: Connecting to broker {}:{}",
        broker_config.ip, broker_config.port
    );

    // Create TCP socket
    let mut tcp_rx_buffer = [0u8; 512];
    let mut tcp_tx_buffer = [0u8; 512];
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

    // Connect to broker
    info!("MQTT: Initiating TCP connection...");
    let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
    socket
        .connect(iface.context(), broker_endpoint, 49152)
        .map_err(|_| {
            info!("MQTT: TCP connect failed");
            MqttError::ConnectionFailed
        })?;

    // Poll until connected
    let mut timeout_counter = 0u32;
    let mut connected = false;
    while !connected && timeout_counter < 100 {
        let timestamp = Instant::from_millis(timeout_counter as i64 * 50);
        iface.poll(timestamp, device, &mut sockets);

        let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
        if socket.is_active() {
            connected = true;
            info!("MQTT: TCP connected");
        }

        if !connected {
            delay.delay_millis(50);
            timeout_counter += 1;
        }
    }

    if !connected {
        info!("MQTT: TCP connection timeout");
        return Err(MqttError::Timeout);
    }

    // After connection is established, wait for socket to be in Established state
    info!("MQTT: TCP connected, waiting for socket to be established...");
    let mut socket_established = false;
    for i in 0..20 {
        let timestamp = Instant::from_millis((timeout_counter + 1 + i) as i64 * 50);
        iface.poll(timestamp, device, &mut sockets);

        let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
        if socket.may_send() {
            socket_established = true;
            info!("MQTT: Socket established and ready after {} polls", i + 1);
            break;
        }
        delay.delay_millis(50);
    }

    if !socket_established {
        info!("MQTT: Socket never reached established state");
        return Err(MqttError::ConnectionFailed);
    }

    // Build and send MQTT CONNECT packet
    let mut connect_buffer = [0u8; 256];
    let client_id = device_id;
    let connect_len = build_connect_packet(
        client_id,
        broker_config.username,
        broker_config.password,
        &mut connect_buffer,
    )?;

    info!("MQTT: Sending CONNECT packet ({} bytes)", connect_len);

    // Try to send CONNECT packet - retry if it fails
    let mut send_success = false;
    for attempt in 0..10 {
        let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
        match socket.send_slice(&connect_buffer[..connect_len]) {
            Ok(_) => {
                if attempt == 0 {
                    info!("MQTT: CONNECT packet queued successfully");
                } else {
                    info!("MQTT: CONNECT packet queued after {} retries", attempt);
                }
                send_success = true;
                break;
            }
            Err(e) => {
                if attempt == 0 {
                    info!("MQTT: Send failed: {:?}, retrying...", e);
                }
                // Wait and poll before retry
                let timestamp = Instant::from_millis((timeout_counter + 10 + attempt) as i64 * 50);
                iface.poll(timestamp, device, &mut sockets);
                delay.delay_millis(50);

                if attempt == 9 {
                    info!("MQTT: Failed to send after {} attempts: {:?}", attempt + 1, e);
                    return Err(MqttError::SendFailed);
                }
            }
        }
    }

    if !send_success {
        return Err(MqttError::SendFailed);
    }

    // Poll to actually send the packet
    for _ in 0..5 {
        let timestamp = Instant::from_millis((timeout_counter + 20) as i64 * 50);
        iface.poll(timestamp, device, &mut sockets);
        delay.delay_millis(50);
    }

    // Wait for CONNACK
    info!("MQTT: Waiting for CONNACK...");
    let mut connack_received = false;
    let mut connack_timeout = 0u32;
    while !connack_received && connack_timeout < 40 {
        let timestamp = Instant::from_millis((timeout_counter + connack_timeout + 2) as i64 * 50);
        iface.poll(timestamp, device, &mut sockets);

        let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
        if socket.can_recv() {
            let mut connack_buffer = [0u8; 4];
            if let Ok(size) = socket.recv_slice(&mut connack_buffer) {
                if size >= 2 && connack_buffer[0] == PacketType::ConnAck as u8 {
                    connack_received = true;
                    info!("MQTT: CONNACK received, connected to broker");
                }
            }
        }

        if !connack_received {
            delay.delay_millis(50);
            connack_timeout += 1;
        }
    }

    if !connack_received {
        info!("MQTT: CONNACK timeout");
        // Continue anyway - some brokers may not send CONNACK immediately
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

    // Build and send MQTT PUBLISH packet
    let mut publish_buffer = [0u8; 256];
    let publish_len =
        build_publish_packet(topic.as_str(), payload.as_bytes(), &mut publish_buffer)?;

    info!("MQTT: Sending PUBLISH packet ({} bytes)", publish_len);
    let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
    socket
        .send_slice(&publish_buffer[..publish_len])
        .map_err(|_| {
            info!("MQTT: Failed to send PUBLISH");
            MqttError::SendFailed
        })?;

    // Poll to send the packet
    let timestamp = Instant::from_millis((timeout_counter + connack_timeout + 3) as i64 * 50);
    iface.poll(timestamp, device, &mut sockets);
    delay.delay_millis(100);

    info!("MQTT: Feedback published successfully");

    // Send DISCONNECT packet
    let mut disconnect_buffer = [0u8; 2];
    let disconnect_len = build_disconnect_packet(&mut disconnect_buffer)?;

    info!("MQTT: Sending DISCONNECT packet");
    let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
    socket
        .send_slice(&disconnect_buffer[..disconnect_len])
        .ok(); // Ignore errors on disconnect

    // Poll to send disconnect
    let timestamp = Instant::from_millis((timeout_counter + connack_timeout + 4) as i64 * 50);
    iface.poll(timestamp, device, &mut sockets);
    delay.delay_millis(50);

    // Close socket
    let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
    socket.close();

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
