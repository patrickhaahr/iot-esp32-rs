extern crate alloc;
use alloc::vec;
use alloc::vec::Vec;
use embedded_io::{ErrorKind, Read, Write};
use esp_mbedtls::{Certificates, Mode, Session, TlsReference, TlsVersion};
use esp_wifi::wifi::WifiDevice;
use log::info;
use serde::Serialize;
use smoltcp::iface::{Interface, SocketSet};
use smoltcp::socket::tcp;
use smoltcp::time::Instant;
use smoltcp::wire::{IpAddress, IpEndpoint, Ipv4Address};

// Load MQTT credentials and broker IP from .env file at compile time
const MQTT_BROKER_IP: &str = env!("MQTT_BROKER_IP");
const MQTT_USERNAME: &str = env!("MQTT_USERNAME");
const MQTT_PASSWORD: &str = env!("MQTT_PASSWORD");

/// MQTT message payload structure
#[derive(Serialize)]
struct ButtonFeedback<'a> {
    button_id: u8,
    emotion: &'a str,
    timestamp: u64,
    datetime: &'a str,
}

/// Map button ID to emotion
/// Button 1 (GPIO32) → "very_happy"
/// Button 2 (GPIO27) → "happy"
/// Button 3 (GPIO26) → "neutral"
/// Button 4 (GPIO25) → "sad"
fn button_to_emotion(button_id: u8) -> &'static str {
    match button_id {
        1 => "very_happy",
        2 => "happy",
        3 => "neutral",
        4 => "sad",
        _ => "unknown",
    }
}

pub struct NetworkStream<'b, 'a, 'd> {
    iface: &'b mut Interface,
    device: &'b mut WifiDevice<'d>,
    sockets: &'b mut SocketSet<'a>,
    handle: smoltcp::iface::SocketHandle,
    poll_at: u64, // Milliseconds
}

impl<'b, 'a, 'd> NetworkStream<'b, 'a, 'd> {
    pub fn new(
        iface: &'b mut Interface,
        device: &'b mut WifiDevice<'d>,
        sockets: &'b mut SocketSet<'a>,
        handle: smoltcp::iface::SocketHandle,
    ) -> Self {
        Self {
            iface,
            device,
            sockets,
            handle,
            poll_at: 0,
        }
    }

    fn poll(&mut self) {
        let timestamp = Instant::from_millis(self.poll_at as i64);
        self.iface.poll(timestamp, self.device, self.sockets);
        self.poll_at = self.poll_at.wrapping_add(10);
    }
}

impl<'b, 'a, 'd> embedded_io::ErrorType for NetworkStream<'b, 'a, 'd> {
    type Error = ErrorKind;
}

impl<'b, 'a, 'd> Read for NetworkStream<'b, 'a, 'd> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            self.poll();
            let socket = self.sockets.get_mut::<tcp::Socket>(self.handle);

            if !socket.is_active() {
                return Err(ErrorKind::NotConnected);
            }

            if socket.can_recv() {
                match socket.recv_slice(buf) {
                    Ok(len) => return Ok(len),
                    Err(_) => return Err(ErrorKind::Other),
                }
            }
        }
    }
}

impl<'b, 'a, 'd> Write for NetworkStream<'b, 'a, 'd> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        loop {
            self.poll();
            let socket = self.sockets.get_mut::<tcp::Socket>(self.handle);

            if !socket.is_active() {
                return Err(ErrorKind::NotConnected);
            }

            if socket.can_send() {
                match socket.send_slice(buf) {
                    Ok(len) => return Ok(len),
                    Err(_) => return Err(ErrorKind::Other),
                }
            }
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub fn publish_button_feedback<'a, 'd>(
    iface: &mut Interface,
    device: &mut WifiDevice<'d>,
    sockets: &mut SocketSet<'a>,
    button_id: u8,
    timestamp: u64,
    client_id: &str,
    tls_ref: TlsReference<'_>,
) -> Result<(), &'static str> {
    info!(
        "MQTT: Preparing to publish feedback for button {}",
        button_id
    );

    // 1. Connect TCP
    let rx_buffer = tcp::SocketBuffer::new(vec![0; 4096]);
    let tx_buffer = tcp::SocketBuffer::new(vec![0; 4096]);
    let tcp_socket = tcp::Socket::new(rx_buffer, tx_buffer);
    let handle = sockets.add(tcp_socket);

    // Parse MQTT_BROKER_IP from .env at runtime (it's a &str constant)
    let ip_parts: Vec<&str> = MQTT_BROKER_IP.split('.').collect();
    let remote_addr = Ipv4Address::new(
        ip_parts[0].parse::<u8>().unwrap_or(192),
        ip_parts[1].parse::<u8>().unwrap_or(168),
        ip_parts[2].parse::<u8>().unwrap_or(8),
        ip_parts[3].parse::<u8>().unwrap_or(222),
    );
    let remote_endpoint = IpEndpoint::new(IpAddress::Ipv4(remote_addr), 8883);

    info!("MQTT: Connecting TCP to {}...", remote_endpoint);
    {
        let socket = sockets.get_mut::<tcp::Socket>(handle);
        socket
            .connect(iface.context(), remote_endpoint, 45678)
            .map_err(|_| "TCP Connect failed")?;
    }

    // Wait for TCP connection
    let mut connection_result = Err("TCP Connection timeout");

    {
        let mut stream = NetworkStream::new(iface, device, sockets, handle);
        let mut connected = false;
        for _ in 0..1000 {
            // 10 second timeout approx
            stream.poll();
            let socket = stream.sockets.get_mut::<tcp::Socket>(stream.handle);
            if socket.state() == tcp::State::Established {
                connected = true;
                break;
            }
            esp_hal::delay::Delay::new().delay_millis(10);
        }

        if connected {
            info!("MQTT: TCP Connected!");
            // 2. Setup TLS (Client with CA)
            let mut certificates = Certificates::new();

            // Use the embedded CA certificate
            // We MUST include the null terminator as required by esp-mbedtls X509::pem
            // The file read tool showed it ends with newline, so we append \0
            const CA_CERT: &[u8] =
                concat!(include_str!("../mosquitto/config/certs/ca.crt"), "\0").as_bytes();

            // Parse the CA certificate
            let ca = esp_mbedtls::X509::pem(CA_CERT).unwrap();
            certificates.ca_chain = Some(ca);

            match Session::new(
                stream,
                Mode::Client { servername: c"" }, // Empty hostname to skip CN check, but verify signature
                TlsVersion::Tls1_3,
                certificates,
                tls_ref,
            ) {
                Ok(mut session) => {
                    info!("MQTT: Performing TLS Handshake...");
                    if let Err(e) = session.connect() {
                        info!("TLS Handshake Error: {:?}", e);
                        connection_result = Err("TLS Handshake Failed");
                    } else {
                        info!("MQTT: TLS Handshake Successful!");
                        connection_result =
                            perform_mqtt_actions(&mut session, client_id, button_id, timestamp);
                    }
                }
                Err(e) => {
                    info!("MQTT: TLS Session Init Failed: {:?}", e);
                    connection_result = Err("TLS Session Init Failed");
                }
            }
        }
    }

    // Clean up socket
    sockets.remove(handle);

    connection_result
}

fn perform_mqtt_actions<T: Read + Write>(
    session: &mut Session<'_, T>,
    client_id: &str,
    button_id: u8,
    timestamp: u64,
) -> Result<(), &'static str> {
    // 3. MQTT Connect
    let client_id_bytes = client_id.as_bytes();
    let username = MQTT_USERNAME.as_bytes();
    let password = MQTT_PASSWORD.as_bytes();

    let mut packet = vec![];
    packet.extend_from_slice(&[0x00, 0x04, b'M', b'Q', b'T', b'T', 0x04, 0xC2, 0x00, 0x3C]);

    packet.push((client_id_bytes.len() >> 8) as u8);
    packet.push((client_id_bytes.len() & 0xFF) as u8);
    packet.extend_from_slice(client_id_bytes);

    packet.push((username.len() >> 8) as u8);
    packet.push((username.len() & 0xFF) as u8);
    packet.extend_from_slice(username);

    packet.push((password.len() >> 8) as u8);
    packet.push((password.len() & 0xFF) as u8);
    packet.extend_from_slice(password);

    let mut connect_packet = vec![0x10];
    encode_remaining_length(packet.len(), &mut connect_packet);
    connect_packet.extend_from_slice(&packet);

    info!("MQTT: Sending CONNECT...");
    session.write(&connect_packet).map_err(|_| "Write Error")?;

    let mut buf = [0u8; 4];
    session.read(&mut buf).map_err(|_| "Read Error")?;

    if buf[0] != 0x20 || buf[3] != 0x00 {
        info!("MQTT: CONNACK Failed: {:?}", buf);
        return Err("MQTT Connection Refused");
    }
    info!("MQTT: Connected to Broker!");

    // 4. Publish
    let topic = b"esp32/feedback";

    // Determine if we're in summer time (approximate check - use current month)
    // For simplicity, we'll assume winter time (CET). In production, you'd calculate the actual month.
    let is_summer_time = false; // TODO: Calculate based on actual date

    // Format datetime string
    let datetime_str = crate::ntp::format_datetime(timestamp, is_summer_time);

    // Get emotion for this button
    let emotion = button_to_emotion(button_id);

    // Create JSON payload
    let feedback = ButtonFeedback {
        button_id,
        emotion,
        timestamp,
        datetime: datetime_str.as_str(),
    };

    // Serialize to JSON using a heapless buffer
    let json_buffer: heapless::Vec<u8, 256> =
        serde_json_core::to_vec(&feedback).map_err(|_| "JSON serialization failed")?;

    let payload = json_buffer.as_slice();

    let mut var_header_payload = vec![];
    var_header_payload.push((topic.len() >> 8) as u8);
    var_header_payload.push((topic.len() & 0xFF) as u8);
    var_header_payload.extend_from_slice(topic);
    var_header_payload.extend_from_slice(payload);

    let mut publish_packet = vec![0x30];
    encode_remaining_length(var_header_payload.len(), &mut publish_packet);
    publish_packet.extend_from_slice(&var_header_payload);

    info!("MQTT: Publishing message...");
    session.write(&publish_packet).map_err(|_| "Write Error")?;
    info!(
        "MQTT: Message Published: {}",
        core::str::from_utf8(payload).unwrap_or("(invalid utf8)")
    );

    let disconnect_packet = [0xE0, 0x00];
    let _ = session.write(&disconnect_packet);

    Ok(())
}

fn encode_remaining_length(mut len: usize, buf: &mut Vec<u8>) {
    loop {
        let mut byte = (len % 128) as u8;
        len /= 128;
        if len > 0 {
            byte |= 0x80;
        }
        buf.push(byte);
        if len == 0 {
            break;
        }
    }
}
