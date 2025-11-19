# IoT ESP32 Rust Project

![Fritzing Circuit Diagram](docs/fritzing.png)

A Rust-based embedded feedback panel for the ESP32 using `esp-hal`. This project implements a smiley feedback system with 4 buttons and corresponding LEDs that publishes ratings to an MQTT broker.

## Features

- **4 Buttons with 4 LEDs** - Smiley feedback panel with ratings:
  - Button 1: Very Happy (Green LED)
  - Button 2: Happy (Yellow LED)
  - Button 3: Neutral (Blue LED)
  - Button 4: Sad (Red LED)
- **Wi-Fi Connectivity** - Connects to Wi-Fi using WPA2 authentication with credentials stored in `.env` file
- **MQTT with Authentication** - Publishes button press events with JSON payloads including timestamps and ratings
- **NTP Time Synchronization** - Syncs with Denmark NTP server (`dk.pool.ntp.org`) with Copenhagen timezone support
- **Button Debouncing** - Software debounce (50ms) prevents false triggers from button bounce
- **Deep Sleep Mode** - Power-saving deep sleep after 10 seconds of inactivity with EXT1 GPIO wakeup
- **LED Feedback** - Visual confirmation with 500ms LED display on button press
- **RTC GPIO Wake** - Uses RTC-capable pins (GPIO25, GPIO26, GPIO27, GPIO32) for deep sleep wakeup

## Development

### Wi-Fi Configuration

Copy the example environment file and configure your Wi-Fi credentials:

```sh
cp .env.example .env
```

Edit `.env` with your Wi-Fi network details:

```
WIFI_SSID=your_wifi_network_name
WIFI_PASSWORD=your_wifi_password
```

### Build and Flash

To build, upload the code, and monitor the ESP32 serial output:

```sh
cargo run --release
```

## MQTT Broker Setup

The project includes a local MQTT broker (Mosquitto) with TLS support using Docker.

### Prerequisites

- [Docker Engine](https://docs.docker.com/engine/install/) installed

### Generate TLS Certificates

Generate self-signed certificates for secure MQTT communication:

```sh
cd mosquitto/certs
./generate-certs.sh
cd ../..
```

This creates:
- `ca.crt` - CA certificate (needed by ESP32 client)
- `server.crt` / `server.key` - Server certificate and key

### Create MQTT User

Create a password file with your MQTT credentials:

```sh
docker run -it --rm -v ./mosquitto/config:/mosquitto/config eclipse-mosquitto mosquitto_passwd -c /mosquitto/config/passwd your_username
```

Replace `your_username` with your desired username. You'll be prompted to enter a password.

**Important:** Update your `.env` file with the same MQTT credentials:

```
MQTT_USERNAME=your_username
MQTT_PASSWORD=your_password
```

To add additional users (without `-c` flag to avoid overwriting):

```sh
docker run -it --rm -v ./mosquitto/config:/mosquitto/config eclipse-mosquitto mosquitto_passwd /mosquitto/config/passwd another_user
```

### Start the MQTT Broker

From the project root directory:

```sh
docker compose up -d
```

The broker will be available on:
- Port `8883` - MQTTS (TLS)
- Port `1883` - Plain MQTT (localhost only, for testing)

### Testing the Connection

**Terminal 1 - Subscribe to a topic:**
```sh
mosquitto_sub -h localhost -p 8883 --cafile mosquitto/certs/ca.crt --insecure -u your_username -P your_password -t "test"
```

**Terminal 2 - Publish a message:**
```sh
mosquitto_pub -h localhost -p 8883 --cafile mosquitto/certs/ca.crt --insecure -u your_username -P your_password -t "test" -m "hello"
```

Replace `your_username` and `your_password` with the credentials you created.

You should see "hello" appear in Terminal 1.

### View Broker Logs

```sh
docker compose logs -f
```

## Documentation

The project documentation is written in [Typst](https://typst.app/).

### Building Documentation

Ensure you have `typst` installed.

To compile the logbook or portfolio:

```sh
# Compile a specific file
typst compile docs/logbog.typst
typst compile docs/arbejds-portfolio.typst
```

### Watch Mode

To automatically recompile when the file changes:

```sh
typst watch docs/logbog.typst docs/logbog.pdf
```
