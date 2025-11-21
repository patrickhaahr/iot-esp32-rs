#!/usr/bin/env bash

# Load MQTT_BROKER_IP from .env file
if [ -f ".env" ]; then
  export $(grep "^MQTT_BROKER_IP" .env | xargs)
else
  echo "Error: .env file not found. Please create .env with MQTT_BROKER_IP."
  exit 1
fi

# Default IP if not set
MQTT_BROKER_IP="${MQTT_BROKER_IP}"

mkdir -p mosquitto/certs
cd mosquitto/certs

# Generate CA (we act as our own CA)
openssl req -new -x509 -days 3650 -extensions v3_ca -keyout ca.key -out ca.crt -nodes -subj "/CN=MyLocalCA"

# Generate Server Key
openssl genrsa -out server.key 2048

# Generate Server CSR
openssl req -out server.csr -key server.key -new -nodes -subj "/CN=$MQTT_BROKER_IP"

# Sign the CSR with our CA
openssl x509 -req -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out server.crt -days 3650

# Cleanup
rm server.csr ca.key ca.srl
# We keep ca.crt just in case, but server.crt/key are what mosquitto needs
