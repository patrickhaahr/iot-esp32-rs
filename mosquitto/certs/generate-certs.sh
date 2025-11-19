#!/bin/bash
# Generate self-signed certificates for Mosquitto MQTT broker
# Run this script from the mosquitto/certs directory

set -e

DAYS_VALID=365
CA_SUBJECT="/CN=MQTT-CA"
SERVER_SUBJECT="/CN=mqtt-server"

echo "Generating CA key and certificate..."
openssl genrsa -out ca.key 2048
openssl req -x509 -new -nodes -key ca.key -sha256 -days $DAYS_VALID -out ca.crt -subj "$CA_SUBJECT"

echo "Generating server key and CSR..."
openssl genrsa -out server.key 2048
openssl req -new -key server.key -out server.csr -subj "$SERVER_SUBJECT"

echo "Signing server certificate with CA..."
openssl x509 -req -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out server.crt -days $DAYS_VALID -sha256

echo "Cleaning up CSR..."
rm server.csr

echo "Setting permissions..."
chmod 644 ca.crt server.crt
chmod 600 ca.key server.key

echo ""
echo "Certificates generated successfully!"
echo "  - ca.crt     : CA certificate (needed by ESP32 client)"
echo "  - ca.key     : CA private key (keep secure)"
echo "  - server.crt : Server certificate"
echo "  - server.key : Server private key"
echo ""
echo "Copy ca.crt to your ESP32 project for TLS verification."
