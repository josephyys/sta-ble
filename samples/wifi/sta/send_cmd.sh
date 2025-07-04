#!/bin/bash
# Usage: ./wifi_connect.sh /dev/ttyS1 "YourSSID" "YourPassword"

# Get parameters
PORT="$1"
SSID="$2"
PASSWORD="$3"

# Validate parameters
if [ -z "$PORT" ] || [ -z "$SSID" ] || [ -z "$PASSWORD" ]; then
    echo "Usage: $0 <port> <ssid> <password>"
    echo "Example: $0 /dev/ttyS1 MyWiFi MySecretPass"
    exit 1
fi

# Configure serial port (macOS uses -f, Linux uses -F)
if [[ "$OSTYPE" == "darwin"* ]]; then
    stty -f "$PORT" 115200 raw -echo
else
    stty -F "$PORT" 115200 raw -echo
fi

# Send WiFi connection command
echo -e "wifi_connect ssid=$SSID pw=$PASSWORD\r\n" > "$PORT"
echo "Command sent to $PORT"