#!/usr/bin/env python3
"""
WiFi Connection Script for nRF7002DK via serial port
Usage: python wifi_connect.py /dev/ttyS1 MyWiFi MySecretPass
"""

import serial
import time
import argparse
import sys

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Connect nRF7002DK to WiFi via serial')
    parser.add_argument('port', help='Serial port (e.g., /dev/ttyS1)')
    parser.add_argument('ssid', help='WiFi SSID')
    parser.add_argument('password', help='WiFi password')
    parser.add_argument('--delay', type=float, default=0.01, 
                        help='Delay between characters in seconds (default: 0.01)')
    args = parser.parse_args()

    try:
        # Open serial port
        ser = serial.Serial(args.port, 115200, timeout=1)
        print(f"Connected to {args.port}")
        
        # Construct command
        command = f"wifi_connect ssid={args.ssid} pw={args.password}\r\n"
        print(f"Sending command: {command.strip()}")
        
        # Send character by character with delay to prevent buffer overflow
        for char in command:
            ser.write(char.encode())
            time.sleep(args.delay)
        
        # Wait for response
        time.sleep(1)
        response = ser.read(ser.in_waiting or 1)
        print(f"Response: {response.decode('utf-8', errors='ignore')}")
        
        ser.close()
        print("Connection closed")
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()