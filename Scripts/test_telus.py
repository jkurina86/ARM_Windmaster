#!/usr/bin/env python3
"""
Test script for Telus UART4 communication.
Sends 'idata\r\n' command and logs the response.

Cross-platform: works on Windows (COM ports) and Linux (/dev/ttyUSB*).

Usage:
    python test_telus.py              # Interactive port selection menu
    python test_telus.py COM6         # Use specific port (Windows)
    python test_telus.py /dev/ttyUSB0 # Use specific port (Linux)
"""

import serial
import time
import sys
from serial_utils import get_port

BAUD = 115200
TIMEOUT = 2.0


def main():
    # Get port from command line or interactive menu
    port_arg = sys.argv[1] if len(sys.argv) > 1 else None
    port = get_port(port_arg)

    print(f"\nOpening {port} at {BAUD} baud...")

    try:
        ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        sys.exit(1)

    # Clear any pending data
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.1)

    # Send command
    cmd = b"idata\r\n"
    print(f"Sending: {cmd}")
    ser.write(cmd)
    ser.flush()

    # Wait for response
    print(f"Waiting for response (timeout={TIMEOUT}s)...")
    time.sleep(0.5)

    # Read response
    response = ser.read(ser.in_waiting or 1)

    # Keep reading if more data available
    while True:
        time.sleep(0.1)
        more = ser.read(ser.in_waiting)
        if not more:
            break
        response += more

    if response:
        print(f"\nReceived {len(response)} bytes:")
        print("-" * 40)
        try:
            print(response.decode('ascii'))
        except:
            print(response)
        print("-" * 40)
    else:
        print("\nNo response received!")

    ser.close()
    print("Done.")


if __name__ == "__main__":
    main()
