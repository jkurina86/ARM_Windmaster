#!/usr/bin/env python3
"""
Test script for Telus UART4 communication.
Sends 'idata\r\n' command and logs the response.
"""

import serial
import time
import sys

PORT = "COM6"
BAUD = 115200
TIMEOUT = 2.0

def main():
    print(f"Opening {PORT} at {BAUD} baud...")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
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
