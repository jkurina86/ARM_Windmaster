#!/usr/bin/env python3
"""
Periodic Telus data collection script.
Sends 'idata\r\n' command every 10 minutes and appends data to reports.csv.

Cross-platform: works on Windows (COM ports) and Linux (/dev/ttyUSB*).

Usage:
    python test_telus_periodic.py              # Interactive port selection menu
    python test_telus_periodic.py COM6         # Use specific port (Windows)
    python test_telus_periodic.py /dev/ttyUSB0 # Use specific port (Linux)
"""

import serial
import time
import sys
import os
from datetime import datetime
from serial_utils import get_port

BAUD = 115200
TIMEOUT = 5.0
INTERVAL_MINUTES = 10
OUTPUT_FILE = "reports.csv"


def send_command(ser: serial.Serial) -> str:
    """Send idata command and return response."""
    ser.reset_input_buffer()

    cmd = b"idata\r\n"
    ser.write(cmd)
    ser.flush()

    # Wait for response to start
    time.sleep(0.5)

    # Read response
    response = ser.read(ser.in_waiting or 1)

    # Keep reading until no more data
    while True:
        time.sleep(0.1)
        more = ser.read(ser.in_waiting)
        if not more:
            break
        response += more

    return response.decode('ascii', errors='replace')


def process_response(response: str, first_run: bool) -> int:
    """
    Process CSV response and write to file.
    Returns number of data rows written.
    """
    lines = response.strip().split('\n')
    if not lines:
        return 0

    # First line should be header
    header = lines[0].strip()
    data_lines = [line.strip() for line in lines[1:] if line.strip()]

    if first_run:
        # Create new file with header and data
        with open(OUTPUT_FILE, 'w') as f:
            f.write(header + '\n')
            for line in data_lines:
                f.write(line + '\n')
    else:
        # Append only data rows
        with open(OUTPUT_FILE, 'a') as f:
            for line in data_lines:
                f.write(line + '\n')

    return len(data_lines)


def main():
    # Get port from command line or interactive menu
    port_arg = sys.argv[1] if len(sys.argv) > 1 else None
    port = get_port(port_arg)

    print(f"\nTelus Periodic Data Collection")
    print(f"Port: {port}, Baud: {BAUD}")
    print(f"Interval: {INTERVAL_MINUTES} minutes")
    print(f"Output: {OUTPUT_FILE}")
    print("-" * 40)

    # Check if output file exists (determines if this is first run)
    first_run = not os.path.exists(OUTPUT_FILE)

    try:
        ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        sys.exit(1)

    print(f"Connected to {port}")
    print("Press Ctrl+C to stop\n")

    try:
        while True:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            print(f"[{timestamp}] Sending idata command...")

            response = send_command(ser)

            if response:
                rows = process_response(response, first_run)
                print(f"[{timestamp}] Received {len(response)} bytes, wrote {rows} data rows")
                first_run = False  # After first successful write, always append
            else:
                print(f"[{timestamp}] No response received")

            # Wait for next interval
            print(f"[{timestamp}] Next collection in {INTERVAL_MINUTES} minutes...\n")
            time.sleep(INTERVAL_MINUTES * 60)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ser.close()
        print("Done.")


if __name__ == "__main__":
    main()
