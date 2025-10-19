#!/usr/bin/env python3
# Simple UART echo script - displays everything received
# Use this to debug if the STM32 is transmitting

import sys
import serial
import serial.tools.list_ports

BAUD = 115200

def pick_com_port():
    ports = list(serial.tools.list_ports.comports())
    print("Available serial ports:")
    if ports:
        for i, p in enumerate(ports, 1):
            desc = f" ({p.description})" if p.description else ""
            print(f"  {i}) {p.device}{desc}")
    else:
        print("  (none detected)")

    user = input("Enter COM port (e.g., COM3) or a number from the list: ").strip()
    if user.upper().startswith("COM"):
        return user
    try:
        idx = int(user)
        if 1 <= idx <= len(ports):
            return ports[idx - 1].device
    except ValueError:
        pass
    return user

def main():
    port = pick_com_port()

    try:
        ser = serial.Serial(
            port=port,
            baudrate=BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        sys.exit(1)

    print(f"\nOpened {ser.port} @ {BAUD} 8N1")
    print("Listening for data (Ctrl+C to exit)...")
    print("=" * 60)

    try:
        while True:
            data = ser.read(1024)
            if data:
                # Try to decode as ASCII
                try:
                    text = data.decode('ascii', errors='replace')
                    print(f"[ASCII] {repr(text)}")
                except:
                    pass
                
                # Also show hex
                hex_str = " ".join(f"{b:02X}" for b in data)
                print(f"[HEX]   {hex_str}")
                print()

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
