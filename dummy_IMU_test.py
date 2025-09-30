#!/usr/bin/env python3
# RS-232 packet sender with START/STOP control (Windows)
# - Prompts for COM port
# - 115200 8-N-1
# - Can run continuously or send a fixed number of packets
# - Sends 10-byte test packets at 20 Hz after "START"
# - Halts on "STOP"

import sys
import time
import struct
import serial
import serial.tools.list_ports

BAUD = 115200
SEND_INTERVAL_S = 0.050  # 20 Hz

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
    return user  # let pyserial try it

def pick_mode():
    print("\nChoose transmit mode:")
    print("  1) Continuous until STOP or Ctrl+C")
    print("  2) Fixed number of messages")
    choice = input("Enter 1 or 2: ").strip()
    if choice == "2":
        n = input("How many packets to send? ").strip()
        try:
            return int(n)
        except ValueError:
            print("Invalid number, defaulting to continuous mode.")
            return None
    return None  # None = continuous

def build_packet(packet_number):
    header = bytes([0xB4, 0xB4])
    body = b'<IMU>' + struct.pack('<I', packet_number)  # little-endian uint32
    return header + body

def hex_str(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def main():
    port = pick_com_port()
    count_limit = pick_mode()

    try:
        ser = serial.Serial(
            port=port,
            baudrate=BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.0,
            write_timeout=1.0,
        )
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        sys.exit(1)

    pkt = build_packet(1)  # Example packet for display
    print(f"\nOpened {ser.port} @ {BAUD} 8N1")
    print(f"Packet (11 bytes): {hex_str(pkt)}")
    if count_limit:
        print(f"Mode: fixed count ({count_limit} packets)")
    else:
        print("Mode: continuous until STOP or Ctrl+C")

    print("\nWaiting for START (send ASCII 'START' from peer)...")

    started = False
    cmd_buf = ""
    last_send = time.monotonic()
    sent_count = 0
    next_send = 0

    try:
        while True:
            # --- read inbound commands ---
            data = ser.read(1024)
            if data:
                try:
                    text = data.decode("ascii", errors="ignore")
                except Exception:
                    text = ""
                for ch in text:
                    if ch.isspace():
                        if cmd_buf:
                            token = cmd_buf.strip().upper()
                            if token == "START":
                                started = True
                                sent_count = 0
                                next_send = time.monotonic()
                                print("[CMD] START → transmitting")
                            elif token == "STOP":
                                started = False
                                print(f"Total packets sent: {sent_count}")
                                print("[CMD] STOP → halted")
                            cmd_buf = ""
                    else:
                        cmd_buf += ch
                        if len(cmd_buf) > 64:
                            cmd_buf = cmd_buf[-64:]
                        # Check for complete commands after appending
                        if cmd_buf.upper() == "START":
                            started = True
                            sent_count = 0
                            next_send = time.monotonic()
                            print("[CMD] START → transmitting")
                            cmd_buf = ""
                        elif cmd_buf.upper() == "STOP":
                            started = False
                            print(f"Total packets sent: {sent_count}")
                            print("[CMD] STOP → halted")
                            cmd_buf = ""

            # --- transmit ---
            now = time.monotonic()
            if started and now >= next_send:
                pkt = build_packet(sent_count + 1)
                ser.write(pkt)
                ser.flush()
                sent_count += 1
                next_send += SEND_INTERVAL_S
                if count_limit and sent_count >= count_limit:
                    print(f"Sent {sent_count} packets → done.")
                    break

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
