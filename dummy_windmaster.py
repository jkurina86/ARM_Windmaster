#!/usr/bin/env python3
# WindMaster Dummy Sensor (Windows)
# - Generates WM_Packet_t structures matching windmaster.h
# - 115200 8-N-1
# - 23-byte packets at 20 Hz after "START"
# - Halts on "STOP"

import sys
import time
import struct
import serial
import serial.tools.list_ports
import math

BAUD = 115200
SEND_INTERVAL_S = 0.050  # 20 Hz
PACKET_SIZE = 23  # WM_Packet_t size

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
    """
    Build a WM_Packet_t structure (23 bytes total):
    
    typedef struct __attribute__((packed)) {
      uint16_t header;        // 0xB4B4 (little-endian)
      uint16_t status;        // Status word
      int16_t U_axis_speed;   // U-axis wind speed
      int16_t V_axis_speed;   // V-axis wind speed
      int16_t W_axis_speed;   // W-axis wind speed
      int16_t SoS;            // Speed of Sound
      int16_t A1;             // Analog Input 1
      int16_t A2;             // Analog Input 2
      int16_t A3;             // Analog Input 3
      int16_t A4;             // Analog Input 4
      int16_t Temp;           // Temperature from PRT
      uint8_t checksum;       // XOR checksum
    } WM_Packet_t;
    """
    
    # Generate some varying dummy data
    t = packet_number * SEND_INTERVAL_S
    
    # Header (constant)
    header = 0xB4B4
    
    # Status word (0 = OK, non-zero = errors)
    status = 0x0000
    
    # Wind speeds (units of 0.01 m/s)
    U_axis_speed = int(500 + 300 * math.sin(t * 0.5))
    V_axis_speed = int(200 + 150 * math.cos(t * 0.4))
    W_axis_speed = int(50 + 30 * math.sin(t * 0.3))
    
    # Speed of Sound (units of 0.01 m/s)
    # Note: This is scaled to fit int16_t range
    SoS = int(3400 + 100 * math.sin(t * 0.1))
    
    # Analog inputs (arbitrary)
    A1 = int(1000 + 100 * math.sin(t * 0.6))
    A2 = int(2000 + 200 * math.cos(t * 0.7))
    A3 = int(1500 + 150 * math.sin(t * 0.8))
    A4 = int(2500 + 250 * math.cos(t * 0.9))
    
    # Temperature from PRT (0.01°C units)
    Temp = int(1500 + 50 * math.sin(t * 0.05))
    
    # Pack the packet without checksum (little-endian)
    # Format: H=uint16, h=int16
    packet_without_checksum = struct.pack(
        '<HHhhhhhhhhh',
        header,         # uint16_t
        status,         # uint16_t
        U_axis_speed,   # int16_t
        V_axis_speed,   # int16_t
        W_axis_speed,   # int16_t
        SoS,            # int16_t
        A1,             # int16_t
        A2,             # int16_t
        A3,             # int16_t
        A4,             # int16_t
        Temp            # int16_t
    )
    
    # Calculate XOR checksum of bytes BETWEEN header and checksum (bytes 2-21, excluding header)
    checksum = 0
    for b in packet_without_checksum[2:]:  # Skip the 2-byte header
        checksum ^= b
    
    # Combine packet with checksum
    packet = packet_without_checksum + bytes([checksum])
    
    return packet

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
    print(f"WindMaster Simulator")
    print(f"Packet size: {len(pkt)} bytes (expected: {PACKET_SIZE})")
    print(f"Packet hex: {hex_str(pkt)}")
    if count_limit:
        print(f"Mode: fixed count ({count_limit} packets)")
    else:
        print("Mode: continuous until STOP or Ctrl+C")

    print("\nWaiting for START (send ASCII 'START' from peer)...")
    print("Packet structure:")
    print("  - header: 0xB4B4")
    print("  - Wind UVW: varying between realistic values (units: 0.01 m/s)")
    print("  - SoS: ~34 m/s (units: 0.01 m/s)")
    print("  - Temp: ~15°C (units: 0.01°C)")
    print("  - Rate: 20 Hz (50ms interval)")
    print()

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
                
                # Show progress every 100 packets
                if sent_count % 100 == 0:
                    elapsed = now - (next_send - sent_count * SEND_INTERVAL_S)
                    actual_rate = sent_count / elapsed if elapsed > 0 else 0
                    print(f"[TX] {sent_count} packets sent (avg rate: {actual_rate:.1f} Hz)")
                
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
