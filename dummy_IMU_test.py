#!/usr/bin/env python3
# VectorNav IMU Dummy Sensor (Windows)
# - Generates VN_Packet_t structures matching vectornav.h
# - 115200 8-N-1
# - 86-byte packets at 20 Hz after "START"
# - Halts on "STOP"

import sys
import time
import struct
import serial
import serial.tools.list_ports
import math

BAUD = 115200
SEND_INTERVAL_S = 0.050  # 20 Hz
PACKET_SIZE = 86  # VN_Packet_t size

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
    Build a VN_Packet_t structure (86 bytes total):
    
    typedef struct __attribute__((packed)) {
      uint8_t sync;            // 0xFA
      uint8_t group;           // 0x01
      uint16_t types_common;   // 0x0000
      uint64_t timegps;        // nanoseconds since 1980
      float yaw, pitch, roll;
      float gyro_x, gyro_y, gyro_z;
      double latitude, longitude, altitude;
      float vel_n, vel_e, vel_d;
      float acc_x, acc_y, acc_z;
      uint16_t checksum;       // big-endian
    } VN_Packet_t;
    """
    
    # Generate some varying dummy data
    t = packet_number * SEND_INTERVAL_S
    
    # Header
    sync = 0xFA
    group = 0x01
    types_common = 0x0000
    
    # Timestamp: nanoseconds since GPS epoch (1980-01-06)
    # Use packet number to simulate time progression
    timegps = int(1.3e18 + packet_number * 50e6)  # ~41 years + packet interval
    
    # Attitude (degrees) - simulate gentle oscillation
    yaw = 45.0 + 10.0 * math.sin(t * 0.5)
    pitch = 5.0 + 3.0 * math.sin(t * 0.3)
    roll = 2.0 + 2.0 * math.cos(t * 0.4)
    
    # Gyro rates (deg/s)
    gyro_x = 0.1 * math.sin(t * 0.7)
    gyro_y = 0.05 * math.cos(t * 0.6)
    gyro_z = 0.02 * math.sin(t * 0.5)
    
    # Position (example: somewhere in the middle of the US)
    latitude = 39.8283 + 0.0001 * math.sin(t * 0.1)
    longitude = -98.5795 + 0.0001 * math.cos(t * 0.1)
    altitude = 500.0 + 10.0 * math.sin(t * 0.2)
    
    # Velocity (m/s) - NED frame
    vel_n = 2.0 + 0.5 * math.sin(t * 0.3)
    vel_e = 1.5 + 0.5 * math.cos(t * 0.4)
    vel_d = -0.1 * math.sin(t * 0.2)
    
    # Acceleration (m/s^2)
    acc_x = 0.05 * math.sin(t * 1.0)
    acc_y = 0.03 * math.cos(t * 1.2)
    acc_z = 9.81 + 0.02 * math.sin(t * 0.8)  # Gravity + small variation
    
    # Pack the packet (little-endian except checksum)
    # Format: B=uint8, H=uint16, Q=uint64, f=float, d=double
    packet_without_checksum = struct.pack(
        '<BBHQffffffdddffffff',
        sync,           # uint8_t
        group,          # uint8_t
        types_common,   # uint16_t (LE)
        timegps,        # uint64_t (LE)
        yaw,            # float (LE)
        pitch,          # float (LE)
        roll,           # float (LE)
        gyro_x,         # float (LE)
        gyro_y,         # float (LE)
        gyro_z,         # float (LE)
        latitude,       # double (LE)
        longitude,      # double (LE)
        altitude,       # double (LE)
        vel_n,          # float (LE)
        vel_e,          # float (LE)
        vel_d,          # float (LE)
        acc_x,          # float (LE)
        acc_y,          # float (LE)
        acc_z           # float (LE)
    )
    
    # Calculate checksum (sum of all bytes including sync, big-endian format)
    # Note: C code sums from index 0 to PACKET_SIZE-2 (all bytes except checksum itself)
    checksum = sum(packet_without_checksum) & 0xFFFF
    
    # Pack checksum as big-endian uint16
    checksum_bytes = struct.pack('>H', checksum)
    
    # Combine
    packet = packet_without_checksum + checksum_bytes
    
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
    print(f"VectorNav IMU Simulator")
    print(f"Packet size: {len(pkt)} bytes (expected: {PACKET_SIZE})")
    print(f"First packet hex (first 32 bytes): {hex_str(pkt[:32])}")
    if count_limit:
        print(f"Mode: fixed count ({count_limit} packets)")
    else:
        print("Mode: continuous until STOP or Ctrl+C")

    print("\nWaiting for START (send ASCII 'START' from peer)...")
    print("Packet structure:")
    print("  - sync: 0xFA")
    print("  - YPR: varying between reasonable values")
    print("  - GPS: lat ~39.8°N, lon ~98.6°W, alt ~500m")
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
