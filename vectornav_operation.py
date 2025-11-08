#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VectorNav Operation Script
Controls VectorNav GPS/IMU sensor via UART commands
"""

import serial
import sys
import time
import struct
from msvcrt import kbhit, getch  # Windows-specific keyboard input

class VectorNavController:
    def __init__(self, port='COM5', baudrate=115200, timeout=1.0):
        """Initialize VectorNav controller with serial connection"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.running = False

    def connect(self):
        """Establish serial connection to VectorNav"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            print(f"Connected to VectorNav on {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False

    def disconnect(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected from VectorNav")

    def send_command(self, command):
        """Send command to VectorNav and wait for echo"""
        if not self.ser or not self.ser.is_open:
            print("Error: Not connected to VectorNav")
            return False

        # Clear input buffer before sending
        self.ser.reset_input_buffer()

        # Send command
        cmd_bytes = (command + '\r\n').encode('ascii')
        self.ser.write(cmd_bytes)
        print(f"[TX] - {command}")

        # Wait for echo response (with timeout)
        start_time = time.time()
        response = ""
        while (time.time() - start_time) < 2.0:  # 2 second timeout
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                try:
                    char = byte.decode('ascii')
                    if char == '\n':
                        # End of line - print response
                        response = response.strip()
                        if response:
                            print(f"[RX] - {response}")
                            return True
                    elif char != '\r':
                        response += char
                except UnicodeDecodeError:
                    pass
            time.sleep(0.01)

        if not response:
            print("[RX] - (No response received)")

        return True

    def async_mode_on(self):
        """Enable VectorNav async mode (20Hz binary output)"""
        print("\n=== Enabling VectorNav Async Mode (20Hz) ===")
        command = "$VNWRG,75,1,20,01,01EA*6441"
        self.send_command(command)
        print("Async mode enabled - VectorNav will send binary packets at 20Hz\n")

    def async_mode_off(self):
        """Disable VectorNav async mode"""
        print("\n=== Disabling VectorNav Async Mode ===")
        command = "$VNWRG,75,0,20,00*F819"
        self.send_command(command)
        print("Async mode disabled\n")

    def parse_vn_packet(self, packet_bytes):
        """
        Parse VectorNav binary packet (86 bytes total):

        Byte offset calculation:
        0:      sync (uint8_t = 1 byte)              -> offset 1
        1:      group (uint8_t = 1 byte)             -> offset 2
        2-3:    types_common (uint16_t = 2 bytes)    -> offset 4
        4-11:   timegps (uint64_t = 8 bytes)         -> offset 12
        12-15:  yaw (float = 4 bytes)                -> offset 16
        16-19:  pitch (float = 4 bytes)              -> offset 20
        20-23:  roll (float = 4 bytes)               -> offset 24
        24-27:  gyro_x (float = 4 bytes)             -> offset 28
        28-31:  gyro_y (float = 4 bytes)             -> offset 32
        32-35:  gyro_z (float = 4 bytes)             -> offset 36
        36-43:  latitude (double = 8 bytes)          -> offset 44
        44-51:  longitude (double = 8 bytes)         -> offset 52
        52-59:  altitude (double = 8 bytes)          -> offset 60
        60-63:  vel_n (float = 4 bytes)              -> offset 64
        64-67:  vel_e (float = 4 bytes)              -> offset 68
        68-71:  vel_d (float = 4 bytes)              -> offset 72
        72-75:  acc_x (float = 4 bytes)              -> offset 76
        76-79:  acc_y (float = 4 bytes)              -> offset 80
        80-83:  acc_z (float = 4 bytes)              -> offset 84
        84-85:  checksum (uint16_t = 2 bytes)        -> offset 86

        Total: 1+1+2+8+(3*4)+(3*4)+(3*8)+(3*4)+(3*4)+2 = 1+1+2+8+12+12+24+12+12+2 = 86 bytes
        """
        if len(packet_bytes) < 86:
            return None

        try:
            # Manual byte-by-byte parsing
            sync = packet_bytes[0]
            group = packet_bytes[1]
            types_common = struct.unpack('<H', packet_bytes[2:4])[0]
            timegps = struct.unpack('<Q', packet_bytes[4:12])[0]
            yaw = struct.unpack('<f', packet_bytes[12:16])[0]
            pitch = struct.unpack('<f', packet_bytes[16:20])[0]
            roll = struct.unpack('<f', packet_bytes[20:24])[0]
            gyro_x = struct.unpack('<f', packet_bytes[24:28])[0]
            gyro_y = struct.unpack('<f', packet_bytes[28:32])[0]
            gyro_z = struct.unpack('<f', packet_bytes[32:36])[0]
            latitude = struct.unpack('<d', packet_bytes[36:44])[0]
            longitude = struct.unpack('<d', packet_bytes[44:52])[0]
            altitude = struct.unpack('<d', packet_bytes[52:60])[0]
            vel_n = struct.unpack('<f', packet_bytes[60:64])[0]
            vel_e = struct.unpack('<f', packet_bytes[64:68])[0]
            vel_d = struct.unpack('<f', packet_bytes[68:72])[0]
            acc_x = struct.unpack('<f', packet_bytes[72:76])[0]
            acc_y = struct.unpack('<f', packet_bytes[76:80])[0]
            acc_z = struct.unpack('<f', packet_bytes[80:84])[0]
            checksum = struct.unpack('>H', packet_bytes[84:86])[0]

            parsed = {
                'sync': sync,
                'group': group,
                'types_common': types_common,
                'timegps': timegps,
                'yaw': yaw,
                'pitch': pitch,
                'roll': roll,
                'gyro_x': gyro_x,
                'gyro_y': gyro_y,
                'gyro_z': gyro_z,
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude,
                'vel_n': vel_n,
                'vel_e': vel_e,
                'vel_d': vel_d,
                'acc_x': acc_x,
                'acc_y': acc_y,
                'acc_z': acc_z,
                'checksum': checksum
            }

            return parsed

        except Exception as e:
            print(f"Error parsing packet: {e}")
            return None

    def print_vn_packet(self, packet_num, parsed):
        """Print parsed VectorNav packet in human-readable format"""
        if parsed is None:
            return

        # Convert GPS time to seconds for readability
        gps_time_s = parsed['timegps'] / 1e9

        print(f"\n{'='*70}")
        print(f"[PKT {packet_num:05d}] VectorNav Binary Packet")
        print(f"{'='*70}")
        print(f"Header:      sync=0x{parsed['sync']:02X} group=0x{parsed['group']:02X} types=0x{parsed['types_common']:04X}")
        print(f"GPS Time:    {parsed['timegps']} ns ({gps_time_s:.3f} s since 1980-01-01)")
        print(f"Orientation: yaw={parsed['yaw']:8.3f}deg pitch={parsed['pitch']:8.3f}deg roll={parsed['roll']:8.3f}deg")
        print(f"Gyro (deg/s):  X={parsed['gyro_x']:8.3f}  Y={parsed['gyro_y']:8.3f}  Z={parsed['gyro_z']:8.3f}")
        print(f"Position:    lat={parsed['latitude']:11.7f}deg lon={parsed['longitude']:11.7f}deg alt={parsed['altitude']:8.2f}m")
        print(f"Velocity(m/s): N={parsed['vel_n']:7.3f}  E={parsed['vel_e']:7.3f}  D={parsed['vel_d']:7.3f}")
        print(f"Accel (m/s2): X={parsed['acc_x']:7.3f}  Y={parsed['acc_y']:7.3f}  Z={parsed['acc_z']:7.3f}")
        print(f"Checksum:    0x{parsed['checksum']:04X}")
        print(f"{'='*70}")

    def echo_mode(self):
        """Echo all received packets from VectorNav until ESC is pressed"""
        print("\n=== VectorNav Echo Mode ===")
        print("Displaying all received packets (parsed binary data)")
        print("Press ESC to exit and return to main menu\n")

        if not self.ser or not self.ser.is_open:
            print("Error: Not connected to VectorNav")
            return

        # Clear input buffer
        self.ser.reset_input_buffer()

        packet_buffer = bytearray()
        packet_count = 0

        while True:
            # Check for ESC key press
            if kbhit():
                key = getch()
                if key == b'\x1b':  # ESC key
                    print("\nExiting echo mode...")
                    break

            # Read available data
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)

                for byte in data:
                    packet_buffer.append(byte)

                    # Check for VectorNav binary packet sync byte (0xFA)
                    if len(packet_buffer) == 1 and packet_buffer[0] == 0xFA:
                        # Start of binary packet
                        continue

                    # Check for ASCII packet (starts with $)
                    if len(packet_buffer) == 1 and packet_buffer[0] == ord('$'):
                        # ASCII packet - collect until newline
                        continue

                    # Handle ASCII packets
                    if len(packet_buffer) > 0 and packet_buffer[0] == ord('$'):
                        if byte == ord('\n'):
                            # End of ASCII packet
                            try:
                                ascii_msg = packet_buffer.decode('ascii').strip()
                                packet_count += 1
                                print(f"[PKT {packet_count:05d}] ASCII: {ascii_msg}")
                            except UnicodeDecodeError:
                                print(f"[PKT {packet_count:05d}] ASCII: (decode error)")
                            packet_buffer.clear()

                    # Handle binary packets (86 bytes total for VectorNav custom output)
                    elif len(packet_buffer) > 0 and packet_buffer[0] == 0xFA:
                        # VN_Packet_t is 86 bytes
                        if len(packet_buffer) >= 86:
                            packet_count += 1

                            # Parse the binary packet
                            parsed = self.parse_vn_packet(packet_buffer[:86])

                            if parsed:
                                self.print_vn_packet(packet_count, parsed)
                            else:
                                # If parsing failed, show hex dump
                                hex_str = ' '.join(f'{b:02X}' for b in packet_buffer[:86])
                                print(f"[PKT {packet_count:05d}] BINARY (parse failed): {hex_str}")

                            packet_buffer.clear()

                    # Handle unexpected data
                    elif len(packet_buffer) > 100:  # Prevent buffer overflow
                        # Unknown packet type or corrupted data
                        hex_str = ' '.join(f'{b:02X}' for b in packet_buffer)
                        print(f"[PKT {packet_count:05d}] UNKNOWN: {hex_str}")
                        packet_buffer.clear()

            time.sleep(0.01)  # Small delay to prevent CPU spinning

        print(f"Echo mode exited. Total packets received: {packet_count}\n")

    def print_menu(self):
        """Print main menu options"""
        print("\n" + "="*50)
        print("VectorNav Operation Menu")
        print("="*50)
        print("1. Async Mode ON  (20Hz binary output)")
        print("2. Async Mode OFF (disable binary output)")
        print("3. Echo Mode      (display all received packets)")
        print("4. Exit")
        print("="*50)

    def run(self):
        """Main program loop"""
        print("VectorNav Operation Script")
        print("-" * 50)

        if not self.connect():
            return

        try:
            while True:
                self.print_menu()
                choice = input("Enter your choice (1-4): ").strip()

                if choice == '1':
                    self.async_mode_on()
                elif choice == '2':
                    self.async_mode_off()
                elif choice == '3':
                    self.echo_mode()
                elif choice == '4':
                    print("\nExiting...")
                    break
                else:
                    print("Invalid choice. Please enter 1-4.")

        except KeyboardInterrupt:
            print("\n\nInterrupted by user (Ctrl+C)")

        finally:
            self.disconnect()


def main():
    """Main entry point"""
    # Default settings - modify as needed
    DEFAULT_PORT = 'COM5'  # VectorNav UART5 port
    DEFAULT_BAUD = 115200

    # Allow command-line arguments for port and baudrate
    port = DEFAULT_PORT
    baudrate = DEFAULT_BAUD

    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        baudrate = int(sys.argv[2])

    print(f"Using port: {port}, baudrate: {baudrate}")

    controller = VectorNavController(port=port, baudrate=baudrate)
    controller.run()


if __name__ == '__main__':
    main()
