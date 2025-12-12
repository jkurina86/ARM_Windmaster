#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WindMaster Operation Script

Controls a Gill WindMaster ultrasonic anemometer via serial/UART.

Designed from "WINDMASTER COMMANDS" manual in Docs.

Features:
- Connect/disconnect to COM port
- Enter/exit Configuration Mode
- Query diagnostics/config (D1/D2/D3)
- Change common settings (M, P, U, J, A, L, N)
- Polled-mode helpers (?, !, &, poll ID)
- Echo mode: display received measurements (ASCII or Binary)
  - ASCII parsing supports Polar and UVW CSV-style payloads.
  - Binary parsing includes Mode 10 (Binary UVW Long) as specified.

Requirements:
- pyserial (pip install pyserial)

Notes:
- WindMaster default serial: 19200 8N1, no flow control.
- Some units output ASCII frames with STX/ETX and checksum; others output
  CSV lines. This script attempts to parse both robustly.
"""

from __future__ import annotations

import sys
import time
import struct
from dataclasses import dataclass
from typing import Optional, List, Dict, Tuple

import serial

# Windows-specific keyboard input (matches vectornav_operation.py style)
from msvcrt import kbhit, getch


STX = 0x02
ETX = 0x03


@dataclass
class WindMasterAsciiMeasurement:
    unit_id: str
    kind: str  # 'polar' or 'uvw'
    units: Optional[str] = None

    # Polar
    direction_deg: Optional[float] = None
    magnitude: Optional[float] = None

    # UVW
    u: Optional[float] = None
    v: Optional[float] = None
    w: Optional[float] = None

    # Optional environmental outputs
    speed_of_sound: Optional[float] = None
    sonic_temp_c: Optional[float] = None

    status: Optional[str] = None
    analog_inputs_v: Optional[List[float]] = None
    prt_temp_c: Optional[float] = None

    raw: Optional[str] = None
    checksum_ok: Optional[bool] = None


@dataclass
class WindMasterBinaryMode10:
    raw_bytes: bytes
    sta_l: int
    sta_h: int
    status: int
    u_raw: int
    v_raw: int
    w_raw: int
    sos_raw: int
    u_mps: float
    v_mps: float
    w_mps: float
    sos_mps: float
    a1: int
    a2: int
    a3: int
    a4: int
    tprt: int
    checksum_expected: int
    checksum_actual: int
    checksum_ok: bool


class WindMasterController:
    def __init__(self, port: str = "COM6", baudrate: int = 19200, timeout: float = 0.25):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            )
            print(f"Connected to WindMaster on {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False

    def disconnect(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected from WindMaster")

    def _require_open(self) -> bool:
        if not self.ser or not self.ser.is_open:
            print("Error: Not connected to WindMaster")
            return False
        return True

    def write_line(self, line: str, eol: str = "\r\n") -> bool:
        """Write an ASCII command line."""
        if not self._require_open():
            return False
        ser = self.ser
        assert ser is not None
        payload = (line + eol).encode("ascii", errors="replace")
        ser.write(payload)
        print(f"[TX] {line}")
        return True

    def read_available_lines(self, max_seconds: float = 1.0) -> List[str]:
        """Read any available ASCII lines (CR/LF terminated) for up to max_seconds."""
        if not self._require_open():
            return []

        ser = self.ser
        assert ser is not None

        end = time.time() + max_seconds
        lines: List[str] = []
        buf = bytearray()

        while time.time() < end:
            n = ser.in_waiting
            if n:
                buf.extend(ser.read(n))
                # Split on LF; keep remainder
                while True:
                    lf = buf.find(b"\n")
                    if lf < 0:
                        break
                    line = buf[: lf + 1]
                    del buf[: lf + 1]
                    try:
                        lines.append(line.decode("ascii", errors="replace").strip("\r\n"))
                    except Exception:
                        lines.append(repr(line))
            else:
                time.sleep(0.01)

        # If buffer ends with CR only, flush it as a line
        if buf:
            try:
                lines.append(buf.decode("ascii", errors="replace").strip("\r\n"))
            except Exception:
                lines.append(repr(buf))

        return [ln for ln in (l.strip() for l in lines) if ln]

    # --- Configuration/diagnostics helpers ---

    def enter_configuration_mode(self, unit_id: Optional[str] = None) -> None:
        """Enter configuration mode: '*' for continuous, or '* <N>' for polled."""
        if unit_id:
            cmd = f"* {unit_id}"
        else:
            cmd = "*"
        self.write_line(cmd)
        # Read a few lines (power-on banner / CONFIGURATION MODE)
        for ln in self.read_available_lines(max_seconds=1.5):
            print(f"[RX] {ln}")

    def exit_to_measurement_mode(self) -> None:
        """Return to measurement mode."""
        self.write_line("Q")

    def request_serial_number(self) -> None:
        self.write_line("D1")
        for ln in self.read_available_lines(max_seconds=1.0):
            print(f"[RX] {ln}")

    def request_software_version(self) -> None:
        self.write_line("D2")
        for ln in self.read_available_lines(max_seconds=1.0):
            print(f"[RX] {ln}")

    def request_current_configuration(self) -> None:
        self.write_line("D3")
        for ln in self.read_available_lines(max_seconds=1.0):
            print(f"[RX] {ln}")

    def set_setting(self, command: str) -> None:
        """Send a setting command (e.g., 'M 2', 'P 25', 'U 1', 'J 2')."""
        self.write_line(command)
        for ln in self.read_available_lines(max_seconds=0.8):
            print(f"[RX] {ln}")

    # --- Polled mode helpers ---

    def polled_enable(self) -> None:
        """Enable polled mode ('?')."""
        self.write_line("?")

    def polled_disable(self) -> None:
        """Disable polled mode ('!')."""
        self.write_line("!")

    def request_unit_id(self) -> None:
        """Request current unit identifier ('&')."""
        self.write_line("&")
        for ln in self.read_available_lines(max_seconds=0.8):
            print(f"[RX] {ln}")

    def poll_once(self, unit_id: str) -> None:
        """Poll a single measurement in polled modes (M3/M4)."""
        self.write_line(unit_id)

    # --- Parsing ---

    @staticmethod
    def _xor_checksum(data: bytes) -> int:
        chk = 0
        for b in data:
            chk ^= b
        return chk & 0xFF

    @staticmethod
    def _try_parse_ascii_frame_from_line(line: str) -> Tuple[str, Optional[bool]]:
        """Normalize an ASCII measurement 'line' into CSV payload.

        Returns:
          (payload_without_framing, checksum_ok or None)

        Supports:
        - STX...ETXCC framing described in manual
        - plain CSV lines (checksum may be last field; checksum_ok=None)
        """
        if not line:
            return "", None

        raw = line
        # Handle cases where terminal decoding replaced STX/ETX with control chars in string.
        # Python string may contain actual \x02/\x03.
        if raw and ord(raw[0]) == STX:
            # Find ETX
            etx_pos = raw.find(chr(ETX))
            if etx_pos != -1:
                between = raw[1:etx_pos]  # between STX and ETX
                after = raw[etx_pos + 1 :]
                # Expect checksum as two hex chars at start of 'after'
                chk_ok: Optional[bool] = None
                if len(after) >= 2 and all(c in "0123456789abcdefABCDEF" for c in after[:2]):
                    expected = int(after[:2], 16)
                    actual = WindMasterController._xor_checksum(between.encode("ascii", errors="replace"))
                    chk_ok = (expected == actual)
                return between.strip(), chk_ok

        # Plain CSV or fixed-field (still comma-separated)
        return raw.strip(), None

    @staticmethod
    def parse_ascii_measurement(line: str) -> Optional[WindMasterAsciiMeasurement]:
        payload, checksum_ok = WindMasterController._try_parse_ascii_frame_from_line(line)
        if not payload:
            return None

        # Measurement lines are comma-separated.
        parts = [p.strip() for p in payload.split(",")]
        if not parts:
            return None

        unit_id = parts[0] if parts[0] else "?"
        m = WindMasterAsciiMeasurement(unit_id=unit_id, kind="unknown", raw=line, checksum_ok=checksum_ok)

        # Some configurations may output a power-on banner or text lines.
        if len(parts) < 3:
            return None

        # Heuristic: UVW has signed U as second field; Polar has non-signed direction.
        second = parts[1]
        is_uvw = second.startswith(("+", "-"))
        is_polar = (not is_uvw)

        try:
            if is_uvw:
                m.kind = "uvw"
                m.u = float(parts[1]) if parts[1] else None
                m.v = float(parts[2]) if len(parts) > 2 and parts[2] else None
                m.w = float(parts[3]) if len(parts) > 3 and parts[3] else None
                m.units = parts[4] if len(parts) > 4 and parts[4] else None
                idx = 5
            else:
                m.kind = "polar"
                m.direction_deg = float(parts[1]) if parts[1] else None
                m.magnitude = float(parts[2]) if parts[2] else None
                m.w = float(parts[3]) if len(parts) > 3 and parts[3] else None
                m.units = parts[4] if len(parts) > 4 and parts[4] else None
                idx = 5

            # After units:
            # - optional SOS and/or sonic temp (A2/A3/A4)
            # - status (always present)
            # - optional A1..A4 (I1)
            # - optional PRT (V2) [but manual notes PRT not available on WindMaster]

            # The status is a 2-hex-digit string (00..0B), but some lines may contain it later.
            def looks_like_status(s: str) -> bool:
                s2 = s.strip()
                return len(s2) == 2 and all(c in "0123456789abcdefABCDEF" for c in s2)

            # Try to consume 0-2 numeric fields (SOS/temp) until we hit status.
            maybe_nums: List[float] = []
            while idx < len(parts) and not looks_like_status(parts[idx]):
                # Some configurations may output blank fields on error.
                if parts[idx] in ("", None):
                    maybe_nums.append(float("nan"))
                else:
                    # Stop if this field contains a trailing 'C' (PRT) or other non-numeric.
                    try:
                        maybe_nums.append(float(parts[idx].rstrip("C")))
                    except ValueError:
                        break
                idx += 1

            if idx < len(parts) and looks_like_status(parts[idx]):
                # Map SOS/temp if they exist and are not NaN.
                if len(maybe_nums) == 1:
                    m.speed_of_sound = maybe_nums[0]
                elif len(maybe_nums) >= 2:
                    m.speed_of_sound = maybe_nums[0]
                    m.sonic_temp_c = maybe_nums[1]
                m.status = parts[idx].upper()
                idx += 1
            else:
                # Couldn't find status; likely not a measurement line.
                return None

            # Remaining fields: analog inputs (4) and/or PRT and/or checksum.
            remaining = [p.strip() for p in parts[idx:] if p.strip()]

            # PRT sometimes has trailing 'C' (e.g. -50.00C)
            analogs: List[float] = []
            prt: Optional[float] = None
            for token in remaining:
                # Skip trailing checksum token if it looks like 2 hex digits.
                if len(token) == 2 and all(c in "0123456789abcdefABCDEF" for c in token):
                    continue
                if token.upper().endswith("C"):
                    try:
                        prt = float(token[:-1])
                        continue
                    except ValueError:
                        pass
                # Otherwise try float
                try:
                    analogs.append(float(token))
                except ValueError:
                    pass

            if analogs:
                m.analog_inputs_v = analogs
            if prt is not None:
                m.prt_temp_c = prt

            return m

        except Exception:
            return None

    @staticmethod
    def parse_binary_mode10(packet: bytes) -> Optional[WindMasterBinaryMode10]:
        """Parse WindMaster Mode 10 (Binary UVW Long) record.

        Manual:
        23 bytes total:
        0xB4 0xB4 StaL StaH Wc1L Wc1H Wc2L Wc2H Wc3L Wc3H SoSL SoSH
        A1L A1H A2H A2L A3H A3L A4H A4L TPrtL TPrtH Checksum

        Note: Byte order for A2..A4 is shown as 'A2H A2L' etc. We follow the manual.
        """
        if len(packet) < 23:
            return None
        if packet[0] != 0xB4 or packet[1] != 0xB4:
            return None

        checksum_expected = packet[22]
        checksum_actual = 0
        for b in packet[0:22]:
            checksum_actual ^= b
        checksum_actual &= 0xFF

        sta_l = packet[2]
        sta_h = packet[3]
        status = sta_l | (sta_h << 8)

        def s16(le_bytes: bytes) -> int:
            return struct.unpack("<h", le_bytes)[0]

        def u16(le_bytes: bytes) -> int:
            return struct.unpack("<H", le_bytes)[0]

        # Standard little-endian for most 16-bit fields.
        u_raw = s16(packet[4:6])
        v_raw = s16(packet[6:8])
        w_raw = s16(packet[8:10])
        # Manual text for Mode 10 says signed, but SoS is physically 300-370m/s,
        # and other modes specify SoS as unsigned. Interpret as uint16.
        sos_raw = u16(packet[10:12])
        a1_raw = s16(packet[12:14])

        # Manual shows swapped order for A2..A4.
        a2_raw = s16(bytes([packet[15], packet[14]]))
        a3_raw = s16(bytes([packet[17], packet[16]]))
        a4_raw = s16(bytes([packet[19], packet[18]]))
        tprt_raw = s16(packet[20:22])

        return WindMasterBinaryMode10(
            raw_bytes=bytes(packet[:23]),
            sta_l=sta_l,
            sta_h=sta_h,
            status=status,
            u_raw=u_raw,
            v_raw=v_raw,
            w_raw=w_raw,
            sos_raw=sos_raw,
            u_mps=u_raw / 100.0,
            v_mps=v_raw / 100.0,
            w_mps=w_raw / 100.0,
            sos_mps=sos_raw / 100.0,
            a1=a1_raw,
            a2=a2_raw,
            a3=a3_raw,
            a4=a4_raw,
            tprt=tprt_raw,
            checksum_expected=checksum_expected,
            checksum_actual=checksum_actual,
            checksum_ok=(checksum_expected == checksum_actual),
        )

    @staticmethod
    def format_ascii_measurement(m: WindMasterAsciiMeasurement) -> str:
        if m.kind == "polar":
            base = f"{m.unit_id} polar: dir={m.direction_deg}deg mag={m.magnitude}{m.units or ''} w={m.w}{m.units or ''}"
        elif m.kind == "uvw":
            base = f"{m.unit_id} uvw: U={m.u}{m.units or ''} V={m.v}{m.units or ''} W={m.w}{m.units or ''}"
        else:
            base = f"{m.unit_id} measurement"

        extras = []
        if m.speed_of_sound is not None:
            extras.append(f"SOS={m.speed_of_sound}m/s")
        if m.sonic_temp_c is not None:
            extras.append(f"Ts={m.sonic_temp_c}C")
        if m.status is not None:
            extras.append(f"status={m.status}")
        if m.checksum_ok is not None:
            extras.append(f"chk={'OK' if m.checksum_ok else 'BAD'}")

        return base + (" | " + " ".join(extras) if extras else "")

    @staticmethod
    def format_binary_mode10(pkt: WindMasterBinaryMode10) -> str:
        # Show every field in the 23-byte record, even if it's zero/unused.
        # This is intentionally verbose for debugging.
        b = pkt.raw_bytes
        hex_bytes = " ".join(f"{x:02X}" for x in b)
        return (
            "B10(23B) "
            f"bytes=[{hex_bytes}] "
            f"StaL=0x{pkt.sta_l:02X} StaH=0x{pkt.sta_h:02X} "
            f"Uraw={pkt.u_raw} Vraw={pkt.v_raw} Wraw={pkt.w_raw} SoSraw={pkt.sos_raw} "
            f"A1={pkt.a1} A2={pkt.a2} A3={pkt.a3} A4={pkt.a4} TPrt={pkt.tprt} "
            f"ChkExp=0x{pkt.checksum_expected:02X} ChkCalc=0x{pkt.checksum_actual:02X} "
            f"| U={pkt.u_mps:.2f} V={pkt.v_mps:.2f} W={pkt.w_mps:.2f}m/s SoS={pkt.sos_mps:.2f}m/s "
            f"status=0x{pkt.status:04X} chk={'OK' if pkt.checksum_ok else 'BAD'}"
        )

    # --- Echoing/stream parsing ---

    def echo_mode(self, max_packets: Optional[int] = None) -> None:
        """Display received measurements until ESC is pressed."""
        print("\n=== WindMaster Echo Mode ===")
        print("Displays received measurements (ASCII or Binary)")
        print("Press ESC to exit and return to main menu\n")

        if not self._require_open():
            return

        ser = self.ser
        assert ser is not None

        # Clear input buffer
        ser.reset_input_buffer()

        buf = bytearray()
        count = 0

        while True:
            if kbhit():
                key = getch()
                if key == b"\x1b":
                    print("\nExiting echo mode...")
                    break

            n = ser.in_waiting
            if n:
                buf.extend(ser.read(n))

                # Consume buffer.
                while buf:
                    # Binary Mode 10 start marker
                    b0 = buf[0]
                    if b0 in (0xB1, 0xB2, 0xB3, 0xB4):
                        # Need at least 2 bytes to confirm repeated marker
                        if len(buf) < 2:
                            break
                        if buf[1] != b0:
                            # Not a valid start; discard one byte
                            buf.pop(0)
                            continue

                        if b0 == 0xB4:
                            needed = 23
                            if len(buf) < needed:
                                break
                            pkt = bytes(buf[:needed])
                            del buf[:needed]
                            parsed = self.parse_binary_mode10(pkt)
                            count += 1
                            if parsed:
                                print(f"[PKT {count:05d}] {self.format_binary_mode10(parsed)}")
                            else:
                                hex_str = " ".join(f"{b:02X}" for b in pkt)
                                print(f"[PKT {count:05d}] BINARY: {hex_str}")
                        else:
                            # For other binary modes, the manual doesn't fully specify record sizes here.
                            # Avoid blocking by discarding the two marker bytes.
                            count += 1
                            print(f"[PKT {count:05d}] Binary start 0x{b0:02X} 0x{b0:02X} (mode not decoded)")
                            del buf[:2]

                    else:
                        # ASCII mode: collect until LF or CR.
                        lf = buf.find(b"\n")
                        cr = buf.find(b"\r")
                        end = -1
                        if lf != -1 and cr != -1:
                            end = min(lf, cr)
                        elif lf != -1:
                            end = lf
                        elif cr != -1:
                            end = cr

                        if end == -1:
                            # Not enough yet; cap buffer growth
                            if len(buf) > 4096:
                                buf.clear()
                            break

                        line_bytes = bytes(buf[:end])
                        # Consume terminator (and optional paired terminator)
                        del buf[: end + 1]
                        if buf and buf[0] in (ord("\n"), ord("\r")):
                            del buf[:1]

                        line = line_bytes.decode("ascii", errors="replace")
                        parsed = self.parse_ascii_measurement(line)
                        if parsed:
                            count += 1
                            print(f"[PKT {count:05d}] {self.format_ascii_measurement(parsed)}")
                        else:
                            # Ignore empty lines; print other text lines
                            s = line.strip()
                            if s:
                                print(f"[RX] {s}")

                    if max_packets is not None and count >= max_packets:
                        print("\nReached packet limit; exiting echo mode...")
                        return

            time.sleep(0.01)

        print(f"Echo mode exited. Total packets displayed: {count}\n")


def print_menu() -> None:
    print("\n" + "=" * 58)
    print("WindMaster Operation Menu")
    print("=" * 58)
    print("1. Enter Configuration Mode (*)")
    print("2. Exit to Measurement Mode (Q)")
    print("3. Diagnostics: Serial Number (D1)")
    print("4. Diagnostics: Software Version (D2)")
    print("5. Diagnostics: Current Configuration (D3)")
    print("6. Set Message Format (M x)")
    print("7. Set Output Rate (P x)")
    print("8. Set Units (U x)")
    print("9. Set Resolution (J x)")
    print("10. Set SOS/Sonic Temp Output (A x)")
    print("11. Set Message Terminator (L x)")
    print("12. Set Unit ID (N <letter>)")
    print("13. Polled: Enable (?)")
    print("14. Polled: Disable (!)" )
    print("15. Polled: Request Unit ID (&)")
    print("16. Polled: Poll Once (send unit letter)")
    print("17. Echo Mode (display incoming data)")
    print("18. Send Raw Command")
    print("19. Exit")
    print("=" * 58)


def main() -> None:
    DEFAULT_PORT = "COM6"
    DEFAULT_BAUD = 19200

    port = DEFAULT_PORT
    baud = DEFAULT_BAUD

    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        baud = int(sys.argv[2])

    print("WindMaster Operation Script")
    print("-" * 58)
    print(f"Using port: {port}, baudrate: {baud}")

    ctrl = WindMasterController(port=port, baudrate=baud)
    if not ctrl.connect():
        return

    try:
        while True:
            print_menu()
            choice = input("Enter your choice (1-19): ").strip()

            if choice == "1":
                unit = input("Unit ID for polled config mode (blank for '*'): ").strip().upper()
                ctrl.enter_configuration_mode(unit_id=unit if unit else None)
            elif choice == "2":
                ctrl.exit_to_measurement_mode()
            elif choice == "3":
                ctrl.request_serial_number()
            elif choice == "4":
                ctrl.request_software_version()
            elif choice == "5":
                ctrl.request_current_configuration()
            elif choice == "6":
                print("Message formats: M1 UVW, M2 Polar, M3 UVW Polled, M4 Polar Polled, M7..M10 Binary")
                x = input("Enter format number (e.g. 2): ").strip()
                ctrl.set_setting(f"M {x}")
            elif choice == "7":
                print("Output rates: P18=1Hz, P19=2Hz, P20=4Hz, P21=5Hz, P22=8Hz, P23=10Hz, P24=16Hz, P25=20Hz, P26=32Hz")
                x = input("Enter rate code (e.g. 25): ").strip()
                ctrl.set_setting(f"P {x}")
            elif choice == "8":
                print("Units: U1 m/s, U2 knots, U3 mph, U4 kph, U5 fpm")
                x = input("Enter unit code (e.g. 1): ").strip()
                ctrl.set_setting(f"U {x}")
            elif choice == "9":
                print("Resolution: J1 normal, J2 high")
                x = input("Enter resolution code (1/2): ").strip()
                ctrl.set_setting(f"J {x}")
            elif choice == "10":
                print("SOS/Sonic Temp: A1 none, A2 SOS, A3 sonic temp, A4 both")
                x = input("Enter A code (1-4): ").strip()
                ctrl.set_setting(f"A {x}")
            elif choice == "11":
                print("Terminator: L1 CRLF, L2 CR")
                x = input("Enter L code (1/2): ").strip()
                ctrl.set_setting(f"L {x}")
            elif choice == "12":
                x = input("Enter new unit letter (A-Z): ").strip().upper()
                if x and len(x) == 1 and "A" <= x <= "Z":
                    ctrl.set_setting(f"N {x}")
                else:
                    print("Invalid unit letter")
            elif choice == "13":
                ctrl.polled_enable()
            elif choice == "14":
                ctrl.polled_disable()
            elif choice == "15":
                ctrl.request_unit_id()
            elif choice == "16":
                x = input("Poll unit letter (A-Z): ").strip().upper()
                if x and len(x) == 1 and "A" <= x <= "Z":
                    ctrl.poll_once(x)
                else:
                    print("Invalid unit letter")
            elif choice == "17":
                ctrl.echo_mode()
            elif choice == "18":
                cmd = input("Raw command to send: ").strip()
                if cmd:
                    ctrl.write_line(cmd)
                    for ln in ctrl.read_available_lines(max_seconds=1.0):
                        print(f"[RX] {ln}")
            elif choice == "19":
                print("\nExiting...")
                break
            else:
                print("Invalid choice. Please enter 1-19.")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user (Ctrl+C)")
    finally:
        ctrl.disconnect()


if __name__ == "__main__":
    main()
