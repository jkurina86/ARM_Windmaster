#!/usr/bin/env python3
"""
Bad Packet Decoder for ARM_Windmaster WindMaster Data
=====================================================

Parses binary bad-packet log files (BAD_*.bin) created by the recorder.
Each entry is a 32-byte WM_BadPacket_t containing a timestamped WindMaster
packet that failed checksum validation or data quality checks.

Usage:
    python bad_packet_decoder.py BAD_2026-02-10-17-20-54.bin --csv output.csv
    python bad_packet_decoder.py BAD_2026-02-10-17-20-54.bin --csv
    python bad_packet_decoder.py ./logs/ --csv
    python bad_packet_decoder.py ./logs/ --csv --output-dir ./output
"""

import struct
import sys
import argparse
import csv
from pathlib import Path
from typing import NamedTuple, List, Optional
from datetime import datetime, timedelta


# WM_BadPacket_t layout (32 bytes, little-endian ARM Cortex-M4):
#   uint32_t timestamp_s      (4B)  seconds since 2000-01-01
#   uint16_t timestamp_ms     (2B)  milliseconds 0-999
#   --- WM_Packet_t (23B packed) ---
#   uint16_t header            (2B)  0xB4B4
#   uint16_t status            (2B)  status word
#   int16_t  U_axis_speed      (2B)  cm/s
#   int16_t  V_axis_speed      (2B)  cm/s
#   int16_t  W_axis_speed      (2B)  cm/s
#   int16_t  SoS               (2B)  speed of sound (0.01 m/s units)
#   int16_t  A1                (2B)  analog input 1
#   int16_t  A2                (2B)  analog input 2
#   int16_t  A3                (2B)  analog input 3
#   int16_t  A4                (2B)  analog input 4
#   int16_t  Temp              (2B)  temperature from PRT
#   uint8_t  checksum          (1B)  XOR checksum
#   --- end WM_Packet_t ---
#   3 bytes trailing padding

ENTRY_FORMAT = '<IHHHhhhhhhhhhBxxx'
ENTRY_SIZE = 32
EPOCH_2000 = datetime(2000, 1, 1, 0, 0, 0)
WM_HEADER_MAGIC = 0xB4B4

assert struct.calcsize(ENTRY_FORMAT) == ENTRY_SIZE


class BadPacketEntry(NamedTuple):
    """32-byte bad packet record matching WM_BadPacket_t"""
    timestamp_s: int        # uint32_t  seconds since 2000-01-01
    timestamp_ms: int       # uint16_t  milliseconds (0-999)
    header: int             # uint16_t  should be 0xB4B4
    status: int             # uint16_t  WindMaster status word
    U_axis_speed: int       # int16_t   cm/s
    V_axis_speed: int       # int16_t   cm/s
    W_axis_speed: int       # int16_t   cm/s
    SoS: int                # int16_t   speed of sound (0.01 m/s)
    A1: int                 # int16_t   analog input 1
    A2: int                 # int16_t   analog input 2
    A3: int                 # int16_t   analog input 3
    A4: int                 # int16_t   analog input 4
    Temp: int               # int16_t   temperature from PRT
    checksum: int           # uint8_t   XOR checksum


def epoch_to_datetime(epoch_s: int) -> datetime:
    """Convert seconds since 2000-01-01 to a datetime object."""
    return EPOCH_2000 + timedelta(seconds=epoch_s)


def decode_status(status: int) -> str:
    """Decode WindMaster status byte into a human-readable reason.

    Status byte meanings (low byte):
      0x00       = OK
      0x01-0x09  = sample failure / hardware error
      0x0A       = gain at maximum (results still OK)
      0x0B       = retries used (results still OK)
    """
    low = status & 0xFF
    if low == 0x00:
        return "OK"
    elif 0x01 <= low <= 0x09:
        return f"sample_fail(0x{low:02X})"
    elif low == 0x0A:
        return "gain_at_max"
    elif low == 0x0B:
        return "retries_used"
    else:
        return f"unknown(0x{low:02X})"


def classify_bad_reason(entry: BadPacketEntry) -> str:
    """Classify why this packet was recorded as bad.

    Two paths lead here:
      1. Checksum failure (wm_validate_packet failed) - raw bytes, possibly garbled
      2. Data quality failure (wm_data_valid failed) - valid structure, bad values
    """
    reasons = []

    if entry.header != WM_HEADER_MAGIC:
        reasons.append("bad_header")

    status_low = entry.status & 0xFF
    if 0x01 <= status_low <= 0x09:
        reasons.append("status_error")

    sentinel = 32767  # INT16_MAX
    if entry.U_axis_speed == sentinel:
        reasons.append("U_sentinel")
    if entry.V_axis_speed == sentinel:
        reasons.append("V_sentinel")
    if entry.W_axis_speed == sentinel:
        reasons.append("W_sentinel")
    if entry.SoS == sentinel:
        reasons.append("SoS_sentinel")

    return "|".join(reasons) if reasons else "checksum_fail"


def parse_bad_packet_file(filepath: Path) -> List[BadPacketEntry]:
    """Parse a BAD_*.bin file into a list of BadPacketEntry records."""
    entries = []
    warnings = []

    file_size = filepath.stat().st_size
    expected = file_size // ENTRY_SIZE
    remainder = file_size % ENTRY_SIZE

    print(f"Parsing: {filepath.name}")
    print(f"File size: {file_size:,} bytes")
    print(f"Expected entries: {expected}")
    if remainder:
        print(f"[WARN] {remainder} trailing bytes (not a multiple of {ENTRY_SIZE})")
    print("-" * 60)

    with open(filepath, 'rb') as f:
        index = 0
        while True:
            data = f.read(ENTRY_SIZE)
            if len(data) == 0:
                break
            if len(data) < ENTRY_SIZE:
                warnings.append(f"Incomplete entry at offset {index * ENTRY_SIZE} ({len(data)} bytes)")
                break

            values = struct.unpack(ENTRY_FORMAT, data)
            entry = BadPacketEntry(*values)

            # Skip zero-filled padding entries (from unfilled buffer tails)
            if entry.timestamp_s == 0 and entry.timestamp_ms == 0 and entry.header == 0:
                index += 1
                continue

            entries.append(entry)
            index += 1

    print(f"[OK] Parsed {len(entries)} bad packet entries ({index - len(entries)} zero-fill skipped)")

    if warnings:
        print(f"\n[WARN] {len(warnings)} warnings:")
        for w in warnings:
            print(f"  - {w}")

    return entries


CSV_COLUMNS = [
    'timestamp',
    'epoch_seconds',
    'ms',
    'header',
    'status',
    'U_axis_speed_cm_s',
    'V_axis_speed_cm_s',
    'W_axis_speed_cm_s',
    'SoS',
    'A1',
    'A2',
    'A3',
    'A4',
    'Temp',
    'checksum',
    'status_decoded',
]


def export_csv(entries: List[BadPacketEntry], output_path: Path):
    """Export bad packet entries to a labeled CSV file."""
    if not entries:
        print("No entries to export")
        return

    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(CSV_COLUMNS)

        for entry in entries:
            dt = epoch_to_datetime(entry.timestamp_s)
            timestamp_str = dt.strftime('%Y-%m-%d %H:%M:%S') + f".{entry.timestamp_ms:03d}"

            writer.writerow([
                timestamp_str,
                entry.timestamp_s,
                entry.timestamp_ms,
                f"0x{entry.header:04X}",
                f"0x{entry.status:04X}",
                entry.U_axis_speed,
                entry.V_axis_speed,
                entry.W_axis_speed,
                entry.SoS,
                entry.A1,
                entry.A2,
                entry.A3,
                entry.A4,
                entry.Temp,
                f"0x{entry.checksum:02X}",
                decode_status(entry.status),
            ])

    print(f"[OK] Exported {len(entries)} entries to {output_path}")


def get_output_path(input_path: Path, output_dir: Optional[Path], ext: str) -> Path:
    """Generate output path from input path and optional output directory."""
    if output_dir:
        output_dir.mkdir(parents=True, exist_ok=True)
        return output_dir / (input_path.stem + ext)
    return input_path.parent / (input_path.stem + ext)


def process_single_file(input_path: Path, csv_output: Optional[Path],
                        auto_csv: bool, output_dir: Optional[Path]) -> bool:
    """Process one BAD_*.bin file. Returns True on success."""
    entries = parse_bad_packet_file(input_path)
    if not entries:
        print("[WARN] No valid entries found")
        return False

    if csv_output:
        export_csv(entries, csv_output)
    elif auto_csv:
        export_csv(entries, get_output_path(input_path, output_dir, '.csv'))

    return True


def main():
    parser = argparse.ArgumentParser(
        description='Decode bad WindMaster packet logs (BAD_*.bin) to CSV',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('input_path', type=Path,
                        help='BAD_*.bin file or directory containing them')
    parser.add_argument('--csv', type=Path, metavar='OUTPUT', nargs='?', const=True,
                        help='Export to CSV (auto-generate name if no path given)')
    parser.add_argument('--output-dir', type=Path, metavar='DIR',
                        help='Directory for auto-generated output files')

    args = parser.parse_args()

    if not args.csv:
        print("[ERROR] Specify --csv to generate output")
        sys.exit(1)

    if not args.input_path.exists():
        print(f"[ERROR] Path not found: {args.input_path}")
        sys.exit(1)

    if args.input_path.is_dir():
        if isinstance(args.csv, Path):
            print("[ERROR] --csv with a specific path is not supported in directory mode")
            print("        Use --csv without a path to auto-generate CSV files")
            sys.exit(1)

        bin_files = sorted(args.input_path.glob('BAD_*.bin'))
        if not bin_files:
            print(f"[INFO] No BAD_*.bin files found in {args.input_path}")
            sys.exit(0)

        print(f"Found {len(bin_files)} BAD_*.bin file(s)")
        print("=" * 60)

        success = 0
        for i, bf in enumerate(bin_files, 1):
            print(f"\n[{i}/{len(bin_files)}] {bf.name}")
            print("-" * 60)
            if process_single_file(bf, None, True, args.output_dir):
                success += 1

        print(f"\n{'=' * 60}")
        print(f"Done: {success}/{len(bin_files)} files processed")
        print(f"{'=' * 60}")

        if success < len(bin_files):
            sys.exit(1)
    else:
        csv_out = args.csv if isinstance(args.csv, Path) else None
        auto = args.csv is True
        if not process_single_file(args.input_path, csv_out, auto, args.output_dir):
            sys.exit(1)
        print("\n[OK] Done")


if __name__ == '__main__':
    main()
