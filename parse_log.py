#!/usr/bin/env python3
"""
Binary Log Parser for ARM_Windmaster Data Recorder
===================================================

Parses binary log files created by the STM32L476 data recorder.
Each record is 128 bytes containing paired WindMaster and VectorNav data.

Usage:
    python parse_log.py log_12345678.bin
    python parse_log.py log_12345678.bin --csv output.csv
    python parse_log.py log_12345678.bin --stats
"""

import struct
import sys
import argparse
from pathlib import Path
from typing import NamedTuple, List
import csv


class RecorderData(NamedTuple):
    """128-byte record structure matching Recorder_Data_t"""
    magic_number: int       # uint32_t
    log_index: int          # uint32_t
    timegps: int            # uint64_t (microseconds)
    yaw: float              # float
    pitch: float            # float
    roll: float             # float
    vel_n: float            # float
    vel_e: float            # float
    vel_d: float            # float
    acc_x: float            # float
    acc_y: float            # float
    acc_z: float            # float
    gyro_x: float           # float
    gyro_y: float           # float
    gyro_z: float           # float
    latitude: float         # double
    longitude: float        # double
    altitude: float         # double
    U_axis_speed: int       # int16_t
    V_axis_speed: int       # int16_t
    W_axis_speed: int       # int16_t
    SoS: int                # int16_t (Speed of Sound)
    Temp: int               # int16_t (Temperature)
    # footer_padding[30] - not parsed


# Struct format string (Little Endian):
# I = uint32_t, Q = uint64_t, f = float, d = double, h = int16_t
RECORD_FORMAT = '<IIQfffffffffdddhhhhhh'
RECORD_SIZE = 128
PARSED_SIZE = struct.calcsize(RECORD_FORMAT)  # Should be 98 bytes (30 bytes padding at end)

MAGIC_NUMBER = 0xFACEFACE


def parse_record(data: bytes) -> RecorderData:
    """Parse a single 128-byte record"""
    if len(data) != RECORD_SIZE:
        raise ValueError(f"Record must be {RECORD_SIZE} bytes, got {len(data)}")
    
    # Unpack the first 98 bytes (30 bytes padding ignored)
    values = struct.unpack(RECORD_FORMAT, data[:PARSED_SIZE])
    
    return RecorderData(*values)


def validate_record(record: RecorderData, index: int) -> List[str]:
    """Validate a record and return list of warnings"""
    warnings = []
    
    if record.magic_number != MAGIC_NUMBER:
        warnings.append(f"Record {index}: Invalid magic number 0x{record.magic_number:08X}")
    
    if record.log_index != index:
        warnings.append(f"Record {index}: Index mismatch (expected {index}, got {record.log_index})")
    
    # Check for reasonable timestamp (not zero and not absurdly large)
    if record.timegps == 0:
        warnings.append(f"Record {index}: Zero timestamp")
    elif record.timegps > 1e15:  # ~31 years in microseconds
        warnings.append(f"Record {index}: Unrealistic timestamp {record.timegps}")
    
    return warnings


def parse_log_file(filepath: Path, verbose: bool = False) -> List[RecorderData]:
    """Parse entire binary log file"""
    records = []
    warnings = []
    
    file_size = filepath.stat().st_size
    expected_records = file_size // RECORD_SIZE
    
    print(f"Parsing: {filepath.name}")
    print(f"File size: {file_size:,} bytes")
    print(f"Expected records: {expected_records}")
    print("-" * 60)
    
    with open(filepath, 'rb') as f:
        index = 0
        while True:
            data = f.read(RECORD_SIZE)
            if len(data) == 0:
                break  # EOF
            
            if len(data) < RECORD_SIZE:
                warnings.append(f"Incomplete record at end of file ({len(data)} bytes)")
                break
            
            try:
                record = parse_record(data)
                records.append(record)
                
                # Validate
                rec_warnings = validate_record(record, index)
                warnings.extend(rec_warnings)
                
                if verbose and index % 100 == 0:
                    print(f"Parsed record {index}: timestamp={record.timegps} µs")
                
                index += 1
                
            except Exception as e:
                warnings.append(f"Error parsing record {index}: {e}")
                break
    
    print(f"✓ Parsed {len(records)} records")
    
    if warnings:
        print(f"\n⚠ {len(warnings)} warnings:")
        for warn in warnings[:10]:  # Show first 10 warnings
            print(f"  - {warn}")
        if len(warnings) > 10:
            print(f"  ... and {len(warnings) - 10} more")
    
    return records


def print_statistics(records: List[RecorderData]):
    """Print statistical summary"""
    if not records:
        print("No records to analyze")
        return
    
    print("\n" + "=" * 60)
    print("STATISTICS")
    print("=" * 60)
    
    print(f"Total records: {len(records)}")
    
    # Time span
    first_time = records[0].timegps
    last_time = records[-1].timegps
    duration_us = last_time - first_time
    duration_s = duration_us / 1e6
    
    print(f"First timestamp: {first_time} µs")
    print(f"Last timestamp:  {last_time} µs")
    print(f"Duration: {duration_s:.2f} seconds ({duration_s/60:.2f} minutes)")
    
    # Sample rate
    if len(records) > 1:
        avg_rate = len(records) / duration_s
        print(f"Average rate: {avg_rate:.2f} Hz")
    
    # Index continuity
    missing_indices = []
    for i in range(1, len(records)):
        expected = records[i-1].log_index + 1
        if records[i].log_index != expected:
            missing_indices.append((expected, records[i].log_index))
    
    if missing_indices:
        print(f"\n⚠ {len(missing_indices)} index discontinuities detected")
    else:
        print("\n✓ All indices are continuous (no dropped packets)")
    
    # Data ranges
    print("\nData Ranges:")
    print(f"  Yaw:   [{min(r.yaw for r in records):.2f}, {max(r.yaw for r in records):.2f}] deg")
    print(f"  Pitch: [{min(r.pitch for r in records):.2f}, {max(r.pitch for r in records):.2f}] deg")
    print(f"  Roll:  [{min(r.roll for r in records):.2f}, {max(r.roll for r in records):.2f}] deg")
    
    u_speeds = [r.U_axis_speed for r in records]
    v_speeds = [r.V_axis_speed for r in records]
    w_speeds = [r.W_axis_speed for r in records]
    
    print(f"  Wind U: [{min(u_speeds)}, {max(u_speeds)}]")
    print(f"  Wind V: [{min(v_speeds)}, {max(v_speeds)}]")
    print(f"  Wind W: [{min(w_speeds)}, {max(w_speeds)}]")


def export_csv(records: List[RecorderData], output_path: Path):
    """Export records to CSV file"""
    if not records:
        print("No records to export")
        return
    
    fieldnames = RecorderData._fields[:-1]  # Exclude padding
    
    with open(output_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Header
        writer.writerow(fieldnames)
        
        # Data
        for record in records:
            writer.writerow(record[:-1])  # Exclude padding
    
    print(f"\n✓ Exported {len(records)} records to {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Parse binary log files from ARM_Windmaster recorder',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('logfile', type=Path, help='Binary log file to parse')
    parser.add_argument('--csv', type=Path, metavar='OUTPUT', help='Export to CSV file')
    parser.add_argument('--stats', action='store_true', help='Print statistics')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    parser.add_argument('--head', type=int, metavar='N', help='Show first N records')
    
    args = parser.parse_args()
    
    if not args.logfile.exists():
        print(f"Error: File not found: {args.logfile}")
        sys.exit(1)
    
    # Parse the file
    records = parse_log_file(args.logfile, verbose=args.verbose)
    
    if not records:
        print("No valid records found")
        sys.exit(1)
    
    # Show first N records
    if args.head:
        print(f"\nFirst {args.head} records:")
        print("-" * 60)
        for i, record in enumerate(records[:args.head]):
            print(f"Record {i}:")
            print(f"  Timestamp: {record.timegps} µs")
            print(f"  YPR: ({record.yaw:.2f}, {record.pitch:.2f}, {record.roll:.2f}) deg")
            print(f"  Wind UVW: ({record.U_axis_speed}, {record.V_axis_speed}, {record.W_axis_speed})")
            print(f"  Position: ({record.latitude:.6f}, {record.longitude:.6f}, {record.altitude:.2f}m)")
            print()
    
    # Statistics
    if args.stats:
        print_statistics(records)
    
    # CSV export
    if args.csv:
        export_csv(records, args.csv)
    
    print("\n✓ Done")


if __name__ == '__main__':
    main()
