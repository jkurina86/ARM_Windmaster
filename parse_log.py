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
from datetime import datetime, timezone, timedelta


class RecorderData(NamedTuple):
    """128-byte record structure matching Recorder_Data_t"""
    magic_number: int       # uint32_t
    log_index: int          # uint32_t
    epoch_seconds: int      # uint32_t (seconds since 2000-01-01)
    ms: int                 # uint16_t (milliseconds fraction within current second)
    timegps: int            # uint64_t (nanoseconds since GPS epoch 1980-01-01)
    yaw: float              # float
    pitch: float            # float
    roll: float             # float
    gyro_x: float           # float
    gyro_y: float           # float
    gyro_z: float           # float
    latitude: float         # double
    longitude: float        # double
    altitude: float         # double
    vel_n: float            # float
    vel_e: float            # float
    vel_d: float            # float
    acc_x: float            # float
    acc_y: float            # float
    acc_z: float            # float
    U_axis_speed: int       # int16_t
    V_axis_speed: int       # int16_t
    W_axis_speed: int       # int16_t
    SoS: int                # int16_t (Speed of Sound)
    Temp: int               # int16_t (Temperature)
    timing_offset_ms: int   # int16_t (WM timestamp - VN timestamp, signed ±10ms max)
    # footer_padding[18] - not parsed (reduced from 20 to maintain 128 bytes)


# Struct format string (Little Endian):
# I = uint32_t, Q = uint64_t, f = float, d = double, h = int16_t, B = uint8_t
# Format: magic(I) index(I) epoch(I) ms(H) pad(2x) timegps(Q) YPR(3f) Gyro(3f) Lat/Lon/Alt(3d) Vel(3f) Acc(3f) WindMaster(6h)
RECORD_FORMAT = '<IIIHxxQffffffdddffffffhhhhhh'
RECORD_SIZE = 128
PARSED_SIZE = struct.calcsize(RECORD_FORMAT)  # 108 bytes parsed (20 bytes padding at end)

MAGIC_NUMBER = 0xFACEFACE
# GPS epoch is January 6, 1980 (not January 1!)
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
# Current GPS-UTC leap second offset (as of 2017, 18 leap seconds)
GPS_LEAP_SECONDS = 18
NS_PER_SECOND = 1_000_000_000


def decode_timegps_ns(timegps_ns: int):
    """Convert VectorNav timegps nanoseconds into datetime and components.
    
    GPS time started on January 6, 1980 and does not include leap seconds.
    To convert to UTC, we subtract the accumulated leap seconds.
    """
    seconds, fractional_ns = divmod(timegps_ns, NS_PER_SECOND)
    # Subtract leap seconds to convert GPS time to UTC
    utc_seconds = seconds - GPS_LEAP_SECONDS
    # datetime only tracks microseconds; keep remaining nanoseconds separately
    dt = GPS_EPOCH + timedelta(seconds=utc_seconds, microseconds=fractional_ns // 1000)
    remaining_ns = fractional_ns % 1000
    components = {
        'year': dt.year,
        'month': dt.month,
        'day': dt.day,
        'hour': dt.hour,
        'minute': dt.minute,
        'second': dt.second,
        'millisecond': dt.microsecond // 1000,
        'microsecond': dt.microsecond % 1000,
        'nanosecond': remaining_ns,
    }
    return dt, components


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
    if record.ms >= 1000:
        warnings.append(f"Record {index}: ms out of range ({record.ms})")
    if record.epoch_seconds == 0 and record.ms == 0:
        warnings.append(f"Record {index}: Zero timestamp")
    
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
    
    print(f"[OK] Parsed {len(records)} records")
    
    if warnings:
        print(f"\n[WARN] {len(warnings)} warnings:")
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

    # Time span - using system timestamps reconstructed from epoch seconds
    first_record = records[0]
    last_record = records[-1]

    # Reconstruct absolute milliseconds from epoch seconds + ms
    first_time_ms = (first_record.epoch_seconds * 1000) + first_record.ms
    last_time_ms = (last_record.epoch_seconds * 1000) + last_record.ms

    duration_ms = last_time_ms - first_time_ms
    duration_s = duration_ms / 1000.0

    print(f"First timestamp: {first_record.epoch_seconds}s + {first_record.ms} ms")
    print(f"Last timestamp:  {last_record.epoch_seconds}s + {last_record.ms} ms")
    print(f"Absolute time range (ms): {first_time_ms}-{last_time_ms}")
    print(f"Duration: {duration_s:.2f} seconds ({duration_s/60:.2f} minutes)")

    # Sample rate
    if len(records) > 1 and duration_s > 0:
        avg_rate = len(records) / duration_s
        print(f"Average rate: {avg_rate:.2f} Hz")
    
    # Index continuity
    missing_indices = []
    for i in range(1, len(records)):
        expected = records[i-1].log_index + 1
        if records[i].log_index != expected:
            missing_indices.append((expected, records[i].log_index))
    
    if missing_indices:
        print(f"\n[WARN] {len(missing_indices)} index discontinuities detected")
    else:
        print("\n[OK] All indices are continuous (no dropped packets)")
    
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

    # Timing offset statistics (nearest-neighbor pairing quality)
    timing_offsets = [r.timing_offset_ms for r in records]
    mean_offset = sum(timing_offsets) / len(timing_offsets)
    abs_offsets = [abs(offset) for offset in timing_offsets]
    mean_abs_offset = sum(abs_offsets) / len(abs_offsets)
    max_abs_offset = max(abs_offsets)

    print("\nTiming Quality (Nearest-Neighbor Pairing):")
    print(f"  Offset (WM-VN): [{min(timing_offsets):+d}, {max(timing_offsets):+d}] ms")
    print(f"  Mean offset:    {mean_offset:+.2f} ms")
    print(f"  Mean |offset|:  {mean_abs_offset:.2f} ms")
    print(f"  Max |offset|:   {max_abs_offset:d} ms")

    # Check if any offsets exceed tolerance
    out_of_tolerance = [offset for offset in abs_offsets if offset > 10]
    if out_of_tolerance:
        print(f"  [WARN] {len(out_of_tolerance)} samples exceed ±10ms tolerance")
    else:
        print(f"  [OK] All samples within ±10ms tolerance")


def export_csv(records: List[RecorderData], output_path: Path):
    """Export records to CSV file"""
    if not records:
        print("No records to export")
        return

    fieldnames = RecorderData._fields

    with open(output_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)

        # Header
        writer.writerow(fieldnames)

        # Data
        for record in records:
            writer.writerow(record)

    print(f"\n[OK] Exported {len(records)} records to {output_path}")
def export_txt(records: List[RecorderData], output_path: Path):
    """Export records to human-readable text file"""
    if not records:
        print("No records to export")
        return

    with open(output_path, 'w') as f:
        f.write("=" * 80 + "\n")
        f.write("ARM_Windmaster Data Log - Human Readable Format\n")
        f.write("=" * 80 + "\n")
        f.write(f"Total Records: {len(records)}\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("=" * 80 + "\n\n")

        for i, record in enumerate(records):
            f.write(f"Record #{i} (Index: {record.log_index})\n")
            f.write("-" * 80 + "\n")

            # Timestamp - milliseconds since 2000-01-01 reconstructed via epoch seconds
            absolute_ms = (record.epoch_seconds * 1000) + record.ms
            _, gps_components = decode_timegps_ns(record.timegps)

            # Convert epoch seconds since 2000-01-01 to datetime
            # Note: RTC stores local time, so epoch_seconds is already local time
            epoch_2000 = datetime(2000, 1, 1, 0, 0, 0)
            dt = epoch_2000 + timedelta(seconds=record.epoch_seconds)
            milliseconds = record.ms

            timestamp_str = dt.strftime('%d-%m-%Y %H:%M:%S') + f".{milliseconds:03d}"

            f.write(f"  Timestamp: {timestamp_str}\n")
            f.write(f"            (epoch_seconds={record.epoch_seconds}, ms={record.ms}, absolute_ms={absolute_ms})\n\n")

            # VectorNav (IMU/GPS) Data
            f.write("  VectorNav Data:\n")
            f.write(f"    GPS Time (raw): {record.timegps} ns since 1980-01-01\n")
            f.write(
                "    GPS Time (UTC): "
                f"{gps_components['year']:04d}-{gps_components['month']:02d}-{gps_components['day']:02d} "
                f"{gps_components['hour']:02d}:{gps_components['minute']:02d}:{gps_components['second']:02d}."
                f"{gps_components['millisecond']:03d}{gps_components['microsecond']:03d}{gps_components['nanosecond']:03d}\n"
            )
            f.write(
                "                    "
                f"(year={gps_components['year']}, month={gps_components['month']}, day={gps_components['day']}, "
                f"hour={gps_components['hour']}, minute={gps_components['minute']}, second={gps_components['second']}, "
                f"millisecond={gps_components['millisecond']}, microsecond={gps_components['microsecond']}, "
                f"nanosecond={gps_components['nanosecond']})\n"
            )
            f.write(f"    Attitude (YPR):  {record.yaw:7.2f} {record.pitch:7.2f} {record.roll:7.2f} deg\n")
            f.write(f"    Velocity (NED):  {record.vel_n:8.3f} {record.vel_e:8.3f} {record.vel_d:8.3f} m/s\n")
            f.write(f"    Accel (XYZ):     {record.acc_x:8.3f} {record.acc_y:8.3f} {record.acc_z:8.3f} m/s^2\n")
            f.write(f"    Gyro (XYZ):      {record.gyro_x:8.3f} {record.gyro_y:8.3f} {record.gyro_z:8.3f} rad/s\n")
            f.write(f"    Position (LLA):  {record.latitude:11.6f} {record.longitude:11.6f} {record.altitude:8.2f}m\n\n")

            # WindMaster Data
            f.write("  WindMaster Data:\n")
            f.write(f"    Wind Speed (UVW): {record.U_axis_speed:6d} {record.V_axis_speed:6d} {record.W_axis_speed:6d} (raw)\n")
            f.write(f"    Speed of Sound:   {record.SoS:6d} (raw)\n")
            f.write(f"    Temperature:      {record.Temp:6d} (raw)\n\n")

            # Timing Quality
            f.write("  Timing Quality:\n")
            f.write(f"    Offset (WM - VN): {record.timing_offset_ms:+4d} ms (nearest-neighbor pairing)\n")

            f.write("\n")

        f.write("=" * 80 + "\n")
        f.write("End of Log\n")
        f.write("=" * 80 + "\n")

    print(f"\n[OK] Exported {len(records)} records to {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Parse binary log files from ARM_Windmaster recorder',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('logfile', type=Path, help='Binary log file to parse')
    parser.add_argument('--csv', type=Path, metavar='OUTPUT', help='Export to CSV file')
    parser.add_argument('--txt', type=Path, metavar='OUTPUT', help='Export to human-readable text file')
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
            absolute_ms = (record.epoch_seconds * 1000) + record.ms
            _, gps_components = decode_timegps_ns(record.timegps)
            print(f"Record {i}:")
            print(f"  Timestamp: {record.epoch_seconds}s + {record.ms} ms (absolute {absolute_ms} ms)")
            print(f"  GPS Time (raw): {record.timegps} ns since 1980-01-01")
            print(
                "  GPS Time (UTC): "
                f"{gps_components['year']:04d}-{gps_components['month']:02d}-{gps_components['day']:02d} "
                f"{gps_components['hour']:02d}:{gps_components['minute']:02d}:{gps_components['second']:02d}."
                f"{gps_components['millisecond']:03d}{gps_components['microsecond']:03d}{gps_components['nanosecond']:03d}"
            )
            print(
                "                 "
                f"(year={gps_components['year']}, month={gps_components['month']}, day={gps_components['day']}, "
                f"hour={gps_components['hour']}, minute={gps_components['minute']}, second={gps_components['second']}, "
                f"millisecond={gps_components['millisecond']}, microsecond={gps_components['microsecond']}, "
                f"nanosecond={gps_components['nanosecond']})"
            )
            print(f"  YPR: ({record.yaw:.2f}, {record.pitch:.2f}, {record.roll:.2f}) deg")
            print(f"  Wind UVW: ({record.U_axis_speed}, {record.V_axis_speed}, {record.W_axis_speed})")
            print(f"  Position: ({record.latitude:.6f}, {record.longitude:.6f}, {record.altitude:.2f}m)")
            print(f"  Timing offset (WM-VN): {record.timing_offset_ms:+d} ms")
            print()
    
    # Statistics
    if args.stats:
        print_statistics(records)
    
    # CSV export
    if args.csv:
        export_csv(records, args.csv)

    # Text export
    if args.txt:
        export_txt(records, args.txt)

    print("\n[OK] Done")


if __name__ == '__main__':
    main()
