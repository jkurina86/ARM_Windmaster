#!/usr/bin/env python3
"""
Log Parser for ARM_Windmaster Data Recorder
===================================================

Parses binary log files created by the data recorder.
Each record is 128 bytes containing paired WindMaster and VectorNav data.

Usage:
    python parse_log.py log_12345678.bin
    python parse_log.py log_12345678.bin --csv output.csv
    python parse_log.py log_12345678.bin --txt output.txt
    python parse_log.py log_12345678.bin --csv --txt
    python parse_log.py log_12345678.bin --csv --output-dir ./output
    
    # Batch process all .bin files in a directory (requires --csv/--txt)
    python parse_log.py ./logs/ --txt
    python parse_log.py ./logs/ --csv --txt
    python parse_log.py ./logs/ --csv --txt --output-dir ./output
"""

import struct
import sys
import argparse
from pathlib import Path
from typing import NamedTuple, List, Optional
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
    SoS: int                # uint16_t (Speed of Sound, 0.01 m/s)
    Temp: int               # int16_t (Temperature)
    timing_offset_ms: int   # int16_t (WM timestamp - VN timestamp, signed ±10ms max)
    # footer_padding[18] - not parsed


# Struct format string (Little Endian):
# I = uint32_t, Q = uint64_t, f = float, d = double, h = int16_t, H = uint16_t
# Format: magic(I) index(I) epoch(I) ms(H) pad(2x) timegps(Q) YPR(3f) Gyro(3f) Lat/Lon/Alt(3d) Vel(3f) Acc(3f)
#         WindMaster(U,V,W = 3h, SoS = H, Temp = h, timing_offset = h)
RECORD_FORMAT = '<IIIHxxQffffffdddffffffhhhHhh'
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
        warnings.append(f"Record {record.log_index}: Invalid magic number 0x{record.magic_number:08X}")

    # Check for reasonable timestamp (not zero and not absurdly large)
    if record.ms >= 1000:
        warnings.append(f"Record {record.log_index}: ms out of range ({record.ms})")
    if record.epoch_seconds == 0 and record.ms == 0:
        warnings.append(f"Record {record.log_index}: Zero timestamp")
    
    return warnings


def parse_log_file(filepath: Path) -> List[RecorderData]:
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
        f.write("ARM_Windmaster Data Log\n")
        f.write(f"Total Records: {len(records)}\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

        for i, record in enumerate(records):
            f.write(f"Record # {record.log_index}\n")
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

            f.write(f"Timestamp: {timestamp_str}\n")

            # VectorNav (IMU/GPS) Data
            f.write("VectorNav Data:\n")
            f.write(f"GPS Time (RAW): {record.timegps} ns since 1980-01-01\n")
            f.write(
                "GPS Time: "
                f"{gps_components['year']:04d}-{gps_components['month']:02d}-{gps_components['day']:02d} "
                f"{gps_components['hour']:02d}:{gps_components['minute']:02d}:{gps_components['second']:02d}."
                f"{gps_components['millisecond']:03d}{gps_components['microsecond']:03d}{gps_components['nanosecond']:03d}\n"
            )
            f.write(f"Attitude (YPR):  {record.yaw:7.2f} {record.pitch:7.2f} {record.roll:7.2f} deg\n")
            f.write(f"Velocity (NED):  {record.vel_n:8.3f} {record.vel_e:8.3f} {record.vel_d:8.3f} m/s\n")
            f.write(f"Accel (XYZ):     {record.acc_x:8.3f} {record.acc_y:8.3f} {record.acc_z:8.3f} m/s^2\n")
            f.write(f"Gyro (XYZ):      {record.gyro_x:8.3f} {record.gyro_y:8.3f} {record.gyro_z:8.3f} rad/s\n")
            f.write(f"Position (LLA):  {record.latitude:11.6f} {record.longitude:11.6f} {record.altitude:8.2f} m\n\n")

            # WindMaster Data
            f.write("WindMaster Data:\n")
            f.write(f"Wind Speed (UVW): {record.U_axis_speed} {record.V_axis_speed} {record.W_axis_speed} mm/s\n")
            f.write(f"Speed of Sound:   {record.SoS/100:.2f} m/s\n")

            # Timing Quality
            f.write(f"Timing Offset: {record.timing_offset_ms:+4d} ms\n")
            f.write("\n")

        f.write("=" * 80 + "\n")
        f.write("End of Log\n")
        f.write("=" * 80 + "\n")

    print(f"\n[OK] Exported {len(records)} records to {output_path}")


def scan_bin_files(directory: Path) -> List[Path]:
    """Scan directory for all .bin files"""
    if not directory.is_dir():
        raise ValueError(f"Not a directory: {directory}")
    
    bin_files = sorted(directory.glob('*.bin'))
    return bin_files


def get_output_path(input_path: Path, output_dir: Optional[Path] = None, extension: str = '.txt') -> Path:
    """Generate output path based on input path and optional output directory"""
    if output_dir:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        return output_dir / (input_path.stem + extension)
    else:
        return input_path.parent / (input_path.stem + extension)


def process_single_file(
    input_path: Path,
    csv_output: Optional[Path] = None,
    txt_output: Optional[Path] = None,
    auto_txt: bool = False,
    auto_csv: bool = False,
    output_dir: Optional[Path] = None
) -> tuple:
    """Process a single log file and return (success, records, errors)"""
    try:
        # Parse the file
        records = parse_log_file(input_path)
        
        if not records:
            return (False, [], ["No valid records found"])
        
        # CSV export
        if csv_output:
            export_csv(records, csv_output)
        elif auto_csv:
            csv_path = get_output_path(input_path, output_dir, '.csv')
            export_csv(records, csv_path)
        
        # Text export
        if txt_output:
            export_txt(records, txt_output)
        elif auto_txt:
            txt_path = get_output_path(input_path, output_dir, '.txt')
            export_txt(records, txt_path)
        
        return (True, records, [])
        
    except Exception as e:
        return (False, [], [str(e)])


def process_batch_files(
    bin_files: List[Path],
    auto_txt: bool = False,
    auto_csv: bool = False,
    output_dir: Optional[Path] = None
) -> dict:
    """Process multiple log files in batch mode"""
    results = {
        'total': len(bin_files),
        'success': 0,
        'failed': 0,
        'errors': [],
        'total_records': 0
    }
    
    print(f"\n{'=' * 60}")
    print(f"BATCH PROCESSING: {len(bin_files)} files")
    print(f"{'=' * 60}\n")
    
    for idx, bin_file in enumerate(bin_files, 1):
        print(f"\n[{idx}/{len(bin_files)}] Processing: {bin_file.name}")
        print("-" * 60)
        
        success, records, errors = process_single_file(
            bin_file,
            csv_output=None,
            txt_output=None,
            auto_txt=auto_txt,
            auto_csv=auto_csv,
            output_dir=output_dir
        )
        
        if success:
            results['success'] += 1
            results['total_records'] += len(records)
            print(f"[OK] Processed {len(records)} records")
        else:
            results['failed'] += 1
            print(f"[FAIL] Failed to process {bin_file.name}")
            for error in errors:
                print(f"  Error: {error}")
                results['errors'].append(f"{bin_file.name}: {error}")
    
    # Print summary
    print(f"\n{'=' * 60}")
    print("BATCH PROCESSING SUMMARY")
    print(f"{'=' * 60}")
    print(f"Total files:     {results['total']}")
    print(f"Successfully:    {results['success']}")
    print(f"Failed:          {results['failed']}")
    print(f"Total records:   {results['total_records']:,}")
    
    if results['errors']:
        print(f"\nErrors encountered:")
        for error in results['errors'][:10]:
            print(f"  - {error}")
        if len(results['errors']) > 10:
            print(f"  ... and {len(results['errors']) - 10} more")
    
    print(f"{'=' * 60}")
    
    return results


def main():
    parser = argparse.ArgumentParser(
        description='Parse binary log files from ARM_Windmaster recorder',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('input_path', type=Path, help='Binary log file or directory containing .bin files')
    parser.add_argument('--csv', type=Path, metavar='OUTPUT', nargs='?', const=True, help='Export to CSV file (auto-generate if no path specified)')
    parser.add_argument('--txt', type=Path, metavar='OUTPUT', nargs='?', const=True, help='Export to text file (auto-generate if no path specified)')
    parser.add_argument('--output-dir', type=Path, metavar='DIR', help='Directory for auto-generated output files')
    
    args = parser.parse_args()

    if not args.csv and not args.txt:
        print("[ERROR] Nothing to do: specify --csv and/or --txt")
        sys.exit(1)
    
    # Check if input exists
    if not args.input_path.exists():
        print(f"Error: Path not found: {args.input_path}")
        sys.exit(1)
    
    # Determine if input is a file or directory
    is_directory = args.input_path.is_dir()
    
    # Validate options for directory mode
    if is_directory:
        # In directory mode, --csv/--txt without path means auto-generate for all files
        # If a path is provided, it's invalid in directory mode
        if args.csv and isinstance(args.csv, Path):
            print("[ERROR] --csv with a specific output path is not supported in directory mode")
            print("        Use --csv without a path to auto-generate CSV files, optionally with --output-dir")
            sys.exit(1)
        
        if args.txt and isinstance(args.txt, Path):
            print("[ERROR] --txt with a specific output path is not supported in directory mode")
            print("        Use --txt without a path to auto-generate TXT files, optionally with --output-dir")
            sys.exit(1)
        
        # Scan for .bin files
        bin_files = scan_bin_files(args.input_path)
        
        if not bin_files:
            print(f"[INFO] No .bin files found in {args.input_path}")
            sys.exit(0)
        
        print(f"Found {len(bin_files)} .bin file(s) in {args.input_path}")
        
        # Process batch - only generate outputs explicitly requested via flags
        results = process_batch_files(
            bin_files,
            auto_txt=bool(args.txt),
            auto_csv=bool(args.csv),
            output_dir=args.output_dir
        )
        
        # Exit with error code if any files failed
        if results['failed'] > 0:
            sys.exit(1)
        
    else:
        # Single file mode
        # Process single file
        success, records, errors = process_single_file(
            args.input_path,
            csv_output=args.csv if isinstance(args.csv, Path) else None,
            txt_output=args.txt if isinstance(args.txt, Path) else None,
            auto_txt=args.txt is True,
            auto_csv=args.csv is True,
            output_dir=args.output_dir
        )
        
        if not success:
            print(f"[ERROR] Failed to process {args.input_path.name}")
            for error in errors:
                print(f"  {error}")
            sys.exit(1)
        
        print("\n[OK] Done")


if __name__ == '__main__':
    main()
