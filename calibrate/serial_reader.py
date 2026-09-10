"""
calibrate/serial_reader.py - Serial data to JSONL converter.

Reads comma-separated serial stream from sensor and saves to formatted JSONL.
Auto-detects format and applies metadata defaults.

Usage:
    python serial_reader.py --port /dev/ttyUSB0 --baud 115200 [--meta key=value]
"""

import argparse
import json
import sys
import time
from typing import Any, Dict, List, Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


# Standardized measurement types
MEASUREMENT_TYPES = {
    "general": "General movement/shaking test",
    "gyro_ref_positive_x": "Gyro positive reference on X axis",
    "gyro_ref_negative_x": "Gyro negative reference on X axis",
    "gyro_ref_positive_y": "Gyro positive reference on Y axis",
    "gyro_ref_negative_y": "Gyro negative reference on Y axis",
    "gyro_ref_positive_z": "Gyro positive reference on Z axis",
    "gyro_ref_negative_z": "Gyro negative reference on Z axis",
    "accel_ref_positive_x": "Accel positive reference on X axis (e.g., level)",
    "accel_ref_negative_x": "Accel negative reference on X axis (e.g., inverted)",
    "accel_ref_positive_y": "Accel positive reference on Y axis",
    "accel_ref_negative_y": "Accel negative reference on Y axis",
    "accel_ref_positive_z": "Accel positive reference on Z axis (1g)",
    "accel_ref_negative_z": "Accel negative reference on Z axis (-1g)",
}

# Default metadata with standardized measurement types
DEFAULT_METADATA: Dict[str, Any] = {
    "sensor_id": "unknown",
    "sample_rate_hz": 100,
    "unit_accel": "g",
    "unit_gyro": "deg/s",
    "unit_mag": "gauss",
    "recording_type": "general",
    "reference_force": 0,
}


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Read serial sensor data and save to JSONL format.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Serial port (e.g., /dev/ttyUSB0, COM3).",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate for serial communication.",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=1,
        help="Read timeout in seconds.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0,
        help="Recording duration in seconds (0 = until Ctrl+C).",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="sensor_data.jsonl",
        help="Output JSONL file path.",
    )
    parser.add_argument(
        "--meta",
        action="append",
        dest="metadata",
        default=[],
        metavar="KEY=VALUE",
        help="Add metadata key=value pair (can be specified multiple times).",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=100,
        help="Number of records to buffer before writing to file.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print detailed progress information.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Test connection without recording data.",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List available serial ports and exit.",
    )
    return parser.parse_args(argv)


def list_serial_ports() -> List[Dict[str, Any]]:
    """List all available serial ports."""
    ports = []
    for port in serial.tools.list_ports.comports():
        ports.append(
            {
                "dev": port.device,
                "description": port.description,
                "manufacturer": port.manufacturer,
                "product": port.product,
            }
        )
    return ports


def parse_serial_line(line: str) -> Optional[Dict[str, Any]]:
    """
    Parse a comma-separated serial line into a structured record.

    Expected formats (timestamp, gyro, accel, mag):
        10 fields: timestamp, gyrox, gyroy, gyroz, accelx, accely, accelz, magx, magy, magz
        9 fields: gyrox, gyroy, gyroz, accelx, accely, accelz, magx, magy, magz
        6 fields: gyrox, gyroy, gyroz, accelx, accely, accelz

    Args:
        line: Raw serial line string

    Returns:
        Parsed record dict or None if parsing fails.
    """
    if not line.strip():
        return None

    # Remove spaces and normalize
    line = line.strip().replace(" ", "")

    parts = line.split(",")

    # Expected: 10, 9, or 6 fields (gyro + accel + mag)
    if len(parts) not in (10, 9, 6):
        # Unknown format, return None
        return None

    # Parse based on field count
    try:
        if len(parts) == 10:
            # Format: timestamp, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z
            timestamp = float(parts[0]) if parts[0] else None
            gyro = [float(parts[1]), float(parts[2]), float(parts[3])]
            accel = [float(parts[4]), float(parts[5]), float(parts[6])]
            mag = [float(parts[7]), float(parts[8]), float(parts[9])]
            has_timestamp = timestamp is not None

        elif len(parts) == 9:
            # Format: gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z
            gyro = [float(parts[0]), float(parts[1]), float(parts[2])]
            accel = [float(parts[3]), float(parts[4]), float(parts[5])]
            mag = [float(parts[6]), float(parts[7]), float(parts[8])]
            has_timestamp = False

        elif len(parts) == 6:
            # Format: gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z
            gyro = [float(parts[0]), float(parts[1]), float(parts[2])]
            accel = [float(parts[3]), float(parts[4]), float(parts[5])]
            mag = [0.0, 0.0, 0.0]  # No magnetometer data
            has_timestamp = False

        else:
            return None

        record = {
            "gyro_x": gyro[0],
            "gyro_y": gyro[1],
            "gyro_z": gyro[2],
            "accel_x": accel[0],
            "accel_y": accel[1],
            "accel_z": accel[2],
            "mag_x": mag[0],
            "mag_y": mag[1],
            "mag_z": mag[2],
        }

        if has_timestamp:
            record["timestamp_ms"] = timestamp

        return record

    except (ValueError, IndexError) as e:
        # Parse error, log and skip
        if args.verbose:
            print(f"Parse error (line: {line[:50]}...): {e}")
        return None


def write_jsonl(filepath: str, records: List[Dict[str, Any]]) -> None:
    """Write records to JSONL file."""
    with open(filepath, "w", encoding="utf-8") as f:
        for record in records:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")


def build_metadata(
    args: argparse.Namespace, reference_force: Optional[str] = None
) -> Dict[str, Any]:
    """Build metadata from arguments, including reference force if specified."""
    # Start with defaults
    metadata: Dict[str, Any] = DEFAULT_METADATA.copy()

    # Apply user-provided metadata
    for key_value in args.metadata or []:
        if "=" in key_value:
            key, value = key_value.split("=", 1)
            metadata[key.strip()] = value.strip()

    # Override recording_type if explicitly set, otherwise use default
    if "recording_type" not in metadata:
        metadata["recording_type"] = DEFAULT_METADATA["recording_type"]

    # If reference_force is specified, it overrides any existing force value
    if reference_force:
        metadata["reference_force"] = reference_force

    return metadata


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point."""
    args = parse_args(argv)

    # Handle --list-ports
    if args.list_ports:
        ports = list_serial_ports()
        if ports:
            print("Available serial ports:")
            for port in ports:
                print(f"  {port['dev']}: {port['description']}")
                if port["manufacturer"]:
                    print(f"    Manufacturer: {port['manufacturer']}")
                if port["product"]:
                    print(f"    Product: {port['product']}")
            return 0
        else:
            print("No serial ports found.")
            return 0

    # Check if serial is available
    if serial is None:
        print("Error: pyserial not installed. Run: pip install pyserial")
        return 1

    # Test connection in dry-run mode
    if args.dry_run:
        print(f"Testing connection to {args.port} at {args.baud} baud...")
        try:
            set = serial.Serial(args.port, args.baud, timeout=args.timeout)
            time.sleep(0.5)
            if set.in_waiting > 0:
                data = set.readline().decode("utf-8", errors="ignore").strip()
                if data:
                    print(f"  Connection OK. Sample data: {data[:80]}...")
                else:
                    print("  Connection OK (no data received).")
            else:
                print("  Connection OK (empty).")
            set.close()
            return 0
        except serial.SerialException as e:
            print(f"Connection error: {e}")
            return 1

    # Open serial port
    print(f"Opening serial port {args.port} at {args.baud} baud...")
    try:
        set = serial.Serial(
            args.port,
            args.baud,
            timeout=args.timeout,
            bytesize=8,
            parity="N",
            stopbits=1,
        )
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        return 1

    if args.verbose:
        print(f"Port opened successfully. Ready to read data.")

    # Build metadata - reference_force can be passed separately for calibration references
    metadata = build_metadata(args)

    # Recording state
    batch: List[Dict[str, Any]] = []
    start_time = time.time()
    total_records = 0
    errors = 0

    try:
        while True:
            # Check duration limit
            if args.duration > 0:
                elapsed = time.time() - start_time
                if elapsed >= args.duration:
                    if args.verbose:
                        print(f"Duration limit ({args.duration}s) reached.")
                    break

            # Read available data
            if set.in_waiting > 0:
                # Read all available data at once
                data = set.read(set.in_waiting).decode("utf-8", errors="ignore")

                # Split into lines
                lines = data.split("\n")

                for line in lines:
                    record = parse_serial_line(line)
                    if record is not None:
                        batch.append(record)
                        total_records += 1

                        # Flush batch when it reaches threshold
                        if len(batch) >= args.batch_size:
                            write_jsonl(args.output, batch)
                            batch.clear()
                            if args.verbose:
                                print(f"  Wrote {len(batch)} records to {args.output}")
                    else:
                        errors += 1

                if args.verbose and total_records % 1000 == 0:
                    print(f"  Total records: {total_records}, Errors: {errors}")

            # Small sleep to avoid busy-waiting
            time.sleep(0.01)

    except KeyboardInterrupt:
        if args.verbose:
            print("\nRecording interrupted by user (Ctrl+C).")
    finally:
        set.close()

    # Write remaining batch
    if batch:
        write_jsonl(args.output, batch)
        if args.verbose:
            print(f"Wrote {len(batch)} remaining records to {args.output}")

    # Print summary
    print("\nRecording complete.")
    print(f"  Total records: {total_records}")
    print(f"  Parse errors: {errors}")
    print(f"  Output file: {args.output}")
    print(f"  Metadata: {json.dumps(metadata, indent=2)}")

    if errors > 0:
        print(f"\nWarning: {errors} records could not be parsed.")
        print("Check data format or baud rate settings.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
