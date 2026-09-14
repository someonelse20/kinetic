"""
calibrate/calibrate.py - CLI tool for sensor calibration.

Groups calibration files by metadata and invokes calibration algorithms.
Minimal input: specify data directory, auto-detect reference vs measurement files.

Measurement types:
    - general_wiggle: General movement/shaking test
    - gyro_ref_positive_x/y/z: Gyro reference on each axis (positive rotation)
    - gyro_ref_negative_x/y/z: Gyro reference on each axis (negative rotation)
    - accel_ref_positive_x/y/z: Accel reference on each axis (positive force)
    - accel_ref_negative_x/y/z: Accel reference on each axis (negative force)
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from numpy import sort

from read_file import read_jsonl

import cal_accel
import cal_gyro


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Run sensor calibration on JSONL data files.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--data-dir",
        type=str,
        default=".",
        help="Directory containing calibration JSONL files.",
    )
    parser.add_argument(
        "--sensor",
        type=str,
        choices=["accel", "gyro", "all"],
        default="all",
        help="Sensor(s) to calibrate (accel, gyro, or all).",
    )
    parser.add_argument(
        "--output",
        type=str,
        choices=["bias", "sensitivity", "alignment", "all"],
        default="all",
        help="Calibration output to compute (bias, sensitivity, alignment, or all).",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print detailed progress information.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="List files to process without running calibration.",
    )
    return parser.parse_args(argv)


def sort_files(jsonl_files: List[Path], verbose: bool):
    files = {}
    for filepath in jsonl_files:
        schema, data = read_jsonl(str(filepath))
        recording_type = ""

        if data and schema:
            recording_type = schema.get("recording_type")
            files[recording_type] = filepath
        else:
            print("could not parse recording type")

    return files


def run_calibration(args, verbose: bool):
    pass


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point."""
    args = parse_args(argv)

    if args.dry_run:
        print(f"Dry run: would calibrate in {args.data_dir}")
        return 0

    result = run_calibration(args, args.verbose)

    if result:
        print("\nResults saved to stdout (pipe to file if needed)")
        print(json.dumps(result, indent=2))
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())
