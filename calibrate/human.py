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


def parse_jsonl(data):
    axes = {
        "gyro": [[], [], []],
        "accel": [[], [], []],
        "mag": [[], [], []],
    }
    axes_index = ("x", "y", "z")
    for rec in data:
        for sensor in ("gyro", "accel", "mag"):
            for i in range(3):
                key = f"{sensor}"
                if key in rec:
                    axes[sensor][i].append(float(rec[key][i]))
    return axes


def run_calibration(args, verbose: bool):
    calibration = {
        "gyro": {
            "bias": [],
            "sensitivity": [],
            "alignment": [],
        },
        "accel": {
            "bias": [],
            "sensitivity": [],
            "alignment": [],
        },
        "mag": {
            "hard_iorn": [],
            "soft_iorn": [],
        },
    }

    calibrate_dir = Path(__file__).resolve().parent
    jsonl_files = list(calibrate_dir.glob("*.jsonl")) + list(
        calibrate_dir.glob("*.JSONL")
    )

    if not jsonl_files:
        print(f"No JSONL files found in {calibrate_dir}")
        return None

    for ref_file in jsonl_files:
        schema, data = read_jsonl(str(ref_file))
        recording_type = ""
        if schema and data:
            recording_type = schema.get("recording_type")
        else:
            print("could not parse file ", ref_file)
            return calibration

        axes = parse_jsonl(data)
        gyro_refs = {
            "x": {"pos": [[]], "neg": [[]]},
            "y": {"pos": [[]], "neg": [[]]},
            "z": {"pos": [[]], "neg": [[]]},
        }
        references = {
            "gyro": {"pos": [[]], "neg": [[]]},
            "accel": {"pos": [[]], "neg": [[]]},
            "mag": {"pos": [[]], "neg": [[]]},
        }
        ref_force = schema.get("reference_force")

        if ref_force == None:
            print("error in metadata: format no reference force found")
            return calibration

        # TODO: Add check for if the reference force is the same across files.
        if "gyro" in str(recording_type):
            # Look for static file for gyro bias calibration (gyro ref with 0 force)
            if ref_force == 0:
                calibration["gyro"]["bias"] = cal_gyro.bias_calibration(
                    axes["gyro"]
                ).tolist()
            elif ref_force > 0:
                references["gyro"]["pos"] = axes["gyro"]
            else:
                references["gyro"]["neg"] = axes["gyro"]
        elif "accel" in str(recording_type):
            # Look for static file for gyro bias calibration while checking if already calculated (accel ref)
            if ref_force == 0 and calibration["gyro"]["bias"] == []:
                calibration["gyro"]["bias"] = cal_gyro.bias_calibration(
                    axes["gyro"]
                ).tolist()
            elif ref_force > 0:
                references["accel"]["pos"] = axes["accel"]
            else:
                references["accel"]["neg"] = axes["accel"]

        # Run gyro and accel sensitivity calibration if positive and negative reference files are found.
        if references["gyro"]["pos"] != [[]] and references["gyro"]["neg"] != [[]]:
            calibration["gyro"]["sensitivity"] = cal_gyro.sensitivity_calibration(
                references["gyro"]["pos"], references["gyro"]["pos"]
            ).tolist()

        if references["accel"]["pos"] != [[]] and references["accel"]["neg"] != [[]]:
            calibration["accel"]["sensitivity"] = cal_accel.sensitivity_calibration(
                references["accel"]["pos"], references["accel"]["pos"]
            ).tolist()

    return calibration


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
