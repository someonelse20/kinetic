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
            "bias": [0, 0, 0],
            "sensitivity": [1, 1, 1],
            "alignment": [[1, 1, 1], [1, 1, 1], [1, 1, 1]],
        },
        "accel": {
            "bias": [0, 0, 0],
            "sensitivity": [1, 1, 1],
            "alignment": [[1, 1, 1], [1, 1, 1], [1, 1, 1]],
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
        reference_schema = None
        if schema and data:
            reference_schema = schema.get("reference")
            if not reference_schema:
                print("could not parse file ", ref_file)
                return calibration
        else:
            print("could not parse file ", ref_file)
            return calibration

        axes = parse_jsonl(data)
        gyro_refs = {
            "x": {"pos": [[]], "neg": [[]]},
            "y": {"pos": [[]], "neg": [[]]},
            "z": {"pos": [[]], "neg": [[]]},
        }
        accel_refs = {
            "x": {"pos": [[]], "neg": [[]]},
            "y": {"pos": [[]], "neg": [[]]},
            "z": {"pos": [[]], "neg": [[]]},
        }

        # TODO: Add check for if the reference force is the same across files.
        if reference_schema.get("sensor") == "gyro":
            # Look for static file for gyro bias calibration (gyro ref with 0 force)
            if reference_schema.get("force") == 0:
                for i in range(3):
                    calibration["gyro"]["bias"][i] = cal_gyro.bias_calibration(
                        axes["gyro"][i]
                    ).tolist()
            elif reference_schema.get("force") > 0:
                gyro_refs[reference_schema.get("axis")]["pos"] = axes["gyro"]
            else:
                gyro_refs[reference_schema.get("axis")]["neg"] = axes["gyro"]
        elif reference_schema.get("sensor") == "accel":
            # Look for static file for gyro bias calibration while checking if already calculated (accel ref)
            if reference_schema.get("force") == 0 and calibration["gyro"]["bias"] == [
                0,
                0,
                0,
            ]:
                for i in range(3):
                    calibration["gyro"]["bias"] = cal_gyro.bias_calibration(
                        axes["gyro"][i]
                    ).tolist()
            elif reference_schema.get("force") > 0:
                accel_refs[reference_schema.get("axis")]["pos"] = axes["accel"]
            else:
                accel_refs[reference_schema.get("axis")]["neg"] = axes["accel"]

        # Run gyro and accel sensitivity calibration if positive and negative reference files are found.
        axes_index = ("x", "y", "z")
        for i in range(3):
            if gyro_refs[axes_index[i]]["pos"] != [[]] and gyro_refs[axes_index[i]][
                "neg"
            ] != [[]]:
                calibration["gyro"]["sensitivity"][i] = (
                    cal_gyro.sensitivity_calibration(
                        gyro_refs[axes_index[i]]["pos"],
                        gyro_refs[axes_index[i]]["neg"],
                    ).tolist()
                )

            if accel_refs[axes_index[i]]["pos"] != [[]] and accel_refs[axes_index[i]][
                "neg"
            ] != [[]]:
                calibration["accel"]["bias"][i] = cal_accel.bias_calibration(
                    accel_refs[axes_index[i]]["pos"],
                    accel_refs[axes_index[i]]["neg"],
                ).tolist()
                calibration["accel"]["sensitivity"][i] = (
                    cal_accel.sensitivity_calibration(
                        accel_refs[axes_index[i]]["pos"],
                        accel_refs[axes_index[i]]["neg"],
                    ).tolist()
                )

    return calibration


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point."""
    args = parse_args(argv)

    if args.dry_run:
        print(f"Dry run: would calibrate in {args.data_dir}")
        return 0

    result = run_calibration(args, args.verbose)

    if result:
        print(result)

    if result and False:
        print("\nResults saved to stdout (pipe to file if needed)")
        print(json.dumps(result, indent=2))
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())
