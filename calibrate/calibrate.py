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

from read_file import read_jsonl


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
        "--algo",
        type=str,
        choices=["accel", "gyro"],
        default="accel",
        help="Sensor type to calibrate (accel or gyro).",
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


def load_calibration_functions(sensor_type: str):
    """
    Import calibration functions from appropriate module.

    Args:
        sensor_type: 'accel' or 'gyro'

    Returns:
        Tuple of (cal_all_func, bias_func, sensitivity_func, alignment_func)
    """
    if sensor_type == "accel":
        from cal_accel import (
            cal_all as cal_all_func,
            cal_bias as bias_func,
            cal_sensitivity as sens_func,
            cal_alignment as align_func,
        )
    else:
        from cal_gyro import (
            cal_all as cal_all_func,
            cal_bias as bias_func,
            cal_sensitivity as sens_func,
            cal_alignment as align_func,
        )
    return cal_all_func, bias_func, sens_func, align_func


def group_files_by_type(
    jsonl_files: List[Path], verbose: bool
) -> Tuple[List[Path], List[Path]]:
    """
    Group JSONL files into reference and measurement groups based on metadata.

    Args:
        jsonl_files: List of file paths to analyze
        verbose: Whether to print progress

    Returns:
        Tuple of (reference_files, measurement_files)
    """
    ref_files: List[Path] = []
    meas_files: List[Path] = []

    for filepath in jsonl_files:
        try:
            schema, data = read_jsonl(str(filepath))
            # Check metadata in data records for recording_type or data_type
            if data and schema:
                meta = schema.get("_schema", {})
                file_type = meta.get("recording_type", meta.get("data_type", ""))
            else:
                file_type = ""

            # Reference files: gyro_ref_* or accel_ref_*
            if file_type and file_type.startswith(("gyro_ref", "accel_ref")):
                ref_files.append(filepath)
            elif file_type in ("reference", "ref", "positive"):
                ref_files.append(filepath)
            elif file_type in ("measurement", "meas", "data"):
                meas_files.append(filepath)
            elif file_type == "both":
                # Mix of ref and meas in same file
                ref_files.append(filepath)
                meas_files.append(filepath)
            else:
                # Default: treat as measurement
                meas_files.append(filepath)
        except Exception as e:
            if verbose:
                print(f"Warning: Could not read {filepath}: {e}")
                meas_files.append(filepath)

    if verbose:
        print(f"Grouped: {len(ref_files)} reference file(s), {len(meas_files)} measurement file(s)")

    return ref_files, meas_files


def extract_reference_arrays(
    jsonl_files: List[Path], verbose: bool
) -> Tuple[List[Any], List[Any]]:
    """
    Extract positive and negative reference arrays from reference files.

    Args:
        jsonl_files: List of reference file paths
        verbose: Whether to print progress

    Returns:
        Tuple of (positive_refs, negative_refs) where each is a list of arrays
    """
    import numpy as np

    positive_refs: List[Any] = []
    negative_refs: List[Any] = []

    for filepath in jsonl_files:
        try:
            schema, data = read_jsonl(str(filepath))

            # Extract reference values from metadata or data records
            # Method 1: Check schema metadata for reference values
            if schema and "_schema" in schema:
                meta = schema.get("_schema", {})
                if "positive_ref" in meta and "negative_ref" in meta:
                    pos = np.array(meta["positive_ref"], dtype=np.float64)
                    neg = np.array(meta["negative_ref"], dtype=np.float64)
                    positive_refs.append(pos)
                    negative_refs.append(neg)
                    if verbose:
                        print(f"  Loaded ref from metadata: {filepath.name}")
                    continue

            # Method 2: Extract from data records and metadata
            # Get measurement type and reference force from metadata
            meta = schema.get("_schema", {}) if schema else {}
            file_type = meta.get("recording_type", "")
            reference_force = meta.get("reference_force")

            # Collect all values for each axis
            axes = {"gyro": [], "accel": [], "mag": []}
            for rec in data:
                for sensor in ("gyro", "accel", "mag"):
                    for axis in ("x", "y", "z"):
                        key = f"{sensor}_{axis}"
                        if key in rec and isinstance(rec[key], (int, float)):
                            axes[sensor].append(float(rec[key]))

            # If we have axis data, create reference arrays
            # Reference force is stored in metadata, use it as the reference value
            if any(len(axes[s]) > 0 for s in ("gyro", "accel", "mag")):
                # Use the last sample as positive (end of recording) and first as negative (start)
                # For reference recordings, these should correspond to the known reference force
                pos_gyro = np.array([axes["gyro"][a][-1] for a in ("x", "y", "z")], dtype=np.float64)
                neg_gyro = np.array([axes["gyro"][a][0] for a in ("x", "y", "z")], dtype=np.float64)
                pos_accel = np.array([axes["accel"][a][-1] for a in ("x", "y", "z")], dtype=np.float64)
                neg_accel = np.array([axes["accel"][a][0] for a in ("x", "y", "z")], dtype=np.float64)

                # Include magnetometer if available
                if axes["mag"] and all(len(axes["mag"][a]) > 0 for a in ("x", "y", "z")):
                    pos_mag = np.array([axes["mag"][a][-1] for a in ("x", "y", "z")], dtype=np.float64)
                    neg_mag = np.array([axes["mag"][a][0] for a in ("x", "y", "z")], dtype=np.float64)
                else:
                    pos_mag = np.zeros(3, dtype=np.float64)
                    neg_mag = np.zeros(3, dtype=np.float64)

                positive_refs.append([pos_gyro, pos_accel, pos_mag])
                negative_refs.append([neg_gyro, neg_accel, neg_mag])
                if verbose:
                    print(f"  Extracted ref from {filepath.name} (type={file_type}, force={reference_force})")

        except Exception as e:
            if verbose:
                print(f"Warning: Could not extract ref from {filepath}: {e}")

    if verbose:
        print(f"Collected: {len(positive_refs)} positive refs, {len(negative_refs)} negative refs")

    return positive_refs, negative_refs


def run_calibration(sensor_type: str, verbose: bool) -> Optional[Dict[str, Any]]:
    """
    Run the full calibration pipeline.

    Returns calibration result dict or None on error.
    """
    try:
        # Load calibration functions
        (
            cal_all_func,
            bias_func,
            sens_func,
            align_func,
        ) = load_calibration_functions(sensor_type)

        # Find and group files
        calibrate_dir = Path(__file__).resolve().parent
        jsonl_files = list(calibrate_dir.glob("*.jsonl")) + list(calibrate_dir.glob("*.JSONL"))

        if not jsonl_files:
            print(f"No JSONL files found in {calibrate_dir}")
            return None

        if args.dry_run:
            print(f"Would process {len(jsonl_files)} file(s)")
            return None

        # Group files
        ref_files, meas_files = group_files_by_type(jsonl_files, verbose)

        if not ref_files:
            print(
                "No reference files found. Please add files with recording_type=gyro_ref_* or accel_ref_* in metadata."
            )
            return None

        if verbose:
            print(f"Reference files: {[f.name for f in ref_files]}")

        # Extract reference arrays
        positive_refs, negative_refs = extract_reference_arrays(ref_files, verbose)

        if not positive_refs or not negative_refs:
            print(
                "Could not extract reference arrays. Check file metadata for recording_type and reference_force."
            )
            return None

        # Run calibration
        bias, sensitivity, alignment = cal_all_func(positive_refs, negative_refs)

        result = {
            "sensor_type": sensor_type,
            "bias": bias.tolist() if hasattr(bias, "tolist") else bias,
            "sensitivity": (
                sensitivity.tolist() if hasattr(sensitivity, "tolist") else sensitivity
            ),
            "alignment": (
                alignment.tolist() if hasattr(alignment, "tolist") else alignment
            ),
            "reference_files": [str(f) for f in ref_files],
            "measurement_files": [str(f) for f in meas_files],
        }

        if verbose:
            print("Calibration complete:")
            print(f"  Bias: {bias}")
            print(f"  Sensitivity: {sensitivity}")
            print("  Alignment:")
            print(alignment)

        return result

    except Exception as e:
        print(f"Calibration error: {e}")
        if verbose:
            import traceback

            traceback.print_exc()
        return None


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point."""
    args = parse_args(argv)

    if args.dry_run:
        print(f"Dry run: would calibrate {args.algo} sensors in {args.data_dir}")
        return 0

    result = run_calibration(args.algo, args.verbose)

    if result:
        print("\nResults saved to stdout (pipe to file if needed)")
        print(json.dumps(result, indent=2))
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())
