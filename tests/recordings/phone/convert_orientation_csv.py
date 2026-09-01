#!/usr/bin/env python3
"""
Convert phone orientation CSV to roll, pitch, yaw format.

Input:
  - Orientation.csv: Time, w, x, y, z, Direct(°), Yaw(°), Pitch(°), Roll(°)

Output:
  - Combined CSV with columns: roll, pitch, yaw
"""

import csv
import os
import sys

RECORDINGS_DIR = "/home/william/Documents/kinetic/tests/recordings/phone"
INPUT_FILE = os.path.join(RECORDINGS_DIR, "Orientation.csv")
OUTPUT_FILE = os.path.join(RECORDINGS_DIR, "orientation_combined.csv")

def read_orientation_data(filepath):
    """Read CSV and return list of (timestamp, [roll, pitch, yaw]) tuples."""
    data = []
    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        # Skip header
        next(reader)
        for row in reader:
            if not row or row == ['']:
                continue
            timestamp = row[0].strip()
            try:
                values = [float(v) for v in row[1:]]
                # Need at least 8 values (w, x, y, z, Direct, Yaw, Pitch, Roll)
                if len(values) < 8:
                    continue
                # Columns: Yaw, Pitch, Roll at indices 5, 6, 7
                # Output order: roll, pitch, yaw
                roll = values[7]
                pitch = values[6]
                yaw = values[5]
                data.append((timestamp, [roll, pitch, yaw]))
            except ValueError:
                continue
    return data

def convert_orientation():
    print(f"Reading {INPUT_FILE}...")
    orientation_data = read_orientation_data(INPUT_FILE)
    print(f"  Read {len(orientation_data)} samples")

    if not orientation_data:
        print("ERROR: File is empty or unreadable")
        sys.exit(1)

    print(f"Done. Writing output to {OUTPUT_FILE}...")

    with open(OUTPUT_FILE, 'w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['roll', 'pitch', 'yaw'])

        for timestamp, values in orientation_data:
            writer.writerow(values)

    print(f"Done. Written {len(orientation_data)} samples to {OUTPUT_FILE}")

if __name__ == '__main__':
    convert_orientation()
