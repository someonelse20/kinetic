#!/usr/bin/env python3
"""
Merge phone sensor CSVs into combined recording format.

Input:
  - Gyroscope.csv: Time, X(rad/s), Y(rad/s), Z(rad/s)
  - Accelerometer.csv: Time, X(m/s²), Y(m/s²), Z(m/s²)
  - Magnetometer.csv: Time, X(µT), Y(µT), Z(µT)

Output:
  Combined CSV with columns: gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z

Timestamp matching is done by index position (1:1 alignment).
Gyro values are converted from rad/s to deg/s.
"""

import csv
import os
import sys

RECORDINGS_DIR = "/home/william/Documents/kinetic/tests/recordings/phone"
GYRO_FILE = os.path.join(RECORDINGS_DIR, "Gyroscope.csv")
ACCEL_FILE = os.path.join(RECORDINGS_DIR, "Accelerometer.csv")
MAG_FILE = os.path.join(RECORDINGS_DIR, "Magnetometer.csv")
OUTPUT_FILE = os.path.join(RECORDINGS_DIR, "merged.csv")

def read_timestamp_data(filepath):
    """Read CSV and return list of (timestamp, [x, y, z]) tuples."""
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
                data.append((timestamp, values))
            except ValueError:
                continue
    return data

def merge_recordings():
    print(f"Reading {GYRO_FILE}...")
    gyro_data = read_timestamp_data(GYRO_FILE)
    print(f"  Read {len(gyro_data)} samples")

    print(f"Reading {ACCEL_FILE}...")
    accel_data = read_timestamp_data(ACCEL_FILE)
    print(f"  Read {len(accel_data)} samples")

    print(f"Reading {MAG_FILE}...")
    mag_data = read_timestamp_data(MAG_FILE)
    print(f"  Read {len(mag_data)} samples")

    if not gyro_data or not accel_data or not mag_data:
        print("ERROR: One or more files are empty or unreadable")
        sys.exit(1)

    # Merge by index position (1:1 alignment)
    output_samples = min(len(gyro_data), len(accel_data), len(mag_data))
    if output_samples == 0:
        print("ERROR: No data found in files")
        sys.exit(1)

    print(f"Done. Merging {output_samples} samples...")
    print(f"Writing merged output to {OUTPUT_FILE}...")

    with open(OUTPUT_FILE, 'w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z', 'mag_x', 'mag_y', 'mag_z'])
        
        for i in range(output_samples):
            gyro = gyro_data[i][1]
            accel = accel_data[i][1]
            mag = mag_data[i][1]
            # Convert gyro from rad/s to deg/s
            gyro_deg = [v * 180.0 / 3.141592653589793 for v in gyro]
            writer.writerow(gyro_deg + accel + mag)

    print(f"Done. Written {output_samples} samples to {OUTPUT_FILE}")

if __name__ == '__main__':
    merge_recordings()
