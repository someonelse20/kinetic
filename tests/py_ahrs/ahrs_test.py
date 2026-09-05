import os
import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../recordings")


def load_recordings(filename=None):
    """Load accelerometer, gyro, and magnetometer data from CSV recordings.

    Args:
        filename: Optional specific CSV filename. If None, loads all CSV files.

    Each CSV has format: gyro x,y,z, accel x,y,z, mag x,y,z
    The file contains N rows, each with 9 comma-separated values.
    """
    if not os.path.isdir(DATA_DIR):
        raise FileNotFoundError(f"Recordings directory not found: {DATA_DIR}")

    if filename:
        csv_files = [filename]
        if not os.path.isfile(os.path.join(DATA_DIR, filename)):
            raise FileNotFoundError(
                f"File not found: {os.path.join(DATA_DIR, filename)}"
            )
    else:
        csv_files = sorted(f for f in os.listdir(DATA_DIR) if f.endswith(".csv"))

    if not csv_files:
        raise FileNotFoundError(
            f"No CSV recordings found in {DATA_DIR}. Expected *.csv files."
        )

    # Each CSV has format: gyro x,y,z, accel x,y,z, mag x,y,z
    csv_arrays = [
        np.genfromtxt(os.path.join(DATA_DIR, f), delimiter=",") for f in csv_files
    ]

    if any(len(arr[0]) != 9 for arr in csv_arrays):
        raise ValueError(
            "CSV files must have 9 columns: gyro x,y,z, accel x,y,z, mag x,y,z"
        )

    # Stack arrays: if multiple CSVs, concatenate along axis 0
    data = np.vstack(csv_arrays)

    if data.shape[0] == 0:
        raise ValueError("CSV files are empty")

    num_samples, num_columns = data.shape
    expected_columns = 9

    if num_columns != expected_columns:
        raise ValueError(
            f"Expected {expected_columns} columns per row (gyro, accel, mag), got {num_columns}"
        )

    # data has shape (num_samples, 9)
    # Each row is: gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z
    gyro_data = data[:, :3]
    acc_data = data[:, 3:6]
    mag_data = data[:, 6:9]

    if (
        gyro_data.shape[0] != acc_data.shape[0]
        or gyro_data.shape[0] != mag_data.shape[0]
    ):
        raise ValueError(
            f"Recording data mismatch: gyro={gyro_data.shape[0]}, acc={acc_data.shape[0]}, mag={mag_data.shape[0]}"
        )

    return acc_data, gyro_data, mag_data


def main(filename=None):
    acc_data, gyro_data, mag_data = load_recordings(filename)
    num_samples = acc_data.shape[0]  # Number of time steps

    ekf = EKF()
    Q = np.zeros((num_samples, 4))  # Allocate array for quaternions

    # acc2q expects tri-axial accelerometer [x, y, z]
    acc0 = np.array([acc_data[0, 0], acc_data[0, 1], acc_data[0, 2]])
    Q[0] = acc2q(acc0)

    # EKF.update expects tri-axial arrays
    for t in range(1, num_samples):
        gyro = np.array([gyro_data[t, 0], gyro_data[t, 1], gyro_data[t, 2]])
        acc = np.array([acc_data[t, 0], acc_data[t, 1], acc_data[t, 2]])
        Q[t] = ekf.update(Q[t - 1], gyro, acc)
        print(Q[t])


filename = "static.csv"
main(filename)
