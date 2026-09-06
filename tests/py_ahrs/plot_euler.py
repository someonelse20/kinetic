import os
import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q
import matplotlib.pyplot as plt

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../recordings")

def load_recordings(filename=None):
    """Load accelerometer, gyro, and magnetometer data from CSV recordings."""
    if not os.path.isdir(DATA_DIR):
        raise FileNotFoundError(f"Recordings directory not found: {DATA_DIR}")

    if filename:
        csv_files = [filename]
        if not os.path.isfile(os.path.join(DATA_DIR, filename)):
            raise FileNotFoundError(f"File not found: {os.path.join(DATA_DIR, filename)}")
    else:
        csv_files = sorted(f for f in os.listdir(DATA_DIR) if f.endswith(".csv"))

    if not csv_files:
        raise FileNotFoundError(f"No CSV recordings found in {DATA_DIR}. Expected *.csv files.")

    csv_arrays = [np.genfromtxt(os.path.join(DATA_DIR, f), delimiter=",") for f in csv_files]

    if any(len(arr[0]) != 9 for arr in csv_arrays):
        raise ValueError("CSV files must have 9 columns: gyro x,y,z, accel x,y,z, mag x,y,z")

    data = np.vstack(csv_arrays)

    if data.shape[0] == 0:
        raise ValueError("CSV files are empty")

    num_samples, num_columns = data.shape
    expected_columns = 9

    if num_columns != expected_columns:
        raise ValueError(
            f"Expected {expected_columns} columns per row (gyro, accel, mag), got {num_columns}"
        )

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

def quaternion_to_euler(q):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).

    Args:
        q: Quaternion [x, y, z, w] or shape (N, 4)

    Returns:
        Euler angles in radians (roll, pitch, yaw)
    """
    # From AHRS documentation
    # q = [x, y, z, w]
    x, y, z, w = q

    # Roll (x-axis rotation)
    roll = np.arctan2(2 * (w * x + y * z), w * w + x * x - y * y - z * z)

    # Pitch (y-axis rotation)
    # Clamp to avoid numerical issues with arctan2
    pitch_arg = 2 * (w * y - z * x)
    pitch_arg = np.clip(pitch_arg, -1.0, 1.0)
    pitch = np.arcsin(pitch_arg)

    # Yaw (z-axis rotation)
    yaw = np.arctan2(2 * (w * z + x * y), w * w - x * x - y * y + z * z)

    return roll, pitch, yaw

def main(filename=None):
    acc_data, gyro_data, mag_data = load_recordings(filename)
    num_samples = acc_data.shape[0]

    ekf = EKF()
    Q = np.zeros((num_samples, 4))

    # Initialize with acc2q
    acc0 = np.array([acc_data[0, 0], acc_data[0, 1], acc_data[0, 2]])
    Q[0] = acc2q(acc0)

    # Run EKF
    for t in range(1, num_samples):
        gyro = np.array([gyro_data[t, 0], gyro_data[t, 1], gyro_data[t, 2]])
        acc = np.array([acc_data[t, 0], acc_data[t, 1], acc_data[t, 2]])
        Q[t] = ekf.update(Q[t - 1], gyro, acc)

    # Convert quaternions to Euler angles
    rolls = np.zeros(num_samples)
    pitches = np.zeros(num_samples)
    yaws = np.zeros(num_samples)

    for t in range(num_samples):
        roll, pitch, yaw = quaternion_to_euler(Q[t])
        rolls[t] = roll
        pitches[t] = pitch
        yaws[t] = yaw

    # Plot results
    plt.figure(figsize=(15, 10))

    # Roll plot
    plt.subplot(3, 1, 1)
    plt.plot(range(num_samples), rolls, label='Roll', linewidth=1)
    plt.xlabel('Sample')
    plt.ylabel('Roll (rad)')
    plt.title('Euler Angles - Roll')
    plt.grid(True, alpha=0.3)
    plt.legend(loc='upper right')

    # Pitch plot
    plt.subplot(3, 1, 2)
    plt.plot(range(num_samples), pitches, label='Pitch', linewidth=1)
    plt.xlabel('Sample')
    plt.ylabel('Pitch (rad)')
    plt.title('Euler Angles - Pitch')
    plt.grid(True, alpha=0.3)
    plt.legend(loc='upper right')

    # Yaw plot
    plt.subplot(3, 1, 3)
    plt.plot(range(num_samples), yaws, label='Yaw', linewidth=1)
    plt.xlabel('Sample')
    plt.ylabel('Yaw (rad)')
    plt.title('Euler Angles - Yaw')
    plt.grid(True, alpha=0.3)
    plt.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig('euler_angles.png', dpi=150)
    print(f"Saved Euler angles plot to euler_angles.png")

if __name__ == "__main__":
    import sys
    filename = sys.argv[1] if len(sys.argv) > 1 else None
    main(filename)
