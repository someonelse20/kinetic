"""
Gyroscope calibration implementation per calibration_formulas.md.

Input format:
- pos_refs: list of numpy arrays from +rotation reference files (e.g., gx+, gy+, gz+)
- neg_refs: list of numpy arrays from -rotation reference files (e.g., gx-, gy-, gz-)

Output:
- bias: 1x3 numpy array (mean output at zero rate)
- sensitivity: 1x3 numpy array (output per unit angular velocity)
"""

import numpy as np


def read_data(file):
    """Read combined IMU data file, extract gyro readings only."""
    gyro_data = []
    with open(file, "r") as f:
        for line in f:
            if "gyro:" in line:
                nums = line.replace("gyro:", "").split(",")
                gyro_data.append([float(n) for n in nums])
    return np.array(gyro_data) if gyro_data else np.array([])


def bias_calibration(refs):
    """
    Eq 6.4.1: b_ω = mean(u_ω) while stationary.

    For gyro, average the zero-rate outputs from a single static reference
    position. Since ω ≈ 0, this converges to the true bias.
    """
    # Stack all measurements into a single array
    all_zero = np.vstack(refs)

    # Compute mean across all samples (Eq 6.4.1)
    bias = np.mean(all_zero, axis=0)

    return bias[0]


def sensitivity_calibration(pos_refs, neg_refs, omega_ref=200.0):
    """
    Eq 6.4.3 / 6.4: s_ω = (|u_+ω| + |u_-ω|) / (2 * ω)

    For each axis, average the outputs at +ω and -ω, take absolute values,
    sum them, and divide by 2 * reference angular velocity.
    """
    # Stack positive refs separately from negative refs
    pos_all = np.vstack(pos_refs)
    neg_all = np.vstack(neg_refs)

    # For each axis (0, 1, 2), compute sensitivity
    sensitivity = np.zeros(3)
    for i in range(3):
        # Average across positive refs only
        avg_plus = np.mean(pos_all[:, i])
        # Average across negative refs only
        avg_minus = np.mean(neg_all[:, i])

        # Eq 6.4.3: use absolute values of both polarities
        sensitivity[i] = (np.abs(avg_plus) + np.abs(avg_minus)) / (2 * omega_ref)

    return sensitivity
