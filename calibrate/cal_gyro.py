"""
Gyroscope calibration implementation per calibration_formulas.md.

Input format:
- pos_refs: list of [xs, ys, zs] arrays from +rotation reference datasets
- neg_refs: list of [xs, ys, zs] arrays from -rotation reference datasets

Each sub-list contains all measurements for one axis across all samples.

Output:
- bias: 1x3 numpy array (mean output at zero rate)
- sensitivity: 1x3 numpy array (output per unit angular velocity)
"""

import numpy as np


def read_data(file):
    """Read combined IMU data file, extract gyro readings only."""
    gyro_data = []
    with open(file, 'r') as f:
        for line in f:
            if 'gyro:' in line:
                nums = line.replace('gyro:', '').split(',')
                gyro_data.append([float(n) for n in nums])
    return np.array(gyro_data) if gyro_data else np.array([])


def bias_calibration(refs):
    """
    Eq 6.4.1: b_ω = mean(u_ω) while stationary.

    For each axis, average the zero-rate outputs from a single static reference
    position. Since ω ≈ 0, this converges to the true bias.
    """
    # refs is list of [xs, ys, zs] arrays, one per reference dataset
    bias = np.zeros(3)
    for axis in range(3):
        # Stack all values for this axis across all references
        all_zero = np.vstack(refs[axis])
        # Compute mean for this axis (Eq 6.4.1)
        bias[axis] = np.mean(all_zero)

    return bias


def sensitivity_calibration(pos_refs, neg_refs, omega_ref=200.0):
    """
    Eq 6.4.3 / 6.4: s_ω = (|u_+ω| + |u_-ω|) / (2 * ω)

    For each axis, average the outputs at +ω and -ω, take absolute values,
    sum them, and divide by 2 * reference angular velocity.
    """
    sensitivity = np.zeros(3)
    for axis in range(3):
        # Get all positive ref values for this axis
        pos_all = np.vstack(pos_refs[axis])
        # Get all negative ref values for this axis
        neg_all = np.vstack(neg_refs[axis])

        # Average across positive and negative refs for this axis
        avg_plus = np.mean(pos_all)
        avg_minus = np.mean(neg_all)

        # Eq 6.4.3: use absolute values of both polarities
        sensitivity[axis] = (np.abs(avg_plus) + np.abs(avg_minus)) / (2 * omega_ref)

    return sensitivity
