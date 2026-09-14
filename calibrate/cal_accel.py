"""
Accelerometer calibration implementation per calibration_formulas.md.

Input format:
- pos_refs: list of [xs, ys, zs] arrays from +1g reference datasets
- neg_refs: list of [xs, ys, zs] arrays from -1g reference datasets

Each sub-list contains all measurements for one axis across all samples.

Output:
- bias: 1x3 numpy array (mean output at zero-g)
- sensitivity: 1x3 numpy array (output per g)
"""

import numpy as np


def read_data(file):
    """Read combined IMU data file, extract accel readings only."""
    accel_data = []
    with open(file, 'r') as f:
        for line in f:
            if 'accel:' in line:
                nums = line.replace('accel:', '').split(',')
                accel_data.append([float(n) for n in nums])
    return np.array(accel_data) if accel_data else np.array([])


def bias_calibration(pos_refs, neg_refs):
    """
    Eq 6.5.2 / 6.8: b_a = (u_+g + u_-g) / 2

    For each axis, average the outputs at +1g and -1g references. The mean of
    these two opposites converges to the true zero-g bias.
    """
    # pos_refs and neg_refs are lists of [xs, ys, zs] arrays
    bias = np.zeros(3)
    for axis in range(3):
        # Get all positive ref values for this axis
        pos_all = np.vstack(pos_refs[axis])
        # Get all negative ref values for this axis
        neg_all = np.vstack(neg_refs[axis])

        # Average across positive refs for this axis
        avg_plus = np.mean(pos_all)
        # Average across negative refs for this axis
        avg_minus = np.mean(neg_all)

        # Eq 6.8: (u_+g + u_-g) / 2
        bias[axis] = (avg_plus + avg_minus) / 2.0

    return bias


def sensitivity_calibration(pos_refs, neg_refs, g_ref=1.0):
    """
    Eq 6.5.2 / 6.9: s_a = (|u_+g| + |u_-g|) / (2 * g)

    For each axis, average the outputs at +1g and -1g, take absolute values,
    sum them, and divide by 2 * reference gravity.
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

        # Eq 6.5.2: use absolute values of both polarities
        sensitivity[axis] = (np.abs(avg_plus) + np.abs(avg_minus)) / (2 * g_ref)

    return sensitivity
