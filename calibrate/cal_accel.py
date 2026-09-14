"""
Accelerometer calibration implementation per calibration_formulas.md.

Input format:
- pos_refs: list of numpy arrays from +1g reference files (e.g., ax+, ay+, az+)
- neg_refs: list of numpy arrays from -1g reference files (e.g., ax-, ay-, az-)

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
    # Compute mean per axis for positive refs
    pos_means = np.zeros((len(pos_refs), 3))
    for idx, pos_ref in enumerate(pos_refs):
        pos_means[idx] = np.mean(pos_ref, axis=0)
    
    # Compute mean per axis for negative refs
    neg_means = np.zeros((len(neg_refs), 3))
    for idx, neg_ref in enumerate(neg_refs):
        neg_means[idx] = np.mean(neg_ref, axis=0)
    
    # Average across all positive refs (for each axis)
    avg_plus = np.mean(pos_means, axis=0)
    # Average across all negative refs (for each axis)
    avg_minus = np.mean(neg_means, axis=0)
    
    # Eq 6.8: (u_+g + u_-g) / 2
    bias = (avg_plus + avg_minus) / 2.0

    return bias


def sensitivity_calibration(pos_refs, neg_refs, g_ref=1.0):
    """
    Eq 6.5.2 / 6.9: s_a = (|u_+g| + |u_-g|) / (2 * g)

    For each axis, average the outputs at +1g and -1g, take absolute values,
    sum them, and divide by 2 * reference gravity.
    """
    # Compute mean per axis for positive refs
    pos_means = np.zeros((len(pos_refs), 3))
    for idx, pos_ref in enumerate(pos_refs):
        pos_means[idx] = np.mean(pos_ref, axis=0)
    
    # Compute mean per axis for negative refs
    neg_means = np.zeros((len(neg_refs), 3))
    for idx, neg_ref in enumerate(neg_refs):
        neg_means[idx] = np.mean(neg_ref, axis=0)
    
    # Average across all positive refs (for each axis)
    avg_plus = np.mean(pos_means, axis=0)
    # Average across all negative refs (for each axis)
    avg_minus = np.mean(neg_means, axis=0)
    
    # Eq 6.5.2: use absolute values of both polarities
    sensitivity = (np.abs(avg_plus) + np.abs(avg_minus)) / (2 * g_ref)

    return sensitivity
