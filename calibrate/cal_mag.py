"""
Magnetometer calibration implementation per calibration_formulas.md.

Input format:
- pos_refs: list of numpy arrays from positive magnetic field references (e.g., mx+, my+, mz+)
- neg_refs: list of numpy arrays from negative magnetic field references (e.g., mx-, my-, mz-)
- ellipsoid_data: list of numpy arrays for ellipsoid fitting (m1, m2, m3, m4, m5)

Output:
- offset: 1x3 numpy array (hard-iron bias)
- sensitivity: 1x3 numpy array (soft-iron scaling)
"""

import numpy as np
from numpy import linalg as la
import os


def read_data(file):
    """Read combined IMU data file, extract mag readings only."""
    mag_data = []
    with open(file, 'r') as f:
        for line in f:
            if 'mag:' in line:
                nums = line.replace('mag:', '').split(',')
                mag_data.append([float(n) for n in nums])
    return np.array(mag_data) if mag_data else np.array([])


def offset_calibration(pos_refs, neg_refs, magnetic_field=0.520411):
    """
    Calibration Method 1: Offset (hard-iron only).
    
    Removes constant magnetic offset. Simplest calibration.
    """
    # Stack all reference measurements
    all_refs = np.vstack(pos_refs + neg_refs)

    # Compute mean magnetic field offset
    h_measured = np.mean(all_refs, axis=0)

    # Return the offset to subtract from raw measurements
    return h_measured


def ellipsoid_calibration(ellipsoid_data, magnetic_field=0.520411):
    """
    Calibration Method 2: Ellipsoid (soft-iron only).
    
    Fits an ellipsoid to magnetometer measurements to correct scaling and
    cross-axis coupling.
    """
    # Stack all ellipsoid calibration data
    dataset = np.vstack(ellipsoid_data)

    x = dataset[:, 0]
    y = dataset[:, 1]
    z = dataset[:, 2]

    # Build the D*d2 matrix for ellipsoid fitting
    D = np.column_stack([
        x**2 + y**2 - 2*z**2,
        x**2 + z**2 - 2*y**2,
        2*x*y,
        2*x*z,
        2*y*z,
        2*x,
        2*y,
        2*z,
        np.ones_like(x)
    ])

    d2 = x**2 + y**2 + z**2

    # Solve for ellipsoid parameters using least squares
    u = la.lstsq(D, d2, rcond=None)[0]

    # Construct the 4x4 ellipsoid matrix
    v = np.zeros(10)
    v[0] = u[0] + u[1] - 1
    v[1] = u[0] - 2*u[1] - 1
    v[2] = u[1] - 2*u[0] - 1
    v[3:10] = u[2:9]

    v = v.reshape(-1, 1)

    # Build 4x4 matrix and solve for center
    A = np.array([
        [v[0][0], v[3][0], v[4][0], v[6][0]],
        [v[3][0], v[1][0], v[5][0], v[7][0]],
        [v[4][0], v[5][0], v[2][0], v[8][0]],
        [v[6][0], v[7][0], v[8][0], v[9][0]]
    ])

    # Solve for center
    center = -np.linalg.solve(A[:3, :3], v[6:9]).reshape(-1, 1)

    # Compute rotation and eigenvalues
    T = np.eye(4)
    T[3, :3] = center.flatten()

    R = T @ A @ T.T

    evals, evecs = la.eig(R[:3, :3] / (-R[3, 3]))

    # Compute radii from eigenvalues
    radii = np.sqrt(1 / np.abs(evals))
    sgns = np.sign(evals)
    radii = radii * sgns

    # Scale ellipsoid to match reference magnetic field (Eq 6.17)
    # Use each eigenvalue for its corresponding axis
    soft_iorn = evecs.T @ np.diag([magnetic_field / radii[0],
                                   magnetic_field / radii[1],
                                   magnetic_field / radii[2]]) @ evecs

    # Compute hard-iron bias as offset of ellipsoid center
    hard_iorn = soft_iorn @ center

    return soft_iorn, np.array([hard_iorn[0][0], hard_iorn[1][0], hard_iorn[2][0]])


def alignment_calibration(pos_refs, neg_refs, ellipsoid_files=None):
    """
    Calibration Method 3: Alignment (full calibration).
    
    Complete calibration with hard-iron bias, soft-iron scaling/coupling,
    and axis alignment.
    """
    if ellipsoid_files is None:
        ellipsoid_files = ["m1", "m2", "m3", "m4", "m5"]

    # Get initial soft-iron and hard-iron parameters from ellipsoid fit
    init_soft_iorn, init_hard_iorn = ellipsoid_calibration(
        [read_data(os.path.join(drone_cal_dir, f)) for f in ellipsoid_files],
        magnetic_field=0.520411
    )

    # Process each axis separately
    v = np.array([])

    for axis in range(3):
        dataset = np.array([])
        if dataset.size == 0:
            dataset = calibrate(read_data(os.path.join(drone_cal_dir, pos_refs[axis])),
                               init_soft_iorn, init_hard_iorn)
        else:
            dataset = np.append(dataset,
                               calibrate(read_data(os.path.join(drone_cal_dir, pos_refs[axis])),
                                        init_soft_iorn, init_hard_iorn),
                               axis=0)

        # Store calibrated values for this axis
        x = dataset[:, 0]
        y = dataset[:, 1]
        z = dataset[:, 2]

        M = np.column_stack([x, y, z])

        ones = np.ones((x.size, 1))

        if v.size == 0:
            v = la.lstsq(M, ones)[0]
        else:
            v = np.append(v, la.lstsq(M, ones)[0], axis=1)

    # Compute alignment matrix from normalized columns
    R = np.array([v[0] / la.norm(v[0]),
                  v[1] / la.norm(v[1]),
                  v[2] / la.norm(v[2])])

    # Transform soft-iron and hard-iron parameters into aligned frame
    S = R.transpose() @ init_soft_iorn
    H = R.transpose() @ init_hard_iorn

    return S, H


def calibrate(points, soft_iorn, hard_iorn):
    """Apply soft-iron and hard-iron calibration to raw measurements."""
    return_list = np.array([])

    for i in range(len(points)):
        if return_list.size == 0:
            return_list = np.array([np.dot(soft_iorn, points[i]) - hard_iorn])
        else:
            return_list = np.append(return_list,
                                   [np.dot(soft_iorn, points[i]) - hard_iorn],
                                   axis=0)

    return return_list


# Drone calibration data directory
drone_cal_dir = "/home/william/Documents/drone/code/flight-controller/calibrate/data"
