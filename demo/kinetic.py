#!/usr/bin/env python3
import math
import numpy as np
from build import kin_wrapper as kin


class EKFDemo:
    """EKF demonstration class with stub implementations."""

    def __init__(self):
        self.state_running = False
        self.state_initialized = False
        self.sim_mode = "all_axis_test"  # or "linear_interpolation"

        # Storage for charting
        self.euler_data = []
        self.true_data = []

        # Configuration
        self.num_points = 100
        self.step_size = 0.1

        self.imu = kin.init_imu(True, 0.00000001, 0, 0, 0)

    # ==================== Test Modes ====================

    def linear_interpolation(self, start_rot=(0, 0, 0), end_rot=(45, 45, 45), duration=10, timestep=0.1):
        """Run interpolation test with start and end rotations

        Args:
            start_rot: Starting Euler angles in degrees (default (0, 0, 0))
            end_rot: Ending Euler angles in degrees (default (45, 45, 45))
            duration: Total duration in seconds (default 10)
            timestep: Simulation timestep in seconds (default 0.1)
        """

        self.imu.dt = timestep

        start_rot_rad = []
        end_rot_rad = []
        for i in range(3):
            start_rot_rad.append(start_rot[i] * np.pi / 180)
            end_rot_rad.append(end_rot[i] * np.pi / 180)

        start_rot_quat = kin.euler_to_quat(arr_to_matrix(start_rot_rad, 3, 1))
        end_rot_quat = kin.euler_to_quat(arr_to_matrix(end_rot_rad, 3, 1))

        quat = kin.fill_matrix(4, 1, 0)
        prev_quat = kin.init_matrix(4, 1)

        accel_m = kin.get_accel(start_rot_quat)
        mag_m = kin.get_mag(start_rot_quat, self.imu.mag_dip)

        kin_quat = kin.imu_init(
            self.imu,
            accel_m.getItem(0),
            accel_m.getItem(1),
            accel_m.getItem(2),
            mag_m.getItem(0),
            mag_m.getItem(1),
            mag_m.getItem(2),
        )

        kin_euler = matrix_to_arr(kin.quat_to_euler(kin_quat))
        true_euler = matrix_to_arr(kin.quat_to_euler(start_rot_quat))

        self.euler_data.append(kin_euler)
        self.true_data.append(true_euler)

        # Run EKF updates
        time = 0
        while time < duration:
            norm_time = time / duration

            for i in range(4):
                prev_quat.setItem(i, quat.getItem(i))

            quat = kin.add_matrix_alloc(
                start_rot_quat,
                kin.scale_matrix_alloc(
                    kin.sub_matrix_alloc(end_rot_quat, start_rot_quat), norm_time
                ),
            )

            gyro_m = kin.get_gyro(prev_quat, quat, timestep)
            accel_m = kin.get_accel(quat)
            mag_m = kin.get_mag(quat, self.imu.mag_dip)

            kin_quat = kin.imu_update(
                self.imu,
                gyro_m.getItem(0),
                gyro_m.getItem(1),
                gyro_m.getItem(2),
                accel_m.getItem(0),
                accel_m.getItem(1),
                accel_m.getItem(2),
                mag_m.getItem(0),
                mag_m.getItem(1),
                mag_m.getItem(2),
            )
            kin_euler = matrix_to_arr(kin.quat_to_euler(kin_quat))
            true_euler = matrix_to_arr(kin.quat_to_euler(quat))

            self.euler_data.append(kin_euler)
            self.true_data.append(true_euler)

            time += timestep

        return self.euler_data, self.true_data

    def all_axis_test(self, num_points=100):
        """Run all-axis test with specified number of steps."""

        timestep = 2 * np.pi / num_points
        self.imu.dt = timestep

        step_counts = [0, np.pi / 2, np.pi]
        euler_orientation = []
        for i in range(3):
            euler_orientation.append(math.sin(step_counts[i]))

        quat = kin.euler_to_quat(arr_to_matrix(euler_orientation, 3, 1))
        prev_quat = kin.init_matrix(4, 1)

        accel_m = kin.get_accel(quat)
        mag_m = kin.get_mag(quat, self.imu.mag_dip)

        kin_quat = kin.imu_init(
            self.imu,
            accel_m.getItem(0),
            accel_m.getItem(1),
            accel_m.getItem(2),
            mag_m.getItem(0),
            mag_m.getItem(1),
            mag_m.getItem(2),
        )

        kin_euler = matrix_to_arr(kin.quat_to_euler(kin_quat))
        true_euler = matrix_to_arr(kin.quat_to_euler(quat))

        self.euler_data.append(kin_euler)
        self.true_data.append(true_euler)

        # Run EKF updates
        for step in range(num_points):
            for i in range(3):
                step_counts[i] += timestep
                euler_orientation[i] = math.sin(step_counts[i])

            for i in range(4):
                prev_quat.setItem(i, quat.getItem(i))

            quat = kin.euler_to_quat(arr_to_matrix(euler_orientation, 3, 1))

            gyro_m = kin.get_gyro(prev_quat, quat, timestep)
            accel_m = kin.get_accel(quat)
            mag_m = kin.get_mag(quat, self.imu.mag_dip)

            kin_quat = kin.imu_update(
                self.imu,
                gyro_m.getItem(0),
                gyro_m.getItem(1),
                gyro_m.getItem(2),
                accel_m.getItem(0),
                accel_m.getItem(1),
                accel_m.getItem(2),
                mag_m.getItem(0),
                mag_m.getItem(1),
                mag_m.getItem(2),
            )
            kin_euler = matrix_to_arr(kin.quat_to_euler(kin_quat))
            true_euler = matrix_to_arr(kin.quat_to_euler(quat))

            self.euler_data.append(kin_euler)
            self.true_data.append(true_euler)

        return self.euler_data, self.true_data


def arr_to_matrix(arr, rows, cols):
    mat = kin.init_matrix(rows, cols)
    for i in range(rows * cols):
        mat.setItem(i, arr[i])
    return mat


def matrix_to_arr(mat):
    arr = []
    for i in range(mat.len()):
        arr.append(mat.getItem(i))
    return arr


if __name__ == "__main__":
    test_demo = EKFDemo()
    test_demo.all_axis_test()

    start_rot = [0, 0, 0]
    end_rot = [45, 45, 45]

    test_demo.linear_interpolation(start_rot, end_rot, 10, 0.1)
