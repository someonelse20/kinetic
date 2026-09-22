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

    """
    def linear_interpolation(self):
        if not self.state_initialized:
            print("Error: EKF not initialized")
            return False

        # Generate interpolated orientations
        true_angles = self._generate_interpolated_orientations()

        # Run EKF on interpolated data
        # Use get_gyro() to compute gyro from orientation changes (matches sim.cpp)
        for i in range(len(true_angles) - 1):
            roll1, pitch1, yaw1 = true_angles[i]
            roll2, pitch2, yaw2 = true_angles[i + 1]
            dt = self.step_size

            # Compute gyro from orientation change (sim.cpp::get_gyro)
            gyro_x, gyro_y, gyro_z = self.get_gyro(
                roll1, pitch1, yaw1, roll2, pitch2, yaw2, dt
            )

            # Compute accel and mag from current orientation
            accel_x, accel_y, accel_z = self.get_accel(roll2, pitch2, yaw2)
            mag_x, mag_y, mag_z = self.get_mag(roll2, pitch2, yaw2, self.mag_dip)

            # Run EKF update
            eul = self.imu_update(accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, dt)

            self.euler_data["roll"].append(eul[0])
            self.euler_data["pitch"].append(eul[1])
            self.euler_data["yaw"].append(eul[2])
            self.true_data["roll"].append(roll2)
            self.true_data["pitch"].append(pitch2)
            self.true_data["yaw"].append(yaw2)

        return True
    """

    def all_axis_test(self, num_points=100):
        """Run all-axis test with specified number of steps."""

        timestep = 2 * np.pi / num_points
        self.imu.dt = timestep

        step_counts = [0, np.pi / 2, np.pi]
        euler_orientation = [0, 0, 0]
        for i in range(3):
            euler_orientation[i] = math.sin(step_counts[i])

        quat = kin.euler_to_quat(arr_to_matrix(euler_orientation, 3, 1))
        prev_quat = kin.init_matrix(4, 1)

        accel_m = kin.get_accel(quat)
        mag_m = kin.get_mag(quat, self.imu.mag_dip)

        kin.imu_init(
            self.imu,
            accel_m.getItem(0),
            accel_m.getItem(1),
            accel_m.getItem(2),
            mag_m.getItem(0),
            mag_m.getItem(1),
            mag_m.getItem(2),
        )

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

    for point in test_demo.euler_data:
        pass
        # print(point)
