#include <iostream>
#include <cmath>

#include "kin_math.h"
#include "kin_types.h"
#include "kin_imu.h"

using namespace std;


int main() {
	/*
	   float a_arr[] = {0, deg_to_rad(57), 0};

	   matrix_t *a = arr_to_matrix(a_arr, 3, 1);
	   matrix_t *rot_mat = euler_to_rot_matrix(a);

	   print_matrix(rot_mat);
	   cout << endl;

	   matrix_t *quat = euler_to_quat(a);
	   matrix_t *quat_rot_mat = quat_to_rot_matrix(quat);
	   print_matrix(quat_rot_mat);
	 */

	// Initialize IMU
	imu_t imu;
	float accel[3] = {0, 0, 9.81};
	float mag[3] = {0, 0, 0};
	imu_init(&imu, accel, mag);

	float new_gyro[3] = {0.1, 0.1, 0.1};
	float new_accel[3] = {0, 0, 9.81};
	float new_mag[3] = {0, 0, 0};
	
	// Update with new measurements
	imu_update(&imu, new_gyro, new_accel, new_mag);
	
	// Get euler angle output
	matrix_t *euler = quat_to_euler(imu.ekf.state);
	
	// Convert output to array
	float *euler_arr = matrix_to_arr(euler);

}

