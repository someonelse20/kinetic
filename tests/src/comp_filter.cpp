#include <iostream>
#include <cmath>

#include "comp_filter.h"
#include "kin_math.h"
#include "kin_types.h"

using namespace std;

matrix_t *update_gyro(matrix_t *prev_state, float *gyro, float dt);
matrix_t *update_accel_mag(float *accel, float *mag);

matrix_t *comp_imu_init(imu_t *imu, float *accel, float *mag) {
	imu->ekf.state = euler_to_quat(update_accel_mag(accel, mag));

	return imu->ekf.state;
}

matrix_t *comp_imu_update(imu_t *imu, float *gyro, float *accel, float *mag) {
	float gain = 0.9;

	matrix_t *gyro_m = update_gyro(imu->ekf.state, gyro, imu->dt);
	matrix_t *accel_mag_m = update_accel_mag(accel, mag);

	// print_matrix(accel_mag_m);
	// cout << endl;

	imu->ekf.state = euler_to_quat(add_matrix(scale_matrix(gyro_m, gain), scale_matrix(accel_mag_m, 1 - gain)));

	return imu->ekf.state;
}


matrix_t *update_gyro(matrix_t *prev_state, float *gyro, float dt) {
	matrix_t *ret = init_matrix(3, 1);


	ret->data[X] = prev_state->data[X] + gyro[X] * dt;
	ret->data[Y] = prev_state->data[Y] + gyro[Y] * dt;
	ret->data[Z] = prev_state->data[Z] + gyro[Z] * dt;

	return ret;
}

matrix_t *update_accel_mag(float *accel, float *mag) {
	matrix_t *ret = init_matrix(3, 1);

	float x = atan2(accel[Y], accel[Z]);
	float y = atan2(-accel[X], sqrt(accel[Y] * accel[Y] * accel[Z] * accel[Z]));
	float z = atan2(mag[Z] * sin(y) - mag[Y] * cos(y), mag[X] * cos(x) + sin(x) * (mag[Y] * sin(y) + mag[Z] * cos(y)));

	ret->data[X] = x;
	ret->data[Y] = y;
	ret->data[Z] = z;

	return ret;

	/*
	matrix_t *mag_tilt_comp_m = init_matrix(3, 3);
	mag_tilt_comp_m->data[0] = cos(x);
	mag_tilt_comp_m->data[1] = sin(x) * sin(y);
	mag_tilt_comp_m->data[2] = sin(x) * cos(y);

	mag_tilt_comp_m->data[3] = 0;
	mag_tilt_comp_m->data[4] = cos(y);
	mag_tilt_comp_m->data[5] = -sin(y);

	mag_tilt_comp_m->data[6] = -sin(x);
	mag_tilt_comp_m->data[7] = cos(x) * sin(y);
	mag_tilt_comp_m->data[8] = cos(x) * cos(y);
	*/
}

