#include <cmath>

#include "reference_ahrs.h"
#include "kin_math.h"

using namespace std;

static matrix_t *update_gyro(matrix_t *prev_state, float *gyro, float dt);
static matrix_t *update_accel_mag(float *accel, float *mag);

matrix_t *gyro_imu_init(imu_t *imu, float *accel, float *mag) {
	imu->ekf.state = fill_matrix(3, 1, 0.f);

	return imu->ekf.state;
}
matrix_t *gyro_imu_update(imu_t *imu, float *gyro, float *accel, float *mag) {
	imu->ekf.state = update_gyro(imu->ekf.state, gyro, imu->dt);

	return imu->ekf.state;
}

matrix_t *comp_imu_init(imu_t *imu, float *accel, float *mag) {
	imu->ekf.state = euler_to_quat(update_accel_mag(accel, mag));

	return imu->ekf.state;
}

matrix_t *comp_imu_update(imu_t *imu, float *gyro, float *accel, float *mag) {
	float gain = 0.9;

	matrix_t *gyro_m = update_gyro(imu->ekf.state, gyro, imu->dt);
	matrix_t *accel_mag_m = update_accel_mag(accel, mag);

	imu->ekf.state = add_matrix_alloc(scale_matrix_alloc(gyro_m, gain), scale_matrix_alloc(accel_mag_m, 1 - gain));

	return imu->ekf.state;
}


matrix_t *update_gyro(matrix_t *prev_state, float *gyro, float dt) {
	matrix_t *ret = init_matrix(3, 1);

	matrix_t *gyro_deg = scale_matrix_alloc(arr_to_matrix(gyro, 3, 1), 180 / M_PI);

	ret->data[X] = prev_state->data[X] + gyro_deg->data[X] * dt;
	ret->data[Y] = prev_state->data[Y] + gyro_deg->data[Y] * dt;
	ret->data[Z] = prev_state->data[Z] + gyro_deg->data[Z] * dt;

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
}

