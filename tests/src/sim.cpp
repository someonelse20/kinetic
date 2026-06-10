#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <math.h>

#include "sim.h"
#include "kin_math.h"
#include "kinetic.h"

using namespace std;

float *rand_rot(int range);

matrix_t *get_gyro(float *gyro_out);
matrix_t *get_accel(matrix_t *);
matrix_t *get_mag(matrix_t *);

sim_t::sim_t(kinetic_t *kinetic) {
	this->kinetic = kinetic;

	this->orientation = init_matrix(4, 1);

	for (int i = 0; i < 3; i++) {
		orientation->data[i] = 0;
	}

	orientation->data[3] = 1;
}

void sim_t::tick() {
	float *gyro_out = rand_rot(10);

	matrix_t *gyro_q = init_matrix(4, 1);
	gyro_q->data[W] = 0;
	for (int i = 0; i < 3; i++) {
		gyro_q->data[i] = gyro_out[i];
	}

	matrix_t *rate_of_change_q = scale_matrix(mul_quat(orientation, gyro_q), 0.5);
	orientation = add_matrix(orientation, rate_of_change_q);
	orientation = normalize_matrix(orientation);

	matrix_t *gyro_m = get_gyro(gyro_out);
	matrix_t *accel_m = get_accel(orientation);
	matrix_t *mag_m = get_mag(orientation);

	gyro = gyro_m->data;
	accel = accel_m->data;
	mag = mag_m->data;
}

void sim_t::loop(void (*update_imu)(kinetic_t*, float*, float*, float*, float)) {
	while (true) {
		tick();
		update_imu(kinetic, gyro, accel, mag, sample_rate_hertz);
		sleep(1 / sample_rate_hertz);
	}
}

float *rand_rot(int range) {
	float* ret = (float *) malloc(3 * sizeof(float));

	for (int i = 0; i < 3; i++) {
		ret[i] = deg_to_rad(((rand() % (range * 200)) - range * 100) / 100.0);
	}

	return ret;
}

matrix_t *get_gyro(float *gyro_out) {
	return arr_to_matrix(gyro_out, 3, true);
}

matrix_t *get_accel(matrix_t *orientation) {
	float g_ref_a[] = {0, 0, 1};
	matrix_t *g_ref_m = arr_to_matrix(g_ref_a, 3, true);
	matrix_t *m = quat_to_rot_matrix(orientation);

	return mul_matrix(trans_matrix(quat_to_rot_matrix(orientation)), g_ref_m);
}

matrix_t *get_mag(matrix_t *orientation) {
	float mag_dip = 0;

	float m_ref_a[] = {cos(mag_dip), 0, sin(mag_dip)};
	matrix_t *m_ref_m = scale_matrix(arr_to_matrix(m_ref_a, 3, true), 1 / (sqrt(pow(cos(mag_dip), 2) + pow(sin(mag_dip), 2))));

	return mul_matrix(trans_matrix(quat_to_rot_matrix(orientation)), m_ref_m);
}

