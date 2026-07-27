#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <math.h>

#include "sim.h"
#include "kin_math.h"
#include "kinetic.h"

using namespace std;

float *rand_rot(int range);

matrix_t *get_gyro(matrix_t *, matrix_t *, float);
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
	float *gyro_out = rand_rot(1);

	matrix_t *gyro_q = init_matrix(4, 1);
	gyro_q->data[W] = 0;
	for (int i = 0; i < 3; i++) {
		gyro_q->data[i] = gyro_out[i];
	}

	matrix_t *rate_of_change_q = scale_matrix(mul_quat(orientation, gyro_q), 0.5);
	orientation = add_matrix(orientation, rate_of_change_q);
	orientation = normalize_matrix(orientation);

	matrix_t *accel_m = get_accel(orientation);
	matrix_t *mag_m = get_mag(orientation);

	gyro = gyro_out;
	accel = accel_m->data;
	mag = mag_m->data;
}

void sim_t::linear_interpolation(matrix_t *start_rot, matrix_t *end_rot, float duration, float timestep) {
	matrix_t *start_accel = get_accel(start_rot);
	matrix_t *start_mag = get_accel(start_rot);

	init_state(kinetic, start_accel->data, start_mag->data);

	for (int time = 0; time <= duration; time += timestep) {
		float norm_time = time / duration;

		matrix_t *prev_orientation = copy_matrix(orientation);
		orientation = add_matrix(start_rot, scale_matrix(sub_matrix(end_rot, start_rot), norm_time));

		matrix_t *gyro_m = get_gyro(prev_orientation, orientation, timestep);
		matrix_t *accel_m = get_accel(orientation);
		matrix_t *mag_m = get_mag(orientation);

		gyro = gyro_m->data;
		accel = accel_m->data;
		mag = mag_m->data;

		update_imu(kinetic, gyro, accel, mag, sample_rate_hertz);

		print();

		sleep(timestep);
	}
}

void sim_t::loop(void (*update_imu)(kinetic_t*, float*, float*, float*, float)) {
	while (true) {
		tick();
		update_imu(kinetic, gyro, accel, mag, sample_rate_hertz);
		sleep(1 / sample_rate_hertz);
	}
}

void sim_t::print() {
		cout << "============================================" << endl;
		cout << "Simulation orientation" << endl;
		cout << "============================================" << endl;

		print_matrix(quat_to_euler(orientation));
		cout << endl;

		cout << "============================================" << endl;
		cout << "Kinetic orientation" << endl;
		cout << "============================================" << endl;

		print_matrix(quat_to_euler(kinetic->state_q));
		cout << endl;
}

float *rand_rot(int range) {
	float* ret = (float *) malloc(3 * sizeof(float));

	for (int i = 0; i < 3; i++) {
		ret[i] = deg_to_rad(((rand() % (range * 200)) - range * 100) / 100.0);
	}

	return ret;
}

matrix_t *get_gyro(matrix_t *q1, matrix_t *q2, float dt) {
	matrix_t *ret = init_matrix(3, 1);

	ret->data[X] = q1->data[W] * q2->data[X] - q1->data[X] * q2->data[W] - q1->data[Y] * q2->data[Z] + q1->data[Z] * q2->data[Y];
	ret->data[Y] = q1->data[W] * q2->data[Y] + q1->data[X] * q2->data[Z] - q1->data[Y] * q2->data[W] - q1->data[Z] * q2->data[X];
	ret->data[Z] = q1->data[W] * q2->data[Z] - q1->data[X] * q2->data[Y] + q1->data[Y] * q2->data[X] - q1->data[Z] * q2->data[W];

	return ret;
}

matrix_t *get_accel(matrix_t *orientation) {
	float g_ref_a[] = {0, 0, 1};
	matrix_t *g_ref_m = arr_to_matrix(g_ref_a, 3, 1);
	// matrix_t *m = quat_to_rot_matrix(orientation);

	return mul_matrix(trans_matrix(quat_to_rot_matrix(orientation)), g_ref_m);
}

matrix_t *get_mag(matrix_t *orientation) {
	float mag_dip = 0;

	float m_ref_a[] = {cos(mag_dip), 0, sin(mag_dip)};
	matrix_t *m_ref_m = scale_matrix(arr_to_matrix(m_ref_a, 3, 1), 1 / (sqrt(pow(cos(mag_dip), 2) + pow(sin(mag_dip), 2))));

	return mul_matrix(trans_matrix(quat_to_rot_matrix(orientation)), m_ref_m);
}

