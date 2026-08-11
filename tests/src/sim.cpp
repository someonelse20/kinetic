#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <math.h>

#include "kin_math.h"
#include "sim.h"

using namespace std;

float *rand_rot(int range);

matrix_t *get_gyro(matrix_t *, matrix_t *, float);
matrix_t *get_accel(matrix_t *);
matrix_t *get_mag(matrix_t *, float);

sim_t::sim_t(imu_t *imu) {
	this->imu = imu;

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
	matrix_t *mag_m = get_mag(orientation, imu->mag_dip);

	gyro = gyro_out;
	accel = accel_m->data;
	mag = mag_m->data;
}

void sim_t::linear_interpolation(matrix_t *start_rot, matrix_t *end_rot, float duration, float timestep, plot_t *Plot) {
	sample_rate_hertz = 1 / timestep;

	imu->dt = timestep;

	matrix_t *start_accel = get_accel(start_rot);
	matrix_t *start_mag = get_mag(start_rot, imu->mag_dip);

	for (int i = 0; i < num_of_algs; i++) {
		ahrs_algs[i]->imu->dt = timestep;
		ahrs_algs[i]->imu_init(ahrs_algs[i]->imu, start_accel->data, start_mag->data);
	}

	for (float time = 0.0; time <= duration; time += timestep) {
		float norm_time = time / duration;

		matrix_t *prev_orientation = copy_matrix(orientation);
		orientation = add_matrix(start_rot, scale_matrix(sub_matrix(end_rot, start_rot), norm_time));

		matrix_t *gyro_m = get_gyro(prev_orientation, orientation, timestep);
		matrix_t *accel_m = get_accel(orientation);
		matrix_t *mag_m = get_mag(orientation, imu->mag_dip); // TODO: Be specific to each ahrs mag_dip not just global mag_dip.

		gyro = gyro_m->data;
		accel = accel_m->data;
		mag = mag_m->data;

		for (int i = 0; i < num_of_algs; i++) {
			ahrs_algs[i]->imu_update(ahrs_algs[i]->imu, gyro, accel, mag);

			if (Plot == NULL) continue;

			if (is_quat(ahrs_algs[i]->imu->ekf.state)) {
				Plot->add_point(quat_to_euler(ahrs_algs[i]->imu->ekf.state), ahrs_algs[i]->name);
			} else {
				Plot->add_point(ahrs_algs[i]->imu->ekf.state, ahrs_algs[i]->name);
			}
		}

		if (Plot != NULL) {
			Plot->add_point(quat_to_euler(orientation), "true");
		}

		int sleep_ms = timestep * 1000; // TODO Add flag for under millisecond timestep and accuracy.
		std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
	}

	if (Plot != NULL) {
		Plot->plot();
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

	print_matrix(quat_to_euler(imu->ekf.state));
	// print_matrix(imu->ekf.state);
	cout << endl;

	/*
	   cout << "============================================" << endl;
	   cout << "Kinetic covariance" << endl;
	   cout << "============================================" << endl;

	   print_matrix(imu->ekf.covariance);
	   cout << endl;
	 */
}

void sim_t::add_ahrs(matrix_t *(*imu_init)(imu_t *imu, float *accel, float *mag), matrix_t *(*imu_update)(imu_t *imu, float *gyro, float *accel, float *mag), std::string name) {
	ahrs_alg_t *new_ahrs = (ahrs_alg_t *)malloc(sizeof(ahrs_alg_t));
	new_ahrs->imu_init = imu_init;
	new_ahrs->imu_update = imu_update;
	new_ahrs->name = name;

	new_ahrs->imu = (imu_t *)malloc(sizeof(imu_t));
	new_ahrs->imu->gyro_noise = imu->gyro_noise;
	new_ahrs->imu->accel_noise = imu->accel_noise;
	new_ahrs->imu->mag_noise = imu->mag_noise;
	new_ahrs->imu->mag_dip = imu->mag_dip;
	new_ahrs->imu->mag_dec = imu->mag_dec;
	new_ahrs->imu->dt = imu->dt;

	ahrs_algs[num_of_algs] = new_ahrs;

	num_of_algs++;
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

	return scale_matrix(ret, 2 / dt);
}

matrix_t *get_accel(matrix_t *orientation) {
	float g_ref_a[] = {0, 0, 1};
	matrix_t *g_ref_m = arr_to_matrix(g_ref_a, 3, 1);
	// matrix_t *m = quat_to_rot_matrix(orientation);

	return mul_matrix(trans_matrix(quat_to_rot_matrix(orientation)), g_ref_m);
}

matrix_t *get_mag(matrix_t *orientation, float mag_dip) {
	float m_ref_a[] = {cos(mag_dip), 0, sin(mag_dip)};
	matrix_t *m_ref_m = scale_matrix(arr_to_matrix(m_ref_a, 3, 1), 1 / (sqrt(pow(cos(mag_dip), 2) + pow(sin(mag_dip), 2))));

	return mul_matrix(trans_matrix(quat_to_rot_matrix(orientation)), m_ref_m);
}

