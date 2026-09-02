#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <iomanip>
#include <cstdlib>
#include <limits>
#include <chrono>
#include <thread>
#include <math.h>
#include <ctime>

#include "kin_math.h"
#include "kin_types.h"
#include "sim.h"

using namespace std;

float *rand_rot(int range);

static matrix_t *get_gyro(matrix_t *, matrix_t *, float);
static matrix_t *get_accel(matrix_t *);
static matrix_t *get_mag(matrix_t *, float);
static float get_error(matrix_t *true_q, matrix_t *estm_q);

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

	matrix_t *rate_of_change_q = scale_matrix_alloc(quat_prod(orientation, gyro_q), 0.5);
	orientation = add_matrix_alloc(orientation, rate_of_change_q);
	normalize_matrix(orientation);

	matrix_t *accel_m = get_accel(orientation);
	matrix_t *mag_m = get_mag(orientation, imu->mag_dip);

	gyro = gyro_out;
	accel = accel_m->data;
	mag = mag_m->data;
}

void sim_t::one_axis_test(int steps, plot_t *Plot) {
	float step_size = 2 * M_PI / steps;
	float timestep = step_size;
	imu->dt = timestep;

	float x_count = 0.f;

	matrix_t *orientation_euler = init_matrix(3, 1);
	orientation_euler->data[X] = sin(x_count);
	orientation_euler->data[Y] = 0.f;
	orientation_euler->data[Z] = 0.f;

	orientation = euler_to_quat(orientation_euler);

	matrix_t *start_accel = get_accel(orientation);
	matrix_t *start_mag = get_mag(orientation, imu->mag_dip);

	for (int i = 0; i < num_of_algs; i++) {
		ahrs_algs[i]->imu->dt = timestep;
		ahrs_algs[i]->imu_init(ahrs_algs[i]->imu, start_accel->data, start_mag->data);
		ahrs_algs[i]->error_buf = (float *)malloc(steps * sizeof(float) + 10); // Add 10 for good measure.
	}

	int count = 0;
	while (x_count <= 2 * M_PI) {
		x_count += step_size;

		orientation_euler->data[X] = sin(x_count);

		matrix_t *prev_orientation = copy_matrix(orientation);
		orientation = euler_to_quat(orientation_euler);

		matrix_t *gyro_m = get_gyro(prev_orientation, orientation, timestep);
		matrix_t *accel_m = get_accel(orientation);
		matrix_t *mag_m = get_mag(orientation, imu->mag_dip); // TODO: Be specific to each ahrs mag_dip not just global mag_dip.

		gyro = gyro_m->data;
		accel = accel_m->data;
		mag = mag_m->data;

		for (int i = 0; i < num_of_algs; i++) {
			ahrs_algs[i]->imu_update(ahrs_algs[i]->imu, gyro, accel, mag);

			if (is_quat(ahrs_algs[i]->imu->ekf.state)) {
				ahrs_algs[i]->error_buf[count] = get_error(orientation, ahrs_algs[i]->imu->ekf.state);
			} else {
				ahrs_algs[i]->error_buf[count] = get_error(orientation, euler_to_quat(ahrs_algs[i]->imu->ekf.state));
			}

			if (Plot == NULL) continue;

			if (is_quat(ahrs_algs[i]->imu->ekf.state)) {
				Plot->add_point(quat_to_euler(ahrs_algs[i]->imu->ekf.state), ahrs_algs[i]->name);
			} else {
				Plot->add_point(ahrs_algs[i]->imu->ekf.state, ahrs_algs[i]->name);
			}
			/*
			 */
		}

		if (Plot != NULL) {
			Plot->add_point(quat_to_euler(orientation), "true");
		}

		count++;
	}

	error_report(steps);

	if (Plot != NULL) {
		Plot->plot("One Axit Test");
	}
}

void sim_t::all_axis_test(int steps, plot_t *Plot) {
	float step_size = 2 * M_PI / steps;
	float timestep = step_size;
	imu->dt = timestep;

	float x_count = 0.f + 0.2;
	float y_count = M_PI / 2.f + 0.2;
	float z_count = M_PI + 0.2;

	matrix_t *orientation_euler = init_matrix(3, 1);
	orientation_euler->data[X] = sin(x_count);
	orientation_euler->data[Y] = sin(y_count);
	orientation_euler->data[Z] = sin(z_count);
	// orientation_euler->data[Z] = 0.f;

	orientation = euler_to_quat(orientation_euler);

	matrix_t *start_accel = get_accel(orientation);
	matrix_t *start_mag = get_mag(orientation, imu->mag_dip);

	/*
	print_matrix(start_accel);
	cout << endl;
	print_matrix(start_mag);
	*/

	for (int i = 0; i < num_of_algs; i++) {
		ahrs_algs[i]->imu->dt = timestep;
		ahrs_algs[i]->imu_init(ahrs_algs[i]->imu, start_accel->data, start_mag->data);
		ahrs_algs[i]->error_buf = (float *)malloc(steps * sizeof(float) + 10); // Add 10 for good measure.

		cout << "=========================" << endl;
		cout << "init simulation orientation" << endl;
		cout << "=========================" << endl;
		print_matrix(quat_to_euler(orientation));
		cout << endl;

		cout << "=========================" << endl;
		cout << "init kinetic orientation" << endl;
		cout << "=========================" << endl;
		print_matrix(quat_to_euler(ahrs_algs[i]->imu->ekf.state));
		cout << endl;
		/*
		*/
	}

	int count = 0;
	while (x_count <= 2 * M_PI) {
		x_count += step_size;
		y_count += step_size;
		z_count += step_size;

		orientation_euler->data[X] = sin(x_count);
		orientation_euler->data[Y] = sin(y_count);
		orientation_euler->data[Z] = sin(z_count);
		// orientation_euler->data[Z] = 0.f;

		matrix_t *prev_orientation = copy_matrix(orientation);
		orientation = euler_to_quat(orientation_euler);

		matrix_t *gyro_m = get_gyro(prev_orientation, orientation, timestep);
		matrix_t *accel_m = get_accel(orientation);
		matrix_t *mag_m = get_mag(orientation, imu->mag_dip); // TODO: Be specific to each ahrs mag_dip not just global mag_dip.

		gyro = gyro_m->data;
		accel = accel_m->data;
		mag = mag_m->data;

		for (int i = 0; i < num_of_algs; i++) {
			ahrs_algs[i]->imu_update(ahrs_algs[i]->imu, gyro, accel, mag);

			if (is_quat(ahrs_algs[i]->imu->ekf.state)) {
				ahrs_algs[i]->error_buf[count] = get_error(orientation, ahrs_algs[i]->imu->ekf.state);
			} else {
				ahrs_algs[i]->error_buf[count] = get_error(orientation, euler_to_quat(ahrs_algs[i]->imu->ekf.state));
			}

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

		count++;

		free_matrix(prev_orientation);
		free_matrix(gyro_m);
		free_matrix(accel_m);
		free_matrix(mag_m);
	}

	error_report(steps);
	/*
	*/

	if (Plot != NULL) {
		Plot->plot("3 Axis Test");
	}

	// free_matrix(orientation_euler);
}

void sim_t::linear_interpolation(matrix_t *start_rot, matrix_t *end_rot, float duration, float timestep, plot_t *Plot) {
	imu->dt = timestep;

	matrix_t *start_accel = get_accel(start_rot);
	matrix_t *start_mag = get_mag(start_rot, imu->mag_dip);

	for (int i = 0; i < num_of_algs; i++) {
		ahrs_algs[i]->imu->dt = timestep;
		ahrs_algs[i]->imu_init(ahrs_algs[i]->imu, start_accel->data, start_mag->data);
		ahrs_algs[i]->error_buf = (float *)malloc(ceil(duration / timestep) * sizeof(float) + 10); // Add 10 for good measure.
		
		cout << "=========================" << endl;
		cout << "init simulation orientation" << endl;
		cout << "=========================" << endl;
		print_matrix(quat_to_euler(orientation));
		cout << endl;

		cout << "=========================" << endl;
		cout << "init kinetic orientation" << endl;
		cout << "=========================" << endl;
		print_matrix(quat_to_euler(ahrs_algs[i]->imu->ekf.state));
		cout << endl;
	}

	int count = 0;
	for (float time = 0.0; time <= duration; time += timestep) {
		float norm_time = time / duration;

		matrix_t *prev_orientation = copy_matrix(orientation);
		orientation = add_matrix_alloc(start_rot, scale_matrix_alloc(sub_matrix_alloc(end_rot, start_rot), norm_time));

		matrix_t *gyro_m = get_gyro(prev_orientation, orientation, timestep);
		matrix_t *accel_m = get_accel(orientation);
		matrix_t *mag_m = get_mag(orientation, imu->mag_dip); // TODO: Be specific to each ahrs mag_dip not just global mag_dip.

		gyro = gyro_m->data;
		accel = accel_m->data;
		mag = mag_m->data;

		for (int i = 0; i < num_of_algs; i++) {
			ahrs_algs[i]->imu_update(ahrs_algs[i]->imu, gyro, accel, mag);

			if (is_quat(ahrs_algs[i]->imu->ekf.state)) {
				ahrs_algs[i]->error_buf[count] = get_error(orientation, ahrs_algs[i]->imu->ekf.state);
			} else {
				ahrs_algs[i]->error_buf[count] = get_error(orientation, euler_to_quat(ahrs_algs[i]->imu->ekf.state));
			}

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

		/*
		   int sleep_ms = timestep * 1000; // TODO Add flag for under millisecond timestep and accuracy.
		   std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
		 */

		count++;
	}

	error_report(count);

	if (Plot != NULL) {
		Plot->plot("Linear Interpolation Test");
	}
}

void sim_t::error_report(int count) {
	int min_spacing = 17;

	for (int i = 0; i < num_of_algs; i++) {
		float sum = 0;
		for (int j = 0; j < count; j++) {
			float error = ahrs_algs[i]->error_buf[j];
			sum += error;

			if (error < abs(ahrs_algs[i]->min_error)) {
				ahrs_algs[i]->min_error = error;
			}

			if (error > abs(ahrs_algs[i]->max_error)) {
				ahrs_algs[i]->max_error = error;
			}
		}

		ahrs_algs[i]->mean_error = sum / count;

		if (ahrs_algs[i]->name.size() >= min_spacing) min_spacing = ahrs_algs[i]->name.size() + 1;
	}

	for (int i = 0; i < min_spacing * num_of_algs; i++) cout << "=";
	cout << endl;
	cout << "Simulation error report:" << endl;
	for (int i = 0; i < min_spacing * num_of_algs; i++) cout << "=";
	cout << endl;

	for (int i = 0; i < num_of_algs; i++) {
		cout << ahrs_algs[i]->name;
		for (int j = 0; j < min_spacing - ahrs_algs[i]->name.size() - 2; j++) cout << " ";
		cout << "| ";
	}
	cout << endl;

	for (int i = 0; i < num_of_algs; i++) {
		cout << "mean: ";
		cout << fixed << setprecision(numeric_limits<float>::max()) << ahrs_algs[i]->mean_error;
		for (int j = 0; j < min_spacing - 16; j++) cout << " ";
		cout << "| ";
	}
	cout << endl;

	for (int i = 0; i < num_of_algs; i++) {
		cout << "min:  ";
		cout << fixed << setprecision(numeric_limits<float>::max()) << ahrs_algs[i]->min_error;
		for (int j = 0; j < min_spacing - 16; j++) cout << " ";
		cout << "| ";
	}
	cout << endl;

	for (int i = 0; i < num_of_algs; i++) {
		cout << "max:  ";
		cout << fixed << setprecision(numeric_limits<float>::max()) << ahrs_algs[i]->max_error;
		for (int j = 0; j < min_spacing - 16; j++) cout << " ";
		cout << "| ";
	}
	cout << endl;

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
	new_ahrs->imu->enu = imu->enu;

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

	return scale_matrix_free(ret, 2 / dt);
}

matrix_t *get_accel(matrix_t *orientation) {
	// float g_ref_a[] = {0, 0, 1};
	float g_ref_a[] = {0, 0, -1};
	matrix_t *g_ref_m = arr_to_matrix(g_ref_a, 3, 1);

	matrix_t *rot_matrix = quat_to_rot_matrix(orientation);
	matrix_t *ret = mul_matrix_free(trans_matrix_free(rot_matrix), g_ref_m);

	free_matrix(g_ref_m);
	return ret;
	/*
	   float g_ref_a[] = {0, 0, 1};
	   matrix_t *g_ref_m = arr_to_matrix(g_ref_a, 3, 1);

	   return mul_matrix(trans_matrix(quat_to_rot_matrix(orientation)), g_ref_m);
	 */
}

matrix_t *get_mag(matrix_t *orientation, float mag_dip) {
	// float m_ref_a[] = {cos(mag_dip), 0, sin(mag_dip)};
	float m_ref_a[] = {0, cos(mag_dip), -sin(mag_dip)};
	matrix_t *m_ref_m = arr_to_matrix(m_ref_a, 3, 1);
	m_ref_m = scale_matrix_free(m_ref_m, 1 / (sqrt(pow(cos(mag_dip), 2) + pow(sin(mag_dip), 2))));

	matrix_t *rot_matrix = quat_to_rot_matrix(orientation);
	matrix_t *ret = mul_matrix_free(trans_matrix_free(rot_matrix), m_ref_m);

	free_matrix(m_ref_m);
	return ret;
	/*
	 */

	/*
	   float m_ref_a[] = {cos(mag_dip), 0, sin(mag_dip)};
	   matrix_t *m_ref_m = scale_matrix(arr_to_matrix(m_ref_a, 3, 1), 1 / (sqrt(pow(cos(mag_dip), 2) + pow(sin(mag_dip), 2))));

	   return mul_matrix(trans_matrix(quat_to_rot_matrix(orientation)), m_ref_m);
	 */
}

static float get_error(matrix_t *true_q, matrix_t *estm_q) {
	/*
	   // Angle between quaternions (maybe) and scaled from 0 to pi/2 to 0 to 1.
	   float angle = 2 * asin(dot_prod(true_q, estm_q));
	   return angle / (M_PI / 2);
	 */
	// Calculate error orientation
	matrix_t *inv = inv_quat(true_q);
	matrix_t *error_q = quat_prod(estm_q, inv);

	// Calculate the angle between the orientations
	float x2 = error_q->data[X] * error_q->data[X];
	float y2 = error_q->data[Y] * error_q->data[Y];
	float z2 = error_q->data[Z] * error_q->data[Z];
	float angle = atan2(sqrt(x2 + y2 + z2), error_q->data[W]);

	free_matrix(inv);
	free_matrix(error_q);
	return angle;

	// Scale from [0 pi/2] to [0 1].
	// return angle / (M_PI);
}

