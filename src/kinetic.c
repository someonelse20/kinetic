#include "kinetic.h"
#include "kin_math.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

// TODO:
// sensor calibration
// seperate mag update
// seperate accel/mag error

// Private function declaration

// Prediction step
static matrix_t *state_prediction(matrix_t *prev_state, float *gyro, float dt);
static matrix_t *state_transition(float *gyro, float dt);
static matrix_t *process_noise_covariance(matrix_t *state, float *gyro_noise, float dt);
static matrix_t *estm_covariance(matrix_t *prev_cov, matrix_t *state_trans_m, matrix_t *proc_noise_cov_m);

// Correction step
static matrix_t *measurement_model(matrix_t *state, matrix_t *g_ref, matrix_t *m_ref);
static matrix_t *measurement_model_jacob(matrix_t *state, matrix_t *g_ref, matrix_t *m_ref);
static matrix_t *measurement_noise_cov(float *accel_noise, float *mag_noise); // NOTE: This is usually static but in the future I want to make the noise dynamicaly change based on linear acceleration and magnetic interferance.
// No idea what this does
// but it seems to have something to do with the derivative
// of the measurement model
static float *jacobian(float *arr, float *quat);

void update_imu(kinetic_t *kinetic, float *gyro, float *accel, float *mag, float dt) {
	/* TODO: this section needs to be moved */

	// NOTE: Maybe try previous orientation or something else instead of estimated orientation
	matrix_t *rot_matrix = quat_to_rot_matrix(kinetic->state_q);

	// TODO: make this run at init, don't need to recalculate every loop
	// NED refrence frame
	// Factor in magnetic dip for m_ref
	float g_ref_a[] = {0, 0, 1};
	float m_ref_a[] = {cos(kinetic->mag_dip), 0, sin(kinetic->mag_dip)};
	matrix_t *g_ref_m = arr_to_matrix(g_ref_a, 3, 1);
	// TODO: find prettier way to do this
	matrix_t *m_ref_m = scale_matrix(arr_to_matrix(m_ref_a, 3, 1), 1 / (sqrt(pow(cos(kinetic->mag_dip), 2) + pow(sin(kinetic->mag_dip), 2))));

	/* end section that needs to be moved */

	// Precition step
	matrix_t *state_pred = state_prediction(kinetic->state_q, gyro, dt);
	matrix_t *state_trans = state_transition(gyro, dt);
	matrix_t *proc_noise_cov = process_noise_covariance(kinetic->state_q, kinetic->gyro_noise, dt);
	matrix_t *estm_cov = estm_covariance(kinetic->estm_covariance, state_trans, proc_noise_cov);

	// Correction step
	matrix_t *expect_g_ref = mul_matrix(trans_matrix(rot_matrix), g_ref_m);
	matrix_t *expect_m_ref = mul_matrix(trans_matrix(rot_matrix), m_ref_m);

	matrix_t *norm_accel_m = normalize_matrix(arr_to_matrix(accel, 3, 1));
	matrix_t *norm_mag_m = normalize_matrix(arr_to_matrix(mag, 3, 1));

	matrix_t *meas_model = measurement_model(kinetic->state_q, expect_g_ref, expect_m_ref);
	matrix_t *meas_model_jacob = measurement_model_jacob(kinetic->state_q, expect_g_ref, expect_m_ref);
	matrix_t *meas_noise_cov = measurement_noise_cov(kinetic->accel_noise, kinetic->mag_noise);

	// Build measurment (not model) matrix
	float meas_m_data[6];
	for (int i; i < 6; i++) {
		if (i < 3) {
			meas_m_data[i] = norm_accel_m->data[i];
		} else {
			meas_m_data[i] = norm_mag_m->data[i - 3];
		}
	}
	matrix_t *meas_m = arr_to_matrix(meas_m_data, 3, 1);

	matrix_t *meas_residual = sub_matrix(meas_m, meas_model);

	matrix_t *mes_model_jacob_trans = trans_matrix(meas_model_jacob);
	matrix_t *mes_pred_covariance = mul_matrix(meas_model_jacob, kinetic->estm_covariance);
	mes_pred_covariance = mul_matrix(mes_pred_covariance, mes_model_jacob_trans);
	mes_pred_covariance = add_matrix(mes_pred_covariance, meas_noise_cov);

	matrix_t *kalman_gain = mul_matrix(kinetic->estm_covariance, mes_model_jacob_trans);
	kalman_gain = mul_matrix(kalman_gain, inv_matrix(mes_pred_covariance));

	kinetic->state_q = add_matrix(kinetic->state_q, mul_matrix(kalman_gain, meas_residual));

	matrix_t *estm_covariance = mul_matrix(kalman_gain, meas_model_jacob);
	estm_covariance = sub_matrix(ident_matrix(4), estm_covariance);
	kinetic->estm_covariance = mul_matrix(estm_covariance, kinetic->estm_covariance);

}

void init_state(kinetic_t *kinetic, float accel[3], float mag[3]) {
	matrix_t *accel_m = arr_to_matrix(accel, 3, 1);
	matrix_t *mag_m = arr_to_matrix(mag, 3, 1);

	matrix_t *accel_x_mag = mul_vector(accel_m, mag_m);
	matrix_t *row_1 = normalize_matrix(mul_vector(accel_x_mag, accel_m));

	matrix_t *row_2 = normalize_matrix(accel_x_mag);

	matrix_t *row_3 = normalize_matrix(accel_m);

	matrix_t *rot_matrix = init_matrix(3, 3);
	for (size_t i = 0; i < 3; i++) {
		rot_matrix->data[3 * i] = row_1->data[i];
	}
	for (size_t i = 0; i < 3; i++) {
		rot_matrix->data[3 * i + 1] = row_2->data[i];
	}
	for (size_t i = 0; i < 3; i++) {
		rot_matrix->data[3 * i + 2] = row_3->data[i];
	}

	print_matrix(rot_matrix);
	kinetic->state_q = rot_matrix_to_quat(rot_matrix);
	printf("\n");
	print_matrix(kinetic->state_q);

	kinetic->estm_covariance = ident_matrix(4);
}

void update_barometer(kinetic_t *kinetic, float altitude, float dt);

void update_gps(kinetic_t *kinetic, float cords[2], float altitude, float dt);

static matrix_t *state_prediction(matrix_t *prev_state, float *gyro, float dt) {
	float *state_q = prev_state->data;

	float estm_state_q[] = {
		state_q[X] + (dt/2) * gyro[X] * state_q[W] - (dt/2) * gyro[Y] * state_q[Z] + (dt/2) * gyro[Z] * state_q[Y],
		state_q[Y] + (dt/2) * gyro[X] * state_q[Z] + (dt/2) * gyro[Y] * state_q[W] - (dt/2) * gyro[Z] * state_q[X],
		state_q[Z] - (dt/2) * gyro[X] * state_q[Y] + (dt/2) * gyro[Y] * state_q[X] + (dt/2) * gyro[Z] * state_q[W],
		state_q[W] - (dt/2) * gyro[X] * state_q[X] - (dt/2) * gyro[Y] * state_q[Y] - (dt/2) * gyro[Z] * state_q[Z],
	};

	return arr_to_matrix(estm_state_q, 4, 1);
}

static matrix_t *state_transition(float *gyro, float dt) {
	float state_trans_data[] = {
		1,                -(dt/2) * gyro[X], -(dt/2) * gyro[Y], -(dt/2) * gyro[Z],
		(dt/2) * gyro[X],                 1,  (dt/2) * gyro[Z], -(dt/2) * gyro[Y],
		(dt/2) * gyro[Y], -(dt/2) * gyro[Z],                 1,  (dt/2) * gyro[X],
		(dt/2) * gyro[Z],  (dt/2) * gyro[Y], -(dt/2) * gyro[X],                 1,
	};

	return arr_to_matrix(state_trans_data, 4, 4);
}

static matrix_t *process_noise_covariance(matrix_t *prev_state, float *gyro_noise, float dt) {
	matrix_t *noise_m;
	float *state_q = prev_state->data;

	float noise_data[] = { // If this doesn't work, put the last row in front
		-state_q[Y],  state_q[X],  state_q[W],
		-state_q[X], -state_q[Y], -state_q[Z],
		 state_q[W], -state_q[Z],  state_q[Y],
		 state_q[Z],  state_q[W], -state_q[X],
	};

	noise_m = arr_to_matrix(noise_data, 4, 3);
	noise_m = scale_matrix(noise_m, dt/2);

	matrix_t *gyro_noise_m = arr_to_matrix(gyro_noise, 1, 3);
	matrix_t *procs_noise_cov_m = mul_matrix(gyro_noise_m, noise_m);

	return mul_matrix(procs_noise_cov_m, trans_matrix(noise_m));
}

static matrix_t *estm_covariance(matrix_t *prev_cov, matrix_t *state_trans_m, matrix_t *proc_noise_cov_m) {
	matrix_t *ret;

	ret = mul_matrix(state_trans_m, ret);
	ret = mul_matrix(ret, trans_matrix(state_trans_m));

	return add_matrix(ret, proc_noise_cov_m);
}

static matrix_t *measurement_model(matrix_t *state, matrix_t *g_ref, matrix_t *m_ref) {
	float mes_model_data[6];

	for (int i; i < 6; i++) {
		if (i < 3) {
			mes_model_data[i] = g_ref->data[i];
		} else {
			mes_model_data[i] = m_ref->data[i - 3];
		}
	}

	return arr_to_matrix(mes_model_data, 6, 1);
}

static matrix_t *measurement_model_jacob(matrix_t *state, matrix_t *g_ref, matrix_t *m_ref) {
	float *g_ref_jacob = jacobian(g_ref->data, state->data);
	float *m_ref_jacob = jacobian(m_ref->data, state->data);
	float mes_model_jacob_data[24]; // 6x4 array

	for (size_t i = 0; i < 6; i++) {
		for (size_t j = 0; j < 4; j++) {
			if (i < 3) {
				mes_model_jacob_data[j * 6 + i] = g_ref_jacob[j * 3 + i];
			} else {
				mes_model_jacob_data[j * 6 + i] = g_ref_jacob[j * 3 + i - 6];
			}
		}
	}

	return arr_to_matrix(mes_model_jacob_data, 6, 4);
}

static matrix_t *measurement_noise_cov(float *accel_noise, float *mag_noise) {
	float noise_covariance_data[36]; // 6x6 array

	for (size_t i = 0; i < 6; i++) {
		for (size_t j = 0; j < 6; j++) {
			if (i != j) {
				noise_covariance_data[i * 6 + j] = 0;
			}

			if (i < 3) {
				noise_covariance_data[i * 6 + j] = accel_noise[i];
			} else {
				noise_covariance_data[i * 6 + j] = mag_noise[i - 3];
			}
		}
	}

	return arr_to_matrix(noise_covariance_data, 6, 6);
}

static float *jacobian(float *arr, float *quat) {
	float *ret = malloc(sizeof(float) * 12); // 3x4 matrix array

	ret[0]  =  arr[X] * quat[W] + arr[Y] * quat[Z] - arr[Z] * quat[Y];
	ret[1]  =  arr[X] * quat[X] + arr[Y] * quat[Y] + arr[Z] * quat[Z];
	ret[2]  = -arr[X] * quat[Y] + arr[Y] * quat[X] - arr[Z] * quat[W];
	ret[3]  = -arr[X] * quat[Z] + arr[Y] * quat[W] + arr[Z] * quat[X];

	ret[4]  = -arr[X] * quat[Z] + arr[Y] * quat[W] + arr[Z] * quat[X];
	ret[5]  =  arr[X] * quat[Y] - arr[Y] * quat[X] + arr[Z] * quat[W];
	ret[6]  =  arr[X] * quat[X] + arr[Y] * quat[Y] + arr[Z] * quat[Z];
	ret[7]  = -arr[X] * quat[W] - arr[Y] * quat[Z] + arr[Z] * quat[Y];

	ret[8]  =  arr[X] * quat[Y] - arr[Y] * quat[X] + arr[Z] * quat[W];
	ret[9]  =  arr[X] * quat[Z] - arr[Y] * quat[W] - arr[Z] * quat[X];
	ret[10] =  arr[X] * quat[W] + arr[Y] * quat[Z] - arr[Z] * quat[Y];
	ret[11] =  arr[X] * quat[X] + arr[Y] * quat[Y] + arr[Z] * quat[Z];

	return ret;
}

