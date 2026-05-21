#include "kinetic.h"
#include "kin_math.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/types.h>

// TODO:
// sensor calibration
// seperate mag update
// seperate accel/mag error

// Private function declaration
void predict_state(kinetic_t *kinetic, double gyro[3], double dt);
void corect_state(kinetic_t *kinetic, double accel[3], double mag[3], double dt);
// No idea what this does
// but it seems to have something to do with the derivative
// of the measurement model
double *jacobian(double *arr, double *quat);

void update_imu(kinetic_t *kinetic, double gyro[3], double accel[3], double mag[3], double dt) {
	predict_state(kinetic, gyro, dt);

	corect_state(kinetic, accel, mag, dt);
}

void init_state(kinetic_t *kinetic, double accel[3], double mag[3]) { // TODO: there's a better way to do this
	matrix_t *accel_m = arr_to_matrix(accel, true);
	matrix_t *mag_m = arr_to_matrix(mag, true);

	matrix_t *accel_x_mag = mul_matrix(accel_m, mag_m);
	matrix_t *row_1 = normalize_matrix(mul_matrix(row_1, accel_m));

	matrix_t *row_2 = normalize_matrix(accel_x_mag);

	matrix_t *row_3 = normalize_matrix(accel_m);

	matrix_t *rot_matrix = init_matrix(3, 3);
	for (size_t i = 0; i < 3; i++) {
		rot_matrix->data[i] = row_1->data[i];
	}
	for (size_t i = 0; i < 3; i++) {
		rot_matrix->data[3 + i] = row_2->data[i];
	}
	for (size_t i = 0; i < 3; i++) {
		rot_matrix->data[6 + i] = row_3->data[i];
	}

	kinetic->state_q = init_matrix(4, 1); // NOTE: some of these indexes might be wrong
	kinetic->state_q->data[X] = 0.5 * sqrt(rot_matrix->data[0] + rot_matrix->data[4] + rot_matrix->data[8] + 1);
	kinetic->state_q->data[Y] = 0.5 * sgn(rot_matrix->data[7] - rot_matrix->data[5]) * sqrt(rot_matrix->data[0] - rot_matrix->data[4] - rot_matrix->data[8] + 1);
	kinetic->state_q->data[Z] = 0.5 * sgn(rot_matrix->data[2] - rot_matrix->data[6]) * sqrt(rot_matrix->data[4] - rot_matrix->data[8] - rot_matrix->data[0] + 1);
	kinetic->state_q->data[W] = 0.5 * sgn(rot_matrix->data[3] - rot_matrix->data[1]) * sqrt(rot_matrix->data[8] - rot_matrix->data[0] - rot_matrix->data[4] + 1);

	kinetic->estm_covariance = ident_matrix(4, 4);
}

void update_barometer(kinetic_t *kinetic, double altitude, double dt);

void update_gps(kinetic_t *kinetic, double cords[2], double altitude, double dt);

void predict_state(kinetic_t *kinetic, double gyro[3], double dt) {
	// Calculate state prediction
	double *state_q = kinetic->state_q->data;
	double estm_state_q[] = {
		state_q[X] + (dt/2) * gyro[X] * state_q[W] - (dt/2) * gyro[Y] * state_q[Z] + (dt/2) * gyro[Z] * state_q[Y],
		state_q[Y] + (dt/2) * gyro[X] * state_q[Z] + (dt/2) * gyro[Y] * state_q[W] - (dt/2) * gyro[Z] * state_q[X],
		state_q[Z] - (dt/2) * gyro[X] * state_q[Y] + (dt/2) * gyro[Y] * state_q[X] + (dt/2) * gyro[Z] * state_q[W],
		state_q[W] - (dt/2) * gyro[X] * state_q[X] - (dt/2) * gyro[Y] * state_q[Y] - (dt/2) * gyro[Z] * state_q[Z],
	};
	kinetic->state_q->data = estm_state_q;

	// Calculate state transition matrix
	matrix_t *state_trans_m = init_matrix(4, 4);
	double state_trans_data[] = {
		1,                -(dt/2) * gyro[X], -(dt/2) * gyro[Y], -(dt/2) * gyro[Z],
		(dt/2) * gyro[X],                 1,  (dt/2) * gyro[Z], -(dt/2) * gyro[Y],
		(dt/2) * gyro[Y], -(dt/2) * gyro[Z],                 1,  (dt/2) * gyro[X],
		(dt/2) * gyro[Z],  (dt/2) * gyro[Y], -(dt/2) * gyro[X],                 1,
	};
	state_trans_m->data = state_trans_data;

	// Calculate process noise covariance matrix
	matrix_t *noise_m = init_matrix(4, 3);
	double noise_data[] = { // If this doesn't work, put the last row in front
		-state_q[Y],  state_q[X],  state_q[W],
		-state_q[X], -state_q[Y], -state_q[Z],
		 state_q[W], -state_q[Z],  state_q[Y],
		 state_q[Z],  state_q[W], -state_q[X],
	};
	noise_m->data = noise_data;
	noise_m = scale_matrix(noise_m, dt/2);

	matrix_t *gyro_noise_m = arr_to_matrix(kinetic->gyro_noise, false);
	matrix_t *procs_noise_cov_m = mul_matrix(gyro_noise_m, noise_m);
	procs_noise_cov_m = mul_matrix(procs_noise_cov_m, trans_matrix(noise_m));

	// matrix_t *kinetic->estm_covariance = mul_matrix(state_trans_m, kinetic->estm_covariance);
	kinetic->estm_covariance = mul_matrix(state_trans_m, kinetic->estm_covariance);
	kinetic->estm_covariance = mul_matrix(kinetic->estm_covariance, trans_matrix(state_trans_m));
	kinetic->estm_covariance = add_matrix(kinetic->estm_covariance, procs_noise_cov_m);

	return;
}

void corect_state(kinetic_t *kinetic, double accel[3], double mag[3], double dt) {
	// NOTE: Maybe try previous orientation or something else instead of estimated orientation
	matrix_t *rot_matrix = quat_to_rot_matrix(kinetic->state_q);

	// TODO: make this run at init, don't need to recalculate every loop
	// NED refrence frame
	// Factor in magnetic dip for m_ref
	double g_ref_a[] = {0, 0, 1};
	double m_ref_a[] = {cos(kinetic->mag_dip), 0, sin(kinetic->mag_dip)};
	matrix_t *g_ref_m = arr_to_matrix(g_ref_a, true);
	// TODO: find prettier way to do this
	matrix_t *m_ref_m = scale_matrix(arr_to_matrix(m_ref_a, true), 1 / (sqrt(pow(cos(kinetic->mag_dip), 2) + pow(sin(kinetic->mag_dip), 2))));

	// NOTE: this does need to be calculated every loop
	matrix_t *expect_g_ref = mul_matrix(trans_matrix(rot_matrix), g_ref_m);
	matrix_t *expect_m_ref = mul_matrix(trans_matrix(rot_matrix), m_ref_m);

	double mes_model_data[6];
	for (int i; i < 6; i++) {
		if (i < 3) {
			mes_model_data[i] = expect_g_ref->data[i];
		} else {
			mes_model_data[i] = expect_m_ref->data[i - 3];
		}
	}
	matrix_t *mes_model = arr_to_matrix(mes_model_data, true);

	double *g_ref_jacob = jacobian(g_ref_a, kinetic->state_q->data);
	double *m_ref_jacob = jacobian(m_ref_a, kinetic->state_q->data);

	double mes_model_jacob_data[24]; // 6x4 array
	for (size_t i = 0; i < 6; i++) {
		for (size_t j = 0; j < 4; j++) {
			if (i < 3) {
				mes_model_jacob_data[j * 6 + i] = g_ref_jacob[j * 3 + i];
			} else {
				mes_model_jacob_data[j * 6 + i] = g_ref_jacob[j * 3 + i - 6];
			}
		}
	}
	matrix_t *mes_model_jacob = init_matrix(6, 4);
	mes_model_jacob->data = mes_model_jacob_data;

	matrix_t *noise_covariance = init_matrix(6, 6);
	double noise_covariance_data[36];
	for (size_t i = 0; i < 6; i++) {
		for (size_t j = 0; j < 6; j++) {
			if (i != j) {
				noise_covariance_data[i * 6 + j] = 0;
			}

			if (i < 3) {
				noise_covariance_data[i * 6 + j] = kinetic->accel_noise[i];
			} else {
				noise_covariance_data[i * 6 + j] = kinetic->mag_noise[i - 3];
			}
		}
	}
	noise_covariance->data = noise_covariance_data;

	matrix_t *norm_accel_m = normalize_matrix(arr_to_matrix(accel, true));
	matrix_t *norm_mag_m = normalize_matrix(arr_to_matrix(mag, true));

	double mes_m_data[6];
	for (int i; i < 6; i++) {
		if (i < 3) {
			mes_m_data[i] = norm_accel_m->data[i];
		} else {
			mes_m_data[i] = norm_mag_m->data[i - 3];
		}
	}
	matrix_t *mes_m = arr_to_matrix(mes_m_data, true);

	matrix_t *mes_residual = sub_matrix(mes_m, mes_model);

	matrix_t *mes_model_jacob_trans = trans_matrix(mes_model_jacob);
	matrix_t *mes_pred_covariance = mul_matrix(mes_model_jacob, kinetic->estm_covariance);
	mes_pred_covariance = mul_matrix(mes_pred_covariance, mes_model_jacob_trans);
	mes_pred_covariance = add_matrix(mes_pred_covariance, noise_covariance);

	matrix_t *kalman_gain = mul_matrix(kinetic->estm_covariance, mes_model_jacob_trans);
	kalman_gain = mul_matrix(kalman_gain, inv_matrix(mes_pred_covariance));

	kinetic->state_q = add_matrix(kinetic->state_q, mul_matrix(kalman_gain, mes_residual));

	matrix_t *estm_covariance = mul_matrix(kalman_gain, mes_model_jacob);
	estm_covariance = sub_matrix(ident_matrix(4, 4), estm_covariance);
	kinetic->estm_covariance = mul_matrix(estm_covariance, kinetic->estm_covariance);
}

double *jacobian(double *arr, double *quat) {
	double *ret = malloc(sizeof(double) * 12); // 3x4 matrix array

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

