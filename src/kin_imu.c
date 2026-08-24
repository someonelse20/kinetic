#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include "kin_types.h"
#include "kin_math.h"
#include "kin_ekf.h"
#include "kin_imu.h"

static matrix_t *state_prediction(matrix_t *prev_state, float *gyro, float dt);
static matrix_t *state_prediction_jacobian(float *gyro, float dt);
static matrix_t *process_noise(matrix_t *prev_state, float gyro_noise, float dt);
static matrix_t *observe_model(matrix_t *state, matrix_t *g_ref, matrix_t *m_ref);
static matrix_t *observe_model_jacobian(matrix_t *state, matrix_t *g_ref, matrix_t *m_ref);
static matrix_t *observe_model_jacobian_helper(matrix_t *ctr_vtr, matrix_t *ref, matrix_t *real, float scalar);

// If other files use this function move over to kin_math
static matrix_t *stack_matrix(const matrix_t *a, const matrix_t *b);

matrix_t *imu_init(imu_t *imu, float *accel, float *mag) {
	// float *accel_cpy = copy_arr(accel, 3);
	// float *mag_cpy = copy_arr(mag, 3);

	// Calculate orientation purly based of the accelerometer and magnetometer to start with.

	matrix_t *accel_m = arr_to_matrix(accel, 3, 1);
	matrix_t *mag_m = arr_to_matrix(mag, 3, 1);

	/* This used to calculate a rotation matrix but there is a bug where the y axis has the wrong
	 * sign (What I was testing it was ~-57 deg when it should be ~57 deg). I have no clue why it
	 * is doing this. It has been replaced with code that calculates the euler angles instead.
	 * I'm leaving this code here in case I find a fix and want to go back to the old code.

	matrix_t *accel_x_mag = cross_prod(accel_m, mag_m);

	matrix_t *collumn_1 = normalize_matrix(cross_prod(accel_x_mag, accel_m));
	matrix_t *collumn_2 = normalize_matrix(accel_x_mag);
	matrix_t *collumn_3 = normalize_matrix(accel_m);

	print_matrix(collumn_1);
	printf("\n");
	print_matrix(collumn_2);
	printf("\n");
	print_matrix(collumn_3);
	printf("\n");

	matrix_t *rot_matrix = init_matrix(3, 3);
	for (size_t i = 0; i < 3; i++) {
		rot_matrix->data[3 * i] = collumn_1->data[i];
	}
	for (size_t i = 0; i < 3; i++) {
		rot_matrix->data[3 * i + 1] = collumn_2->data[i];
	}
	for (size_t i = 0; i < 3; i++) {
		rot_matrix->data[3 * i + 2] = collumn_3->data[i];
	}

	print_matrix(rot_matrix);
	printf("\n");

	imu->ekf.state = rot_matrix_to_quat(rot_matrix);
	*/

	matrix_t *state_euler = init_matrix(3, 1);
	state_euler->data[X] = atan2(accel[Y], accel[Z]);
	state_euler->data[Y] = atan2(-accel[X], sqrt(accel[Y] * accel[Y] + accel[Z] * accel[Z]));

	matrix_t *west = cross_prod(accel_m, mag_m);
	matrix_t *north = cross_prod(west, accel_m);
	matrix_t *frame = fill_matrix(3, 1, 0.f);
	frame->data[X] = 1.f;
	state_euler->data[Z] = atan2(dot_prod(west, frame), dot_prod(north, frame));

	imu->ekf.state = euler_to_quat(state_euler);

	// Initialize rest of variables

	imu->ekf.covariance = ident_matrix(4);

	// NED reference frame
	// Factor in magnetic dip for m_ref
	if (imu->mag_dip == 0.f) {
		printf("Invalid mag dip. When setting mag dip to zero weird things happen.\n");
		printf("Resetting mag dip to 0.000001\n");
		imu->mag_dip = 0.000001;
	}

	float g_ref_a[] = {0, 0, 1};
	float m_ref_a[] = {cos(imu->mag_dip), 0, sin(imu->mag_dip)};
	matrix_t *m_ref_m = arr_to_matrix(m_ref_a, 3, 1);
	imu->g_ref = arr_to_matrix(g_ref_a, 3, 1);
	imu->m_ref = scale_matrix_alloc(m_ref_m, 1 / (sqrt(pow(cos(imu->mag_dip), 2) + pow(sin(imu->mag_dip), 2))));


	// Use static noise for now.

	imu->proc_noise = init_matrix(3, 3);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			uint8_t index = i * 3 + j;
			if (i == j) {
				imu->proc_noise->data[index] = imu->gyro_noise * imu->gyro_noise;
			} else {
				imu->proc_noise->data[index] = 0.0;
			}
		}
	}

	imu->meas_noise = init_matrix(6, 6);
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			uint8_t index = i * 6 + j;
			float noise = (i < 3) ? imu->accel_noise : imu->mag_noise;
			if (i == j) {
				imu->meas_noise->data[index] = noise * noise;
			} else {
				imu->meas_noise->data[index] = 0;
			}
		}
	}

	free_matrix(accel_m);
	free_matrix(mag_m);
	free_matrix(state_euler);
	free_matrix(west);
	free_matrix(north);
	free_matrix(frame);
	free_matrix(m_ref_m);

	return imu->ekf.state;
}

uint8_t imu_deinit(imu_t *imu) {
	ekf_deinit(&imu->ekf);

	free_matrix(imu->g_ref);
	free_matrix(imu->m_ref);
	free_matrix(imu->proc_noise);
	free_matrix(imu->meas_noise);

	return 0;
}

matrix_t *imu_update(imu_t *imu, float *gyro, float *accel, float *mag) {
	float *gyro_cpy = copy_arr(gyro, 3);
	float *accel_cpy = copy_arr(accel, 3);
	float *mag_cpy = copy_arr(mag, 3);

	matrix_t *accel_m = arr_to_matrix(accel_cpy, 3, 1);
	matrix_t *mag_m = arr_to_matrix(mag_cpy, 3, 1);

	matrix_t *meas = stack_matrix(accel_m, mag_m);
	matrix_t *state_pred = state_prediction(imu->ekf.state, gyro_cpy, imu->dt);
	matrix_t *state_pred_jacob = state_prediction_jacobian(gyro_cpy, imu->dt);
	matrix_t *obsv_model = observe_model(state_pred, imu->g_ref, imu->m_ref);
	matrix_t *obsv_model_jacob = observe_model_jacobian(state_pred, imu->g_ref, imu->m_ref);
	matrix_t *proc_noise = process_noise(imu->ekf.state, imu->gyro_noise, imu->dt);

	ekf_update(&imu->ekf, meas, state_pred, state_pred_jacob, obsv_model, obsv_model_jacob, proc_noise, imu->meas_noise);

	// Normalize output.
	normalize_matrix(imu->ekf.state);

	free(gyro_cpy);
	free(accel_cpy);
	free(mag_cpy);
	free_matrix(accel_m);
	free_matrix(mag_m);
	free_matrix(meas);
	free_matrix(state_pred);
	free_matrix(state_pred_jacob);
	free_matrix(obsv_model);
	free_matrix(obsv_model_jacob);
	free_matrix(proc_noise);

	return imu->ekf.state;
}

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

static matrix_t *state_prediction_jacobian(float *gyro, float dt) {
	float state_trans_data[] = {
		1,                -(dt/2) * gyro[X], -(dt/2) * gyro[Y], -(dt/2) * gyro[Z],
		(dt/2) * gyro[X],                 1,  (dt/2) * gyro[Z], -(dt/2) * gyro[Y],
		(dt/2) * gyro[Y], -(dt/2) * gyro[Z],                 1,  (dt/2) * gyro[X],
		(dt/2) * gyro[Z],  (dt/2) * gyro[Y], -(dt/2) * gyro[X],                 1,
	};

	return arr_to_matrix(state_trans_data, 4, 4);
}

static matrix_t *process_noise(matrix_t *prev_state, float gyro_noise, float dt) {
	matrix_t *ret;

	float noise_data[] = {
		-prev_state->data[Y],  prev_state->data[X],  prev_state->data[W],
		-prev_state->data[X], -prev_state->data[Y], -prev_state->data[Z],
		 prev_state->data[W], -prev_state->data[Z],  prev_state->data[Y],
		 prev_state->data[Z],  prev_state->data[W], -prev_state->data[X],
	};

	matrix_t *noise_m = arr_to_matrix(noise_data, 4, 3);
	noise_m = scale_matrix_free(noise_m, dt/2);

	matrix_t *noise_trans = trans_matrix_alloc(noise_m);

	ret = scale_matrix_free(noise_m, gyro_noise * gyro_noise);
	ret = mul_matrix_free(ret, noise_trans);

	free_matrix(noise_trans);
	return ret;
}

static matrix_t *observe_model(matrix_t *state_pred, matrix_t *g_ref, matrix_t *m_ref) {
	matrix_t *ret = init_matrix(6, 1);

	matrix_t *rot_matrix = quat_to_rot_matrix(state_pred);
	matrix_t *rot_matrix_trans = trans_matrix_free(rot_matrix);

	matrix_t *accel_model = mul_matrix_alloc(rot_matrix_trans, g_ref);
	matrix_t *mag_model = mul_matrix_alloc(rot_matrix_trans, m_ref);

	for (int i = 0; i < 6; i++) {
		if (i < 3) {
			ret->data[i] = accel_model->data[i];
		} else {
			ret->data[i] = mag_model->data[i - 3];
		}
	}

	// free_matrix(rot_matrix);
	free_matrix(rot_matrix_trans);
	free_matrix(accel_model);
	free_matrix(mag_model);
	return ret;
}

static matrix_t *observe_model_jacobian(matrix_t *state_pred, matrix_t *g_ref, matrix_t *m_ref) {
	/*
	 * H(^Q_t) = 2 * [U_g [U_g + ^Q_w * G]x + (^Q_v * G)I_3 - G * ^Q_v^T]
	 *               [U_r [U_r + ^Q_w * R]x + (^Q_v * R)I_3 - R * ^Q_v^T]
	 */

	matrix_t *state_pred_real = init_matrix(3, 1); // = ^Q_v = {^Q_x, ^Q_y, ^Q_z}
	state_pred_real->data[X] = state_pred->data[X];
	state_pred_real->data[Y] = state_pred->data[Y];
	state_pred_real->data[Z] = state_pred->data[Z];

	float state_pred_scalar = state_pred->data[W]; // = ^Q_w

	matrix_t *accel_ctr_vctr = mul_matrix_free(skew_symm_matrix(g_ref), state_pred_real); // = U_g
	matrix_t *mag_ctr_vctr = mul_matrix_free(skew_symm_matrix(m_ref), state_pred_real); // = U_r

	matrix_t *accel_model = observe_model_jacobian_helper(accel_ctr_vctr, g_ref, state_pred_real, state_pred_scalar);
	matrix_t *mag_model = observe_model_jacobian_helper(mag_ctr_vctr, m_ref, state_pred_real, state_pred_scalar);

	matrix_t *stack = stack_matrix(accel_model, mag_model);

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 4; j++) {
			int index = i * 4 + j;
			if (i < 3) {
				stack->data[index] = accel_model->data[index];
			}
			else {
				stack->data[index] = mag_model->data[(i - 3) * 4 + j];
			}
		}
	}

	free_matrix(state_pred_real);
	free_matrix(accel_ctr_vctr);
	free_matrix(mag_ctr_vctr);
	free_matrix(accel_model);
	free_matrix(mag_model);
	return scale_matrix_free(stack, 2);
	/*
	*/
	// return init_matrix(6, 4);
}

static matrix_t *observe_model_jacobian_helper(matrix_t *ctr_vtr, matrix_t *ref, matrix_t *real, float scalar) {
	matrix_t *ret = init_matrix(3, 4);

	matrix_t *ref_scale = scale_matrix_alloc(ref, scalar);
	matrix_t *ref_scale_p_ctr = add_matrix_free(ref_scale, ctr_vtr);

	// NOTE: If buggy, add_matrix may not be communative.
	matrix_t *skew_matrix = skew_symm_matrix(ref_scale_p_ctr); // = [ctr_vtr + scalar * ref]x

	float dot = dot_prod(real, ref);

	matrix_t *real_trans = trans_matrix_alloc(real);

	matrix_t *ref_x_real_t = mul_matrix_alloc(ref, real_trans);

	matrix_t *real_half = scale_matrix_free(ident_matrix(3), dot); // = (real * ref) * I_3 - ref * real^T
	real_half = sub_matrix_free(real_half, ref_x_real_t);

	matrix_t *model_left = add_matrix_free(skew_matrix, real_half);
	/*
	*/

	/* Build matrix with structure:
	 * ctr_vtr = u
	 * real_half = r
	 * u_1 r_11 r_12 r_13
	 * u_2 r_21 r_22 r_23
	 * u_3 r_31 r_32 r_33
	 */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			int index = i * 4 + j;
			if (j == 0) {
				ret->data[index] = ctr_vtr->data[i];
			}
			else {
				ret->data[index] = model_left->data[i * 4 + (j - 1)];
			}
		}
	}

	/*
	*/
	free_matrix(ref_scale_p_ctr);
	free_matrix(real_trans);
	free_matrix(ref_x_real_t);
	free_matrix(real_half);
	free_matrix(model_left);
	return ret;
}

static matrix_t *stack_matrix(const matrix_t *a, const matrix_t *b) {
	if (a->rows != b->rows || a->cols != b->cols) return NULL;

	matrix_t *ret = init_matrix(a->rows * 2, a->cols);

	uint8_t rows = a->rows * 2;
	uint8_t cols = a->cols;

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			int index = i * cols + j;
			if (i < a->rows) {
				ret->data[index] = a->data[index];
			}
			else {
				ret->data[index] = b->data[(i - a->rows) * cols + j];
			}
		}
	}

	return ret;
}

