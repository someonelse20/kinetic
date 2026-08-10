#include "kin_types.h"
#include "kin_math.h"

#include <stdio.h>

uint8_t ekf_init(ekf_t *ekf, matrix_t *state, matrix_t *covariance) {
	// NOTE: Copying may not be necessary for most uses but will cause errors if args are freed.
	ekf->state = copy_matrix(state);
	ekf->covariance = copy_matrix(covariance);

	return 0;
}

uint8_t ekf_update(ekf_t *ekf, matrix_t *meas, matrix_t *state_pred, matrix_t *state_pred_jacob, matrix_t *obsv_model, matrix_t *obsv_model_jacob, matrix_t *proc_noise, matrix_t *meas_noise) {
	// NOTE: It might not be necessary to copy these but this is just to ensure the values don't change.
	matrix_t *prev_state = copy_matrix(ekf->state); // X_k-1
	matrix_t *prev_cov = copy_matrix(ekf->covariance); // P_k-1

	/* ========================== State Prediction ========================= */
	// state_pred = ^X_k :: This isn't calculated here because different implementations calcualte it differently.

	/* ======================= Covariance Prediction ======================= */
	// state_pred_jacob = F_K :: Not calculated for same reason as ^X_k.

	matrix_t *cov_pred; // ^P_k = F_k * P_k-1 *F_k^T + Q_k (proc_noise)
	cov_pred = mul_matrix(state_pred_jacob, prev_cov); 
	cov_pred = mul_matrix(cov_pred, trans_matrix(state_pred_jacob));
	cov_pred = add_matrix(cov_pred, proc_noise);
	/*
	*/

	/* ============================ Kalman Gain ============================ */
	// obsv_model_jacob = H_k (not h_k) :: Not calculated for same reason as ^X_k.
	matrix_t *obsv_model_jacob_trans = trans_matrix(obsv_model_jacob); // H_k^T :: Variable used twice so no need to recompute.
	matrix_t *meas_pred_cov; // S_k = H_k * ^P_k * H_k^T + R_k (meas_noise) :: Measurement predicted covariance. Seperate variable to make computation simpler.
	meas_pred_cov = mul_matrix(obsv_model_jacob, cov_pred);
	meas_pred_cov = mul_matrix(meas_pred_cov, obsv_model_jacob_trans);
	meas_pred_cov = add_matrix(meas_pred_cov, meas_noise);

	matrix_t *kalman_gain; // K_k = ^P_k * H_k^T * S_k^-1
	kalman_gain = mul_matrix(cov_pred, obsv_model_jacob_trans);
	kalman_gain = mul_matrix(kalman_gain, inv_matrix(meas_pred_cov));
	/*
	*/

	/* ============================ State Update =========================== */
	matrix_t *meas_residual; // V_k = Z_k - h(^X-k) (not H_k) :: Again a seperate variable to make computation simpler.
	meas_residual = sub_matrix(meas, obsv_model);

	matrix_t *state = add_matrix(state_pred, mul_matrix(kalman_gain, meas_residual)); // X_k = ^X_k + K_k * (Z_k - h(^X-k)) (not H_k)
	/*
	*/

	/* ========================= Covariance Update ========================= */

	matrix_t *covariance; // P_k = (I - K_k * H_k) * ^P_k
	// TODO: Check if the dimentions of the identity matrix are the same of the kalman gain.
	covariance = sub_matrix(ident_matrix(kalman_gain->rows), mul_matrix(kalman_gain, obsv_model_jacob));
	covariance = mul_matrix(covariance, cov_pred);

	ekf->state = state;
	ekf->covariance = covariance;

	free_matrix(prev_state);
	free_matrix(prev_cov);
	free_matrix(cov_pred);
	free_matrix(obsv_model_jacob_trans);
	free_matrix(meas_pred_cov);
	free_matrix(kalman_gain);
	free_matrix(meas_residual);
	/*
	*/

	return 0;
}

