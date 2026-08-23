#include "kin_types.h"
#include "kin_math.h"

#include <stdio.h>

/*
static matrix_t cov_pred;
static matrix_t state_pred_jacob_trans;
static matrix_t obsv_model_jacob_trans;
static matrix_t meas_pred_cov;
static matrix_t meas_pred_cov_inv;
static matrix_t kalman_gain;
static matrix_t meas_residual;
static matrix_t kalman_x_obsv;
*/

uint8_t ekf_init(ekf_t *ekf, matrix_t *state, matrix_t *covariance) {
	// NOTE: Copying may not be necessary for most uses but will cause errors if args are freed.
	ekf->state = copy_matrix(state);
	ekf->covariance = copy_matrix(covariance);

	return 0;
}

uint8_t ekf_deinit(ekf_t *ekf) {
	free_matrix(ekf->state);
	free_matrix(ekf->covariance);

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
	matrix_t *state_pred_jacob_trans = trans_matrix_alloc(state_pred_jacob);
	cov_pred = mul_matrix_alloc(state_pred_jacob, prev_cov); 
	cov_pred = mul_matrix_free(cov_pred, state_pred_jacob_trans);
	cov_pred = add_matrix_free(cov_pred, proc_noise);
	/*
	*/

	/* ============================ Kalman Gain ============================ */
	// obsv_model_jacob = H_k (not h_k) :: Not calculated for same reason as ^X_k.
	matrix_t *obsv_model_jacob_trans = trans_matrix_alloc(obsv_model_jacob); // H_k^T :: Variable used twice so no need to recompute.
	matrix_t *meas_pred_cov; // S_k = H_k * ^P_k * H_k^T + R_k (meas_noise) :: Measurement predicted covariance. Seperate variable to make computation simpler.
	meas_pred_cov = mul_matrix_alloc(obsv_model_jacob, cov_pred);
	meas_pred_cov = mul_matrix_free(meas_pred_cov, obsv_model_jacob_trans);
	meas_pred_cov = add_matrix_free(meas_pred_cov, meas_noise);

	matrix_t *kalman_gain; // K_k = ^P_k * H_k^T * S_k^-1
	matrix_t *meas_pred_cov_inv = inv_matrix(meas_pred_cov);
	kalman_gain = mul_matrix_alloc(cov_pred, obsv_model_jacob_trans);
	kalman_gain = mul_matrix_free(kalman_gain, meas_pred_cov_inv);
	/*
	*/

	/* ============================ State Update =========================== */
	matrix_t *meas_residual; // V_k = Z_k - h(^X-k) (not H_k) :: Again a seperate variable to make computation simpler.
	meas_residual = sub_matrix_alloc(meas, obsv_model);

	matrix_t *kalman_x_resid = mul_matrix_alloc(kalman_gain, meas_residual);

	// add_matrix(state_pred, kalman_x_resid, ekf->state); // X_k = ^X_k + K_k * (Z_k - h(^X-k)) (not H_k)
	ekf->state = add_matrix_alloc(state_pred, kalman_x_resid); // X_k = ^X_k + K_k * (Z_k - h(^X-k)) (not H_k)
	/*
	*/

	/* ========================= Covariance Update ========================= */

	matrix_t *covariance; // P_k = (I - K_k * H_k) * ^P_k
	// TODO: Check if the dimentions of the identity matrix are the same of the kalman gain.
	matrix_t *kalman_x_obsv = mul_matrix_alloc(kalman_gain, obsv_model_jacob);
	covariance = sub_matrix_free(ident_matrix(kalman_gain->rows), kalman_x_obsv);
	covariance = mul_matrix_free(covariance, cov_pred);

	move_matrix(covariance, ekf->covariance);
	free_matrix(covariance);

	free_matrix(prev_state);
	free_matrix(prev_cov);
	free_matrix(cov_pred);
	free_matrix(state_pred_jacob_trans);
	free_matrix(obsv_model_jacob_trans);
	free_matrix(meas_pred_cov);
	free_matrix(meas_pred_cov_inv);
	free_matrix(kalman_gain);
	free_matrix(meas_residual);
	free_matrix(kalman_x_resid);
	free_matrix(kalman_x_obsv);
	/*
	*/

	return 0;
}

