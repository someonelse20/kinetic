#ifndef KIN_EKF_H
#define KIN_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "kin_types.h"

/**
 * @brief Initialize the EKF state and covariance matrix.
 *
 * Copies the initial state and covariance matrix into the EKF structure.
 * The copies are retained even if the original arguments are freed, preventing
 * dangling pointer issues.
 *
 * @param ekf Pointer to the EKF structure to initialize (must be allocated)
 * @param state Initial state vector (state)
 * @param variance Initial covariance matrix
 * @return 0 on success, non-zero on error
 */
uint8_t ekf_init(ekf_t *ekf, matrix_t *state, matrix_t *variance);

/**
 * @brief Deinitialize the EKF and free allocated resources.
 *
 * Frees the state and covariance matrices stored in the EKF structure.
 * Safe to call multiple times (idempotent).
 *
 * @param ekf Pointer to the EKF structure to deinitialize
 * @return 0 on success, non-zero on error
 */
uint8_t ekf_deinit(ekf_t *ekf);

/**
 * @brief Perform the EKF update step using prediction + measurement correction.
 *
 * Implements the extended Kalman filter update cycle:
 * 1. Covariance prediction: P = F * P * F^T + Q
 * 2. Kalman gain: K = P * H^T * (H * P * H^T + R)^-1
 * 3. State update: X = X + K * (Z - H * X)
 * 4. Covariance update: P = (I - K * H) * P
 *
 * The state prediction (X^) and state transition Jacobian (F) are provided as
 * arguments since different EKF implementations compute them differently.
 *
 * @param ekf Pointer to the EKF structure (uses prev_state internally)
 * @param meas Measurement vector (Z_k)
 * @param state_pred Predicted state (X^)
 * @param state_pred_jacob State transition Jacobian (F_k)
 * @param obsv_model Observation model output (H * X)
 * @param obsv_model_jacob Observation model Jacobian (H_k)
 * @param proc_noise Process noise covariance (Q_k)
 * @param meas_noise Measurement noise covariance (R_k)
 * @return 0 on success, non-zero on error
 */
uint8_t ekf_update(ekf_t *ekf, matrix_t *meas, matrix_t *state_pred, matrix_t *state_pred_jacob,
                   matrix_t *obsv_model, matrix_t *obsv_model_jacob, matrix_t *proc_noise, matrix_t *meas_noise);

#ifdef __cplusplus
}
#endif

#endif
