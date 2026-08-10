#ifndef KIN_EKF_H
#define KIN_EKF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "kin_types.h"

uint8_t ekf_init(ekf_t *ekf, matrix_t *state, matrix_t *variance);

uint8_t ekf_update(ekf_t *ekf, matrix_t *meas, matrix_t *state_pred, matrix_t *state_pred_jacob, matrix_t *obsv_model, matrix_t *obsv_model_jacob, matrix_t *proc_noise, matrix_t *meas_noise);

#ifdef __cplusplus
}
#endif

#endif
