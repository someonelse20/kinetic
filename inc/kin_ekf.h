#ifndef KIN_EKF_H
#define KIN_EKF_H

#include <cstdint>
#ifdef __cplusplus
extern "C" {
#endif

#include "kin_types.h"

uint8_t ekf_init(ekf_t *ekf, matrix_t *init_state);

uint8_t ekf_update(ekf_t *ekf, matrix_t *meas, matrix_t *proc_noise, matrix_t *meas_noise);

#ifdef __cplusplus
}
#endif

#endif
