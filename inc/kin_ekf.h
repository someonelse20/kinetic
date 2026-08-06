#ifndef KIN_EKF_H
#define KIN_EKF_H

#include <cstdint>
#ifdef __cplusplus
extern "C" {
#endif

#include "kin_types.h"

uint8_t ekf_init(ekf_t *ekf, matrix_t *state, matrix_t *variance);

uint8_t ekf_update(ekf_t *ekf, ekf_input_t input);

#ifdef __cplusplus
}
#endif

#endif
