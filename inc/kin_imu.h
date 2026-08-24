#ifndef KIN_IMU_H
#define KIN_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "kin_types.h"

// Move these to source file when done debugging.
uint8_t calibrate_gyro_accel(matrix_t *value, matrix_t *alignment, matrix_t *sensitivity, matrix_t *bias);
uint8_t calibrate_mag(matrix_t *value, matrix_t *soft_iorn, matrix_t *hard_iorn);

matrix_t *imu_init(imu_t *imu, float *accel, float *mag);

uint8_t imu_deinit(imu_t *imu);

matrix_t *imu_update(imu_t *imu, float *gyro, float *accel, float *mag);

#ifdef __cplusplus
}
#endif

#endif
