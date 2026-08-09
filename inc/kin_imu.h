#ifndef KIN_IMU_H
#define KIN_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "kin_types.h"

matrix_t *imu_init(imu_t *imu, float *accel, float *mag);

matrix_t *imu_update(imu_t *imu, float *gyro, float *accel, float *mag);

#ifdef __cplusplus
}
#endif

#endif
