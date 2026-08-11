#ifndef COMP_FILTER_H
#define COMP_FILTER_H

#include "kin_types.h"

matrix_t *gyro_imu_init(imu_t *imu, float *accel, float *mag);
matrix_t *gyro_imu_update(imu_t *imu, float *gyro, float *accel, float *mag);

matrix_t *comp_imu_init(imu_t *imu, float *accel, float *mag);
matrix_t *comp_imu_update(imu_t *imu, float *gyro, float *accel, float *mag);

#endif
