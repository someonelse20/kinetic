#ifndef KIN_TYPES_H
#define KIN_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/types.h>
#include <stdint.h>

typedef struct {
	uint8_t rows;
	uint8_t cols;
	float *data;
} matrix_t;

typedef struct {
	matrix_t *state;
	matrix_t *covariance;
} ekf_t;

typedef struct {
	float gyro_noise;
	float accel_noise;
	float mag_noise;
	float mag_dip;
	float mag_dec;
	float dt;

	bool enu;

	ekf_t ekf;

	matrix_t *m_ref;
	matrix_t *g_ref;
	matrix_t *proc_noise;
	matrix_t *meas_noise;
} imu_t;

#ifdef __cplusplus
}
#endif

#endif
