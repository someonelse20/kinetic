#ifndef KIN_TYPES_H
#define KIN_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
	uint8_t rows;
	uint8_t cols;
	float *data;
} matrix_t;

typedef struct {
	// function pointers
	matrix_t (*predict_state)();
	matrix_t (*predict_cov)();

	matrix_t (*correct_state)();
	matrix_t (*correct_cov)();

	// private variables
	matrix_t *state;
	matrix_t *variance;
} ekf_t;

typedef struct {
	float attitude_q[4];
	float attitude_e[3];

	float velocity[3];
	float velocity_total;

	float cords[2];
	float altitude;

	float gyro_noise;
	float accel_noise;
	float mag_noise;
	float mag_dip;

	matrix_t *state_q;

	matrix_t *g_ref;
	matrix_t *m_ref;
	matrix_t *estm_covariance;
} kinetic_t;

#ifdef __cplusplus
}
#endif

#endif
