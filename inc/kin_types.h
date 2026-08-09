#ifndef KIN_TYPES_H
#define KIN_TYPES_H

#include <sys/types.h>
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
	matrix_t *state_pred;
	matrix_t *observe_model;
	matrix_t *observe_model_jacob;
	matrix_t *state_trans_model;
	matrix_t *state_trans_model_jacob;
	matrix_t *proc_noise;
	matrix_t *meas_noise;
	matrix_t *meas;
} ekf_input_t;

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

	ekf_t *ekf;

	matrix_t *m_ref;
	matrix_t *g_ref;
	matrix_t *proc_noise;
	matrix_t *meas_noise;
} imu_t;

// TODO: Remove if not used.
typedef struct {
	matrix_t *data;
	matrix_t *noise;
	ulong timestamp;
} sensor_t;

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
