#ifndef KINETIC_H
#define KINETIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "kin_math.h"
#include <stdbool.h>

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

void init_state(kinetic_t *kinetic, float *accel, float *mag);

void update_imu(kinetic_t *kinetic, float *gyro, float *accel, float *mag, float dt);

void update_barometer(kinetic_t *kinetic, float altitude, float dt);

void update_gps(kinetic_t *kinetic, float cords[2], float altitude, float dt);

#ifdef __cplusplus
}
#endif

#endif
