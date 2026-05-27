#ifndef KINETIC_H
#define KINETIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "kin_math.h"
#include <stdbool.h>

typedef struct {
	double attitude_q[4];
	double attitude_e[3];

	double velocity[3];
	double velocity_total;

	double cords[2];
	double altitude;

	double gyro_noise[3];
	double accel_noise[3];
	double mag_noise[3];
	double mag_dip;

	matrix_t *state_q;
	matrix_t *estm_covariance;

	bool initialized;
} kinetic_t;

void init_state(kinetic_t *kinetic, double *accel, double *mag);

void update_imu(kinetic_t *kinetic, double *gyro, double *accel, double *mag, double dt);

void update_barometer(kinetic_t *kinetic, double altitude, double dt);

void update_gps(kinetic_t *kinetic, double cords[2], double altitude, double dt);

#ifdef __cplusplus
}
#endif

#endif
