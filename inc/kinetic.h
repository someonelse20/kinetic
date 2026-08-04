#ifndef KINETIC_H
#define KINETIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "kin_types.h"

void init_state(kinetic_t *kinetic, float *accel, float *mag);

void update_imu(kinetic_t *kinetic, float *gyro, float *accel, float *mag, float dt);

void update_barometer(kinetic_t *kinetic, float altitude, float dt);

void update_gps(kinetic_t *kinetic, float cords[2], float altitude, float dt);

#ifdef __cplusplus
}
#endif

#endif
