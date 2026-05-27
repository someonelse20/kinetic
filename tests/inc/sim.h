#ifndef SIM_H
#define SIM_H

#include "kin_math.h"
#include "kinetic.h"

class sim_t {
	private:
		kinetic_t *kinetic;

	public:
		matrix_t *orientation;

		double *gyro;
		double *accel;
		double *mag;

		double sample_rate_hertz = 1;

		sim_t(kinetic_t *kinetic);

		void tick(void (*update_imu)(kinetic_t*, double*, double*, double*, double));

		void loop(void (*update_imu)(kinetic_t*, double*, double*, double*, double));
};

#endif
