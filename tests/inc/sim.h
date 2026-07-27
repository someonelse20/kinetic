#ifndef SIM_H
#define SIM_H

#include "kin_math.h"
#include "kinetic.h"

class sim_t {
	private:

	public:
		kinetic_t *kinetic;

		matrix_t *orientation;

		float *gyro;
		float *accel;
		float *mag;

		float sample_rate_hertz = 1;

		sim_t(kinetic_t *kinetic);

		void tick();

		void linear_interpolation(matrix_t *start_rot, matrix_t *end_rot, float duration, float timestep);

		void loop(void (*update_imu)(kinetic_t*, float*, float*, float*, float));

		void print();
};

#endif
