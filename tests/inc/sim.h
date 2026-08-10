#ifndef SIM_H
#define SIM_H

#include "kin_types.h"
#include "plot.h"

typedef struct {
	matrix_t (*imu_update);
} test_t;

class sim_t {
	private:

	public:
		kinetic_t *kinetic;

		imu_t *imu;

		matrix_t *orientation;

		float *gyro;
		float *accel;
		float *mag;

		float sample_rate_hertz = 1;

		sim_t(imu_t *imu);

		void tick();

		void linear_interpolation(matrix_t *start_rot, matrix_t *end_rot, float duration, float timestep, plot_t *Plot = NULL);

		void linear_interpolation_comp(matrix_t *start_rot, matrix_t *end_rot, float duration, float timestep, plot_t *Plot = NULL);

		// TODO: Migrate kinetic_t over to new imu_t.
		// void loop(void (*update_imu)(kinetic_t*, float*, float*, float*, float));

		void print();
};

#endif
