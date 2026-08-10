#ifndef SIM_H
#define SIM_H

#include <string>

#include "kin_types.h"
#include "plot.h"

typedef struct ahrs_alg_t {
	matrix_t *(*imu_init)(imu_t *imu, float *accel, float *mag);
	matrix_t *(*imu_update)(imu_t *imu, float *gyro, float *accel, float *mag);
	std::string name = "EKF";
} ahrs_alg_t;

class sim_t {
	private:

	public:
		int num_of_algs = 1;
		ahrs_alg_t ahrs_algs[3];

		imu_t *imu;

		matrix_t *orientation;

		float *gyro;
		float *accel;
		float *mag;

		float sample_rate_hertz = 1;

		sim_t(imu_t *imu);

		void tick();

		void linear_interpolation(matrix_t *start_rot, matrix_t *end_rot, float duration, float timestep, plot_t *Plot = NULL);

		// TODO: Migrate kinetic_t over to new imu_t.
		// void loop(void (*update_imu)(kinetic_t*, float*, float*, float*, float));

		void print();
};

#endif
