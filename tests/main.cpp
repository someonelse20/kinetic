#include <unistd.h>
#include <math.h>

#include "reference_ahrs.h"
#include "kin_math.h"
#include "kin_imu.h"
#include "plot.h"
#include "sim.h"

using namespace std;

int main() {
	// cout << "hello world!" << endl;

	imu_t imu;
	// kinetic.mag_dip = 67 * (180 / M_PI);
	imu.mag_dip = 0.000001;
	/*
	imu.gyro_noise = 0.3;
	imu.accel_noise = 0.5;
	imu.mag_noise = 0.8;
	*/
	imu.gyro_noise = 0.f;
	imu.accel_noise = 0.f;
	imu.mag_noise = 0.f;
	/*
	*/
	/*
	imu.gyro_noise = 1.0 * (180 / M_PI);
	imu.accel_noise = 0.f;
	imu.mag_noise = 0.f;
	*/

	sim_t sim(&imu);

	matrix_t *start_rot = init_matrix(4, 1);
	start_rot->data[X] = 0.0;
	start_rot->data[Y] = 0.0;
	start_rot->data[Z] = 0.0;
	start_rot->data[W] = 1.0;

	matrix_t *end_rot = init_matrix(4, 1);
	end_rot->data[X] = 0.7071068;
	end_rot->data[Y] = 0.0;
	end_rot->data[Z] = 0.0;
	end_rot->data[W] = 0.7071068;

	plot_t Plot;

	sim.add_ahrs(imu_init, imu_update, "EKF");
	sim.add_ahrs(gyro_imu_init, gyro_imu_update, "gyro-only");
	sim.add_ahrs(comp_imu_init, comp_imu_update, "comp-filter");

	sim.linear_interpolation(start_rot, end_rot, 1, 0.1, &Plot);
}

