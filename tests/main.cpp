#include <unistd.h>
#include <math.h>

#include "kin_imu.h"
#include "kin_math.h"
#include "kin_types.h"
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

	ahrs_alg_t ahrs_ekf;
	ahrs_ekf.imu_init = imu_init;
	ahrs_ekf.imu_update = imu_update;
	ahrs_ekf.name = "EKF";

	sim.num_of_algs = 1;
	sim.ahrs_algs[0] = ahrs_ekf;

	sim.linear_interpolation(start_rot, end_rot, 1, 0.1, &Plot);
	// sim.linear_interpolation(start_rot, end_rot, 5, 1, &Plot);

	/*
	imu_t comp_imu;
	sim_t comp_sim(&comp_imu);

	plot_t comp_Plot;

	comp_sim.linear_interpolation_comp(start_rot, end_rot, 5, 1, &comp_Plot);
	*/

	/*
	sim.tick();
	init_state(sim.kinetic, sim.accel, sim.mag);

	for (int i = 0; i < 20; i++) {
		sim.tick();
		update_imu(&kinetic, sim.gyro, sim.accel, sim.mag, 1.0);

		cout << "============================================" << endl;
		cout << "Simulation orientation" << endl;
		cout << "============================================" << endl;
		print_matrix(quat_to_euler(sim.orientation));
		cout << endl;
		cout << "============================================" << endl;
		cout << "Kinetic orientation" << endl;
		cout << "============================================" << endl;
		print_matrix(quat_to_euler(sim.kinetic->state_q));
		cout << endl;

		// break;

		sleep(1);
	}
	*/
}

