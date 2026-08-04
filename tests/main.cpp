#include <iostream>
#include <unistd.h>
#include <math.h>

#include "kin_math.h"
#include "kinetic.h"
#include "sim.h"

using namespace std;

int main() {
	// cout << "hello world!" << endl;

	kinetic_t kinetic;
	// kinetic.mag_dip = 67 * (180 / M_PI);
	kinetic.mag_dip = 0.000001;
	kinetic.gyro_noise = 1.0 * (180 / M_PI);
	kinetic.accel_noise = 0.f;
	kinetic.mag_noise = 0.f;

	sim_t sim(&kinetic);

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

	sim.linear_interpolation(start_rot, end_rot, 5, 1, &Plot);

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

