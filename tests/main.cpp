#include <iostream>
#include <unistd.h>
#include <math.h>

#include "kin_math.h"
#include "kinetic.h"
#include "sim.h"

using namespace std;

int main() {
	// cout << "hello world!" << endl;

	matrix_t *m = init_matrix(3, 3);
	float arr[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
	m->data = arr;

	// print_matrix(m);

	kinetic_t kinetic;
	kinetic.mag_dip = 67 * (180 / M_PI);
	kinetic.gyro_noise = 0.0001;
	kinetic.accel_noise = 0.0001;
	kinetic.mag_noise = 0.0001;

	sim_t sim(&kinetic);

	sim.sample_rate_hertz = 1;

	sim.tick();
	init_state(sim.kinetic, sim.accel, sim.mag);

	while (1) {
		sim.tick();
		update_imu(&kinetic, sim.gyro, sim.accel, sim.mag, 1.0);

		print_matrix(quat_to_euler(sim.orientation));
		cout << endl;
		print_matrix(quat_to_euler(sim.kinetic->state_q));
		cout << endl;
		/*
		print_matrix(sim.kinetic->state_q);
		cout << endl;
		*/

		sleep(1);
	}
}

