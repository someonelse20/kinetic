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
	kinetic.mag_dip = 67 * (180 / M_PI);
	kinetic.gyro_noise = 1.0 * (180 / M_PI);
	kinetic.accel_noise = 0.0;
	kinetic.mag_noise = 0.0;

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

		// break;

		sleep(1);
	}
}

