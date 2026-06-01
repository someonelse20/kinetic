#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "kin_math.h"
#include "kinetic.h"
#include "sim.h"

using namespace std;

int main() {
	// cout << "hello world!" << endl;

	matrix_t *m = init_matrix(3, 3);
	double arr[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
	m->data = arr;

	kinetic_t kinetic;
	kinetic.mag_dip = 67 * (180 / M_PI);

	sim_t sim(&kinetic);

	sim.tick(update_imu);
	// init_state(&kinetic, sim.accel, sim.mag);

	double accel[] = {0.01, 0.51, 0.49};
	double mag[] = {1.01, 0.01, 0.01};
	init_state(&kinetic, accel, mag);

	printf("\n");
	print_matrix(sim.orientation);
	cout << endl;
	print_matrix(quat_to_euler(kinetic.state_q));

	/*
	cout << endl;
	print_matrix(m);
	*/

	/*
	while (1) {
		sim.tick(update_imu);

		// print_matrix(quat_to_euler(sim.orientation));
		cout << endl;
		sleep(1);
	}
	*/
}

