#include <iostream>
#include <unistd.h>

#include "kin_math.h"
#include "kinetic.h"
#include "sim.h"

using namespace std;

int main() {
	cout << "hello world!" << endl;

	matrix_t *m = init_matrix(3, 3);
	double arr[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
	m->data = arr;
	print_matrix(m);

	kinetic_t kinetic;

	sim_t sim(&kinetic);

	sim.tick(update_imu);
	init_state(&kinetic, sim.accel, sim.mag);

	/*
	print_matrix(quat_to_euler(sim.orientation));
	cout << endl;
	print_matrix(quat_to_euler(kinetic.state_q));
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

