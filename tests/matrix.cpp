#include <iostream>
#include <threads.h>

#include "kin_math.h"

using namespace std;

int main() {
	matrix_t *ref = init_matrix(3, 3);
	float arr[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
	ref->data = arr;

	// print_matrix(ref);
	// cout << endl;

	matrix_t *m = init_matrix(3, 3);
	float arr2[] = { 0.2042344, -0.5080490, -0.8367643,
		         0.7018977,  0.6718354, -0.2365943,
		         0.6823694, -0.5390023,  0.4938103};
	m->data = arr2;

	matrix_t *quat = rot_matrix_to_quat(m);

	matrix_t *two = init_matrix(2, 2);
	two->data[0] = 1; two->data[1] = 2;
	two->data[2] = 3; two->data[3] = 4;

	matrix_t *three = init_matrix(3, 3);
	three->data[0] =  4;
	three->data[1] =  3;
	three->data[2] =  8;
	three->data[3] =  6;
	three->data[4] =  2;
	three->data[5] =  5;
	three->data[6] =  1;
	three->data[7] =  5;
	three->data[8] =  9;

	matrix_t *ajt = ajt_matrix(three);
	matrix_t *inv = inv_matrix(three);



	float accel_noise = 1.1;
	float mag_noise = 2.2;

	float noise_covariance_data[36];         // 6x6 array

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			if (i != j) {
				noise_covariance_data[i * 6 + j] = 0;
				continue;
			}

			if (i < 3) {
				// noise_covariance_data[i * 6 + j] = accel_noise[i];
				noise_covariance_data[i * 6 + j] = accel_noise;
			} else {
				// noise_covariance_data[i * 6 + j] = mag_noise[i - 3];
				noise_covariance_data[i * 6 + j] = mag_noise;
			}
		}
	}

	print_matrix(arr_to_matrix(noise_covariance_data, 6, 6));

}

