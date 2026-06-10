#include <iostream>

#include "kin_math.h"

using namespace std;

int main() {
	matrix_t *ref = init_matrix(3, 3);
	float arr[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
	ref->data = arr;

	print_matrix(ref);

	cout << endl;

	matrix_t *m = init_matrix(3, 3);
	float arr2[] = { 0.2042344, -0.5080490, -0.8367643,
	                  0.7018977,  0.6718354, -0.2365943,
	                  0.6823694, -0.5390023,  0.4938103};
	m->data = arr2;

	matrix_t *quat = rot_matrix_to_quat(m);
	print_matrix(quat);
}

