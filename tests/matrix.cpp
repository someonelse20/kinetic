#include <iostream>
#include <cmath>

#include "kin_math.h"
#include "kin_types.h"

using namespace std;


int main() {
	float a_arr[] = {0, deg_to_rad(57), 0};

	matrix_t *a = arr_to_matrix(a_arr, 3, 1);
	matrix_t *rot_mat = euler_to_rot_matrix(a);

	print_matrix(rot_mat);
	cout << endl;

	matrix_t *quat = euler_to_quat(a);
	matrix_t *quat_rot_mat = quat_to_rot_matrix(quat);
	print_matrix(quat_rot_mat);
}

