#include <iostream>
#include <cmath>

#include "kin_math.h"
#include "kin_types.h"

using namespace std;


int main() {
	float a_arr[] = {1, 2, 3, 4};
	float b_arr[] = {5, 6, 7, 8};

	matrix_t *a = arr_to_matrix(a_arr, 4, 1);
	matrix_t *b = arr_to_matrix(b_arr, 4, 1);

	// cout << dot_prod(a, b) << endl;
	cout << sqrt(0) << endl;;
}

