#include <iostream>
#include <cmath>

#include "kin_math.h"
#include "kin_types.h"

using namespace std;


int main() {
	float a_arr[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

	matrix_t *a = arr_to_matrix(a_arr, 3, 3);

	matrix_t *ajt = ajt_matrix(a);

	print_matrix(a);
	cout << endl;
	print_matrix(ajt);

	free_matrix(a);
	free_matrix(ajt);
}

