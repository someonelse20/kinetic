#include <iostream>

#include "kin_math.h"
#include "kin_types.h"

using namespace std;

static matrix_t *stack_matrix(const matrix_t *a, const matrix_t *b) {
	if (a->rows != b->rows || a->cols != b->cols) return NULL;

	matrix_t *ret = init_matrix(a->rows * 2, a->cols);

	uint8_t rows = a->rows * 2;
	uint8_t cols = a->cols;

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			int index = i * cols + j;
			if (i < a->rows) {
				ret->data[index] = a->data[index];
			}
			else {
				ret->data[index] = b->data[(i - a->rows) * cols + j];
			}
		}
	}

	return ret;
}

int main() {
	float a_arr[] = {1, 2, 3, 4};
	float b_arr[] = {5, 6, 7, 8};

	matrix_t *a = arr_to_matrix(a_arr, 2, 2);
	matrix_t *b = arr_to_matrix(b_arr, 2, 2);

	print_matrix(stack_matrix(a, b));
}

