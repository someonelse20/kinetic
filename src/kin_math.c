/*
 * Copyright (c) 2024 Rubik Proxy. All rights reserved.
 * https://github.com/rubikproxy/matrix.h
 *
 * This software is provided for educational purposes and inspired by
 * the matrix library from rubikproxy.
 */

#include "kin_math.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

matrix_t *init_matrix(size_t rows, size_t cols) {
	matrix_t *matrix = (matrix_t *)malloc(sizeof(matrix_t));
	matrix->rows = rows;
	matrix->cols = cols;
	matrix->data = (double *)malloc(rows * cols * sizeof(double));
	return matrix;
}

void free_matrix(matrix_t *matrix) {
	free(matrix->data);
	free(matrix);
}

matrix_t *arr_to_matrix(double *arr, size_t size, bool vert) {
	matrix_t *matrix;
	if (vert) {
		matrix = init_matrix(size, 1);
	} else {
		matrix = init_matrix(1, size);
	}
	matrix->data = arr;
	return matrix;
}

void print_matrix(const matrix_t *matrix) {
	for (size_t i = 0; i < matrix->rows; i++) {
		for (size_t j = 0; j < matrix->cols; j++) {
			printf("%f ", matrix->data[i * matrix->cols + j]);
		}
		printf("\n");
	}
}

matrix_t *add_matrix(const matrix_t *a, const matrix_t *b) {
	if (a->rows != b->rows || a->cols != b->cols) return NULL;
	matrix_t *result = init_matrix(a->rows, a->cols);
	for (size_t i = 0; i < a->rows * a->cols; i++) {
		result->data[i] = a->data[i] + b->data[i];
	}
	return result;
}

matrix_t *sub_matrix(const matrix_t *a, const matrix_t *b) {
	if (a->rows != b->rows || a->cols != b->cols) return NULL;
	matrix_t *result = init_matrix(a->rows, a->cols);
	for (size_t i = 0; i < a->rows * a->cols; i++) {
		result->data[i] = a->data[i] - b->data[i];
	}
	return result;
}

matrix_t *mul_matrix(const matrix_t *a, const matrix_t *b) {
	if (a->cols != b->rows) return NULL;
	matrix_t *result = init_matrix(a->rows, b->cols);
	for (size_t i = 0; i < a->rows; i++) {
		for (size_t j = 0; j < b->cols; j++) {
			result->data[i * b->cols + j] = 0;
			for (size_t k = 0; k < a->cols; k++) {
				result->data[i * b->cols + j] += a->data[i * a->cols + k] * b->data[k * b->cols + j];
			}
		}
	}
	return result;
}

matrix_t *scale_matrix(matrix_t *matrix, double scalar) {
	matrix_t *result = init_matrix(matrix->rows, matrix->cols);
	for (size_t i = 0; i < matrix->rows * matrix->cols; i++) {
		result->data[i] = scalar * matrix->data[i];
	}
	return result;
}

matrix_t *trans_matrix(const matrix_t *matrix) {
	matrix_t *result = init_matrix(matrix->cols, matrix->rows);
	for (size_t i = 0; i < matrix->rows; i++) {
		for (size_t j = 0; j < matrix->cols; j++) {
			result->data[j * matrix->rows + i] = matrix->data[i * matrix->cols + j];
		}
	}
	return result;
}

double matrix_determinant(const matrix_t *matrix) {
	if (matrix->rows != matrix->cols) return 0;
	return 0;
}

matrix_t *ident_matrix(size_t size) {
	matrix_t *matrix = init_matrix(size, size);
	for (size_t i = 0; i < size; i++) {
		for (size_t j = 0; j < size; j++) {
			matrix->data[i * size + j] = (i == j) ? 1.0 : 0.0;
		}
	}
	return matrix;
}

matrix_t *inv_matrix(matrix_t *matrix) {
	matrix_t *ret = init_matrix(matrix->rows, matrix->cols);
	for (size_t i = 0; i < matrix->rows * matrix->cols; i++) {
		ret->data[i] = 1 / matrix->data[i];
	}
	return ret;
}

double matrix_norm(const matrix_t *matrix) {
	double norm = 0;
	for (size_t i = 0; i < matrix->rows * matrix->cols; i++) {
		norm += matrix->data[i] * matrix->data[i];
	}
	return sqrt(norm);
}

matrix_t *normalize_matrix(matrix_t *matrix) {
	return scale_matrix(matrix, 1 / matrix_norm(matrix));
}

eigen_t *matrix_eigen(const matrix_t *matrix_t) {

}

matrix_t *euler_to_quat(matrix_t *matrix) {
	matrix_t *ret = init_matrix(4, 1);

	double u = matrix->data[X] / 2;
	double v = matrix->data[Y] / 2;
	double w = matrix->data[Z] / 2;

	ret->data[W] = cos(u) * cos(v) * cos(w) + sin(u) * sin(v) * sin(w);
	ret->data[X] = sin(u) * cos(v) * cos(w) - cos(u) * sin(v) * sin(w);
	ret->data[Y] = cos(u) * sin(v) * cos(w) + sin(u) * cos(v) * sin(w);
	ret->data[Z] = cos(u) * cos(v) * sin(w) - sin(u) * sin(v) * cos(w);

	return ret;
}

matrix_t *quat_to_euler(matrix_t *matrix) {
	matrix_t *ret = init_matrix(3, 1);
	// matrix_t *normed = normalize_matrix(matrix);
	matrix_t *normed = matrix;

	double w2 = pow(normed->data[W], 2);
	double x2 = pow(normed->data[X], 2);
	double y2 = pow(normed->data[Y], 2);
	double z2 = pow(normed->data[Z], 2);

	ret->data[X] = atan2(
		2 * (normed->data[W] * normed->data[X] + normed->data[Y] * normed->data[Z]), 
		1 - 2 * (normed->data[X] * normed->data[X] + normed->data[Y] * normed->data[Y])
	);
	ret->data[Y] = asin(2 * (normed->data[W] * normed->data[Y] - normed->data[Z] * normed->data[X]));
	ret->data[Z] = atan2(
		2 * (normed->data[W] * normed->data[Z] + normed->data[X] * normed->data[Y]), 
		1 - 2 * (normed->data[Y] * normed->data[Y] + normed->data[Z] * normed->data[Z])
	);

	for (int i = 0; i < 3; i++) {
		ret->data[i] = rad_to_deg(ret->data[i]);
	}

	return ret;
}

matrix_t *quat_to_rot_matrix(matrix_t *matrix) { // NOTE: There seems to be multiple ways to do this
	if (matrix->rows != 4 || matrix->cols != 1) return 0;
	matrix_t *result = init_matrix(3, 3);
	double *data = matrix->data;
	double result_d[] = { // 3x3 matrix
		pow(data[W], 2) + pow(data[X], 2) - pow(data[Y], 2) - pow(data[Z], 2),
		2 * (data[X] * data[Y] - data[W] * data[Z]),
		2 * (data[X] * data[Z] + data[W] * data[Y]),

		2 * (data[X] * data[Y] + data[W] * data[Z]),
		pow(data[W], 2) - pow(data[X], 2) + pow(data[Y], 2) - pow(data[Z], 2),
		2 * (data[Y] * data[Z] - data[W] * data[X]),

		2 * (data[X] * data[Z] - data[W] * data[Y]),
		2 * (data[W] * data[X] + data[Y] * data[Z]),
		pow(data[W], 2) - pow(data[X], 2) - pow(data[Y], 2) + pow(data[Z], 2),
	};
	result->data = result_d;
	return  result;
}

matrix_t *rot_matrix_to_quat(matrix_t *matrix) {
	if (matrix->rows != 3 || matrix->cols != 3) return 0;
	matrix_t *K = init_matrix(4, 4);

	K->data[0]  = matrix->data[0] - matrix->data[4] - matrix->data[8];
	K->data[1]  = matrix->data[3] - matrix->data[1];
	K->data[2]  = matrix->data[6] + matrix->data[2];
	K->data[3]  = matrix->data[5] + matrix->data[7];

	K->data[4]  = matrix->data[3] + matrix->data[1];
	K->data[5]  = matrix->data[4] + matrix->data[0] + matrix->data[8];
	K->data[6]  = matrix->data[7] + matrix->data[5];
	K->data[7]  = matrix->data[6] + matrix->data[2];

	K->data[8]  = matrix->data[6] + matrix->data[2];
	K->data[9]  = matrix->data[7] + matrix->data[5];
	K->data[10] = matrix->data[8] + matrix->data[0] + matrix->data[4];
	K->data[11] = matrix->data[1] + matrix->data[3];

	K->data[12] = matrix->data[5] + matrix->data[7];
	K->data[13] = matrix->data[6] + matrix->data[2];
	K->data[14] = matrix->data[1] + matrix->data[3];
	K->data[15] = matrix->data[0] + matrix->data[4] + matrix->data[8];

	K = scale_matrix(K, 0.333333);

	return K;
}

matrix_t *mul_quat(const matrix_t *a, const matrix_t *b) {
	matrix_t *ret = init_matrix(4, 1);

	ret->data[X] = a->data[W] * b->data[X] + a->data[X] * b->data[W] + a->data[Y] * b->data[Z] - a->data[Z] * b->data[Y];
	ret->data[Y] = a->data[W] * b->data[Y] - a->data[X] * b->data[Z] + a->data[Y] * b->data[W] + a->data[Z] * b->data[X];
	ret->data[Z] = a->data[W] * b->data[Z] + a->data[X] * b->data[Y] - a->data[Y] * b->data[X] + a->data[Z] * b->data[W];
	ret->data[W] = a->data[W] * b->data[W] - a->data[X] * b->data[X] - a->data[Y] * b->data[Y] - a->data[Z] * b->data[Z];

	return  ret;
}

matrix_t *mul_vector(const matrix_t *a, const matrix_t *b) {
	matrix_t *ret = init_matrix(3, 1);

	ret->data[X] = a->data[Y] * b->data[Z] - a->data[Z] * b->data[Y];
	ret->data[Y] = a->data[Z] * b->data[X] - a->data[X] * b->data[Z];
	ret->data[Z] = a->data[X] * b->data[Y] - a->data[Y] * b->data[X];

	return ret;
}

double vector_dot(const matrix_t *a, const matrix_t *b) {
	double ret = 0;
	for (int i = 0; i < 3; i++) {
		ret += a->data[i] * b->data[i];
	}
	return ret;
}

int sgn(double x) {
	if (x == 0) {
		return 0;
	} else if (x > 0) {
		return 1;
	} else {
		return -1;
	}
}

double rad_to_deg(double rad) {
	return rad * 180 / M_PI;
}

double deg_to_rad(double deg) {
	return deg * M_PI / 180;
}

