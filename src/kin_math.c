/*
 * Copyright (c) 2024 Rubik Proxy. All rights reserved.
 * https://github.com/rubikproxy/matrix.h
 *
 * This software is provided for educational purposes and inspired by
 * the matrix library from rubikproxy.
 */

#include "kin_math.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

matrix_t *copy_matrix(matrix_t *matrix) {
	matrix_t *ret = (matrix_t *)malloc(sizeof(matrix_t));
	ret->rows = matrix->rows;
	ret->cols = matrix->cols;
	ret->data = (float *)malloc(ret->rows * ret->cols * sizeof(float));
	ret->data = (float *)memcpy(ret->data, matrix->data, ret->rows * ret->cols * sizeof(float));
	return ret;
}

matrix_t *init_matrix(uint8_t rows, uint8_t cols) {
	matrix_t *matrix = (matrix_t *)malloc(sizeof(matrix_t));
	matrix->rows = rows;
	matrix->cols = cols;
	matrix->data = (float *)malloc(rows * cols * sizeof(float));
	return matrix;
}

void free_matrix(matrix_t *matrix) {
	free(matrix->data);
	free(matrix);
}

matrix_t *arr_to_matrix(float *arr, uint8_t rows, uint8_t cols) {
	matrix_t *matrix = init_matrix(rows, cols);

	for (uint8_t i = 0; i < rows * cols; i++) {
		matrix->data[i] = arr[i];
	}

	return matrix;
}

void print_matrix(const matrix_t *matrix) {
	for (uint8_t i = 0; i < matrix->rows; i++) {
		for (uint8_t j = 0; j < matrix->cols; j++) {
			printf("%f ", matrix->data[i * matrix->cols + j]);
		}
		printf("\n");
	}
}

void print_arr(const float *arr, uint8_t size) {
	for (uint8_t i = 0; i < size; i++) {
		printf("%f\n", arr[i]);
	}
}

matrix_t *add_matrix(const matrix_t *a, const matrix_t *b) {
	if (a->rows != b->rows || a->cols != b->cols) return NULL;
	matrix_t *result = init_matrix(a->rows, a->cols);
	for (uint8_t i = 0; i < a->rows * a->cols; i++) {
		result->data[i] = a->data[i] + b->data[i];
	}
	return result;
}

matrix_t *sub_matrix(const matrix_t *a, const matrix_t *b) {
	if (a->rows != b->rows || a->cols != b->cols) return NULL;
	matrix_t *result = init_matrix(a->rows, a->cols);
	for (uint8_t i = 0; i < a->rows * a->cols; i++) {
		result->data[i] = a->data[i] - b->data[i];
	}
	return result;
}

matrix_t *mul_matrix(const matrix_t *a, const matrix_t *b) {
	if (a->cols != b->rows) return NULL;
	matrix_t *result = init_matrix(a->rows, b->cols);
	for (uint8_t i = 0; i < a->rows; i++) {
		for (uint8_t j = 0; j < b->cols; j++) {
			result->data[i * b->cols + j] = 0;
			for (uint8_t k = 0; k < a->cols; k++) {
				result->data[i * b->cols + j] += a->data[i * a->cols + k] * b->data[k * b->cols + j];
			}
		}
	}
	return result;
}

matrix_t *scale_matrix(matrix_t *matrix, float scalar) {
	matrix_t *result = init_matrix(matrix->rows, matrix->cols);
	for (uint8_t i = 0; i < matrix->rows * matrix->cols; i++) {
		result->data[i] = scalar * matrix->data[i];
	}
	return result;
}

matrix_t *trans_matrix(const matrix_t *matrix) {
	matrix_t *result = init_matrix(matrix->cols, matrix->rows);
	for (uint8_t i = 0; i < matrix->rows; i++) {
		for (uint8_t j = 0; j < matrix->cols; j++) {
			result->data[j * matrix->rows + i] = matrix->data[i * matrix->cols + j];
		}
	}
	return result;
}

matrix_t *ident_matrix(uint8_t size) {
	matrix_t *matrix = init_matrix(size, size);
	for (uint8_t i = 0; i < size; i++) {
		for (uint8_t j = 0; j < size; j++) {
			matrix->data[i * size + j] = (i == j) ? 1.0 : 0.0;
		}
	}
	return matrix;
}

float matrix_det(const matrix_t *matrix) { // This function is ai generated with some modification.
	if (matrix->rows != matrix->cols) return -1; // TODO: find better error code

	uint8_t size = matrix->rows;

	if (size == 1)
		return matrix->data[0];

	if (size == 2)
		return matrix->data[0] * matrix->data[3] - matrix->data[1] * matrix->data[2];

	float ret = 0;
	for (uint8_t j = 0; j < size; j++) {
		matrix_t *minor = init_matrix(size - 1, size - 1);
		uint8_t mi = 0;
		for (uint8_t i = 1; i < size; i++) {
			uint8_t mj = 0;
			for (uint8_t k = 0; k < size; k++) {
				if (k == j) continue;
				minor->data[mi * (size - 1) + mj] = matrix->data[i * size + k];
				mj++;
			}
			mi++;
		}

		float sign = (j % 2) ? -1 : 1;
		ret += sign * matrix->data[j] * matrix_det(minor);
		free_matrix(minor);
	}

	return ret;
}

float matrix_norm(const matrix_t *matrix) {
	float norm = 0;
	for (uint8_t i = 0; i < matrix->rows * matrix->cols; i++) {
		norm += matrix->data[i] * matrix->data[i];
	}
	return sqrt(norm);
}

float matrix_minor(const matrix_t *matrix, uint8_t row, uint8_t col) {
	if (matrix->cols != matrix->rows) return -1;
	if (matrix->cols < 2) return -1;
	uint8_t size = matrix->rows;

	uint8_t index = 0;
	matrix_t *minor_m = init_matrix(size - 1, size - 1);
	for (uint8_t i = 0; i < size; i++) {
		for (uint8_t j = 0; j < size; j++) {
			if (i == row || j == col)
				continue;

			minor_m->data[index] = matrix->data[i * size + j];
			index++;
		}
	}

	return matrix_det(minor_m);
}

matrix_t *inv_matrix(matrix_t *matrix) {
	if (matrix->cols != matrix->rows) return NULL;
	if (matrix->cols < 2) return NULL;

	float det = matrix_det(matrix);
	if (det == 0) return NULL; // TODO also error out if very very close to zero

	return scale_matrix(ajt_matrix(matrix), 1.0 / det);
}

matrix_t *ajt_matrix(matrix_t *matrix) {
	if (matrix->cols != matrix->rows) return NULL;
	if (matrix->cols < 2) return NULL;

	matrix_t *ret = init_matrix(matrix->rows, matrix->cols);
	uint8_t size = matrix->rows;

	for (uint8_t i = 0; i < size; i++) {
		for (uint8_t j = 0; j < size; j++) {
			ret->data[i * size + j] = pow(-1.0, i + j + 2) * matrix_minor(matrix, i, j);
			// ret->data[i * size + j] = matrix_minor(matrix, i, j);
		}
	}

	return trans_matrix(ret);
}

matrix_t *normalize_matrix(matrix_t *matrix) {
	return scale_matrix(matrix, 1 / matrix_norm(matrix));
}

eigen_t *matrix_eigen(const matrix_t *matrix_t) {

}

matrix_t *euler_to_quat(matrix_t *matrix) {
	matrix_t *ret = init_matrix(4, 1);

	float u = matrix->data[X] / 2;
	float v = matrix->data[Y] / 2;
	float w = matrix->data[Z] / 2;

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

	float w2 = pow(normed->data[W], 2);
	float x2 = pow(normed->data[X], 2);
	float y2 = pow(normed->data[Y], 2);
	float z2 = pow(normed->data[Z], 2);

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
	if (matrix->rows != 4 || matrix->cols != 1) return NULL;
	matrix_t *result = init_matrix(3, 3);
	float *data = matrix->data;

	result->data[0] = pow(data[W], 2) + pow(data[X], 2) - pow(data[Y], 2) - pow(data[Z], 2);
	result->data[1] = 2 * (data[X] * data[Y] - data[W] * data[Z]);
	result->data[2] = 2 * (data[X] * data[Z] + data[W] * data[Y]);

	result->data[3] = 2 * (data[X] * data[Y] + data[W] * data[Z]);
	result->data[4] = pow(data[W], 2) - pow(data[X], 2) + pow(data[Y], 2) - pow(data[Z], 2);
	result->data[5] = 2 * (data[Y] * data[Z] - data[W] * data[X]);

	result->data[6] = 2 * (data[X] * data[Z] - data[W] * data[Y]);
	result->data[7] = 2 * (data[W] * data[X] + data[Y] * data[Z]);
	result->data[8] = pow(data[W], 2) - pow(data[X], 2) - pow(data[Y], 2) + pow(data[Z], 2);

	return result;
}

matrix_t *rot_matrix_to_quat(matrix_t *matrix) { // this function is from https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	if (matrix->rows != 3 || matrix->cols != 3) return NULL;
	matrix_t *ret = init_matrix(4, 1);

	float tr = matrix->data[0] + matrix->data[4] + matrix->data[8];

	if (tr > 0) {
		float S = sqrt(1 + tr) * 2;
		ret->data[W] = 0.25 * S;
		ret->data[X] = (matrix->data[7] - matrix->data[5]) / S;
		ret->data[Y] = (matrix->data[2] - matrix->data[6]) / S;
		ret->data[Z] = (matrix->data[3] - matrix->data[1]) / S;
	} else if ((matrix->data[0] > matrix->data[4])&(matrix->data[0] > matrix->data[8])) { // if the first diagonal is the largest
		float S = sqrt(1 + matrix->data[0] - matrix->data[4] - matrix->data[8]) * 2;
		ret->data[W] = (matrix->data[7] - matrix->data[5]) / S;
		ret->data[X] = 0.25 * S;
		ret->data[Y] = (matrix->data[1] + matrix->data[3]) / S;
		ret->data[Z] = (matrix->data[2] + matrix->data[6]) / S;
	} else if (matrix->data[4] > matrix->data[8]) {
		float S = sqrt(1 + matrix->data[4] - matrix->data[0] - matrix->data[8]) * 2;
		ret->data[W] = (matrix->data[2] - matrix->data[6]) / S;
		ret->data[X] = (matrix->data[1] + matrix->data[3]) / S;
		ret->data[Y] = 0.25 * S;
		ret->data[Z] = (matrix->data[5] + matrix->data[7]) / S;
	} else {
		float S = sqrt(1 + matrix->data[8] - matrix->data[0] - matrix->data[4]) * 2;
		ret->data[W] = (matrix->data[3] - matrix->data[1]) / S;
		ret->data[X] = (matrix->data[2] + matrix->data[6]) / S;
		ret->data[Y] = (matrix->data[5] + matrix->data[7]) / S;
		ret->data[Z] = 0.25 * S;
	}

	return ret;
}

matrix_t *mul_quat(const matrix_t *a, const matrix_t *b) {
	matrix_t *ret = init_matrix(4, 1);

	ret->data[X] = a->data[W] * b->data[X] + a->data[X] * b->data[W] + a->data[Y] * b->data[Z] - a->data[Z] * b->data[Y];
	ret->data[Y] = a->data[W] * b->data[Y] - a->data[X] * b->data[Z] + a->data[Y] * b->data[W] + a->data[Z] * b->data[X];
	ret->data[Z] = a->data[W] * b->data[Z] + a->data[X] * b->data[Y] - a->data[Y] * b->data[X] + a->data[Z] * b->data[W];
	ret->data[W] = a->data[W] * b->data[W] - a->data[X] * b->data[X] - a->data[Y] * b->data[Y] - a->data[Z] * b->data[Z];

	return ret;
}

matrix_t *mul_vector(const matrix_t *a, const matrix_t *b) {
	matrix_t *ret = init_matrix(3, 1);

	ret->data[X] = a->data[Y] * b->data[Z] - a->data[Z] * b->data[Y];
	ret->data[Y] = a->data[Z] * b->data[X] - a->data[X] * b->data[Z];
	ret->data[Z] = a->data[X] * b->data[Y] - a->data[Y] * b->data[X];

	return ret;
}

float vector_dot(const matrix_t *a, const matrix_t *b) {
	float ret = 0;
	for (int i = 0; i < 3; i++) {
		ret += a->data[i] * b->data[i];
	}
	return ret;
}

int sgn(float x) {
	if (x > 0) {
		return 1;
	} else if (x < 0) {
		return -1;
	} else {
		return 0;
	}
}

float rad_to_deg(float rad) {
	return rad * 180 / M_PI;
}

float deg_to_rad(float deg) {
	return deg * M_PI / 180;
}

