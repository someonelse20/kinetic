/*
 * Copyright (c) 2024 Rubik Proxy. All rights reserved.
 * https://github.com/rubikproxy/matrix.h
 *
 * This software uses and is inspired by
 * the matrix library from rubikproxy.
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "kin_error.h"
#include "kin_types.h"
#include "kin_math.h"

matrix_t *copy_matrix(const matrix_t *matrix) {
	matrix_t *ret = init_matrix(matrix->rows, matrix->cols);

	for (int i = 0; i < matrix->rows * matrix->cols; i++) {
		ret->data[i] = matrix->data[i];
	}

	return ret;
}

matrix_t *init_matrix(uint8_t rows, uint8_t cols) {
	matrix_t *matrix = (matrix_t *)malloc(sizeof(matrix_t));
	matrix->rows = rows;
	matrix->cols = cols;
	matrix->data = (float *)malloc(rows * cols * sizeof(float));
	return matrix;
}

matrix_t *fill_matrix(uint8_t rows, uint8_t cols, float value) {
	matrix_t *ret = init_matrix(rows, cols);

	for (int i = 0; i < rows * cols; i++) {
		ret->data[i] = value;
	}

	return ret;
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

float *matrix_to_arr(matrix_t *matrix) {
	float *arr = (float *)malloc(matrix->rows * matrix->cols * sizeof(float));

	for (uint8_t i = 0; i < matrix->rows * matrix->cols; i++) {
		arr[i] = matrix->data[i];
	}

	return arr;
}

float *copy_arr(const float *arr, uint8_t size) {
	float *ret = (float *)malloc(sizeof(arr));

	for (int i = 0; i < size; i++) {
		ret[i] = arr[i];
	}

	return ret;
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

uint8_t move_matrix(const matrix_t *src, matrix_t *dest) {
	if (src->rows != dest->rows || src->cols != dest->cols) error_handler(MATRIX_DIMENTION_ERROR);

	for (int i = 0; i < src->rows * src->cols; i++) {
		dest->data[i] = src->data[i];
	}

	return 0;
}

uint8_t trans_matrix(const matrix_t *matrix, matrix_t *ret) {
	for (uint8_t i = 0; i < matrix->rows; i++) {
		for (uint8_t j = 0; j < matrix->cols; j++) {
			ret->data[j * matrix->rows + i] = matrix->data[i * matrix->cols + j];
		}
	}

	return 0;
}

uint8_t add_matrix(const matrix_t *a, const matrix_t *b, matrix_t *ret) {
	if (a->rows != b->rows || a->cols != b->cols) error_handler(MATRIX_DIMENTION_ERROR);

	for (uint8_t i = 0; i < a->rows * a->cols; i++) {
		ret->data[i] = a->data[i] + b->data[i];
	}

	return 0;
}

uint8_t sub_matrix(const matrix_t *a, const matrix_t *b, matrix_t *ret) {
	if (a->rows != b->rows || a->cols != b->cols) error_handler(MATRIX_DIMENTION_ERROR);

	for (uint8_t i = 0; i < a->rows * a->cols; i++) {
		ret->data[i] = a->data[i] - b->data[i];
	}

	return 0;
}

uint8_t mul_matrix(const matrix_t *a, const matrix_t *b, matrix_t *ret) {
	if (a->cols != b->rows) error_handler(MATRIX_DIMENTION_ERROR);

	for (uint8_t i = 0; i < a->rows; i++) {
		for (uint8_t j = 0; j < b->cols; j++) {
			ret->data[i * b->cols + j] = 0;
			for (uint8_t k = 0; k < a->cols; k++) {
				ret->data[i * b->cols + j] += a->data[i * a->cols + k] * b->data[k * b->cols + j];
			}
		}
	}

	return 0;
}

uint8_t scale_matrix(const matrix_t *matrix, const float scalar, matrix_t *ret) {
	for (uint8_t i = 0; i < matrix->rows * matrix->cols; i++) {
		ret->data[i] = scalar * matrix->data[i];
	}

	return 0;
}

matrix_t *trans_matrix_alloc(const matrix_t *matrix) {
	matrix_t *ret = init_matrix(matrix->cols, matrix->rows);
	trans_matrix(matrix, ret);
	return ret;
}

matrix_t *add_matrix_alloc(const matrix_t *a, const matrix_t *b) {
	matrix_t *ret = init_matrix(a->rows, a->cols);
	add_matrix(a, b, ret);
	return ret;
}

matrix_t *sub_matrix_alloc(const matrix_t *a, const matrix_t *b) {
	matrix_t *ret = init_matrix(a->rows, a->cols);
	sub_matrix(a, b, ret);
	return ret;
}

matrix_t *mul_matrix_alloc(const matrix_t *a, const matrix_t *b) {
	matrix_t *ret = init_matrix(a->rows, b->cols);
	mul_matrix(a, b, ret);
	return ret;
}

matrix_t *scale_matrix_alloc(const matrix_t *matrix, const float scalar) {
	matrix_t *ret = init_matrix(matrix->rows, matrix->cols);
	scale_matrix(matrix, scalar, ret);
	return ret;
}

matrix_t *trans_matrix_free(matrix_t *matrix) {
	matrix_t *ret = trans_matrix_alloc(matrix);
	free_matrix(matrix);
	return ret;
}

matrix_t *add_matrix_free(matrix_t *a, const matrix_t *b) {
	matrix_t *ret = add_matrix_alloc(a, b);
	free_matrix(a);
	return ret;
}

matrix_t *sub_matrix_free(matrix_t *a, const matrix_t *b) {
	matrix_t *ret = sub_matrix_alloc(a, b);
	free_matrix(a);
	return ret;
}

matrix_t *mul_matrix_free(matrix_t *a, const matrix_t *b) {
	matrix_t *ret = mul_matrix_alloc(a, b);
	free_matrix(a);
	return ret;
}

matrix_t *scale_matrix_free(matrix_t *matrix, float scalar) {
	matrix_t *ret = scale_matrix_alloc(matrix, scalar);
	free_matrix(matrix);
	return ret;
}

float matrix_det(const matrix_t *matrix) { // This function is ai generated with some modification.
	if (matrix->rows != matrix->cols) error_handler(MATRIX_DIMENTION_ERROR);

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
	/*
	float norm = 0;
	for (uint8_t i = 0; i < matrix->rows * matrix->cols; i++) {
		norm += matrix->data[i] * matrix->data[i];
	}
	return sqrt(norm);
	*/
	return 1;
}

float matrix_minor(const matrix_t *matrix, uint8_t row, uint8_t col) {
	if (matrix->cols != matrix->rows) error_handler(MATRIX_DIMENTION_ERROR);
	if (matrix->cols < 2) error_handler(MATRIX_DIMENTION_ERROR);
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

	float det = matrix_det(minor_m);

	free_matrix(minor_m);
	return det;
}

matrix_t *inv_quat(const matrix_t *matrix) {
	// Computation of a unit quaternion (normal of 1) is much easier.
	// q^-1 = q^8 / ||q||^2

	if (!is_quat(matrix)) error_handler(MATRIX_DIMENTION_ERROR);

	matrix_t *ret;

	matrix_t *quat;

	// Normalize matrix if needed.
	float norm = matrix_norm(matrix);
	if (abs(norm) < 0.99) {
		quat = normalize_matrix_alloc(matrix);
	} else {
		quat = copy_matrix(matrix);
	}

	ret = scale_matrix_alloc(quat_conjugate(quat), 1 / (norm * norm));

	free_matrix(quat);
	return ret;
}

matrix_t *inv_matrix(const matrix_t *matrix) {
	if (matrix->cols != matrix->rows) error_handler(MATRIX_DIMENTION_ERROR);
	if (matrix->cols < 2) error_handler(MATRIX_DIMENTION_ERROR);

	float det = matrix_det(matrix);
	// Technically if the detrement is 0 the math is saying there isn't an inverse.
	// But sometimes the detrement is actually just really close to zero and the
	// rest of the program still expects an output. Setting the detrement to almost
	// zero returns an output of pretty much all zeros which satisfies the rest of the
	// math.
	if (det == 0.f) det = 0.000001; // NOTE: This isn't pretty but gets the job done.
	// TODO: See if returning a matrix filled with zeros works as well.
	
	matrix_t *adj = ajt_matrix(matrix);
	matrix_t *ret = scale_matrix_free(adj, 1.0 / det);

	return ret;
}

matrix_t *ajt_matrix(const matrix_t *matrix) {
	if (matrix->cols != matrix->rows) error_handler(MATRIX_DIMENTION_ERROR);
	if (matrix->cols < 2) error_handler(MATRIX_DIMENTION_ERROR);

	matrix_t *ajt = init_matrix(matrix->rows, matrix->cols);
	uint8_t size = matrix->rows;

	for (uint8_t i = 0; i < size; i++) {
		for (uint8_t j = 0; j < size; j++) {
			ajt->data[i * size + j] = pow(-1.0, i + j + 2) * matrix_minor(matrix, i, j);
			// ret->data[i * size + j] = matrix_minor(matrix, i, j);
		}
	}

	matrix_t *ret = trans_matrix_free(ajt);

	return ret;
}

uint8_t normalize_matrix(matrix_t *matrix) {
	float norm = matrix_norm(matrix);

	/*
	// Prevent divide by zero.
	if (norm == 0.f) {
		// Quaternions are a special case where a unit quaternion is {0, 0, 0, 1} not {0, 0, 0, 0}.
		if (is_quat(matrix)) {
			matrix->data[X] = 0.f;
			matrix->data[Y] = 0.f;
			matrix->data[Z] = 0.f;
			matrix->data[W] = 1.f;

			return 1;
		}
		for (int i = 0; i < matrix->rows * matrix->cols; i ++) { // NOTE: When optimizing, this may need to be uint16 over uint8 because it could have max value of uint8 * uint8.
			matrix->data[i] = 0.f;
		}
		return 1; // TODO: Change this when full error codes are implemented.
	}

	// NOTE: Copying may not be nessesary.
	matrix_t *matrix_cpy = copy_matrix(matrix);
	scale_matrix(matrix_cpy, 1 / norm, matrix);
	free_matrix(matrix_cpy);
	*/

	return 0;
}

matrix_t *normalize_matrix_alloc(const matrix_t *matrix) {
	matrix_t *ret = copy_matrix(matrix);
	normalize_matrix(ret);
	return ret;
}

matrix_t *skew_symm_matrix(const matrix_t *matrix) {
	if (!is_vector(matrix)) {
		printf("For now skew_symm_matrix only works for 3x1 vectors.");
		error_handler(MATRIX_DIMENTION_ERROR);
	}

	matrix_t *ret = init_matrix(3, 3);

	ret->data[0] = 0;
	ret->data[1] = -matrix->data[Z];
	ret->data[2] = matrix->data[Y];

	ret->data[3] = matrix->data[Z];
	ret->data[4] = 0;
	ret->data[5] = -matrix->data[X];

	ret->data[6] = -matrix->data[Y];
	ret->data[7] = matrix->data[X];
	ret->data[8] = 0;

	return ret;
}

matrix_t *euler_to_quat(const matrix_t *matrix) {
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

	/* Not sure if this is left over code.
	   float w2 = pow(normed->data[W], 2);
	   float x2 = pow(normed->data[X], 2);
	   float y2 = pow(normed->data[Y], 2);
	   float z2 = pow(normed->data[Z], 2);
	 */

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

matrix_t *quat_to_rot_matrix(const matrix_t *matrix) { // NOTE: There seems to be multiple ways to do this
	if (matrix->rows != 4 || matrix->cols != 1) error_handler(MATRIX_DIMENTION_ERROR);
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

matrix_t *rot_matrix_to_quat(const matrix_t *matrix) { // this function is from https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	if (matrix->rows != 3 || matrix->cols != 3) error_handler(MATRIX_DIMENTION_ERROR);
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

bool is_quat(const matrix_t *matrix) {
	if (matrix->rows == 4 && matrix->cols == 1) {
		return true;
	} else {
		return false;
	}
}

bool is_vector(const matrix_t *matrix) {
	if (matrix->rows == 3 && matrix->cols == 1) { // Vertical vector.
		return true;
	} else if (matrix->rows == 1 && matrix->cols == 3) { // Horizontal vector.
		return true;
	} else {
		return false;
	}
}

float dot_prod(const matrix_t *a, const matrix_t *b) {
	// Must have 1 column and same amount of rows.
	if (a->cols != 1 || b->cols != 1) error_handler(MATRIX_DIMENTION_ERROR);
	if (a->rows != b->rows) error_handler(MATRIX_DIMENTION_ERROR);

	float ret = 0;
	for (int i = 0; i < 3; i++) {
		ret += a->data[i] * b->data[i];
	}
	return ret;
}

matrix_t *quat_prod(const matrix_t *a, const matrix_t *b) {
	if (!is_quat(a) || !is_quat(b)) error_handler(MATRIX_DIMENTION_ERROR);

	matrix_t *ret = init_matrix(4, 1);

	ret->data[X] = a->data[W] * b->data[X] + a->data[X] * b->data[W] + a->data[Y] * b->data[Z] - a->data[Z] * b->data[Y];
	ret->data[Y] = a->data[W] * b->data[Y] - a->data[X] * b->data[Z] + a->data[Y] * b->data[W] + a->data[Z] * b->data[X];
	ret->data[Z] = a->data[W] * b->data[Z] + a->data[X] * b->data[Y] - a->data[Y] * b->data[X] + a->data[Z] * b->data[W];
	ret->data[W] = a->data[W] * b->data[W] - a->data[X] * b->data[X] - a->data[Y] * b->data[Y] - a->data[Z] * b->data[Z];

	return ret;
}

matrix_t *cross_prod(const matrix_t *a, const matrix_t *b) {
	if (!is_vector(a) || !is_vector(b)) error_handler(MATRIX_DIMENTION_ERROR);

	matrix_t *ret = init_matrix(3, 1);

	ret->data[X] = a->data[Y] * b->data[Z] - a->data[Z] * b->data[Y];
	ret->data[Y] = a->data[Z] * b->data[X] - a->data[X] * b->data[Z];
	ret->data[Z] = a->data[X] * b->data[Y] - a->data[Y] * b->data[X];

	return ret;
}

matrix_t *quat_conjugate(const matrix_t *matrix) {
	if (!is_quat(matrix)) error_handler(MATRIX_DIMENTION_ERROR);

	matrix_t *ret = init_matrix(4, 1);
	ret->data[X] = -matrix->data[X];
	ret->data[Y] = -matrix->data[Y];
	ret->data[Z] = -matrix->data[Z];
	ret->data[W] =  matrix->data[W];

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

