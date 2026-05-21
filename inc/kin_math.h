/*
 * Copyright (c) 2024 Rubik Proxy. All rights reserved.
 * https://github.com/rubikproxy/matrix.h
 *
 * This software is provided for educational purposes and inspired by
 * the matrix library from rubikproxy.
 */

#ifndef KIN_MATH_H
#define KIN_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>

// Defines for vector/quaternion indexes
// quaternions are in the format x y z w
#define X 0
#define Y 1
#define Z 2
#define W 3

typedef struct {
	size_t rows;
	size_t cols;
	double *data;
} matrix_t;

void free_matrix(matrix_t *matrix);
matrix_t *arr_to_matrix(double *arr, bool vert);
matrix_t *init_matrix(size_t rows, size_t cols);
void print_matrix(const matrix_t *matrix);

matrix_t *add_matrix(const matrix_t *a, const matrix_t *b);
matrix_t *sub_matrix(const matrix_t *a, const matrix_t *b);
matrix_t *mul_matrix(const matrix_t *a, const matrix_t *b);
matrix_t *scale_matrix(matrix_t *matrix, double scalar);

matrix_t *trans_matrix(matrix_t *matrix);
matrix_t *ident_matrix(size_t rows, size_t cols);

matrix_t *inv_matrix(matrix_t *matrix);
double matrix_norm(const matrix_t *matrix);
matrix_t *normalize_matrix(matrix_t *matrix);

matrix_t *euler_to_quat(matrix_t *matrix);
matrix_t *quat_to_euler(matrix_t *matrix);
matrix_t *quat_to_rot_matrix(matrix_t *matrix);

int sgn(double x);

#ifdef __cplusplus
}
#endif

#endif
