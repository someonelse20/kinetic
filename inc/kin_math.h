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
	float *data;
} matrix_t;

typedef struct {
	matrix_t *vector;
	float value;
} eigen_t;

void free_matrix(matrix_t *matrix);
matrix_t *init_matrix(size_t rows, size_t cols);
matrix_t *arr_to_matrix(float *arr, size_t rows, size_t cols);
void print_matrix(const matrix_t *matrix);
void print_arr(const float *arr, size_t size);

matrix_t *add_matrix(const matrix_t *a, const matrix_t *b);
matrix_t *sub_matrix(const matrix_t *a, const matrix_t *b);
matrix_t *mul_matrix(const matrix_t *a, const matrix_t *b);
matrix_t *scale_matrix(matrix_t *matrix, float scalar);

matrix_t *trans_matrix(const matrix_t *matrix);
matrix_t *ident_matrix(size_t size);

matrix_t *inv_matrix(matrix_t *matrix);
float matrix_norm(const matrix_t *matrix);
matrix_t *normalize_matrix(matrix_t *matrix);

eigen_t *eigen_matrix(const matrix_t *matrix_t);

matrix_t *euler_to_quat(matrix_t *matrix);
matrix_t *quat_to_euler(matrix_t *matrix);
matrix_t *quat_to_rot_matrix(matrix_t *matrix);
matrix_t *rot_matrix_to_quat(matrix_t *matrix);

matrix_t *mul_quat(const matrix_t *a, const matrix_t *b);
matrix_t *mul_vector(const matrix_t *a, const matrix_t *b);
float vector_dot(const matrix_t *a, const matrix_t *b);

int sgn(float x);
float rad_to_deg(float rad);
float deg_to_rad(float deg);

#ifdef __cplusplus
}
#endif

#endif
