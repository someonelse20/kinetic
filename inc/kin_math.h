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

#include <stdint.h>
#include <stddef.h>

#include "kin_types.h"

// Defines for vector/quaternion indexes
// quaternions are in the format x y z w
#define X 0
#define Y 1
#define Z 2
#define W 3

typedef struct {
	matrix_t *vector;
	float value;
} eigen_t;

void free_matrix(matrix_t *matrix);
matrix_t *copy_matrix(matrix_t *matrix);
matrix_t *init_matrix(uint8_t rows, uint8_t cols);
matrix_t *fill_matrix(uint8_t rows, uint8_t cols, float value);
matrix_t *arr_to_matrix(float *arr, uint8_t rows, uint8_t cols);
void print_matrix(const matrix_t *matrix);
void print_arr(const float *arr, uint8_t size);

matrix_t *add_matrix(const matrix_t *a, const matrix_t *b);
matrix_t *sub_matrix(const matrix_t *a, const matrix_t *b);
matrix_t *mul_matrix(const matrix_t *a, const matrix_t *b);
matrix_t *scale_matrix(matrix_t *matrix, float scalar);

matrix_t *trans_matrix(const matrix_t *matrix);
matrix_t *ident_matrix(uint8_t size);

float matrix_det(const matrix_t *matrix);
float matrix_norm(const matrix_t *matrix);
float matrix_minor(const matrix_t *matrix, uint8_t row, uint8_t col);
matrix_t *inv_matrix(matrix_t *matrix);
matrix_t *ajt_matrix(matrix_t *matrix);
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
