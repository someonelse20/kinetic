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

#include <stdbool.h>
#include <stdint.h>

#include "kin_types.h"

// Defines for vector/quaternion indexes
// quaternions are in the format x y z w
#define X 0
#define Y 1
#define Z 2
#define W 3

void free_matrix(matrix_t *matrix);
matrix_t *copy_matrix(const matrix_t *matrix);
matrix_t *init_matrix(uint8_t rows, uint8_t cols);
matrix_t *fill_matrix(uint8_t rows, uint8_t cols, float value);
matrix_t *ident_matrix(uint8_t size);
matrix_t *arr_to_matrix(float *arr, uint8_t rows, uint8_t cols);
float *matrix_to_arr(matrix_t *matrix);
float *copy_arr(const float *arr, uint8_t size);
void print_matrix(const matrix_t *matrix);
void print_arr(const float *arr, uint8_t size);
uint8_t move_matrix(const matrix_t *src, matrix_t *dest);

uint8_t trans_matrix(const matrix_t *matrix, matrix_t *ret);
uint8_t add_matrix(const matrix_t *a, const matrix_t *b, matrix_t *ret);
uint8_t sub_matrix(const matrix_t *a, const matrix_t *b, matrix_t *ret);
uint8_t mul_matrix(const matrix_t *a, const matrix_t *b, matrix_t *ret);
uint8_t scale_matrix(const matrix_t *matrix, const float scalar, matrix_t *ret);

matrix_t *trans_matrix_alloc(const matrix_t *matrix);
matrix_t *add_matrix_alloc(const matrix_t *a, const matrix_t *b);
matrix_t *sub_matrix_alloc(const matrix_t *a, const matrix_t *b);
matrix_t *mul_matrix_alloc(const matrix_t *a, const matrix_t *b);
matrix_t *scale_matrix_alloc(const matrix_t *matrix, const float scalar);

matrix_t *trans_matrix_free(matrix_t *matrix);
matrix_t *add_matrix_free(matrix_t *a, const matrix_t *b);
matrix_t *sub_matrix_free(matrix_t *a, const matrix_t *b);
matrix_t *mul_matrix_free(matrix_t *a, const matrix_t *b);
matrix_t *scale_matrix_free(matrix_t *matrix, const float scalar);

float matrix_det(const matrix_t *matrix);
float matrix_norm(const matrix_t *matrix);
float matrix_minor(const matrix_t *matrix, uint8_t row, uint8_t col);
matrix_t *inv_quat(const matrix_t *matrix);
matrix_t *inv_matrix(const matrix_t *matrix);
matrix_t *ajt_matrix(const matrix_t *matrix);
matrix_t *skew_symm_matrix(const matrix_t *matrix);
uint8_t normalize_matrix(matrix_t *matrix);
matrix_t *normalize_matrix_alloc(const matrix_t *matrix);

matrix_t *quat_to_euler(matrix_t *matrix);
matrix_t *euler_to_quat(const matrix_t *matrix);
matrix_t *quat_to_rot_matrix(const matrix_t *matrix);
matrix_t *rot_matrix_to_quat(const matrix_t *matrix);

bool is_quat(const matrix_t *matrix);
bool is_vector(const matrix_t *matrix);
float dot_prod(const matrix_t *a, const matrix_t *b);
matrix_t *cross_prod(const matrix_t *a, const matrix_t *b);
matrix_t *quat_prod(const matrix_t *a, const matrix_t *b);
matrix_t *quat_conjugate(const matrix_t *matrix);

int sgn(float x);
float rad_to_deg(float rad);
float deg_to_rad(float deg);

#ifdef __cplusplus
}
#endif

#endif
