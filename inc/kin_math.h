#ifndef KIN_MATH_H
#define KIN_MATH_H

#include "kin_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and allocate a matrix with given dimensions.
 *
 * @param rows Number of rows
 * @param cols Number of columns
 * @return Pointer to newly allocated matrix, or NULL on failure
 */
matrix_t *init_matrix(uint8_t rows, uint8_t cols);

/**
 * @brief Create a deep copy of a matrix.
 *
 * @param matrix Source matrix to copy
 * @return Pointer to new matrix with copied data
 */
matrix_t *copy_matrix(const matrix_t *matrix);

/**
 * @brief Allocate and fill a matrix with a single value.
 *
 * @param rows Number of rows
 * @param cols Number of columns
 * @param value Value to fill each element with
 * @return Pointer to newly allocated matrix
 */
matrix_t *fill_matrix(uint8_t rows, uint8_t cols, float value);

/**
 * @brief Create an identity matrix of given size.
 *
 * @param size Dimension (for square matrices)
 * @return Pointer to identity matrix
 */
matrix_t *ident_matrix(uint8_t size);

/**
 * @brief Free memory allocated for a matrix.
 *
 * @param matrix Matrix to free
 */
void free_matrix(matrix_t *matrix);

/**
 * @brief Convert array of floats to matrix.
 *
 * @param arr Input array of size rows * cols
 * @param rows Number of rows
 * @param cols Number of columns
 * @return Pointer to newly allocated matrix
 */
matrix_t *arr_to_matrix(float *arr, uint8_t rows, uint8_t cols);

/**
 * @brief Convert matrix to array of floats.
 *
 * @param matrix Input matrix
 * @return Pointer to array of size rows * cols
 */
float *matrix_to_arr(matrix_t *matrix);

/**
 * @brief Copy an array of floats.
 *
 * @param arr Input array
 * @param size Number of elements
 * @return Pointer to new array
 */
float *copy_arr(const float *arr, uint8_t size);

/**
 * @brief Copy matrix contents from source to destination.
 *
 * @param src Source matrix
 * @param dest Destination matrix
 * @return 0 on success
 */
uint8_t move_matrix(const matrix_t *src, matrix_t *dest);

/**
 * @brief Transpose a matrix.
 *
 * @param matrix Input matrix
 * @param ret Output matrix for transposed result
 * @return 0 on success
 */
uint8_t trans_matrix(const matrix_t *matrix, matrix_t *ret);

/**
 * @brief Allocate and transpose a matrix.
 *
 * @param matrix Input matrix
 * @return Pointer to transposed matrix, or NULL on failure
 */
matrix_t *trans_matrix_alloc(const matrix_t *matrix);

/**
 * @brief Add two matrices element-wise.
 *
 * @param a First matrix
 * @param b Second matrix
 * @param ret Output for result
 * @return 0 on success
 */
uint8_t add_matrix(const matrix_t *a, const matrix_t *b, matrix_t *ret);

/**
 * @brief Subtract two matrices element-wise.
 *
 * @param a Minuend matrix
 * @param b Subtrahend matrix
 * @param ret Output for result
 * @return 0 on success
 */
uint8_t sub_matrix(const matrix_t *a, const matrix_t *b, matrix_t *ret);

/**
 * @brief Multiply two matrices (A * B).
 *
 * @param a Left matrix
 * @param b Right matrix
 * @param ret Output for result
 * @return 0 on success
 */
uint8_t mul_matrix(const matrix_t *a, const matrix_t *b, matrix_t *ret);

/**
 * @brief Scale a matrix by a scalar value.
 *
 * @param matrix Input matrix
 * @param scalar Scaling factor
 * @param ret Output for result
 * @return 0 on success
 */
uint8_t scale_matrix(const matrix_t *matrix, const float scalar, matrix_t *ret);

/**
 * @brief Allocate and perform matrix addition.
 *
 * @param a First matrix
 * @param b Second matrix
 * @return Pointer to result matrix, or NULL on failure
 */
matrix_t *add_matrix_alloc(const matrix_t *a, const matrix_t *b);

/**
 * @brief Allocate and perform matrix subtraction.
 *
 * @param a Minuend matrix
 * @param b Subtrahend matrix
 * @return Pointer to result matrix, or NULL on failure
 */
matrix_t *sub_matrix_alloc(const matrix_t *a, const matrix_t *b);

/**
 * @brief Allocate and perform matrix multiplication.
 *
 * @param a Left matrix
 * @param b Right matrix
 * @return Pointer to result matrix, or NULL on failure
 */
matrix_t *mul_matrix_alloc(const matrix_t *a, const matrix_t *b);

/**
 * @brief Allocate and perform scalar multiplication.
 *
 * @param matrix Input matrix
 * @param scalar Scaling factor
 * @return Pointer to result matrix, or NULL on failure
 */
matrix_t *scale_matrix_alloc(const matrix_t *matrix, const float scalar);

/**
 * @brief Allocate, transpose, and free input matrix.
 *
 * @param matrix Input matrix to transpose and free
 * @return Pointer to transposed matrix, or NULL on failure
 */
matrix_t *trans_matrix_free(matrix_t *matrix);

/**
 * @brief Allocate, add, and free first matrix.
 *
 * @param a First matrix
 * @param b Second matrix
 * @return Pointer to result matrix, or NULL on failure
 */
matrix_t *add_matrix_free(matrix_t *a, const matrix_t *b);

/**
 * @brief Allocate, subtract, and free first matrix.
 *
 * @param a Minuend matrix
 * @param b Subtrahend matrix
 * @return Pointer to result matrix, or NULL on failure
 */
matrix_t *sub_matrix_free(matrix_t *a, const matrix_t *b);

/**
 * @brief Allocate, multiply, and free first matrix.
 *
 * @param a Left matrix
 * @param b Right matrix
 * @return Pointer to result matrix, or NULL on failure
 */
matrix_t *mul_matrix_free(matrix_t *a, const matrix_t *b);

/**
 * @brief Allocate, scale, and free input matrix.
 *
 * @param matrix Input matrix
 * @param scalar Scaling factor
 * @return Pointer to result matrix, or NULL on failure
 */
matrix_t *scale_matrix_free(matrix_t *matrix, float scalar);

/**
 * @brief Compute determinant of a square matrix.
 *
 * @param matrix Square matrix
 * @return Determinant value, or 0 on failure
 */
float matrix_det(const matrix_t *matrix);

/**
 * @brief Compute Euclidean norm (Frobenius) of a matrix.
 *
 * @param matrix Input matrix
 * @return Norm value
 */
float matrix_norm(const matrix_t *matrix);

/**
 * @brief Compute minor determinant at given row/column.
 *
 * @param matrix Square matrix
 * @param row Row index
 * @param col Column index
 * @return Minor determinant, or 0 on failure
 */
float matrix_minor(const matrix_t *matrix, uint8_t row, uint8_t col);

/**
 * @brief Compute inverse of a quaternion represented as matrix.
 *
 * Uses the unit quaternion formula: q^-1 = conj(q) / ||q||^2
 *
 * @param matrix Quaternion matrix (4x1)
 * @return Inverse quaternion matrix, or NULL on failure
 */
matrix_t *inv_quat(const matrix_t *matrix);

/**
 * @brief Compute inverse of a square matrix.
 *
 * Uses adjugate/determinant method.
 *
 * @param matrix Square matrix
 * @return Inverse matrix, or NULL on failure
 */
matrix_t *inv_matrix(const matrix_t *matrix);

/**
 * @brief Compute adjugate (adjoint) of a square matrix.
 *
 * @param matrix Square matrix
 * @return Adjugate matrix, or NULL on failure
 */
matrix_t *ajt_matrix(const matrix_t *matrix);

/**
 * @brief Normalize a matrix (scale to unit norm).
 *
 * Special case: unit quaternion is {0, 0, 0, 1} if norm is 0.
 *
 * @param matrix Input/output matrix to normalize
 * @return 0 on success, 1 if normalization failed
 */
uint8_t normalize_matrix(matrix_t *matrix);

/**
 * @brief Allocate and normalize a matrix.
 *
 * @param matrix Input matrix
 * @return Normalized matrix, or NULL on failure
 */
matrix_t *normalize_matrix_alloc(const matrix_t *matrix);

/**
 * @brief Create a skew-symmetric matrix from a 3D vector.
 *
 * @param vector 3x1 vector
 * @return 3x3 skew-symmetric matrix, or NULL on failure
 */
matrix_t *skew_symm_matrix(const matrix_t *vector);

/**
 * @brief Convert Euler angles (XYZ) to quaternion.
 *
 * @param euler 3x1 vector [roll, pitch, yaw]
 * @return Quaternion matrix (4x1), or NULL on failure
 */
matrix_t *euler_to_quat(const matrix_t *euler);

/**
 * @brief Convert quaternion to Euler angles (XYZ).
 *
 * @param quat Quaternion matrix (4x1)
 * @return Euler angles in degrees [roll, pitch, yaw], or NULL on failure
 */
matrix_t *quat_to_euler(matrix_t *quat);

/**
 * @brief Convert quaternion to rotation matrix.
 *
 * @param quat Quaternion matrix (4x1)
 * @return 3x3 rotation matrix, or NULL on failure
 */
matrix_t *quat_to_rot_matrix(const matrix_t *quat);

/**
 * @brief Convert rotation matrix to quaternion.
 *
 * @param matrix 3x3 rotation matrix
 * @return Quaternion matrix (4x1), or NULL on failure
 */
matrix_t *rot_matrix_to_quat(const matrix_t *matrix);

/**
 * @brief Convert Euler angles (XYZ) to rotation matrix.
 *
 * Rotation order: Z * Y * X (yaw * pitch * roll)
 *
 * @param euler 3x1 vector [roll, pitch, yaw]
 * @return 3x3 rotation matrix, or NULL on failure
 */
matrix_t *euler_to_rot_matrix(const matrix_t *euler);

/**
 * @brief Check if matrix is a valid quaternion.
 *
 * @param matrix Input matrix
 * @return true if 4x1, false otherwise
 */
bool is_quat(const matrix_t *matrix);

/**
 * @brief Check if matrix is a valid vector.
 *
 * @param matrix Input matrix
 * @return true if 3x1 or 1x3, false otherwise
 */
bool is_vector(const matrix_t *matrix);

/**
 * @brief Compute dot product of two column vectors.
 *
 * Both matrices must have 1 column and the same number of rows.
 *
 * @param a First column vector
 * @param b Second column vector
 * @return Dot product value, or 0 on failure
 */
float dot_prod(const matrix_t *a, const matrix_t *b);

/**
 * @brief Multiply two quaternions (q1 * q2).
 *
 * @param a First quaternion (4x1)
 * @param b Second quaternion (4x1)
 * @return Product quaternion, or NULL on failure
 */
matrix_t *quat_prod(const matrix_t *a, const matrix_t *b);

/**
 * @brief Compute cross product of two vectors.
 *
 * @param a First vector (3x1)
 * @param b Second vector (3x1)
 * @return Cross product vector, or NULL on failure
 */
matrix_t *cross_prod(const matrix_t *a, const matrix_t *b);

/**
 * @brief Compute conjugate of a quaternion.
 *
 * @param quat Input quaternion (4x1)
 * @return Conjugate quaternion, or NULL on failure
 */
matrix_t *quat_conjugate(const matrix_t *quat);

/**
 * @brief Convert ENU vector to NED.
 *
 * @param v_enu Input ENU vector
 * @return 0 on success
 */
uint8_t *enu_to_ned(matrix_t *v_enu);

/**
 * @brief Convert NED vector to ENU.
 *
 * @param v_ned Input NED vector
 * @return 0 on success
 */
uint8_t *ned_to_enu(matrix_t *v_ned);

/**
 * @brief Convert radians to degrees.
 *
 * @param rad Angle in radians
 * @return Angle in degrees
 */
float rad_to_deg(float rad);

/**
 * @brief Convert degrees to radians.
 *
 * @param deg Angle in degrees
 * @return Angle in radians
 */
float deg_to_rad(float deg);

/**
 * @brief Print a matrix to stdout in row-major format.
 *
 * @param matrix Input matrix to print
 */
void print_matrix(const matrix_t *matrix);

/**
 * @brief Print an array of floats to stdout.
 *
 * @param arr Input array
 * @param size Number of elements
 */
void print_arr(const float *arr, uint8_t size);

/**
 * @brief Compute sign of a float.
 *
 * @param x Input value
 * @return 1 if positive, -1 if negative, 0 if zero
 */
int sgn(float x);

#ifdef __cplusplus
}
#endif

#endif
