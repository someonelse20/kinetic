#ifndef KIN_ERROR_H
#define KIN_ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Matrix dimension error.
 *
 * Triggered when matrix operations involve incompatible dimensions.
 */
#define MATRIX_DIMENSION_ERROR 10

/**
 * @brief Matrix inversion error.
 *
 * Triggered when a matrix cannot be inverted (e.g., singular matrix).
 */
#define MATRIX_INV_ERROR 11

/**
 * @brief Error code lookup table for human-readable error messages.
 *
 * Index corresponds to error_code values.
 */
static const char* ERROR_CODE_STR[] = {
	"", // 0 - No error
	"", // 1 - Reserved
	"", // 2 - Reserved
	"", // 3 - Reserved
	"", // 4 - Reserved
	"", // 5 - Reserved
	"", // 6 - Reserved
	"", // 7 - Reserved
	"", // 8 - Reserved
	"", // 9 - Reserved
	"MATRIX_DIMENSION_ERROR", // 10
	"MATRIX_INV_ERROR", // 11
	"", // 12 - Reserved
	"", // 13 - Reserved
	"", // 14 - Reserved
	"", // 15 - Reserved
	"", // 16 - Reserved
	"", // 17 - Reserved
	"", // 18 - Reserved
	"", // 19 - Reserved
};

/**
 * @brief Handle an error code by printing a descriptive message.
 *
 * @param error_code Error code to handle
 */
void error_handler(const uint8_t error_code);

#ifdef __cplusplus
}
#endif

#endif
