#ifndef KIN_ERROR_H
#define KIN_ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MATRIX_DIMENTION_ERROR 10

// Error code lookup table.
static const char* ERROR_CODE_STR[] = {
	"", // 0
	"", // 1
	"", // 2
	"", // 3
	"", // 4
	"", // 5
	"", // 6
	"", // 7
	"", // 8
	"", // 9
	"MATRIX_DIMENTION_ERROR", // 10
	"", // 11
	"", // 12
	"", // 13
	"", // 14
	"", // 15
	"", // 16
	"", // 17
	"", // 18
	"", // 19
};

void error_handler(const uint8_t error_code);

#ifdef __cplusplus
}
#endif

#endif
