#include <stdlib.h>
#include <stdio.h>

#include "kin_error.h"

void error_handler(const uint8_t error_code) {
	const char* error_str = ERROR_CODE_STR[error_code];
	fprintf(stderr, "Process exited with error code: %i %s\n", error_code, error_str);
	exit(1);
}

