#ifndef PLOT_H
#define PLOT_H

#include <string>

#include "kin_types.h"

class plot_t {
	public:

	void plot();

	void add_point(matrix_t *orientation, std::string type = "true");
};

#endif
