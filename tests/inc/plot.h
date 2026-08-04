#ifndef PLOT_H
#define PLOT_H

#include <string>

#include "kin_math.h"

using namespace std;

class plot_t {
	public:

	void init();

	void plot();

	void add_point(matrix_t *orientation, string type = "true");
};

#endif
