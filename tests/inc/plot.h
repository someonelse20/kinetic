#ifndef PLOT_H
#define PLOT_H

#include <string>

#include "kin_math.h"

using namespace std;

void init();

void plot();

void add_point(matrix_t *orientation, string type = "refrence");

#endif
