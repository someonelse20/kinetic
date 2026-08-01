#include <stdio.h>
#include <math.h>

#include "plot.h"
#include "kin_math.h"

FILE *xdata;
FILE *ydata;
FILE *zdata;
FILE *gp;

float axis_ranges[3] = {0, 0, 0};

void init() {
	gp = popen("gnuplot -persist", "w");
	if (!gp) {
		fprintf(stderr, "Error opening GNUplot\n");
		return;
	}

	xdata = fopen("xdata.txt", "w");
	if (!xdata) {
		fprintf(stderr, "Error opening x data file\n");
		pclose(gp);
		return;
	}

	ydata = fopen("ydata.txt", "w");
	if (!ydata) {
		fprintf(stderr, "Error opening y data file\n");
		pclose(gp);
		return;
	}

	zdata = fopen("zdata.txt", "w");
	if (!zdata) {
		fprintf(stderr, "Error opening z data file\n");
		pclose(gp);
		return;
	}
}

void plot() {
	fclose(xdata);
	fclose(ydata);
	fclose(zdata);

	 fprintf(gp, "set ylabel 'deg'\n"); // Y-axis label

	fprintf(gp, "set multiplot layout 3,1 rows\n");

	fprintf(gp, "set tmargin at screen 1.00; set bmargin at screen 0.75\n");
	fprintf(gp, "plot 'xdata.txt' with lines title 'sin(x)'\n"); // Plot command

	fprintf(gp, "set tmargin at screen 0.65; set bmargin at screen 0.40\n");
	fprintf(gp, "plot 'ydata.txt' with lines title 'cos(x)'\n"); // Plot command

	fprintf(gp, "set tmargin at screen 0.30; set bmargin at screen 0.05\n");
	fprintf(gp, "plot 'zdata.txt' with lines title 'tan(x)'\n"); // Plot command

	fflush(gp);
	pclose(gp);
}

void add_point(matrix_t *orientation, string type) {
	fprintf(xdata, "%f %f\n", axis_ranges[X], orientation->data[X]);
	axis_ranges[X]++;

	fprintf(ydata, "%f %f\n", axis_ranges[Y], orientation->data[Y]);
	axis_ranges[Y]++;

	fprintf(zdata, "%f %f\n", axis_ranges[Z], orientation->data[Z]);
	axis_ranges[Z]++;
}

int main() {
	init();

	for (double x = 0; x < 10; x += 0.1) {
		matrix_t *foo = init_matrix(3, 1);
		foo->data[X] = sin(x);
		foo->data[Y] = cos(x);
		foo->data[Z] = tan(x);
		add_point(foo);
	}

	plot();

	return 0;
}
