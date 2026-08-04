#include <iostream>
#include <stdio.h>
#include <math.h>

#include "plot.h"
#include "kin_math.h"

FILE *true_x_data;
FILE *true_y_data;
FILE *true_z_data;
FILE *estm_x_data;
FILE *estm_y_data;
FILE *estm_z_data;
FILE *gp;

float true_axis_ranges[3] = {0, 0, 0};
float estm_axis_ranges[3] = {0, 0, 0};

void plot_t::init() {
	gp = popen("gnuplot -persist", "w");
	if (!gp) {
		fprintf(stderr, "Error opening GNUplot\n");
		return;
	}

	true_x_data = fopen("true_x_data.txt", "w");
	if (!true_x_data) {
		fprintf(stderr, "Error opening x data file\n");
		pclose(gp);
		return;
	}

	true_y_data = fopen("true_y_data.txt", "w");
	if (!true_y_data) {
		fprintf(stderr, "Error opening y data file\n");
		pclose(gp);
		return;
	}

	true_z_data = fopen("true_z_data.txt", "w");
	if (!true_z_data) {
		fprintf(stderr, "Error opening z data file\n");
		pclose(gp);
		return;
	}

	estm_x_data = fopen("estm_x_data.txt", "w");
	if (!estm_x_data) {
		fprintf(stderr, "Error opening x data file\n");
		pclose(gp);
		return;
	}

	estm_y_data = fopen("estm_y_data.txt", "w");
	if (!estm_y_data) {
		fprintf(stderr, "Error opening y data file\n");
		pclose(gp);
		return;
	}

	estm_z_data = fopen("estm_z_data.txt", "w");
	if (!estm_z_data) {
		fprintf(stderr, "Error opening z data file\n");
		pclose(gp);
		return;
	}
}

void plot_t::plot() {
	fclose(true_x_data);
	fclose(true_y_data);
	fclose(true_z_data);
	fclose(estm_x_data);
	fclose(estm_y_data);
	fclose(estm_z_data);

	fprintf(gp, "set ylabel 'deg'\n");  // Y-axis label
	fprintf(gp, "set term qt font 'Arial,8'\n");
	fprintf(gp, "set multiplot layout 3,1 rows\n");

	fprintf(gp, "plot 'true_x_data.txt' lt rgb 'forest-green' with lines title 'True', ");
	fprintf(gp, "'estm_x_data.txt' lt rgb 'medium-blue' with lines title 'Estmated'\n");

	fprintf(gp, "plot 'true_y_data.txt' lt rgb 'forest-green' with lines title 'True', ");
	fprintf(gp, "'estm_y_data.txt' lt rgb 'medium-blue' with lines title 'Estmated'\n");

	fprintf(gp, "plot 'true_z_data.txt' lt rgb 'forest-green' with lines title 'True', ");
	fprintf(gp, "'estm_z_data.txt' lt rgb 'medium-blue' with lines title 'Estmated'\n");

	fflush(gp);
	pclose(gp);
}

void plot_t::add_point(matrix_t *orientation, string type) {

	if (type == "true") {
		fprintf(true_x_data, "%f %f\n", true_axis_ranges[X], orientation->data[X]);
		true_axis_ranges[X]++;

		fprintf(true_y_data, "%f %f\n", true_axis_ranges[Y], orientation->data[Y]);
		true_axis_ranges[Y]++;

		fprintf(true_z_data, "%f %f\n", true_axis_ranges[Z], orientation->data[Z]);
		true_axis_ranges[Z]++;
	} else if (type == "estm") {
		fprintf(estm_x_data, "%f %f\n", estm_axis_ranges[X], orientation->data[X]);
		estm_axis_ranges[X]++;

		fprintf(estm_y_data, "%f %f\n", estm_axis_ranges[Y], orientation->data[Y]);
		estm_axis_ranges[Y]++;

		fprintf(estm_z_data, "%f %f\n", estm_axis_ranges[Z], orientation->data[Z]);
		estm_axis_ranges[Z]++;
	} else {
		cout << "Incorrect type: " << type << " select 'true' or 'estm'" << endl;
	}
}

/*
int main() {
	plot_t plot;
	plot.init();

	for (double x = 0; x < 10; x += 0.1) {
		matrix_t *True = init_matrix(3, 1);
		True->data[X] = sin(x);
		True->data[Y] = cos(x);
		True->data[Z] = tan(x);
		plot.add_point(True, "true");

		matrix_t *Estm = init_matrix(3, 1);
		Estm->data[X] = cos(x);
		Estm->data[Y] = sin(x);
		Estm->data[Z] = atan(x);
		plot.add_point(Estm, "estm");
	}

	plot.plot();

	return 0;
}
*/
