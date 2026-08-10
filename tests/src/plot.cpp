#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>

#include "plot.h"
#include "kin_math.h"

using namespace std;

struct Data_type {
	FILE *x_file;
	FILE *y_file;
	FILE *z_file;

	string name;
	float axis_ranges[3] = {0, 0, 0};
};

Data_type *get_type(string name);

FILE *open_file(string filename);

FILE *gp;
string gp_input;

int num_of_types = 0;
string type_names[100];
Data_type type_buf[100];

void plot_t::plot() {
	if (num_of_types == 0) {
		return;
	}

	for (int i = 0; i < num_of_types; i++) {
		fclose(type_buf[i].x_file);
		fclose(type_buf[i].y_file);
		fclose(type_buf[i].z_file);
	}

	gp = popen("gnuplot -persist", "w");

	if (!gp) {
		cerr << "Error opening GNUplot" << endl;
		return;
	}

	fprintf(gp, "set ylabel 'deg'\n");
	fprintf(gp, "set term qt font 'Arial,8'\n");
	fprintf(gp, "set multiplot layout 3,1 rows\n");

	string begin_name = type_buf[0].name;
	gp_input = "plot '" + begin_name + "_x_data.txt' with lines title '" + begin_name + "', ";
	fprintf(gp, gp_input.c_str());
	for (int i = 1; i < num_of_types; i++) {
		string name = type_buf[i].name;

		gp_input = "'" + name + "_x_data.txt' with lines title '" + name + "'";
		fprintf(gp, gp_input.c_str());
	}
	fprintf(gp, "\n");

	begin_name = type_buf[0].name;
	gp_input = "plot '" + begin_name + "_y_data.txt' with lines title '" + begin_name + "', ";
	fprintf(gp, gp_input.c_str());
	for (int i = 1; i < num_of_types; i++) {
		string name = type_buf[i].name;

		gp_input = "'" + name + "_y_data.txt' with lines title '" + name + "'";
		fprintf(gp, gp_input.c_str());
	}
	fprintf(gp, "\n");

	begin_name = type_buf[0].name;
	gp_input = "plot '" + begin_name + "_z_data.txt' with lines title '" + begin_name + "', ";
	fprintf(gp, gp_input.c_str());
	for (int i = 1; i < num_of_types; i++) {
		string name = type_buf[i].name;

		gp_input = "'" + name + "_z_data.txt' with lines title '" + name + "'";
		fprintf(gp, gp_input.c_str());
	}
	fprintf(gp, "\n");

	fflush(gp);
	pclose(gp);

	num_of_types = 0;
}

void plot_t::add_point(matrix_t *orientation, string type) {
	Data_type *data_type = get_type(type);

	fprintf(data_type->x_file, "%f %f\n", data_type->axis_ranges[X], orientation->data[X]);
	data_type->axis_ranges[X] += 1;

	fprintf(data_type->y_file, "%f %f\n", data_type->axis_ranges[Y], orientation->data[Y]);
	data_type->axis_ranges[Y] += 1;

	fprintf(data_type->z_file, "%f %f\n", data_type->axis_ranges[Z], orientation->data[Z]);
	data_type->axis_ranges[Z] += 1;
}

Data_type *get_type(string name) {
	for (int i = 0; i < num_of_types; i++) {
		if (type_buf[i].name == name) {
			cout << name + " matching to " + name + " success!" << endl;
			print_arr(type_buf[i].axis_ranges, 3);
			cout << endl;
			return &type_buf[i];
		}
	}

	// cout << " matching to " + name + " failure!" << endl;

	// Create new data type.

	type_buf[num_of_types].name = name;

	type_buf[num_of_types].axis_ranges[X] = 0;
	type_buf[num_of_types].axis_ranges[Y] = 0;
	type_buf[num_of_types].axis_ranges[Z] = 0;

	type_buf[num_of_types].x_file = open_file(name + "_x_data.txt");
	type_buf[num_of_types].y_file = open_file(name + "_y_data.txt");
	type_buf[num_of_types].z_file = open_file(name + "_z_data.txt");

	num_of_types++;

	return &type_buf[num_of_types - 1];
}

FILE *open_file(string filename) {
	FILE *File = fopen(filename.c_str(), "w");
	if (!File) {
		fprintf(stderr, "Error opening x data file\n");
		pclose(gp);
		return NULL;
	}
	return File;
}

