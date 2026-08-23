#include <iostream>
#include <fstream>

#include "kin_ekf.h"
#include "kin_imu.h"
#include "plot.h"

// #define PLOT

using namespace std;

typedef struct {
	float gyro[3];
	float accel[3];
	float mag[3];
} datapoint_t;

const static string record_dir = "../tests/recordings/";
const static string filename = "static.csv";

// Stores up to 1000 lines.
datapoint_t datapoints_buf[1000];
int datapoints = 0;

static void read_file();

int main() {
	read_file();

	#ifdef PLOT
	plot_t plot;
	#endif

	imu_t imu;
	imu.mag_dip = 0.000001;
	imu.gyro_noise = 0.3;
	imu.accel_noise = 0.5;
	imu.mag_noise = 0.8;
	imu.dt = 0.01;

	imu_init(&imu, datapoints_buf[0].accel, datapoints_buf[0].mag);

	for (int i = 1; i < datapoints; i++) {
		// cout << datapoints_buf[i].gyro[0] << "," << datapoints_buf[i].gyro[1] << "," << datapoints_buf[i].gyro[2] << endl;
		imu_update(&imu, datapoints_buf[i].gyro, datapoints_buf[i].accel, datapoints_buf[i].mag);

		#ifdef PLOT
		plot.add_point(imu.ekf.state, "EKF");
		#endif
	}

	imu_deinit(&imu);

	#ifdef PLOT
	plot.plot("Data Recording Test");
	#endif

	return 0;
}

static void read_file() {
	ifstream file(record_dir + filename);

	string line;
	int line_count = 0;
	int nums_index = 0;
	int line_index = 0;
	while (getline(file, line)) {
		string nums_str[9];

		nums_index = 0;
		line_index = 0;

		for (auto &ch : line) {
			if (ch == ',') {
				nums_index++;
				line_index = 0;
			} else {
				if (line_index < 10) {
					nums_str[nums_index][line_index] = ch;
					line_index++;
				}
			}
		}

		for (int i = 0; i < 3; i++) {
			datapoints_buf[line_count].gyro[i] = atof(nums_str[i].c_str());
		}
		for (int i = 0; i < 3; i++) {
			datapoints_buf[line_count].accel[i] = atof(nums_str[i + 3].c_str());
		}
		for (int i = 0; i < 3; i++) {
			datapoints_buf[line_count].mag[i] = atof(nums_str[i + 6].c_str());
		}

		line_count++;
	}

	datapoints = line_count;

	file.close();
}

