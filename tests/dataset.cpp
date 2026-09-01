#include <functional>
#include <iostream>
#include <fstream>
#include <cmath>

#include "kin_imu.h"
#include "kin_math.h"
#include "kin_types.h"
#include "plot.h"

#define PLOT

using namespace std;

typedef struct {
	float gyro[3];
	float accel[3];
	float mag[3];
} datapoint_t;

const static string record_dir = "../tests/recordings/";
const static string filename = "phone.csv";
const static string ref_filename = "phone_orientation.csv";

// Stores up to 2000 lines.
datapoint_t datapoints_buf[2000];
int datapoints = 0;

matrix_t *ref_datapoints_buf[3000];
int ref_datapoints = 0;

static void read_file();
static void read_refrence_file();
static void free_ref_datapoints_buf();
static matrix_t *get_accel(matrix_t *orientation);
static matrix_t *get_mag(matrix_t *orientation, float mag_dip);

int main() {
	read_file();
	read_refrence_file();

	#ifdef PLOT
	plot_t plot;
	#endif

	imu_t imu;
	// imu.mag_dip = deg_to_rad(54.7);
	imu.mag_dip = 54.7;
	// imu.mag_dip = 0.000001;
	imu.gyro_noise = 0.3;
	imu.accel_noise = 0.5;
	imu.mag_noise = 0.8;
	// imu.dt = 0.01;
	imu.dt = 0.0717;
	// imu.dt = 0.071693;

	float gyro_bias_a[] = {
		/*
		   3.5890706303030306,
		   0.19436924,
		   3.469256772121212
		 */
		3.4,
		-0.9,
		0.05
	};

	float gyro_sensitivity_a[] = {
		0.8409686965306123,
		0.8787611643169447,
		0.875307229757525
		/*
		 */
	};

	matrix_t *gyro_bias = arr_to_matrix(gyro_bias_a, 3, 1);
	matrix_t *gyro_sensitivity = arr_to_matrix(gyro_sensitivity_a, 3, 1);
	matrix_t *gyro_alignment = ident_matrix(3);
	/*
	   matrix_t *gyro_bias = fill_matrix(3, 1, 0.f);
	   matrix_t *gyro_sensitivity = fill_matrix(3, 1, 1.f);
	 */

	imu_init(&imu, datapoints_buf[0].accel, datapoints_buf[0].mag);

	/*
	cout << "========== Init Kinetic State ==========" << endl;
	print_matrix(quat_to_euler(imu.ekf.state));
	cout << endl;

	cout << "========== Init Refrence State ==========" << endl;
	print_matrix(ref_datapoints_buf[0]);
	*/

	for (int i = 1; i < datapoints; i++) {
		matrix_t *gyro_m = arr_to_matrix(datapoints_buf[i].gyro, 3, 1);
		// calibrate_gyro_accel(gyro_m, gyro_alignment, gyro_sensitivity, gyro_bias);
		// cout << gyro_m->data[X] << ", " << gyro_m->data[Y] << ", " << gyro_m->data[Z] << endl;
		matrix_t *accel_m = arr_to_matrix(datapoints_buf[i].accel, 3, 1);
		matrix_t *mag_m = arr_to_matrix(datapoints_buf[i].mag, 3, 1);
		enu_to_ned(accel_m);
		// enu_to_ned(mag_m);

		float gyro[3];
		float mag[3];
		for (int j = 0; j < 3; j++) {
			gyro[j] = deg_to_rad(gyro_m->data[j]);
			// gyro[j] = gyro_m->data[j];
			// mag[j] = mag_m->data[j];
			mag[j] = datapoints_buf[i].mag[j];
		}
		// cout << datapoints_buf[i].gyro[0] << "," << datapoints_buf[i].gyro[1] << "," << datapoints_buf[i].gyro[2] << endl;
		imu_update(&imu, gyro, accel_m->data, mag_m->data);

		matrix_t *refrence = copy_matrix(ref_datapoints_buf[i]);
		// enu_to_ned(refrence);

		cout << endl << "========== Kinetic State ==========" << endl;
		print_matrix(quat_to_euler(imu.ekf.state));

		cout << endl << "========== True State ==========" << endl;
		print_matrix(refrence);

		/*
		cout << endl << "========== True Accel ==========" << endl;
		normalize_matrix(accel_m);
		print_matrix(accel_m);

		cout << endl << "========== Expected accel ==========" << endl;
		print_matrix(get_accel(euler_to_quat(refrence)));
		*/

		/*
		cout << endl << "========== True Mag ==========" << endl;
		matrix_t *mag_m = arr_to_matrix(mag, 3, 1);
		normalize_matrix(mag_m);
		print_matrix(mag_m);

		cout << endl << "========== Expected Mag ==========" << endl;
		print_matrix(get_mag(euler_to_quat(ref_datapoints_buf[i]), imu.mag_dip));
		*/

		#ifdef PLOT
		plot.add_point(quat_to_euler(imu.ekf.state), "EKF");
		plot.add_point(refrence);
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

static void read_refrence_file() {
	ifstream file(record_dir + ref_filename);

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

		float datapoint[3];

		for (int i = 0; i < 3; i++) {
			datapoint[i] = atof(nums_str[i].c_str());
		}

		// cout << line_count << endl;
		ref_datapoints_buf[line_count] = arr_to_matrix(datapoint, 3, 1);

		line_count++;
	}

	ref_datapoints = line_count;

	file.close();
}

static void free_ref_datapoints_buf() {

}

static matrix_t *get_accel(matrix_t *orientation) {
	float g_ref_a[] = {0.f, 0.f, 1.f};
	matrix_t *g_ref_m = arr_to_matrix(g_ref_a, 3, 1);

	matrix_t *rot_matrix = quat_to_rot_matrix(orientation);
	matrix_t *ret = mul_matrix_free(trans_matrix_free(rot_matrix), g_ref_m);

	free_matrix(g_ref_m);
	return ret;
	/*
	   float g_ref_a[] = {0, 0, 1};
	   matrix_t *g_ref_m = arr_to_matrix(g_ref_a, 3, 1);

	   return mul_matrix(trans_matrix(quat_to_rot_matrix(orientation)), g_ref_m);
	 */
}

static matrix_t *get_mag(matrix_t *orientation, float mag_dip) {
	float m_ref_a[] = {cos(mag_dip), 0, sin(mag_dip)};
	matrix_t *m_ref_m = arr_to_matrix(m_ref_a, 3, 1);
	m_ref_m = scale_matrix_free(m_ref_m, 1 / (sqrt(pow(cos(mag_dip), 2) + pow(sin(mag_dip), 2))));

	matrix_t *rot_matrix = quat_to_rot_matrix(orientation);
	matrix_t *ret = mul_matrix_free(trans_matrix_free(rot_matrix), m_ref_m);

	free_matrix(m_ref_m);
	return ret;
	/*
	 */

	/*
	   float m_ref_a[] = {cos(mag_dip), 0, sin(mag_dip)};
	   matrix_t *m_ref_m = scale_matrix(arr_to_matrix(m_ref_a, 3, 1), 1 / (sqrt(pow(cos(mag_dip), 2) + pow(sin(mag_dip), 2))));

	   return mul_matrix(trans_matrix(quat_to_rot_matrix(orientation)), m_ref_m);
	 */
}

