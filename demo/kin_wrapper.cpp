// #include <pybind11/pybind11.h>

#include "kin_imu.h"
#include "kin_math.h"

// namespace py = pybind11;

int add(int i, int j) {
	imu_t imu;
	float a[] = {0, 0, 0};
	float m[] = {0, 0, 0};
	imu_init(&imu, a, m);
	return i + j;
}

int main() {
	add(1, 2);
}

/*
PYBIND11_MODULE(kin_wrapper, m, py::mod_gil_not_used()) {
	py::class_<matrix_t>(m, "matrix_t")
		.def(py::init<>())
		.def_readwrite("rows", &matrix_t::rows)
		.def_readwrite("cols", &matrix_t::cols)
		.def_readwrite("data", &matrix_t::data);

	py::class_<ekf_t>(m, "ekf_t")
		.def(py::init<>())
		.def_readwrite("state", &ekf_t::state)
		.def_readwrite("covariance", &ekf_t::covariance);

	py::class_<imu_t>(m, "imu_t")
		.def(py::init<>())
		.def_readwrite("gyro_noise", &imu_t::gyro_noise)
		.def_readwrite("accel_noise", &imu_t::accel_noise)
		.def_readwrite("mag_noise", &imu_t::mag_noise)
		.def_readwrite("mag_dip", &imu_t::mag_dip)
		.def_readwrite("mag_dec", &imu_t::mag_dec)
		.def_readwrite("dt", &imu_t::dt)
		.def_readwrite("enu", &imu_t::enu)
		.def_readwrite("ekf", &imu_t::ekf)
		.def_readwrite("m_ref", &imu_t::m_ref)
		.def_readwrite("g_ref", &imu_t::g_ref)
		.def_readwrite("proc_noise", &imu_t::proc_noise)
		.def_readwrite("meas_noise", &imu_t::meas_noise);

    m.doc() = "pybind11 kinetic wrapper";

    m.def("add", &add, "A function that adds two numbers");

	m.def("imu_init", &imu_init);
	m.def("imu_deinit", &imu_deinit);
	m.def("imu_update", &imu_update);
}
*/
