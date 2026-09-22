#include <cstdlib>
#include <pybind11/pybind11.h>

#include "kin_imu.h"
#include "kin_math.h"
#include "kin_types.h"
#include "sim.h"

namespace py = pybind11;

imu_t *init_imu(bool enu, float mag_dip, float gyro_noise, float accel_noise, float mag_noise) {
	imu_t *imu = (imu_t *)malloc(sizeof(imu_t));
	imu->enu = enu;
	imu->mag_dip = mag_dip;
	imu->gyro_noise = gyro_noise;
	imu->accel_noise = accel_noise;
	imu->mag_noise = mag_noise;

	return imu;
}

PYBIND11_MODULE(kin_wrapper, m, py::mod_gil_not_used()) {
    // Matrix type bindings
    py::class_<matrix_t>(m, "matrix_t")
        .def(py::init<>())
        .def_readwrite("rows", &matrix_t::rows)
        .def_readwrite("cols", &matrix_t::cols)
        .def_readwrite("data", &matrix_t::data)
        .def("len", [](const matrix_t& m) { return m.rows * m.cols; })
        .def("getData", [](const matrix_t& m) { return m.data; })
        .def("getItem", [](const matrix_t& m, size_t i) { return m.data[i]; })
        .def("setItem", [](matrix_t& m, size_t i, float val) { m.data[i] = val; });

    // EKF type bindings
    py::class_<ekf_t>(m, "ekf_t")
        .def(py::init<>())
        .def_readwrite("state", &ekf_t::state)
        .def_readwrite("covariance", &ekf_t::covariance);

    // IMU type bindings
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

    // Core IMU/EKF functions - pybind11 handles array conversion automatically
	/*
    m.def("imu_init", &imu_init);
    m.def("imu_deinit", &imu_deinit);
    m.def("imu_update", &imu_update);
	*/
    m.def("imu_init", [](imu_t *imu, float ax, float ay, float az, float mx, float my, float mz) {
			float accel[] = {ax, ay, az};
			float mag[] = {mx, my, mz};
			return py::cast((matrix_t*)imu_init(imu, accel, mag));
			});
    m.def("imu_deinit", &imu_deinit);
	m.def("imu_update", [](imu_t *imu, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
			float gyro[] = {gx, gy, gz};
			float accel[] = {ax, ay, az};
			float mag[] = {mx, my, mz};
			return py::cast((matrix_t*)imu_update(imu, gyro, accel, mag));
			});


	// Test utilities
	m.def("get_gyro", &get_gyro);
	m.def("get_accel", &get_accel);
	m.def("get_mag", &get_mag);
	m.def("init_imu", &init_imu);

    // Matrix utilities
    m.def("init_matrix", [](int rows, int cols) {
        return py::cast(init_matrix(rows, cols));
    });
    m.def("free_matrix", &free_matrix);

    // Quaternion utilities
    m.def("quat_to_euler", [](const matrix_t& m) {
        return py::cast((matrix_t*)quat_to_euler((matrix_t*)&m));
    });
    m.def("euler_to_quat", [](const matrix_t& m) {
        return py::cast(euler_to_quat((matrix_t*)&m));
    });
    m.def("normalize_quaternion", [](const matrix_t& m) {
        return py::cast((int)normalize_matrix((matrix_t*)&m));
    });
    m.def("inv_quaternion", [](const matrix_t& m) {
        return py::cast(inv_quat((matrix_t*)&m));
    });
    m.def("quat_multiply", [](const matrix_t& a, const matrix_t& b) {
        return py::cast(quat_prod((const matrix_t*)&a, (const matrix_t*)&b));
    });

    m.doc() = "pybind11 kinetic wrapper";
}
