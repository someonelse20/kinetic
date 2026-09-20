#include <pybind11/pybind11.h>

#include "kin_imu.h"
#include "kin_math.h"

namespace py = pybind11;

PYBIND11_MODULE(kin_wrapper, m, py::mod_gil_not_used()) {
    // Matrix type bindings
    py::class_<matrix_t>(m, "matrix_t")
        .def(py::init<>())
        .def_readwrite("rows", &matrix_t::rows)
        .def_readwrite("cols", &matrix_t::cols)
        .def_readwrite("data", &matrix_t::data)
        .def("__len__", [](const matrix_t& m) { return m.rows * m.cols; })
        .def("__getitem__", [](const matrix_t& m, size_t i) { return m.data[i]; })
        .def("__setitem__", [](matrix_t& m, size_t i, float val) { m.data[i] = val; });

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
    m.def("imu_init", &imu_init);
    m.def("imu_deinit", &imu_deinit);
    m.def("imu_update", &imu_update);
	/*
	*/

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
