cmake_minimum_required(VERSION 3.13)

project(kinetic_demo LANGUAGES C CXX)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(KINETIC_PATH ..)
include_directories(${KINETIC_PATH}/inc)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(kin_wrapper MODULE kin_wrapper.cpp ${KINETIC_PATH}/src/kin_error.c ${KINETIC_PATH}/src/kin_math.c ${KINETIC_PATH}/src/kin_ekf.c ${KINETIC_PATH}/src/kin_imu.c)
install(TARGETS kin_wrapper DESTINATION .)

