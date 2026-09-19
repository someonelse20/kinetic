cmake_minimum_required(VERSION 3.13)

project(kinetic_demo C CXX ASM)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(KINETIC_PATH ..)
include(../kinetic_import.cmake)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(kin_wrapper kin_wrapper.cpp)
install(TARGETS kin_wrapper DESTINATION .)
