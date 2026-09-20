cmake_minimum_required(VERSION 3.13)

project(kinetic_demo LANGUAGES C CXX)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(KINETIC_PATH ..)
include(../kinetic_import.cmake)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(kin_wrapper kin_wrapper.cpp)
install(TARGETS kin_wrapper DESTINATION .)

add_executable(test kin_wrapper.cpp)
target_link_libraries(test PRIVATE kinetic)

