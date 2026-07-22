cmake_minimum_required(VERSION 3.14)
project(kinetic VERSION 0.0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Library source files
set(KINETIC_SOURCES
    ${CMAKE_SOURCE_DIR}/src/kin_ekf.c
    ${CMAKE_SOURCE_DIR}/src/kinetic.c
    ${CMAKE_SOURCE_DIR}/src/kin_math.c
)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/tests/inc)

# Library target
add_library(kinetic STATIC ${KINETIC_SOURCES})
target_include_directories(kinetic PUBLIC ${CMAKE_SOURCE_DIR}/inc ${CMAKE_SOURCE_DIR}/tests/inc)
target_link_libraries(kinetic PUBLIC m)

set_target_properties(kinetic PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# Static object file for sim (added individually if needed)
add_library(sim_objects STATIC tests/src/sim.cpp)
target_include_directories(sim_objects PUBLIC ${CMAKE_SOURCE_DIR}/inc ${CMAKE_SOURCE_DIR}/tests/inc)
target_link_libraries(sim_objects PRIVATE kinetic)

# Executable targets - main (C++ simulation test)
add_executable(main tests/main.cpp)
target_link_libraries(main PRIVATE kinetic sim_objects pybind11::embed)

# Executable target - matrix (C math unit tests, only links to kin_math.c from kinetic lib)
add_executable(matrix tests/matrix.cpp)
target_link_libraries(matrix PRIVATE kinetic)

