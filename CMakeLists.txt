cmake_minimum_required(VERSION 3.10)
project(kinetic VERSION 0.0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Library source files
set(KINETIC_SOURCES
    ${CMAKE_SOURCE_DIR}/src/kin_ekf.c
    ${CMAKE_SOURCE_DIR}/src/kinetic.c
    ${CMAKE_SOURCE_DIR}/src/kin_math.c

    ${CMAKE_SOURCE_DIR}/tests/src/sim.cpp
)

include_directories(${CMAKE_SOURCE_DIR}/inc)

add_library(kinetic STATIC ${KINETIC_SOURCES})
target_include_directories(kinetic PUBLIC ${CMAKE_SOURCE_DIR}/inc ${CMAKE_SOURCE_DIR}/tests/inc)
target_link_libraries(kinetic PUBLIC m)

set_target_properties(kinetic PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# Test executables
add_executable(tests ${CMAKE_SOURCE_DIR}/tests/main.cpp)
target_link_libraries(tests PRIVATE kinetic)

add_executable(matrix_test ${CMAKE_SOURCE_DIR}/tests/matrix.cpp)
target_link_libraries(matrix_test PRIVATE kinetic)
