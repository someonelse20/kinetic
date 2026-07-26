cmake_minimum_required(VERSION 3.20)
project(Kinetic VERSION 1.0.0 LANGUAGES CXX C)

# Use system Python 3.14 for pybind11 (latest available)
set(PYTHON_EXECUTABLE "/usr/bin/python" CACHE STRING "Python executable for pybind11")
find_package(Python COMPONENTS Interpreter REQUIRED)
message(STATUS "Using Python ${Python_VERSION} at ${PYTHON_EXECUTABLE}")

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/inc)

# Kinetic library (core C code)
add_library(kinetic STATIC src/kinetic.c)
target_include_directories(kinetic PUBLIC ${CMAKE_SOURCE_DIR}/inc)

# Set compiler flags for optimization and pybind11 compatibility  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")

#==============================================================================
# GNUPLOT-BASED PLOTTING MODULE (pure C++, no Python required!)
#==============================================================================

# Plotting library (CSV export, SVG generation via gnuplot)
add_library(plot STATIC
    src/plot.cpp           # CSV export + SVG generation
    src/plot_view.cpp      # Browser opener + HTML viewer
    src/plot_browse.cpp    # Cross-platform browser launcher
)
target_include_directories(plot PUBLIC ${CMAKE_SOURCE_DIR}/tests/inc)

# Find gnuplot if available on system, make optional
find_program(GNUPLOT_EXECUTABLE gnuplot QUIET)
if(GNUPLOT_EXECUTABLE)
    message(STATUS "Gnuplot found: ${GNUPLOT_EXECUTABLE}")
    target_include_directories(plot PUBLIC ${CMAKE_SOURCE_DIR}/tests/src)
else()
    message(STATUS "WARNING: Gnuplot not found - SVG generation will use embedded methods")
endif()

# Install plot files (optional, for distribution)
install(DIRECTORY src/ DESTINATION include/kinetic_tests FILES_MATCHING CONFIGURE_MODE)

#==============================================================================
# TEST TARGETS
#==============================================================================

add_executable(kinetic_math_test tests/math/main.cpp)
target_link_libraries(kinetic_math_test PRIVATE kinetic)

# Simple gnuplot test (standalone, no pybind11 dependency)
add_executable(plot_quick_test tests/src/plot_gnuplot.cpp)
target_link_libraries(plot_quick_test PRIVATE plot m)

#==============================================================================
# OPTIONAL: Python-interop test for embedding (requires system libpython)
#==============================================================================
option(ENABLE_PYTHON_INTEROP "Enable Python interop (pybind11)" OFF)

if(ENABLE_PYTHON_INTEROP AND PYTHON_EXECUTABLE MATCHES "python3\\.14")
    # Note: Requires system libpython shared library (python3.14m.so)
    # For pybind11 embedding without dev packages, use workaround: comment out below
    # message(STATUS "Python interop enabled - requires libpython3.14m.so")
    # add_library(Python::Module IMPORTED INTERFACE SYSTEM_INCLUDE_DIRS "/usr/include/python3.14")
endif()
