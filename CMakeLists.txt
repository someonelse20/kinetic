cmake_minimum_required(VERSION 3.20)
project(Kinetic VERSION 1.0.0 LANGUAGES CXX C)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/inc)

# Kinetic library (core C code)
add_library(kinetic STATIC src/kin_math.c src/kinetic.c)
target_include_directories(kinetic PUBLIC ${CMAKE_SOURCE_DIR}/inc)
target_link_libraries(kinetic PRIVATE m)

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

#==============================================================================
# GNUPLOT-BASED PLOTTING MODULE
#==============================================================================

# Plotting library (CSV export, SVG generation via gnuplot)
# add_library(plot STATIC
#     tests/src/plot.cpp           # CSV export + SVG generation
#     tests/src/plot_view.cpp      # Browser opener + HTML viewer
#     tests/src/plot_browse.cpp    # Cross-platform browser launcher
# )
# target_include_directories(plot PUBLIC ${CMAKE_SOURCE_DIR}/tests/inc)

# Find gnuplot if available on system, make optional
find_program(GNUPLOT_EXECUTABLE gnuplot QUIET)
if(GNUPLOT_EXECUTABLE)
    message(STATUS "Gnuplot found: ${GNUPLOT_EXECUTABLE}")
	# target_include_directories(plot PUBLIC ${CMAKE_SOURCE_DIR}/tests/src)
else()
    message(STATUS "WARNING: Gnuplot not found - SVG generation will use embedded methods")
endif()

# Install plot files (optional, for distribution)
# install(DIRECTORY src/ DESTINATION include/kinetic_tests)

#==============================================================================
# TEST TARGETS
#==============================================================================

add_executable(tests tests/main.cpp)
target_include_directories(tests PRIVATE ${CMAKE_SOURCE_DIR}/tests/inc)
target_link_libraries(tests PRIVATE kinetic)

# Link sim.cpp with tests
target_sources(tests PRIVATE tests/src/sim.cpp tests/src/plot.cpp)

add_executable(matrix tests/matrix.cpp)
target_include_directories(matrix PRIVATE ${CMAKE_SOURCE_DIR}/tests/inc)
target_link_libraries(matrix PRIVATE kinetic)

