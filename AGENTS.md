# kinetic - Agent Guidance

## Project Overview
C/C++ sensor fusion library implementing EKF (extended Kalman filter) for inertial measurement unit data.

## Required First Steps
1. **Fill source files**: `kin_ekf.c`, `kinetic.c`, and `kin_math.c` are empty stubs that need implementation matching the `.h` declarations.
2. **Create CMakeLists.txt**: Currently empty; must define targets, include paths (`inc/`), and linking rules for all source/headers.

## Directory Structure
- `src/`: Main .c files (currently empty - needs population)
- `inc/`: Header files with public API declarations
- `build/`: Generated CMake output directory (empty)

## Key Headers to Implement
- `kinetic.h`: Main struct definition (currently just typedef placeholder)
- `kin_math.h`: Matrix operations for Kalman filter state estimation
- `kin_ekf.h`: EKF implementation interface

## Typical Build Commands
```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

Then test or deploy the resulting library/executable.
