# kinetic - Agent Guidance

## Project Overview
C/C++ sensor fusion library implementing EKF (extended Kalman filter) for IMU data. Uses CMake build system.

## Architecture
- **Core**: `src/` - C implementations of EKF, kinematic model, matrix operations
- **Headers**: `inc/` - Public API declarations (kinetic.h, kin_math.h, kin_ekf.h)
- **Tests**: `tests/` - C unit tests for kinetic math, C++ simulations

## Build Commands
```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```
Build artifacts go to `build/`; omit from version control.

## Implementation Status
- **kin_math.c**: Matrix operations (initially populated, see file)
- **kinetic.c**: EKF state estimation and sensor updates
- **kin_ekf.c**: Extended Kalman filter logic (currently empty stub)
- **tests/main.cpp**: Main driver; requires full library before building

## Notes
- Test suite in `tests/` uses C++ simulations and expects complete implementation
- Library statically linked against math functions (`target_link_libraries(kinetic PUBLIC m)`)
