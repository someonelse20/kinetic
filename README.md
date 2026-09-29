# Kinetic
Sensor fusion library that provides an Extended Kalman Filter (EKF) implementation for IMU data processing. It fuses accelerometer, gyroscope, and magnetometer measurements to estimate orientation.

## Build
This project uses cmake to compile.

```bash
mkdir build && cd build
cmake .. && make
```

## Quickstart
Copy the repository into your project
```bash
git clone https://github.com/someonelse20/kinetic.git
```
Include kinetic_import.cmake and set KINETIC_PATH
```cmake
set(KINETIC_PATH kinetic)
include(kinetic/kinetic_import.cmake)
```

## Usage
The primary API is exposed through kin_imu.h
```c
#include "kin_imu.h"

// Initialize IMU
imu_t imu;
float accel[3] = {0, 0, 1};
float mag[3] = {0, 0, 0};
imu_init(&imu, accel, mag);

// Update with new measurements
imu_update(&imu, new_gyro, new_accel, new_mag);

// Get euler angle output
matrix_t *euler = quat_to_euler(imu.ekf.state);

// Convert output to array
float *euler_arr = matrix_to_arr(euler);
```

## Architecture
Kinetic uses quaternions in x, y, z, w format (little-endian convention).

- kin_imu.h - Primary IMU API (imu_init, imu_update)
- kin_ekf.h - EKF core functions
- kin_types.h - Data structures
- kin_math.h - Matrix operations
- kin_error.h - Error codes
