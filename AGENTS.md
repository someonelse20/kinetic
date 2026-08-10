# kinetic - Agent Guidance

## Project Overview

C/C++ sensor fusion library implementing EKF (extended Kalman filter) for IMU data. Uses CMake build system.

## Architecture

- **Core**: `src/` - C implementations of EKF, kinematic model, matrix operations
- **Headers**: `inc/` - Public API declarations (kinetic.h, kin_math.h, kin_ekf.h)
- **Tests**: `tests/` - C unit tests for kinetic math, C++ simulations

## Build Commands

```bash
cd build
make
```

Build artifacts go to `build/`; omit from version control.

## Implementation Status

### Core Files

- **kin_math.c** (427 lines, ~12KB): Fully implemented matrix and quaternion operations
  - Matrix ops: init/free/copy/fill/arr_to_matrix/print, add/sub/mul, scale/transpose/identity
  - Determinant/inverse via adjugate transpose
  - Quaternion ops: mul, quat_to_euler (XYZ, deg output), quat_to_rot_matrix, rot_matrix_to_quat
  - Vector ops: dot product, cross product (mul_vector)
  - Utilities: sgn(), normalize_matrix, skew_symm_matrix, euler_to_quat
  - Missing: `matrix_eigen()` (stub), `normalize_matrix()` uses L2 norm (not unit quaternion norm)

- **kinetic.c** (299 lines, ~10KB): EKF state estimation and IMU sensor updates
  - `init_state()`: Computes initial quaternion from accel/mag vectors, sets NED reference frames
  - `update_imu()`: Full EKF cycle with prediction + correction for IMU data
  - Prediction: `state_prediction()`, `state_transition()`, `process_noise_covariance()`, `pred_covariance()`
  - Correction: `measurement_model()`, `measurement_model_jacob()`, `measurement_noise_cov()`
  - State: quaternion only (`kinetic->state_q`), covariance initialized as 4x4 identity

- **kin_ekf.c** (80 lines, ~3KB): Partially implemented EKF interface
  - `ekf_init()`: Copies state and covariance (handles free-on-args)
  - `ekf_update()`: Mostly commented-out; 4 EKF steps defined but not executed
  - Expects: `state_pred`, `state_pred_jacob`, `obsv_model`, `obsv_model_jacob`, `proc_noise`, `meas_noise`
  - Currently prints debug output but doesn't return updated state/covariance

- **src/**: C implementations of EKF, kinematic model, matrix/quat operations
- **inc/**: Public API (kinetic.h, kin_math.h, kin_ekf.h)
- **tests/**: C unit tests, C++ simulations
- **EKF math**: Based on AHRS EKF documentation (https://ahrs.readthedocs.io/en/latest/filters/ekf.html)

### Build Commands

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

Artifacts go to `build/`; omit from version control.

## Kinetic Structure

```c
typedef struct {
    matrix_t state_q;        // 4x1 quaternion (XYZW)
    matrix_t estim_covariance; // 4x4 covariance
    matrix_t g_ref;          // 3x1 gravity reference (NED: 0,0,1)
    matrix_t m_ref;          // 3x1 magnetometer reference (incl. mag_dip)
    float gyro_noise;
    float accel_noise;
    float mag_noise;
    float mag_dip;
} kinetic_t;
```

## Critical Behaviors and Patterns

### Simulation Behavior (tests/src/sim.cpp)

**Linear Interpolation Mode**: Uses `linear_interpolation()` with zero external noise inputs:

- Gyro data is interpolated between two rotations via quaternion math: `rate_of_change_q = quat_multiply(start_rot, end_rot)`
- Acceleration is computed as reference gravity rotated by current orientation: `get_accel(q) = R^T * [0, 0, 1]^T`
- Magnetometer is computed with no magnetic dip (or uses kinetic's mag_dip if available): `get_mag(q) = R^T * [cos(0), 0, sin(0)]^T`
- **Zero randomness**: No noise injected in get_accel() or get_mag() during linear interpolation - this is deterministic test mode

**Expected Measurements vs Actual Data**: The simulation generates "expected" measurements based on interpolated orientation but does NOT add sensor noise. When kinetic's EKF uses these measurements, it should converge to the true state if the measurement model is initialized correctly.

### EKF State Initialization Pattern (src/kinetic.c)

**init_state() initializes from accelerometer and magnetometer**:

```c
void init_state(kinetic_t *kinetic, float accel[3], float mag[3]) {
    // Computes orientation quaternion purely from acceleration and magnetometer vectors
    kinetic->state_q = rot_matrix_to_quat(rot_matrix);  // Line ~135

    // Reference frames initialized in NED format (Line ~142-145)
    kinetic->g_ref = arr_to_matrix({0, 0, 1}, 3, 1);
    kinetic->m_ref = scale_matrix(arr_to_matrix(cos(mag_dip), 0, sin(mag_dip)), ...);
}
```

**Key bug pattern identified**: During first measurement update, `init_state()` calculates the initial state from acceleration and magnetometer vectors but does NOT initialize other state variables (like gyro bias). This can cause divergence on subsequent updates if:

- Measurement model is initialized AFTER init_state() instead of using actual sensor values
- Magnetometer reference in get_mag() uses wrong function call or hardcoded value

### Measurement Model Location (src/kinetic.c ~line 57)

**update_imu() measurement residual calculation**:

```c
meas_residual = meas_model - meas_model_jacob * state_pred;
```

where `meas_model` contains both acceleration and magnetometer reference values from kinetic->accel_ref and kinetic->mag_ref. This must be populated BEFORE calling update_imu(), typically during init_state().

### Quaternion Operations (src/kin_math.c)

**Available operations**:

- `quat_to_euler(q)` - Convert quaternion to Euler angles (XYZ order, deg output)
- `quat_to_rot_matrix(q)` - 3x3 rotation matrix from quaternion
- `rot_matrix_to_quat(R)` - Quaternion from 3x3 rotation matrix
- `mul_quat(q1, q2)` - Quaternion multiplication
- `scale_matrix(M, s)` - Scale matrix by scalar
- `normalize_matrix(M)` - Normalize to prevent numerical drift

### Matrix Operations (src/kin_math.c)

**All operations**:

- `add_matrix(a, b)`, `sub_matrix(a, b)`
- `mul_matrix(A, B)` - Matrix multiplication
- `trans_matrix(M)` - Transpose
- `inv_matrix(M)` - Inverse
- `init_matrix(rows, cols)` - Zero-initialized matrix
- `ident_matrix(n)` - Identity matrix
- `arr_to_matrix(data, rows, cols)` - Fill from array
- `scale_matrix(M, s)` - Scale all elements

### Test Framework (tests/)

**Simulation types**:

1. `sim_t::tick()` - Continuous loop mode with random gyro inputs
2. `sim_t::linear_interpolation()` - Deterministic interpolation between orientations
3. `sim_t::loop()` - Generic update callback pattern

**Build for tests**:

```bash
mkdir -p kinetic/build && cd kinetic/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

### Common Pitfalls

1. **Measurement model not initialized**: Ensure `kinetic->accel_ref` and `kinetic->mag_ref` are set via init_state() BEFORE first update_imu() call, not after.

2. **Magnetometer reference mismatch**: get_mag() in simulation may use hardcoded `mag_dip=0` instead of kinetic's mag_dip field - causes divergence if magnetic dip varies between simulation and EKF.

3. **Zero-noise test mode**: During linear interpolation, no noise is added to measurements. This tests perfect sensor convergence but doesn't represent real IMU behavior. Add noise injection for realistic testing.

4. **Euler angle conversion**: Always check output order - quat_to_euler() uses XYZ Euler sequence in degrees.

### Debugging Tips

**Trace measurement model initialization**:

```bash
mkdir -p kinetic/build && cd kinetic/build
cmake .. -DCMAKE_BUILD_TYPE=Release
./tests  # Watch initial state and measurement model divergence
```

**Verify quaternion consistency**: After every operation, normalize matrices with `normalize_matrix()` to prevent numerical drift.

- EKF math is based on but not constrained to https://ahrs.readthedocs.io/en/latest/filters/ekf.html
