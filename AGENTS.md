# kinetic

C/C++ sensor fusion library implementing EKF (extended Kalman filter) for IMU data. Uses CMake build system.

## Build

```bash
mkdir -p build && cd build && cmake ..
cd .. && ./build/tests      # Run linear interpolation test
cd .. && ./build/matrix     # Run matrix operations test
```

Build artifacts go to `build/`; omit from version control.

## Architecture

**EKF math**: Based on AHRS EKF documentation (https://ahrs.readthedocs.io/en/latest/filters/ekf.html)

**Coordinate system**:
- Quaternion format: x y z w (little-endian convention)
- Reference frames: NED (North-East-Down)

**Reference frames**:
- `accel_ref`: Always [0, 0, 1] (gravity in NED)
- `mag_ref`: Depends on magnetic dip angle, computed as `[cos(dip), 0, sin(dip)]`
- `g_ref`: Always [0, 0, 1] (gravity direction)
- `m_ref`: Depends on magnetic dip angle, normalized

## Testing

Two primary test modes:

1. **Linear interpolation** (`tests/main.cpp`, `tests/src/sim.cpp`):
   - Interpolates between two orientations over time
   - Runs EKF updates on interpolated "true" orientation
   - Uses zero-noise deterministic mode: no noise injected during interpolation
   - Outputs CSV data to `build/estm_*.txt` and `build/true_*.txt`

2. **Debug divergence** (`tests/debug_divergence.cpp`):
   - Prints diagnostic info about measurement model
   - Shows reference frame values before/after initialization
   - Useful for debugging EKF convergence behavior

**Zero-noise test mode**: During linear interpolation, no noise is added to measurements. This tests perfect sensor convergence but doesn't represent real IMU behavior.

**Default test**: `sim.all_axis_test(100)` runs all-axis test with 100 steps.

## Key Implementation Details

- `update_imu()`: Core EKF update function using `state_pred` (prediction step) instead of `prev_state` for rotation matrix computation
- `linear_interpolation()`: Creates synthetic IMU data by interpolating between orientations
- `init_state()`: Initializes EKF from raw accelerometer/magnetometer readings
- `get_accel()`: Computes expected accelerometer reading given current orientation and gravity vector
- `get_mag()`: Computes expected magnetometer reading given current orientation and magnetic dip angle
- `plot_t::add_point()` / `plot_t::plot()`: CSV export + SVG generation via gnuplot; outputs to `*_x_data.txt`, `*_y_data.txt`, `*_z_data.txt`

## Platform Notes

- Gnuplot optional for SVG generation in plotting module
- Link with `-lm` for math library
