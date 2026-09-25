#ifndef KIN_IMU_H
#define KIN_IMU_H

/** @file
 * @brief Kinetic IMU Sensor Fusion Library
 * 
 * This library provides an Extended Kalman Filter (EKF) implementation for
 * IMU data processing. It fuses accelerometer, gyroscope, and magnetometer
 * measurements to estimate orientation (roll, pitch, yaw).
 * 
 * @mainpage Kinetic Documentation
 * 
 * Kinetic is a C/C++ sensor fusion library implementing an Extended Kalman
 * Filter (EKF) for IMU data. It uses CMake as its build system and includes
 * tools for calibration and testing.
 * 
 * @section quickstart Quick Start
 * 
 * The primary API is exposed through kin_imu.h:
 * 
 * @code
 * #include "kin_imu.h"
 * 
 * // Initialize IMU
 * imu_t imu;
 * float accel[3] = {0, 0, 9.81};
 * float mag[3] = {0, 0, 0};
 * imu_init(&imu, accel, mag);
 * 
 * // Update with new measurements
 * imu_update(&imu, new_gyro, new_accel, new_mag);
 * 
 * // Get euler angle output
 * matrix_t *euler = quat_to_euler(imu.ekf.state);
 *
 * // Convert output to array
 * float *euler_arr = matrix_to_arr(euler);
 * @endcode
 * 
 * @section architecture Architecture
 * 
 * The library uses NED (North-East-Down) reference frames with quaternions
 * in x, y, z, w format (little-endian convention). The EKF state includes:
 * - Quaternion for orientation
 * - Bias corrections
 * - Covariance matrix
 * 
 * @section files Header Files
 * 
 * - kin_imu.h - Primary IMU API (imu_init, imu_update)
 * - kin_ekf.h - EKF core functions
 * - kin_types.h - Data structures
 * - kin_math.h - Matrix operations
 * - kin_error.h - Error codes
 */

#include "kin_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize IMU EKF from accelerometer and magnetometer readings.
 *
 * Computes initial orientation using accelerometer/magnetometer fusion,
 * initializes reference frames (g_ref, m_ref) based on magnetic dip angle,
 * and sets up noise covariance matrices.
 *
 * @param imu Pointer to imu_t structure to initialize (must be allocated)
 * @param accel 3-element gravity vector [ax, ay, az]
 * @param mag   3-element magnetic field vector [mx, my, mz]
 * @return EKF state quaternion, or NULL on failure
 */
matrix_t *imu_init(imu_t *imu, float *accel, float *mag);

/**
 * @brief Deinitialize IMU and free all allocated resources.
 *
 * Frees EKF state, reference frames, and noise covariance matrices.
 * Safe to call multiple times (idempotent).
 *
 * @param imu Pointer to imu_t structure to deinitialize
 * @return 0 on success, non-zero on error
 */
uint8_t imu_deinit(imu_t *imu);

/**
 * @brief Perform one EKF update step with IMU and sensor measurements.
 *
 * Updates the EKF state using:
 * - Gyro integration for state prediction (delta-angle method)
 * - Accelerometer and magnetometer for measurement correction
 *
 * @param imu Pointer to imu_t structure
 * @param gyro  3-element gyro reading [wx, wy, wz] in rad/s
 * @param accel 3-element accelerometer reading [ax, ay, az] in m/s²
 * @param mag   3-element magnetometer reading [mx, my, mz] in μT
 * @return EKF state quaternion, or NULL on failure
 */
matrix_t *imu_update(imu_t *imu, float *gyro, float *accel, float *mag);

/**
 * @brief Calibrate IMU using reference measurements.
 *
 * Applies sensitivity, alignment, and bias corrections to raw IMU data.
 *
 * @param value       Raw measurement vector to calibrate
 * @param alignment   Alignment matrix (3x3)
 * @param sensitivity Sensitivity matrix (3x3, diagonal)
 * @param bias        Bias vector (3x1)
 * @return 0 on success, non-zero on error
 */
uint8_t calibrate_gyro_accel(matrix_t *value, matrix_t *alignment, matrix_t *sensitivity, matrix_t *bias);

/**
 * @brief Calibrate magnetometer using soft and hard iron compensation.
 *
 * Corrects for magnetic field distortions from nearby ferromagnetic materials.
 *
 * @param value    Raw magnetometer measurement (3x1)
 * @param soft_iorn Soft iron correction matrix (3x3)
 * @param hard_iorn Hard iron offset vector (3x1)
 * @return 0 on success, non-zero on error
 */
uint8_t calibrate_mag(matrix_t *value, matrix_t *soft_iorn, matrix_t *hard_iorn);

#ifdef __cplusplus
}
#endif

#endif
