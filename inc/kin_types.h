#ifndef KIN_TYPES_H
#define KIN_TYPES_H

#include <sys/types.h>
#include <stdint.h>

/* Quaternion/vector axis identifiers - available for both C and C++ */
#ifndef X
#define X 0
#endif
#ifndef Y
#define Y 1
#endif
#ifndef Z
#define Z 2
#endif
#ifndef W
#define W 3
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Matrix structure for linear algebra operations.
 *
 * Stores a 2D matrix of float values in row-major order.
 */
typedef struct {
	uint8_t rows;  ///< Number of rows in the matrix
	uint8_t cols;  ///< Number of columns in the matrix
	float *data;  ///< Pointer to contiguous array of size rows * cols
} matrix_t;

/**
 * @brief Extended Kalman Filter state structure.
 *
 * Holds the EKF state vector and covariance matrix.
 */
typedef struct {
	matrix_t *state;  ///< State vector X (typically 4x1 for quaternion)
	matrix_t *covariance;  ///< Covariance matrix P
} ekf_t;

/**
 * @brief IMU structure containing EKF and sensor configuration.
 *
 * Central structure for IMU state estimation, noise parameters,
 * and reference frames.
 */
typedef struct {
	/** Sensor noise parameters (used for covariance matrices) */
	float gyro_noise;  ///< Gyroscope noise variance
	float accel_noise;  ///< Accelerometer noise variance
	float mag_noise;  ///< Magnetometer noise variance

	/** Magnetic field configuration */
	float mag_dip;  ///< Magnetic dip angle (for reference frame computation)
	float mag_dec;  ///< Magnetic declination (unused, retained for future use)

	/** System timing */
	float dt;  ///< Time step for EKF integration

	/** Coordinate system configuration */
	bool enu;  ///< true for ENU (East-North-Up), false for NED (North-East-Down)

	/** Extended Kalman Filter instance */
	ekf_t ekf;

	/** Reference frames for measurement models */
	matrix_t *m_ref;  ///< Normalized magnetic reference vector
	matrix_t *g_ref;  ///< Gravity reference vector [0, 0, 1] in NED

	/** Noise covariance matrices */
	matrix_t *proc_noise;  ///< Process noise covariance Q (3x3)
	matrix_t *meas_noise;  ///< Measurement noise covariance R (6x6)
} imu_t;

#ifdef __cplusplus
}
#endif

#endif
