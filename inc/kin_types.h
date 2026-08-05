#ifndef KIN_TYPES_H
#define KIN_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
	uint8_t rows;
	uint8_t cols;
	float *data;
} matrix_t;

// TODO: Move over function pointers to main function arguments.
//       Maybe use a struct to avoid unwieldy number of function args.
typedef struct {
	// Function pointers
	// Required
	matrix_t *(*state_trans_model)(matrix_t *prev_state, matrix_t *contrl_vector);
	matrix_t *(*observe_model)(matrix_t *);
	matrix_t *(*state_trans_model_jacob)(matrix_t *prev_state);
	matrix_t *(*observe_model_jacob)(matrix_t *state_pred);

	// Implementation specific
	matrix_t *(*contrl_input_model)(matrix_t *);
	matrix_t *(*contrl_vector)();

	// Output variables
	matrix_t *state;
	matrix_t *covariance;
} ekf_t;

typedef struct {
	float attitude_q[4];
	float attitude_e[3];

	float velocity[3];
	float velocity_total;

	float cords[2];
	float altitude;

	float gyro_noise;
	float accel_noise;
	float mag_noise;
	float mag_dip;

	matrix_t *state_q;

	matrix_t *g_ref;
	matrix_t *m_ref;
	matrix_t *estm_covariance;
} kinetic_t;

#ifdef __cplusplus
}
#endif

#endif
