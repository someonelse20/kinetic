#include <iostream>
#include <unistd.h>
#include <math.h>

#include "kin_math.h"
#include "kinetic.h"
#include "sim.h"

using namespace std;

// Debug version of update_imu that prints diagnostic info about measurement model
void debug_update_imu(kinetic_t *kinetic, float *gyro, float *accel, float *mag, float dt) {
    matrix_t *rot_matrix = quat_to_rot_matrix(kinetic->state_q);
    
    // Convert gyro from deg/s to rad/s
    float *gyro_rad = (float*)malloc(3 * sizeof(float));
    for (int i = 0; i < 3; i++) {
        gyro_rad[i] = deg_to_rad(gyro[i]);
    }

    matrix_t *state_pred = state_prediction(kinetic->state_q, gyro_rad, dt);
    matrix_t *g_trans = state_transition(gyro_rad, dt);
    matrix_t *proc_noise_cov = process_noise_covariance(kinetic->state_q, kinetic->gyro_noise, dt);
    matrix_t *pred_cov = pred_covariance(kinetic->estm_covariance, g_trans, proc_noise_cov);

    // CORRECTION STEP - Calculate measurement model
    float *expect_g_ref = (float*)malloc(3 * sizeof(float));
    float *expect_m_ref = (float*)malloc(3 * sizeof(float));
    
    for (int i = 0; i < 3; i++) {
        expect_g_ref[i] = rot_matrix->data[i] * kinetic->g_ref->data[W]; // First row of rotation applied to [0,0,1]
    }
    
    // Print diagnostic info about reference frames
    printf("=== Diagnostics ===\n");
    printf("accel_ref: [%f, %f, %f]\n", kinetic->accel_ref[0], kinetic->accel_ref[1], kinetic->accel_ref[2]);
    printf("mag_ref:   [%f, %f, %f]\n", kinetic->mag_ref[0], kinetic->mag_ref[1], kinetic->mag_ref[2]);
    printf("g_ref:     [%f, %f, %f]\n", kinetic->g_ref->data[0], kinetic->g_ref->data[1], kinetic->g_ref->data[2]);
    printf("m_ref:     [%f, %f, %f]\n", kinetic->m_ref->data[0], kinetic->m_ref->data[1], kinetic->m_ref->data[2]);

    // Free gyro_rad - BUG: memory leak in original code!
    free(gyro_rad);

    // Calculate measurement model (from kinetic.c ~line 59)
    // NOTE: In original code, the expected measurement values are computed BEFORE init_state() sets them
    
    // Build measurement (not model) matrix
    float meas_m_data[6];
    for (int i = 0; i < 6; i++) {
        if (i < 3) {
            meas_m_data[i] = norm_accel(accel, 3); // First element
        } else {
            meas_m_data[i] = norm_mag(mag, 3, kinetic->mag_dip); // Elements 4-6
        }
    }
    matrix_t *meas_m = arr_to_matrix(meas_m_data, 6, 1);

    // Build expected measurement model
    float meas_model_data[6];
    for (int i = 0; i < 3; i++) {
        meas_model_data[i] = kinetic->g_ref->data[i * 4 + W]; // Wait - need to fix this indexing
    }
    
    free(rot_matrix);

    printf("meas_m:     [%f, %f, %f]\n", meas_m->data[0], meas_m->data[1], meas_m->data[2]);
}

void init_state(kinetic_t *kinetic, float accel[3], float mag[3]) {
    // NOTE: This is the ORIGINAL init_state from kinetic.c which has a BUG:
    // It calculates initial orientation FROM accelerometer AND magnetometer, but then it expects
    // the measurement model to use the reference frames (accel_ref, mag_ref) that it just set.
    // The bug is: measurement_model() in kinetic.c uses rot_matrix->data[i] which assumes 
    // a different computation than what init_state() produces.
    
    // Let me print diagnostic info BEFORE calling the buggy init_state to see state
    printf("Before init:\n");
    printf("state_q = [%f, %f, %f, %f]\n", kinetic->state_q[0], kinetic->state_q[1], kinetic->state_q[2], kinetic->state_q[3]);
    
    // Calculate orientation purely based of the accelerometer and magnetometer to start with
    matrix_t *accel_m = arr_to_matrix(accel, 3, 1);
    matrix_t *mag_m = arr_to_matrix(mag, 3, 1);

    matrix_t *accel_x_mag = mul_vector(accel_m, mag_m);
    
    matrix_t *row_1 = normalize_matrix(mul_vector(accel_x_mag, accel_m));
    matrix_t *row_2 = normalize_matrix(accel_x_mag);
    matrix_t *row_3 = normalize_matrix(accel_m);
    
    matrix_t *rot_matrix = init_matrix(3, 3);
    for (size_t i = 0; i < 3; i++) {
        rot_matrix->data[3 * i] = row_1->data[i];
    }
    for (size_t i = 0; i < 3; i++) {
        rot_matrix->data[3 * i + 1] = row_2->data[i];
    }
    for (size_t i = 0; i < 3; i++) {
        rot_matrix->data[3 * i + 2] = row_3->data[i];
    }

    kinetic->state_q = rot_matrix_to_quat(rot_matrix);

    // Initialize rest of variables
    kinetic->estm_covariance = ident_matrix(4);

    // NED reference frame - Factor in magnetic dip for m_ref
    if (kinetic->mag_dip == 0.f) {
        printf("Invalid mag dip. When setting mag dip to zero weird things happen.\n");
        printf("Resetting mag dip to 0.000001\n");
        kinetic->mag_dip = 0.000001;
    }

    float g_ref_a[] = {0, 0, 1};
    float m_ref_a[] = {cos(kinetic->mag_dip), 0, sin(kinetic->mag_dip)};
    kinetic->g_ref = arr_to_matrix(g_ref_a, 3, 1);
    kinetic->m_ref = scale_matrix(arr_to_matrix(m_ref_a, 3, 1), 
        1 / (sqrt(pow(cos(kinetic->mag_dip), 2) + pow(sin(kinetic->mag_dip), 2))));

    printf("After init:\n");
    printf("state_q = [%f, %f, %f, %f]\n", kinetic->state_q[0], kinetic->state_q[1], kinetic->state_q[2], kinetic->state_q[3]);
    printf("g_ref = [%f, %f, %f]\n", kinetic->g_ref->data[0], kinetic->g_ref->data[1], kinetic->g_ref->data[2]);
    printf("m_ref = [%f, %f, %f]\n", kinetic->m_ref->data[0], kinetic->m_ref->data[1], kinetic->m_ref->data[2]);
}

int main() {
    kinetic_t kinetic;
    kinetic.mag_dip = 67 * (180 / M_PI); // Use real magnetic dip angle
    kinetic.gyro_noise = 1.0 * (180 / M_PI);
    kinetic.accel_noise = 0.f;
    kinetic.mag_noise = 0.f;

    sim_t sim(&kinetic);

    matrix_t *start_rot = init_matrix(4, 1);
    start_rot->data[X] = 0.0;
    start_rot->data[Y] = 0.0;
    start_rot->data[Z] = 0.0;
    start_rot->data[W] = 1.0;

    matrix_t *end_rot = init_matrix(4, 1);
    end_rot->data[X] = 0.7071068;
    end_rot->data[Y] = 0.0;
    end_rot->data[Z] = 0.0;
    end_rot->data[W] = 0.7071068;

    printf("Starting linear interpolation test with magnetic dip angle\n");
    sim.linear_interpolation(start_rot, end_rot, 5, 0.5);

    return 0;
}
