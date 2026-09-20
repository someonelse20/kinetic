#!/usr/bin/env python3
"""Kinetic EKF Web Demo - Demonstrates the kinetic library through a browser interface."""

import json
import math
import random
import sys

try:
    from build import kin_wrapper
except ImportError:
    print("Error: Run `cd demo && pip install -e .` first")
    sys.exit(1)


class DemoState:
    def __init__(self):
        self.running = False
        self.paused = False
        self.step = 0
        self.init_done = False

        # EKF instance - using raw C structures via pybind11
        self.imu = kin_wrapper.imu_t()
        self.ekf = kin_wrapper.ekf_t()

        # Data buffers
        self.quat_data = []
        self.euler_data = []
        self.step_data = []

        # IMU noise
        self.noise_scale = 0.02


state = DemoState()


def generate_sensor_noise(value):
    """Generate Gaussian noise scaled to realistic IMU values."""
    if state.noise_scale <= 0:
        return value
    noise = (
        random.gauss(0, state.noise_scale * abs(value))
        if value != 0
        else random.gauss(0, state.noise_scale)
    )
    return value + noise


def interpolate_orientation(step, total_steps=100):
    """
    Interpolate between two orientations over time.
    Creates smooth rotation motion similar to vehicle movement.
    """
    progress = step / total_steps

    # Complex multi-axis rotation with slight frequency variation
    qx = math.sin(progress * 6.0) * math.sin(progress * 3.0)
    qy = math.cos(progress * 4.0) * math.sin(progress * 2.0)
    qz = math.sin(progress * 5.0)
    qw = math.cos(progress * 3.0) * math.cos(progress * 2.0) + 0.7

    # Normalize quaternion
    norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    return qx / norm, qy / norm, qz / norm, qw / norm


def run_demo_step():
    """Run one EKF update step with synthetic IMU measurements."""
    # 1. Generate "true" orientation at this step
    qx_true, qy_true, qz_true, qw_true = interpolate_orientation(state.step, 100)

    # 2. Convert true quaternion to Euler angles for reference
    m_true = kin_wrapper.init_matrix(1, 4)
    m_true[0] = qx_true
    m_true[1] = qy_true
    m_true[2] = qz_true
    m_true[3] = qw_true
    euler_true = kin_wrapper.quat_to_euler(m_true)

    # 3. Generate noisy IMU measurements from this orientation
    # Read true quaternion components from EKF state
    qx_t = state.ekf.state[0]
    qy_t = state.ekf.state[1]
    qz_t = state.ekf.state[2]
    qw_t = state.ekf.state[3]

    # Generate gyro (angular velocity) measurements
    progress = state.step / 100.0
    wx = (math.cos(progress * 3.0) * math.cos(progress * 2.0) + 0.7) * 10.0
    wy = -(math.sin(progress * 3.0) * math.cos(progress * 2.0) + 0.7) * 8.0
    wz = (math.cos(progress * 4.0) * math.sin(progress * 2.0)) * 12.0

    gyro_meas = [
        generate_sensor_noise(wx),
        generate_sensor_noise(wy),
        generate_sensor_noise(wz),
    ]

    # Generate accelerometer measurements (gravity transformed by rotation)
    ax = generate_sensor_noise(9.81 * qx_t)
    ay = generate_sensor_noise(9.81 * qy_t)
    az = generate_sensor_noise(9.81 * qz_t)

    # Generate magnetometer measurements (earth's magnetic field transformed)
    bx = generate_sensor_noise(50.0 * qx_t + 20.0)
    by = generate_sensor_noise(50.0 * qy_t + 15.0)
    bz = generate_sensor_noise(50.0 * qz_t + 5.0)

    # 4. Update EKF with measurements
    state.imu.gyro_noise = 0.001
    state.imu.accel_noise = 0.001
    state.imu.mag_noise = 0.001

    # Call C functions via pybind11 with float pointers
    a_arr = [ax, ay, az]
    m_arr = [bx, by, bz]
    kin_wrapper.imu_update(state.imu, gyro_meas, a_arr, m_arr)

    # 5. Extract new quaternion components
    qx_new = state.ekf.state[0]
    qy_new = state.ekf.state[1]
    qz_new = state.ekf.state[2]
    qw_new = state.ekf.state[3]

    # 6. Convert to Euler angles for display
    m_new = kin_wrapper.init_matrix(1, 4)
    m_new[0] = qx_new
    m_new[1] = qy_new
    m_new[2] = qz_new
    m_new[3] = qw_new
    euler_new = kin_wrapper.quat_to_euler(m_new)

    # 7. Store data for visualization
    state.quat_data.append((qx_new, qy_new, qz_new, qw_new))
    state.euler_data.append((euler_new[0], euler_new[1], euler_new[2]))
    state.step_data.append(state.step)

    # Keep arrays manageable
    if len(state.quat_data) > 500:
        state.quat_data = state.quat_data[-250:]
        state.euler_data = state.euler_data[-250:]
        state.step_data = state.step_data[-250:]

    # 8. Return JSON payload for the web client
    return {
        "step": state.step,
        "quat": [
            round(qx_new, 6),
            round(qy_new, 6),
            round(qz_new, 6),
            round(qw_new, 6),
        ],
        "euler": [
            round(euler_new[0], 2),
            round(euler_new[1], 2),
            round(euler_new[2], 2),
        ],
        "euler_true": [
            round(euler_true[0], 2),
            round(euler_true[1], 2),
            round(euler_true[2], 2),
        ],
        "covariance": [
            round(state.ekf.covariance[0], 2),
            round(state.ekf.covariance[1], 2),
            round(state.ekf.covariance[2], 2),
            round(state.ekf.covariance[3], 2),
        ],
    }


def init_ekf():
    """Initialize the EKF with a starting orientation."""
    global state

    initial_quat = [0.0, 0.0, 0.0, 1.0]

    # Initialize IMU with initial orientation
    state.imu = kin_wrapper.imu_t()
    state.imu.gyro_noise = 0.001
    state.imu.accel_noise = 0.001
    state.imu.mag_noise = 0.001
    state.imu.mag_dip = 65.0
    state.imu.dt = 0.01

    # Initialize EKF state
    state.ekf = kin_wrapper.ekf_t()
    state.ekf.state = kin_wrapper.init_matrix(1, 4)
    state.ekf.state[0] = initial_quat[0]
    state.ekf.state[1] = initial_quat[1]
    state.ekf.state[2] = initial_quat[2]
    state.ekf.state[3] = initial_quat[3]

    state.ekf.covariance = kin_wrapper.init_matrix(4, 1)
    state.ekf.covariance[0] = 1.0
    state.ekf.covariance[1] = 1.0
    state.ekf.covariance[2] = 1.0
    state.ekf.covariance[3] = 1.0

    # Clear data arrays
    state.quat_data = []
    state.euler_data = []
    state.step_data = []

    state.init_done = True


def main():
    """Run the web server and open the demo."""
    print("Starting Kinetic EKF Web Demo...")
    print("Server will be available at http://localhost:8082")
    print("Press Ctrl+C to stop")

    init_ekf()

    # Run a few demo steps to populate initial data
    for _ in range(10):
        run_demo_step()

    from http.server import HTTPServer, BaseHTTPRequestHandler

    class DemoHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                with open("demo.html", "r") as f:
                    html = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(html.encode("utf-8"))
            elif self.path == "/data":
                data = json.dumps(run_demo_step())
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                self.wfile.write(data.encode("utf-8"))
            else:
                self.send_response(404)
                self.end_headers()

        def log_message(self, format, *args):
            pass

    server = HTTPServer(("127.0.0.1", 8082), DemoHandler)
    server.serve_forever()


if __name__ == "__main__":
    main()
