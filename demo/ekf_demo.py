#!/usr/bin/env python3
"""
Kinetic EKF Demo - Python stubs matching kin_wrapper.cpp API.
Based on AHRS EKF math: https://ahrs.readthedocs.io/en/latest/filters/ekf.html
"""
import json
import os
import numpy as np
from http.server import HTTPServer, SimpleHTTPRequestHandler
from threading import Thread


class EKFDemo:
    """EKF demonstration class with stub implementations."""

    def __init__(self):
        self.state_running = False
        self.state_initialized = False
        self.sim_mode = "linear_interpolation"  # or "all_axis_test"
        
        # Storage for charting
        self.euler_data = {"roll": [], "pitch": [], "yaw": []}
        self.true_data = {"roll": [], "pitch": [], "yaw": []}
        
        # Configuration
        self.num_points = 100
        self.step_size = 0.1

    # ==================== Core EKF API (matches kin_wrapper.cpp) ====================

    def imu_init(self, gyro_noise=0.001, accel_noise=0.01, mag_noise=0.01,
                 mag_dip=45.0, dt=0.01):
        """Initialize EKF with sensor noise parameters."""
        self.state_initialized = True
        return True

    def imu_update(self, accel_x, accel_y, accel_z,
                   mag_x, mag_y, mag_z, dt):
        """Core EKF update step with accelerometer and magnetometer."""
        # Placeholder: would call kinetic::update_imu()
        # Returns Euler angles (roll, pitch, yaw) in radians
        return 0.0, 0.0, 0.0

    def init_state(self, accel_x, accel_y, accel_z,
                   mag_x, mag_y, mag_z):
        """Initialize EKF from raw sensor readings."""
        self.state_initialized = True
        return True

    def get_accel(self, roll, pitch, yaw):
        """Compute expected accelerometer reading."""
        # Placeholder: kinetic::get_accel()
        # rot_matrix_trans * g_ref where g_ref = [0, 0, 1]
        return 0.0, 0.0, 0.0

    def get_mag(self, roll, pitch, yaw, dip_angle):
        """Compute expected magnetometer reading."""
        # Placeholder: kinetic::get_mag()
        # rot_matrix_trans * m_ref where m_ref depends on mag_dip
        return 0.0, 0.0, 0.0

    # ==================== Test Modes ====================

    def linear_interpolation(self):
        """Run linear interpolation test."""
        if not self.state_initialized:
            print("Error: EKF not initialized")
            return False

        # Generate interpolated orientations
        true_angles = self._generate_interpolated_orientations()

        # Run EKF on interpolated data
        for i, (roll, pitch, yaw) in enumerate(true_angles):
            dt = self.step_size
            # Simulate raw IMU measurements (with small noise for realism)
            meas = self._simulate_imu_measurement(roll, pitch, yaw)
            eul = self.imu_update(meas[0], meas[1], meas[2],
                                 meas[3], meas[4], meas[5], dt)

            self.euler_data["roll"].append(eul[0])
            self.euler_data["pitch"].append(eul[1])
            self.euler_data["yaw"].append(eul[2])
            self.true_data["roll"].append(roll)
            self.true_data["pitch"].append(pitch)
            self.true_data["yaw"].append(yaw)

        return True

    def all_axis_test(self, num_points=100):
        """Run all-axis test with specified number of steps."""
        if not self.state_initialized:
            print("Error: EKF not initialized")
            return False

        # Generate test data for all axes
        test_data = self._generate_all_axis_data(num_points)

        # Run EKF updates
        for i, (roll, pitch, yaw) in enumerate(test_data):
            dt = self.step_size
            meas = self._simulate_imu_measurement(roll, pitch, yaw)
            eul = self.imu_update(meas[0], meas[1], meas[2],
                                 meas[3], meas[4], meas[5], dt)

            self.euler_data["roll"].append(eul[0])
            self.euler_data["pitch"].append(eul[1])
            self.euler_data["yaw"].append(eul[2])
            self.true_data["roll"].append(roll)
            self.true_data["pitch"].append(pitch)
            self.true_data["yaw"].append(yaw)

        return True

    # ==================== Internal Helpers ====================

    def _generate_interpolated_orientations(self):
        """Generate orientations via linear interpolation."""
        angles = []
        # Start at identity
        a1 = (0.0, 0.0, 0.0)
        # Rotate to final orientation (example: 30° pitch, 45° yaw)
        a2 = (30.0 * np.pi / 180.0, 45.0 * np.pi / 180.0, 60.0 * np.pi / 180.0)

        for i in range(self.num_points):
            t = i / (self.num_points - 1)
            roll = a1[0] + t * (a2[0] - a1[0])
            pitch = a1[1] + t * (a2[1] - a1[1])
            yaw = a1[2] + t * (a2[2] - a1[2])
            angles.append((roll, pitch, yaw))
        return angles

    def _generate_all_axis_data(self, num_points):
        """Generate test data rotating through all axes."""
        data = []
        roll = pitch = yaw = 0.0
        step = 0.1

        for i in range(num_points):
            # Rotate around each axis sequentially
            roll += step
            pitch += step
            yaw += step

            # Normalize to keep angles reasonable
            roll = (roll + np.pi) % (2 * np.pi) - np.pi
            pitch = (pitch + np.pi) % (2 * np.pi) - np.pi
            yaw = (yaw + np.pi) % (2 * np.pi) - np.pi

            data.append((roll, pitch, yaw))
        return data

    def _simulate_imu_measurement(self, roll, pitch, yaw):
        """Simulate raw IMU measurements from orientation."""
        # Placeholder: would use kinetic rotation matrix
        # For now, return the orientation itself as "measured"
        # In real code, this would be rot_matrix_trans * g_ref/m_ref
        return roll, pitch, yaw, 0.0, 0.0, 0.0

    # ==================== Output & Status ====================

    def plot_data(self, output_dir="build"):
        """Export data to CSV for external plotting."""
        os.makedirs(output_dir, exist_ok=True)

        # Export EKF estimates
        with open(os.path.join(output_dir, "estm_roll.txt"), "w") as f:
            for val in self.euler_data["roll"]:
                f.write(f"{val}\n")

        with open(os.path.join(output_dir, "estm_pitch.txt"), "w") as f:
            for val in self.euler_data["pitch"]:
                f.write(f"{val}\n")

        with open(os.path.join(output_dir, "estm_yaw.txt"), "w") as f:
            for val in self.euler_data["yaw"]:
                f.write(f"{val}\n")

        # Export true values
        with open(os.path.join(output_dir, "true_roll.txt"), "w") as f:
            for val in self.true_data["roll"]:
                f.write(f"{val}\n")

        with open(os.path.join(output_dir, "true_pitch.txt"), "w") as f:
            for val in self.true_data["pitch"]:
                f.write(f"{val}\n")

        with open(os.path.join(output_dir, "true_yaw.txt"), "w") as f:
            for val in self.true_data["yaw"]:
                f.write(f"{val}\n")

        print(f"Data exported to {output_dir}/")

    def get_demo_status(self):
        """Return current demo state for web UI."""
        return {
            "running": self.state_running,
            "initialized": self.state_initialized,
            "mode": self.sim_mode,
            "points": len(self.euler_data["roll"])
        }


def run_server(port=8081):
    """Start the HTTP server."""
    demo = EKFDemo()

    def run_test():
        """Run the selected test mode."""
        if demo.sim_mode == "linear_interpolation":
            demo.linear_interpolation()
        else:
            demo.all_axis_test(demo.num_points)

        demo.plot_data()

    # Start test in background thread
    test_thread = Thread(target=run_test, daemon=True)
    test_thread.start()

    # Simple handler that serves static files and runs demo on request
    class DemoHandler(SimpleHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/run-demo":
                run_test()
                self.send_response(200)
                self.send_header("Content-type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps(demo.get_demo_status()).encode())
            elif self.path.startswith("/data/"):
                filename = self.path[6:]  # Remove "/data/"
                try:
                    with open(filename, "r") as f:
                        self.send_response(200)
                        self.send_header("Content-type", "text/plain")
                        self.end_headers()
                        self.wfile.write(f.read().encode())
                except FileNotFoundError:
                    self.send_error(404, "File not found")
            else:
                super().do_GET()

    server = HTTPServer(("", port), DemoHandler)
    print(f"Server running on http://localhost:{port}")
    print("Press Ctrl+C to stop")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.shutdown()


def run_server_from_path(port=8081, demo_path=None):
    """Start server with demo file specified by path."""
    if demo_path:
        demo = EKFDemo()
    else:
        demo = EKFDemo()

    def run_test():
        if demo.sim_mode == "linear_interpolation":
            demo.linear_interpolation()
        else:
            demo.all_axis_test(demo.num_points)
        demo.plot_data()

    test_thread = Thread(target=run_test, daemon=True)
    test_thread.start()

    class DemoHandler(SimpleHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/run-demo":
                run_test()
                self.send_response(200)
                self.send_header("Content-type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps(demo.get_demo_status()).encode())
            elif self.path.startswith("/data/"):
                filename = self.path[6:]
                try:
                    with open(filename, "r") as f:
                        self.send_response(200)
                        self.send_header("Content-type", "text/plain")
                        self.end_headers()
                        self.wfile.write(f.read().encode())
                except FileNotFoundError:
                    self.send_error(404, "File not found")
            else:
                super().do_GET()

    server = HTTPServer(("", port), DemoHandler)
    print(f"Server running on http://localhost:{port}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.shutdown()


if __name__ == "__main__":
    run_server()
