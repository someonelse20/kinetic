#!/usr/bin/env python3
import numpy as np
from build import kin_wrapper as kin


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
        self.mag_dip = 45.0  # Magnetic dip angle (degrees to radians)

        self.imu = kin.imu_t

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

    def _quat_to_rot_matrix(self, roll, pitch, yaw):
        """Convert Euler angles to rotation matrix."""
        # AHRS EKF uses quaternion-based rotation
        # Build rotation matrix from Euler angles (NED convention)
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)

        R = np.array([
            [cr * cy - sr * sp * sy, -cr * sy - sr * sp * cy, sr * cp],
            [sr * cy + cr * sp * sy, -sr * sy + cr * sp * cy, -cr * cp],
            [-sr * sp * cy - cr * cp * sy, sr * sp * sy - cr * cp * cy, cp * cp]
        ])
        return R

    def get_accel(self, roll, pitch, yaw):
        """Compute expected accelerometer reading.
        
        Matches sim.cpp::get_accel()
        rot_matrix_trans * g_ref where g_ref = [0, 0, 1]
        """
        R = self._quat_to_rot_matrix(roll, pitch, yaw)
        g_ref = np.array([0.0, 0.0, 1.0])
        
        # Apply rotation and transpose (equivalent to inverse for orthogonal matrices)
        accel = R.T.dot(g_ref)
        return accel[0], accel[1], accel[2]

    def get_mag(self, roll, pitch, yaw, dip_angle):
        """Compute expected magnetometer reading.
        
        Matches sim.cpp::get_mag()
        rot_matrix_trans * m_ref where m_ref depends on mag_dip
        m_ref = [cos(dip), 0, sin(dip)] normalized
        """
        R = self._quat_to_rot_matrix(roll, pitch, yaw)
        
        # m_ref vector from sim.cpp line 464-465
        m_ref = np.array([np.cos(dip_angle), 0.0, np.sin(dip_angle)])
        
        # Normalize (sim.cpp line 468)
        norm = np.sqrt(np.cos(dip_angle)**2 + np.sin(dip_angle)**2)
        m_ref = m_ref / norm
        
        # Apply rotation and transpose
        mag = R.T.dot(m_ref)
        return mag[0], mag[1], mag[2]

    # ==================== Test Modes ====================

    def linear_interpolation(self):
        """Run linear interpolation test."""
        if not self.state_initialized:
            print("Error: EKF not initialized")
            return False

        # Generate interpolated orientations
        true_angles = self._generate_interpolated_orientations()

        # Run EKF on interpolated data
        # Use get_gyro() to compute gyro from orientation changes (matches sim.cpp)
        for i in range(len(true_angles) - 1):
            roll1, pitch1, yaw1 = true_angles[i]
            roll2, pitch2, yaw2 = true_angles[i + 1]
            dt = self.step_size
            
            # Compute gyro from orientation change (sim.cpp::get_gyro)
            gyro_x, gyro_y, gyro_z = self.get_gyro(roll1, pitch1, yaw1, 
                                                    roll2, pitch2, yaw2, dt)
            
            # Compute accel and mag from current orientation
            accel_x, accel_y, accel_z = self.get_accel(roll2, pitch2, yaw2)
            mag_x, mag_y, mag_z = self.get_mag(roll2, pitch2, yaw2, self.mag_dip)
            
            # Run EKF update
            eul = self.imu_update(accel_x, accel_y, accel_z,
                                 mag_x, mag_y, mag_z, dt)

            self.euler_data["roll"].append(eul[0])
            self.euler_data["pitch"].append(eul[1])
            self.euler_data["yaw"].append(eul[2])
            self.true_data["roll"].append(roll2)
            self.true_data["pitch"].append(pitch2)
            self.true_data["yaw"].append(yaw2)

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

    def get_gyro(self, euler_x1, euler_y1, euler_z1, euler_x2, euler_y2, euler_z2, dt):
        """Compute gyro rate from two consecutive Euler orientations.
        
        Matches sim.cpp::get_gyro()
        Uses quaternion cross-product formula:
        gyro = 2 * dQ/dt * Q_inv where dQ = Q2 - Q1
        
        Args:
            euler_x1, euler_y1, euler_z1: First orientation (raw Euler angles)
            euler_x2, euler_y2, euler_z2: Second orientation (raw Euler angles)
            dt: Time step between measurements
        
        Returns:
            (gyro_x, gyro_y, gyro_z) in rad/s, scaled by 2/dt
        """
        # Convert Euler to quaternions (AHRS EKF convention)
        q1 = self._euler_to_quat(euler_x1, euler_y1, euler_z1)
        q2 = self._euler_to_quat(euler_x2, euler_y2, euler_z2)
        
        # Compute delta quaternion: dQ = Q2 - Q1 (sim.cpp line 439-441)
        dQ_x = q2[0] - q1[0]
        dQ_y = q2[1] - q1[1]
        dQ_z = q2[2] - q1[2]
        dQ_w = q2[3] - q1[3]
        
        # Quaternion cross-product formula (sim.cpp lines 439-441)
        # gyro = 2 * dQ/dt * Q_inv
        # Q_inv = conjugate for unit quaternions
        gyro_x = 2 * (dQ_w * q1[0] - dQ_x * q1[3] - dQ_y * q1[2] + dQ_z * q1[1])
        gyro_y = 2 * (dQ_w * q1[1] + dQ_x * q1[2] - dQ_y * q1[3] - dQ_z * q1[0])
        gyro_z = 2 * (dQ_w * q1[2] - dQ_x * q1[1] + dQ_y * q1[0] - dQ_z * q1[3])
        
        # Scale by 2/dt (sim.cpp line 443)
        gyro_x *= 2.0 / dt
        gyro_y *= 2.0 / dt
        gyro_z *= 2.0 / dt
        
        return gyro_x, gyro_y, gyro_z

    def _euler_to_quat(self, euler_x, euler_y, euler_z):
        """Convert raw Euler angles to quaternion.
        
        Matches kin_math.c::euler_to_quat() exactly.
        Uses half-angle trigonometry: u=v=w=angle/2
        Returns quaternion in kinetic's x y z w order.
        """
        # Half-angles (kin_math.c lines 416-418)
        u = euler_x / 2.0
        v = euler_y / 2.0
        w = euler_z / 2.0
        
        # cos(u), cos(v), cos(w), sin(u), sin(v), sin(w)
        cu = np.cos(u)
        cv = np.cos(v)
        cw = np.cos(w)
        su = np.sin(u)
        sv = np.sin(v)
        sw = np.sin(w)
        
        # kin_math.c lines 420-423
        qw = cu * cv * cw + su * sv * sw
        qx = su * cv * cw - cu * sv * sw
        qy = cu * sv * cw + su * cv * sw
        qz = cu * cv * sw - su * sv * cw
        
        # Return in kinetic's x y z w order
        return qx, qy, qz, qw

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
