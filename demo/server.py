#!/usr/bin/env python3
"""Simple HTTP server for Kinetic EKF demo."""

import sys
import os
import json
from http.server import HTTPServer, SimpleHTTPRequestHandler

# Add project root to path so kinetic can be imported
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from demo.kinetic import EKFDemo


class DemoHandler(SimpleHTTPRequestHandler):
    """Custom handler that intercepts /api/simulate requests."""

    def do_GET(self):
        # Strip leading /demo if present for file serving
        path = self.path
        if path.startswith("/demo"):
            path = path[5:]

        if path == "/api/simulate":
            self._run_simulation()
        elif path == "/api/linear":
            self._run_linear_interpolation()
        else:
            super().do_GET()

    def do_POST(self):
        # Strip leading /demo if present for file serving
        path = self.path
        if path.startswith("/demo"):
            path = path[5:]

        if path == "/api/linear":
            self._run_linear_interpolation_post()
        else:
            super().do_POST()

    def _run_simulation(self):
        """Run the all_axis_test simulation and return results."""
        try:
            demo = EKFDemo()
            ekf_data, true_data = demo.all_axis_test(100)

            response = {"ekf": ekf_data, "true": true_data}

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(response).encode())

        except Exception as e:
            error_response = {"error": str(e)}
            self.send_response(500)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(error_response).encode())

    def _run_linear_interpolation(self):
        """Run the linear interpolation simulation with default parameters."""
        try:
            demo = EKFDemo()
            ekf_data, true_data = demo.linear_interpolation()

            response = {"ekf": ekf_data, "true": true_data}

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(response).encode())

        except Exception as e:
            error_response = {"error": str(e)}
            self.send_response(500)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(error_response).encode())

    def _run_linear_interpolation_post(self):
        """Run the linear interpolation simulation with POST parameters."""
        try:
            print("[DEBUG] _run_linear_interpolation_post called")
            print("[DEBUG] Path:", self.path)
            print("[DEBUG] Headers:", dict(self.headers))

            # Parse JSON body
            content_length = int(self.headers.get("Content-Length", 0))
            print("[DEBUG] Content-Length:", content_length)
            body = self.rfile.read(content_length).decode("utf-8")
            print("[DEBUG] Body length:", len(body))
            print("[DEBUG] Body:", body)
            params = json.loads(body) if body else {}

            print("[DEBUG] Params:", params)

            start_rot = params.get("startRot", [0, 0, 0])
            end_rot = params.get("endRot", [45, 45, 45])

            print("Using start_rot:", start_rot, "end_rot:", end_rot)

            demo = EKFDemo()
            ekf_data, true_data = demo.linear_interpolation(start_rot, end_rot)

            print("Generated data lengths:", len(ekf_data), len(true_data))

            response = {"ekf": ekf_data, "true": true_data}

            response_json = json.dumps(response)
            encoded = response_json.encode("utf-8")
            print("[DEBUG] About to write JSON:", len(encoded), "bytes")
            self.wfile.write(encoded)
            self.wfile.flush()
            print("[DEBUG] Wrote JSON")

        except json.JSONDecodeError as e:
            error_response = {"error": "Invalid JSON body", "detail": str(e)}
            error_json = json.dumps(error_response)
            self.send_response(400)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(error_json)))
            self.end_headers()
            self.wfile.write(error_json.encode())
        except Exception as e:
            error_response = {"error": "Server error", "detail": str(e)}
            error_json = json.dumps(error_response)
            self.send_response(500)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(error_json)))
            self.end_headers()
            self.wfile.write(error_json.encode())


def run_server(port=8081, root=None):
    """Start the demo server."""
    # Default to demo directory for file serving
    if root is None:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    server_address = ("", port)
    httpd = HTTPServer(server_address, DemoHandler)
    print(f"🧭 Kinetic EKF Demo Server running on http://localhost:{port}")
    print(f"   📊 Serve files from: {root}")
    print(f"   📊 Open http://localhost:{port}/index.html to view the demo")
    httpd.serve_forever()


if __name__ == "__main__":
    run_server()
