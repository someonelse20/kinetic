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

        # Extract query string before stripping it for route comparison
        query_string = "" if "?" not in path else path.split("?")[1]

        # Strip query string for route comparison
        if "?" in path:
            path = path.split("?")[0]

        if path == "/api/simulate":
            self._run_simulation()
        elif path == "/api/linear":
            self._run_linear_interpolation(query_string)
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

    def _run_linear_interpolation(self, query_string=None):
        """Run the linear interpolation simulation with parameters from URL query string.
        
        Args:
            query_string: The query string portion of the URL (e.g., 'startRot=[...]&endRot=[...]').
                        If None, will be extracted from self.path.
        """
        try:
            # Parse query parameters from URL (e.g., /api/linear?startRot=[0,0,0]&endRot=[45,45,45])
            import urllib.parse
            
            # Use provided query_string or extract from self.path
            if query_string is None:
                query_string = self.path.split("?")[1] if "?" in self.path else ""
            
            # parse_qs interprets brackets as array delimiters, so we need to handle this
            # We'll manually parse the query string instead
            params = {}
            if query_string:
                for pair in query_string.split("&"):
                    if "=" in pair:
                        key, value = pair.split("=", 1)
                        if key not in params:
                            params[key] = []
                        params[key].append(urllib.parse.unquote(value))

            # Parse startRot and endRot as JSON arrays
            start_rot = [0, 0, 0]
            end_rot = [45, 45, 45]

            if "startRot" in params and params["startRot"]:
                try:
                    start_rot = json.loads(params["startRot"][0])
                except json.JSONDecodeError:
                    pass

            if "endRot" in params and params["endRot"]:
                try:
                    end_rot = json.loads(params["endRot"][0])
                except json.JSONDecodeError:
                    pass

            print("[DEBUG] GET linear params: start_rot=%s, end_rot=%s" % (start_rot, end_rot))

            demo = EKFDemo()
            ekf_data, true_data = demo.linear_interpolation(start_rot, end_rot)

            print("[DEBUG] Generated %d data points" % len(ekf_data))

            response = {"ekf": ekf_data, "true": true_data}

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(response).encode())

        except json.JSONDecodeError as e:
            error_response = {"error": "Invalid JSON in query parameters", "detail": str(e)}
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
