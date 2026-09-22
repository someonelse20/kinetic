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
        if self.path == '/api/simulate':
            self._run_simulation()
        else:
            super().do_GET()
    
    def _run_simulation(self):
        """Run the all_axis_test simulation and return results."""
        try:
            demo = EKFDemo()
            ekf_data, true_data = demo.all_axis_test(100)
            
            response = {
                'ekf': ekf_data,
                'true': true_data
            }
            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode())
            
        except Exception as e:
            error_response = {'error': str(e)}
            self.send_response(500)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(error_response).encode())


def run_server(port=8081):
    """Start the demo server."""
    server_address = ('', port)
    httpd = HTTPServer(server_address, DemoHandler)
    print(f"🧭 Kinetic EKF Demo Server running on http://localhost:{port}")
    print("   📊 Open this URL to view the interactive demo")
    httpd.serve_forever()


if __name__ == '__main__':
    run_server()
