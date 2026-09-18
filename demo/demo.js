// Kinetic EKF Demo - Standalone JavaScript Implementation
// This simulates kinetic's EKF behavior for demonstration purposes

(function() {
    'use strict';

    // EKF State
    const state = {
        quaternion: [0, 0, 0, 1],  // x, y, z, w
        rotationMatrix: new Array(9).fill(0),
        covarP: new Array(4 * 4).fill(0),
        covarQ: new Array(4 * 4).fill(0),
        covarR: new Array(3 * 3).fill(0),
        time: 0,
        running: false,
        dataIndex: 0,
        measuredData: [],
        trueData: [],
        estimatedData: []
    };

    // EKF Parameters (mimicking kinetic configuration)
    const EPSILON = 1e-10;

    // Utility Functions
    function abs(x) { return x < 0 ? -x : x; }
    function sqrt(x) { return x > 0 ? Math.sqrt(x) : 0; }
    function sign(x) { return x > 0 ? 1 : (x < 0 ? -1 : 0); }

    // Quaternion Math
    function qNorm(q) {
        return sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    }

    function normalizeQuaternion(q) {
        const n = qNorm(q);
        if (n < EPSILON) return [0, 0, 0, 1];
        return [q[0]/n, q[1]/n, q[2]/n, q[3]/n];
    }

    function qToRotationMatrix(q) {
        const x = q[0], y = q[1], z = q[2], w = q[3];
        return [
            1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w,
            2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w,
            2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y
        ];
    }

    function rotationMatrixToQuaternion(R) {
        const tr = R[0] + R[4] + R[8];
        if (tr > 0) {
            const s = 0.5 / sqrt(tr + 1);
            return [
                (R[2] - R[6]) * s,
                (R[3] - R[5]) * s,
                (R[5] + R[3] - R[6] + R[1]) * s, // Simplified
                0.5 * s * (tr + 1)
            ];
        } else {
            // Find max diagonal element
            if (R[0] > R[4] && R[0] > R[8]) {
                const s = 0.5 / sqrt(R[0] + R[8] + R[4]);
                return [0.5 * s * (R[2] - R[6]), s * (R[0] + R[5]), s * (R[1] + R[3]), s * (R[0] + R[8] + R[4])];
            } else if (R[4] > R[8]) {
                const s = 0.5 / sqrt(R[0] + R[4] + R[8]);
                return [s * (R[1] - R[5]), 0.5 * s * (R[0] + R[4]), s * (R[3] + R[6]), s * (R[0] + R[4] + R[8])];
            } else {
                const s = 0.5 / sqrt(R[8] + R[4] + R[0]);
                return [s * (R[3] + R[5]), s * (R[2] + R[6]), 0.5 * s * (R[8] + R[4]), s * (R[0] + R[4] + R[8])];
            }
        }
    }

    // EKF Core Functions
    function predict(dt) {
        // State prediction (kinematic model)
        const q = state.quaternion;
        const w = 0.1; // Simulated angular velocity
        const dq = [
            w * (q[1]*q[3] - q[2]*q[0]),
            w * (q[0]*q[3] + q[1]*q[2]),
            w * (q[0]*q[1] - q[2]*q[3]),
            0
        ];
        
        // Update quaternion
        state.quaternion = normalizeQuaternion([
            q[0] + dq[0]*dt,
            q[1] + dq[1]*dt,
            q[2] + dq[2]*dt,
            q[3] + dq[3]*dt
        ]);
        
        // Update rotation matrix
        state.rotationMatrix = qToRotationMatrix(state.quaternion);
        
        // Simplified covariance prediction
        // In full kinetic implementation, this uses Jacobians
    }

    function update(measurement) {
        const R = state.rotationMatrix;
        const q = state.quaternion;
        
        // Compute expected measurement (measurement model)
        // In kinetic: accel = rot_matrix_trans * g_ref, mag = rot_matrix_trans * m_ref
        const gRef = [0, 0, 1]; // Gravity in NED
        const magRef = [0.5, 0, 0.866]; // Magnetic reference
        
        const expectedAccel = [
            R[0]*gRef[0] + R[1]*gRef[1] + R[2]*gRef[2],
            R[3]*gRef[0] + R[4]*gRef[1] + R[5]*gRef[2],
            R[6]*gRef[0] + R[7]*gRef[1] + R[8]*gRef[2]
        ];
        
        const expectedMag = [
            R[0]*magRef[0] + R[1]*magRef[1] + R[2]*magRef[2],
            R[3]*magRef[0] + R[4]*magRef[1] + R[5]*magRef[2],
            R[6]*magRef[0] + R[7]*magRef[1] + R[8]*magRef[2]
        ];
        
        // Measurement residual
        const residual = [
            measurement.accX - expectedAccel[0],
            measurement.accY - expectedAccel[1],
            measurement.accZ - expectedAccel[2]
        ];
        
        // Kalman gain (simplified computation)
        const K = [
            [0.1, 0, 0, 0],
            [0, 0.1, 0, 0],
            [0, 0, 0.1, 0]
        ];
        
        // State update
        const dq = [
            K[0][0] * residual[0],
            K[1][1] * residual[1],
            K[2][2] * residual[2],
            0
        ];
        
        state.quaternion = normalizeQuaternion([
            q[0] + dq[0],
            q[1] + dq[1],
            q[2] + dq[2],
            q[3]
        ]);
        
        state.rotationMatrix = qToRotationMatrix(state.quaternion);
    }

    function eulerFromQuaternion(q) {
        const x = q[0], y = q[1], z = q[2], w = q[3];
        
        const roll = Math.atan2(2*(x*w + y*z), 1 - 2*(x*x + y*y));
        const pitch = Math.asin(Math.clamp(2*(y*w - z*x), -1, 1));
        const yaw = Math.atan2(2*(z*w + x*y), 1 - 2*(y*y + z*z));
        
        return [roll, pitch, yaw];
    }

    // Demo Simulation
    function generateSampleData() {
        const data = [];
        let roll = 0, pitch = 0, yaw = 0;
        const dt = 0.1;
        
        for (let i = 0; i < 100; i++) {
            const time = i * dt;
            
            // True orientation (smooth rotation)
            roll = Math.sin(time * 0.5) * 0.3;
            pitch = Math.cos(time * 0.3) * 0.2;
            yaw = (time * 0.2) % (2 * Math.PI);
            
            // Simulate IMU measurements with noise
            const q = eulerFromQuaternion([roll, pitch, yaw]);
            
            // Simulate accelerometer (gravity in different orientation)
            const accX = 9.81 * (Math.sin(q[1]) * Math.cos(q[0]) + 0.01 * (Math.random() - 0.5));
            const accY = 9.81 * (Math.cos(q[1]) * Math.cos(q[0]) + 0.01 * (Math.random() - 0.5));
            const accZ = 9.81 * (Math.sin(q[1]) * Math.sin(q[0]) + 0.01 * (Math.random() - 0.5));
            
            // Simulate magnetometer
            const magX = 0.5 + 0.3 * Math.sin(yaw + 0.3 * (Math.random() - 0.5));
            const magY = 0.1 * (Math.random() - 0.5);
            const magZ = 0.866 + 0.2 * Math.cos(yaw + 0.2 * (Math.random() - 0.5));
            
            data.push({
                time,
                accX, accY, accZ,
                magX, magY, magZ,
                trueRoll: roll, truePitch: pitch, trueYaw: yaw
            });
        }
        
        return data;
    }

    // Chart Drawing
    const canvas = document.getElementById('chart');
    const ctx = canvas.getContext('2d');
    let chartData = [];

    function drawChart() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        const padding = 40;
        const chartWidth = canvas.width - padding * 2;
        const chartHeight = canvas.height - padding * 2;
        
        // Draw grid
        ctx.strokeStyle = 'rgba(148, 163, 184, 0.2)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let i = 0; i <= 4; i++) {
            const y = padding + (chartHeight / 4) * i;
            ctx.moveTo(padding, y);
            ctx.lineTo(canvas.width - padding, y);
        }
        ctx.stroke();
        
        if (chartData.length < 2) return;
        
        // Draw EKF Estimate
        ctx.strokeStyle = '#3b82f6';
        ctx.lineWidth = 2;
        ctx.beginPath();
        for (let i = 0; i < chartData.length; i++) {
            const x = padding + (chartData[i].time / chartData[chartData.length - 1].time) * chartWidth;
            const y = chartHeight - padding - (chartData[i].yaw + Math.PI) / (2 * Math.PI) * chartHeight;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        
        // Draw True Orientation
        ctx.strokeStyle = '#22c55e';
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        for (let i = 0; i < chartData.length; i++) {
            const x = padding + (chartData[i].time / chartData[chartData.length - 1].time) * chartWidth;
            const y = chartHeight - padding - (chartData[i].trueYaw + Math.PI) / (2 * Math.PI) * chartHeight;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
    }

    function updateDisplay() {
        const quat = state.quaternion;
        const [roll, pitch, yaw] = eulerFromQuaternion(quat);
        
        document.getElementById('quat').textContent = 
            `x: ${quat[0].toFixed(3)}, y: ${quat[1].toFixed(3)}, z: ${quat[2].toFixed(3)}, w: ${quat[3].toFixed(3)}`;
        document.getElementById('roll').textContent = `${(roll * 180 / Math.PI).toFixed(2)}°`;
        document.getElementById('pitch').textContent = `${(pitch * 180 / Math.PI).toFixed(2)}°`;
        document.getElementById('yaw').textContent = `${(yaw * 180 / Math.PI).toFixed(2)}°`;
        
        chartData.push({ time: state.time, yaw: yaw * 180 / Math.PI });
        if (chartData.length > 100) chartData.shift();
        drawChart();
    }

    // Event Handlers
    function setStatus(msg, type = 'info') {
        const el = document.getElementById('status');
        el.textContent = msg;
        el.className = 'status ' + (type === 'error' ? 'error' : (type === 'info' ? 'info' : ''));
    }

    document.getElementById('btn-sample').addEventListener('click', () => {
        if (state.running) return;
        
        state.running = true;
        state.dataIndex = 0;
        state.measuredData = generateSampleData();
        state.trueData = state.measuredData.map(d => ({ ...d }));
        state.estimatedData = [];
        state.time = 0;
        
        document.getElementById('btn-sample').disabled = true;
        document.getElementById('btn-live').disabled = true;
        document.getElementById('btn-stop').disabled = true;
        document.getElementById('btn-reset').disabled = true;
        
        setStatus('Loading sample data...', 'info');
    });

    document.getElementById('btn-live').addEventListener('click', () => {
        if (!state.running) {
            // Start live demo from current state
            setStatus('Live demo started', 'info');
        }
    });

    document.getElementById('btn-stop').addEventListener('click', () => {
        state.running = false;
        setStatus('Demo paused', 'info');
        document.getElementById('btn-stop').disabled = true;
    });

    document.getElementById('btn-reset').addEventListener('click', () => {
        state.running = false;
        state.quaternion = [0, 0, 0, 1];
        state.time = 0;
        state.dataIndex = 0;
        state.measuredData = [];
        state.trueData = [];
        state.estimatedData = [];
        chartData = [];
        
        document.getElementById('quat').textContent = 'x: 0.000, y: 0.000, z: 0.000, w: 1.000';
        document.getElementById('roll').textContent = '0.00°';
        document.getElementById('pitch').textContent = '0.00°';
        document.getElementById('yaw').textContent = '0.00°';
        
        document.getElementById('btn-sample').disabled = false;
        document.getElementById('btn-live').disabled = false;
        document.getElementById('btn-stop').disabled = true;
        document.getElementById('btn-reset').disabled = false;
        
        setStatus('Ready', 'info');
    });

    document.getElementById('file-upload').addEventListener('change', async (e) => {
        const file = e.target.files[0];
        if (!file) return;
        
        setStatus('Parsing CSV...', 'info');
        
        const text = await file.text();
        const lines = text.trim().split('\n');
        
        if (lines.length < 2) {
            setStatus('Error: CSV too short or empty', 'error');
            return;
        }
        
        // Parse CSV (expects: time,acc_x,acc_y,acc_z,mag_x,mag_y,mag_z)
        const data = [];
        for (let i = 1; i < lines.length; i++) {
            const parts = lines[i].split(',').map(s => parseFloat(s));
            if (parts.length === 7 && !isNaN(parts[0])) {
                data.push({
                    time: parts[0],
                    accX: parts[1], accY: parts[2], accZ: parts[3],
                    magX: parts[4], magY: parts[5], magZ: parts[6]
                });
            }
        }
        
        if (data.length === 0) {
            setStatus('Error: No valid data rows found', 'error');
            return;
        }
        
        state.running = true;
        state.dataIndex = 0;
        state.measuredData = data;
        state.trueData = []; // No true data for uploaded files
        state.estimatedData = [];
        state.time = data[0].time;
        
        document.getElementById('file-upload').value = '';
        document.getElementById('btn-sample').disabled = true;
        document.getElementById('file-upload').disabled = true;
        
        setStatus(`Loaded ${data.length} data points. Running EKF...`, 'info');
    });

    // Animation Loop
    function animate() {
        if (!state.running) {
            requestAnimationFrame(animate);
            return;
        }
        
        // Simulate sensor updates
        if (state.dataIndex < state.measuredData.length) {
            const measurement = state.measuredData[state.dataIndex];
            state.time = measurement.time;
            
            // Predict
            predict(0.1);
            
            // Update
            update(measurement);
            
            // Store
            state.estimatedData.push({ time: state.time, yaw: eulerFromQuaternion(state.quaternion)[2] });
            state.trueData.push({ time: state.time, yaw: measurement.trueYaw });
            
            state.dataIndex++;
        } else {
            state.running = false;
        }
        
        updateDisplay();
        requestAnimationFrame(animate);
    }

    // Initialize
    setStatus('Ready. Click "Load Sample Data" to begin.', 'info');
})();

// Math.clamp polyfill
Math.clamp = Math.clamp || function(value, min, max) {
    return Math.min(Math.max(value, min), max);
};
