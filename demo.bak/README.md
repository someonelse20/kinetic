# Kinetic Web Demo

A web-based demonstration of the kinetic EKF sensor fusion library.

## How to use

1. Open `index.html` in a browser (no server required).
2. Click "Load Sample Data" to see the EKF in action, or
3. Upload a CSV file with IMU data (columns: time, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z).
4. Watch the orientation estimate update in real-time.

## Data Format

For custom CSV uploads, use this format:

```
time,acc_x,acc_y,acc_z,mag_x,mag_y,mag_z
0.0,9.81,0.0,0.0,0.5,0.0,0.866
0.1,9.81,0.0,0.0,0.4,0.0,0.917
...
```

## Live Demo

- **Start**: Click "Start Live Demo" to simulate IMU data
- **Stop**: Click "Stop Demo" to pause
- **Reset**: Click "Reset" to clear estimates
