# IMU Calibration Formulas

Source: Madgwick PhD Thesis, Chapter 6

---

## 1. Gyroscope Calibration

### 1.1 Calibration Model

**Vector form (Eq 6.2):**
```
ω = C_R^(-1) * S_ω * (u_ω - b_ω)
```

**Expanded 3×3 form (Eq 6.3):**
```
[ ωx ]   [ r_ω11 r_ω12 r_ω13 ] [ s_ωx  0    0    ]^-1 [ u_ωx  ]
[ ωy ] = [ r_ω21 r_ω22 r_ω23 ] [ 0    s_ωy 0    ] [ -b_ωy ]
[ ωz ]   [ r_ω31 r_ω32 r_ω33 ] [ 0    0    s_ωz ] [  u_ωz ]
```

Where:
- ω = calibrated angular velocity (degrees/sec)
- u_ω = uncalibrated gyroscope output (lsb)
- C_R = rotation matrix for gyroscope alignment to calibrated frame
- S_ω = diagonal sensitivity matrix (lsb per degrees/sec)
- b_ω = gyroscope bias vector (lsb)

### 1.2 Calibration Steps

**Step 1: Bias (Eq 6.4.1)**
```
b_ω = mean(u_ω)  while stationary
```

**Step 2: Sensitivity (Eq 6.4.3, Eq 6.4)**
```
s_ω = (|u_+ω| + |u_-ω|) / (2 * ω)
```
- ω = reference angular velocity (200°/s from 33⅓ RPM turntable)
- u_+ω, u_-ω = axis outputs at +ω and -ω references

**Step 3: Alignment (Eq 6.4.4, Eq 6.5)**
```
C_R ~ [ (k * S_ω^-1 * (u_ωx - b_ω) * k)^T,
        (k * S_ω^-1 * (u_ωy - b_ω) * k)^T,
        (k * S_ω^-1 * (u_ωz - b_ω) * k)^T ]^T
```
- Construct 3 columns from orthogonal dataset measurements
- Compute best-fit rotation matrix from approximation

---

## 2. Accelerometer Calibration

### 2.1 Calibration Model

**Vector form (Eq 6.6):**
```
a = C_R^(-1) * S_a * (u_a - b_a)
```

**Expanded 3×3 form (Eq 6.7):**
```
[ ax ]   [ ra11 ra12 ra13 ] [ sax  0    0    ]^-1 [ uax  ]
[ ay ] = [ ra21 ra22 ra23 ] [ 0    say 0    ] [ -bay ]
[ az ]   [ ra31 ra32 ra33 ] [ 0    0    saz ] [  uaz ]
```

Where:
- a = calibrated acceleration (g)
- u_a = uncalibrated accelerometer output (lsb)
- C_R = rotation matrix for accelerometer alignment to calibrated frame
- S_a = diagonal sensitivity matrix (lsb per g)
- b_a = accelerometer bias vector (lsb)

### 2.2 Calibration Steps

**Step 1: Collect dataset**
- 6 orientations on level surface, each axis aligned with gravity (+1g and -1g)

**Step 2: Bias (Eq 6.5.2, Eq 6.8)**
```
b_a = (u_+g + u_-g) / 2
```
- u_+g, u_-g = axis outputs at +1g and -1g references

**Step 3: Sensitivity (Eq 6.5.2, Eq 6.9)**
```
s_a = (|u_+g| + |u_-g|) / (2 * g)
```
- g = 1 (normal gravity)

**Step 4: Alignment (Eq 6.5.3, Eq 6.10)**
```
C_R ~ [ (k * S_a^-1 * (u_ax - b_a) * k)^T,
        (k * S_a^-1 * (u_ay - b_a) * k)^T,
        (k * S_a^-1 * (u_az - b_a) * k)^T ]^T
```
- Construct 3 columns from orthogonal dataset measurements
- Compute best-fit rotation matrix from approximation

### 2.3 Inclination Angle Formulas (Eq 6.11–6.14)

**Normalize measurements:**
```
n̂ = u / ||u||   (uncalibrated)
â = a / ||a||   (calibrated)
```

**Inclination angles:**
```
θx = atan2(ny, nz)
θy = atan2(nz, nx)
θz = atan2(nx, ny)
```

---

## 3. Magnetometer Calibration

### 3.1 Calibration Model

**Vector form (Eq 6.15):**
```
m = S * u - h
```

**Expanded 3×3 form (Eq 6.16):**
```
[ mx ]   [ sxx sxy sxz ] [ ux ]   [ hx ]
[ my ] = [ syx syy syz ] [ uy ] - [ hy ]
[ mz ]   [ szx szy szz ] [ uz ]   [ hz ]
```

Where:
- m = calibrated magnetometer measurement (Gauss)
- u = uncalibrated output (lsb)
- S = soft-iron matrix (scales and couples axes)
- h = hard-iron bias (Gauss)

### 3.2 Calibration Method 1: Offset (Hard-Iron Only)

**Purpose:** Removes constant magnetic offset; simplest calibration.

**Bias formula:**
```
h = S^-1 * h_measured
```

**Measurement:**
```
h_measured = mean(u)  while stationary in uniform magnetic field
```

**Result:**
```
u_calibrated = u - h_measured
```

**Limitation:** No axis scaling or cross-axis coupling correction.

---

### 3.3 Calibration Method 2: Ellipsoid (Soft-Iron Only)

**Purpose:** Fits an ellipsoid to magnetometer measurements to correct scaling and cross-axis coupling.

**Model (S^-1 form):**
```
(u - h)^T * S^-1 * (u - h) = 1
```

Where S^-1 is 3×3 symmetric positive-definite:
```
S^-1 = [ a   b   c ]
       [ b   d   e ]
       [ c   e   f ]
```

**Measurement procedure:**
- Collect N samples in varying magnetic fields (e.g., rotate device 360°)
- Build matrix equation: A * x = b
- Solve for ellipsoid parameters

**Matrix form:**
```
A = [ u1^T * u1     u1^T * u2     ... ]
    [ u2^T * u1     u2^T * u2     ... ]
    [   ...              ...       ]
    [ un^T * u1     un^T * u2     ... ]

b = [ 1 ]
    [ 1 ]
    [   ... ]
    [ 1 ]
```

**Solve:**
```
x = (A^T * A)^(-1) * A^T * b
```

Where x = [a, b, c, d, e, f]^T

**Compute S:**
```
S = S^-1^-1
```

**Calibrated measurement:**
```
u_calibrated = S * (u - h)
```

**Limitation:** Assumes symmetric ellipsoid; does not separate hard-iron bias cleanly.

---

### 3.4 Calibration Method 3: Alignment (Full Calibration)

**Purpose:** Complete calibration with hard-iron bias, soft-iron scaling/coupling, and axis alignment.

**Vector model:**
```
m = C_R^(-1) * S * (u - h)
```

Where:
- C_R = rotation matrix for magnetometer alignment
- S = soft-iron matrix (3×3, symmetric)
- h = hard-iron bias vector

**Measurement procedure:**
- Collect N samples in varying magnetic fields (e.g., 3D rotation)
- Build linear system for S and h
- Solve for S and h
- Compute C_R from alignment dataset

**Two-stage solution:**

**Stage 1: Solve for S and h**
```
m_i = S * u_i - h   for all measurements i
```

**Stage 2: Solve for alignment matrix C_R**
```
C_R approximates the rotation from measured frame to calibrated frame
```

**Implementation:**
```
h = S^-1 * S * h_measured   (iterative refinement)
C_R = best_fit_rotation(m_calibrated_directions, u_raw_directions)
```

**Calibrated measurement:**
```
m_calibrated = C_R^(-1) * S * (u - h)
```

**Limitation:** Requires more measurement diversity and computational complexity.

---

## 4. Summary of Parameters

| Parameter | Symbol | Description | Units |
|-----------|--------|-------------|-------|
| Gyro bias | b_ω | Zero-rate output offset | lsb |
| Gyro sensitivity | s_ω | Output per unit angular velocity | lsb/(°/s) |
| Gyro alignment | C_R | Rotation matrix | dimensionless |
| Accel bias | b_a | Zero-g offset | lsb |
| Accel sensitivity | s_a | Output per g | lsb/g |
| Accel alignment | C_R | Rotation matrix | dimensionless |
| Mag hard-iron | h | Constant offset | Gauss |
| Mag soft-iron | S | 3×3 scaling/coupling matrix | dimensionless |
| Mag alignment | C_R | Rotation matrix | dimensionless |

---

*Generated from Madgwick PhD Thesis, Chapter 6*
