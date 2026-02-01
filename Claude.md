Goal:
rebuild the ops9 theory into a usable system.

Hardware:
1. amt103
2. wit hwt903
3. esp32

Protocol:
uart for imu

read this before you start:
# **Planar Positioning System Algorithm Explanation**

## **1. System Overview**

### **Core Principle: Sensor Fusion**
The system combines **inertial measurement** (gyroscope) with **wheel odometry** to achieve accurate planar positioning. This fusion compensates for the weaknesses of each method:
- **Gyroscope alone**: Accumulates drift over time
- **Odometry alone**: Susceptible to wheel slip and requires accurate heading

### **Key Innovation**
The system introduces an **auxiliary coordinate system** to correct **mechanical non-orthogonality errors** in the wheel arrangement, which is critical for high precision.

---

## **2. Mathematical Foundation**

### **Coordinate Systems**
1. **Navigation Coordinate System (NCS)**: Global reference frame, fixed at initialization
2. **Body Coordinate System (BCS)**: Attached to the moving platform
3. **Auxiliary Coordinate System (ACS)**: Accounts for wheel non-orthogonality

### **Notation**
- \( \theta \): Heading angle (yaw)
- \( \omega_x, \omega_y, \omega_z \): Angular velocities
- \( \Delta \): Small change/increment
- \( R \): Wheel radius
- \( \gamma \): Non-orthogonality angle between wheels

---

## **3. Core Algorithm Components**

### **3.1 Gyroscope Error Compensation Pipeline**

#### **Step 1: Innovation Adaptive Kalman Filter (IAKF)**
**Purpose**: Remove Gaussian white noise from raw gyroscope readings.

**State Equation**:
\[
X_{k+1} = X_k + w_k
\]
**Measurement Equation**:
\[
Z_{k+1} = X_{k+1} + v_{k+1}
\]
Where:
- \( X \): True angular velocity (state)
- \( Z \): Measured angular velocity
- \( w \): System noise (covariance \( Q \))
- \( v \): Measurement noise (covariance \( R \))

**Adaptive Update of Q**:
\[
Q_{k+1} = K_{k+1}^2 \cdot C_{k+1}
\]
\[
C_{k+1} = \frac{1}{N} \sum_{j=j_0}^{k+1} (Z_j - X_j)^2
\]
Where \( N \) is the sliding window size (typically 10).

#### **Step 2: Temperature Drift Compensation**
**Model**: Linear relationship between temperature and zero-bias:
\[
\Delta I_t = k \cdot \Delta T + b
\]
Where:
- \( \Delta I_t \): Temperature-induced drift
- \( \Delta T \): Temperature change
- \( k, b \): Fitted coefficients using least squares

**Implementation**:
1. Collect temperature-drift pairs during warm-up
2. Perform linear regression to get \( k, b \)
3. Apply compensation: \( \omega_{corrected} = \omega_{raw} - (k\Delta T + b) \)

#### **Step 3: Threshold Processing**
**Purpose**: Eliminate tiny residual drifts that could accumulate.

**Algorithm**:
```
if |ω_compensated| < threshold (0.15°/s):
    ω_final = 0
else:
    ω_final = ω_compensated
```

#### **Step 4: Dynamic State Compensation**

**A. Stationary State Detection & Compensation**
1. **Detection**: Both conditions for 20 consecutive samples (100ms):
   - |ω| < 0.04°/s
   - Encoder changes < 1 count
2. **Compensation (Progressive Convergence)**:
   ```
   while |ω_bias| > 0.0001°/s:
       ω_bias = ω_bias - sign(ω_bias) * 0.0001
   ```

**B. Vibration State Detection & Compensation**
1. **Detection**: Spectral analysis shows dominant frequency component
2. **Compensation Methods**:
   - **Zeroing**: Set ω = 0 during vibration
   - **Scaling**: ω_final = ω × scale_factor (e.g., 0.4)

---

### **3.2 Attitude Solver using Quaternions**

#### **Why Quaternions?**
- Avoid gimbal lock
- Efficient computation (no trigonometric functions in update)
- Numerically stable

#### **Quaternion Representation**
\[
Q = q_0 + q_1 i + q_2 j + q_3 k
\]
with constraint: \( q_0^2 + q_1^2 + q_2^2 + q_3^2 = 1 \)

#### **Attitude Update using Second-Order Runge-Kutta**
**Differential Equation**:
\[
\dot{Q} = \frac{1}{2} \Omega Q
\]
Where \( \Omega \) is the skew-symmetric matrix of angular velocities:
\[
\Omega = \begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z \\
\omega_x & 0 & \omega_z & -\omega_y \\
\omega_y & -\omega_z & 0 & \omega_x \\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
\]

**Runge-Kutta Solution (2nd Order)**:
```
k1 = 0.5 * Ω(t_n, Q_n) * Q_n * dt
k2 = 0.5 * Ω(t_n + dt, Q_n + k1) * (Q_n + k1) * dt
Q_{n+1} = Q_n + (k1 + k2)/2
```

#### **Quaternion to Euler Angles**
\[
\text{roll} = \arctan\left(\frac{2(q_0q_1 + q_2q_3)}{1 - 2(q_1^2 + q_2^2)}\right)
\]
\[
\text{pitch} = \arcsin(2(q_0q_2 - q_3q_1))
\]
\[
\text{yaw} = \arctan\left(\frac{2(q_0q_3 + q_1q_2)}{1 - 2(q_2^2 + q_3^2)}\right)
\]

---

### **3.3 Coordinate Calculation with Non-Orthogonal Correction**

#### **Problem Statement**
Wheels are never perfectly orthogonal. Actual angle = \( 90° ± \delta \).

#### **Auxiliary Coordinate System Solution**

**Case 1: Acute Angle (α < 90°)**
```
[X_body]   [cosγ  sinγ] [X_aux]
[Y_body] = [sinγ  cosγ] [Y_aux]
```
Where \( γ = (90° - α)/2 \)

**Case 2: Obtuse Angle (α > 90°)**
```
[X_body]   [cosγ  -sinγ] [X_aux]
[Y_body] = [-sinγ  cosγ] [Y_aux]
```

#### **Complete Coordinate Calculation Pipeline**

**Step 1: Calculate Auxiliary Displacement**
For each wheel (i = 0,1):
\[
\text{displacement}_i = \frac{2\pi R_i \cdot \Delta \text{encoder}_i}{4096 \cdot \cos(2\gamma)}
\]

**Step 2: Convert to Body Coordinates**
Using the transformation matrix above.

**Step 3: Fuse with Heading Angle**
Average heading during step:
\[
\theta_{avg} = \frac{\theta_{k-1} + \theta_k}{2}
\]

**Step 4: Calculate Navigation Coordinates**
\[
\begin{bmatrix}
X_n \\
Y_n
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta_{avg} & -\sin\theta_{avg} \\
\sin\theta_{avg} & \cos\theta_{avg}
\end{bmatrix}
\begin{bmatrix}
X_b \\
Y_b
\end{bmatrix}
\]

**Step 5: Integrate Position**
\[
\text{Position}_k = \text{Position}_{k-1} + \Delta \text{Position}
\]

---

## **4. Calibration Procedures**

### **4.1 Mechanical Parameter Calibration**
**Method**: Move known distance L in X and Y directions.

**Equations**:
\[
L_x = \frac{2\pi R_0 \cdot \text{enc}_0}{4096 \sin(\alpha/2)} + \frac{2\pi R_1 \cdot \text{enc}_1}{4096 \sin(\alpha/2)}
\]
\[
L_y = \frac{2\pi R_0 \cdot \text{enc}_0'}{4096 \cos(\alpha/2)} + \frac{2\pi R_1 \cdot \text{enc}_1'}{4096 \cos(\alpha/2)}
\]

Solve for \( R_0, R_1, \alpha \).

### **4.2 Gyroscope Calibration**
1. **Zero-bias**: Average reading when stationary
2. **Temperature coefficients**: Linear fit of drift vs. temperature
3. **Scale factor**: Compare integrated angle with known rotation

---

## **5. Implementation Architecture**

### **Data Flow**
```
Sensors → Error Compensation → Attitude Update → Coordinate Transformation → Position Output
    ↑          ↑                    ↑
Calibration  Parameters          Heading Feedback
```

### **Timing Requirements**
- **Sampling rate**: 200Hz (5ms period)
- **Algorithm latency**: < 2.5ms (to allow for I/O)
- **Output rate**: 100-200Hz

---

## **6. Key Mathematical Insights**

### **Error Propagation Model**
Total position error grows as:
\[
\epsilon_{total} = \epsilon_{gyro} \cdot t + \epsilon_{encoder} \cdot d + \epsilon_{mech}
\]
Where:
- \( \epsilon_{gyro} \): Gyro drift rate (°/hr)
- \( \epsilon_{encoder} \): Encoder scale error (%)
- \( \epsilon_{mech} \): Mechanical misalignment error

### **Why This Fusion Works**
1. **Short-term**: Odometry provides accurate displacement
2. **Long-term**: Gyroscope maintains accurate heading
3. **Mutual correction**: Each sensor corrects the other's weaknesses

---

## **7. Practical Considerations for ESP32 Implementation**

### **Numerical Stability**
- Use 32-bit floating point (ESP32 has FPU)
- Regular quaternion normalization: \( Q = Q / \|Q\| \)
- Handle encoder overflow properly (equation 4.19 in paper)

### **Real-time Constraints**
- Interrupt-driven sampling
- Fixed-point arithmetic for time-critical sections
- Pre-compute trigonometric values where possible

### **Memory Usage**
- Store calibration parameters in NVS/Flash
- Maintain sliding window for adaptive Kalman filter
- Buffer recent positions for velocity calculation

---

## **8. Algorithm Summary**

The system's accuracy comes from three key innovations:

1. **Multi-stage gyro compensation**: Static + dynamic + temperature
2. **Quaternion-based attitude**: Stable, efficient, no gimbal lock
3. **Auxiliary coordinate system**: Corrects mechanical imperfections

**Final Output**:
- Position (x, y) in millimeters
- Heading (θ) in degrees
- Velocity (vx, vy) in mm/s
- Update rate: 200Hz

This algorithm achieves **<2cm error per meter traveled** and **<0.1° heading accuracy** when properly calibrated and implemented.