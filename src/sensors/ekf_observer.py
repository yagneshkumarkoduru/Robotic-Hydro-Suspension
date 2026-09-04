"""
Extended Kalman Filter (EKF) Sensor Fusion for Active Suspension State Estimation
Author: Yagnesh Kumar Koduru
Domain: State Estimation, Sensor Fusion, Autonomous Mechatronics

Fuses noisy MPU6050 vertical accelerometry (accelerometer + gyro) and 
VL53L0X Time-of-Flight (ToF) road preview distance to estimate:
- States: [z_s - z_us, dot{z}_s, z_us - z_r, dot{z}_us]^T
- Road disturbance profile z_r(t)
"""

import numpy as np


class SuspensionEKF:
    def __init__(self, ms=15.0, mus=2.5, ks=950.0, cs=45.0, kt=6500.0, ct=5.0, dt=0.001):
        self.ms = ms
        self.mus = mus
        self.ks = ks
        self.cs = cs
        self.kt = kt
        self.ct = ct
        self.dt = dt

        # State vector: x = [z_s - z_us, dot{z}_s, z_us - z_r, dot{z}_us]^T
        self.x = np.zeros((4, 1))

        # Continuous-time state transition matrix A
        self.A_c = np.array([
            [0.0,                   1.0,               0.0,                  -1.0],
            [-self.ks / self.ms,   -self.cs / self.ms, 0.0,           self.cs / self.ms],
            [0.0,                   0.0,               0.0,                   1.0],
            [self.ks / self.mus,    self.cs / self.mus, -self.kt / self.mus, -(self.cs + self.ct) / self.mus]
        ])

        # Input matrix B (for actuator force F_act)
        self.B_c = np.array([
            [0.0],
            [1.0 / self.ms],
            [0.0],
            [-1.0 / self.mus]
        ])

        # Discretize using first-order Taylor expansion: F = I + A*dt
        self.F = np.eye(4) + self.A_c * dt
        self.B = self.B_c * dt

        # Measurement matrix H
        # Measurements: y = [a_s (accelerometer), d_susp (rattlespace sensor)]^T
        # a_s = ddot{z}_s = (-ks*x_1 - cs*x_2 + cs*x_4 + F_act) / ms
        self.H = np.array([
            [-self.ks / self.ms, -self.cs / self.ms, 0.0, self.cs / self.ms],
            [1.0,                0.0,               0.0, 0.0]
        ])

        # Process noise covariance Q
        self.Q = np.diag([1e-6, 1e-4, 1e-4, 1e-3])

        # Measurement noise covariance R (IMU noise + ToF noise)
        self.R = np.diag([0.05, 1e-4])

        # Error covariance estimate P
        self.P = np.eye(4) * 0.01

    def predict(self, u_force):
        """EKF Prediction step with control input F_act."""
        self.x = self.F @ self.x + self.B * u_force
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x

    def update(self, z_meas, u_force):
        """
        EKF Correction step with sensor measurements:
        z_meas = [measured_a_s, measured_susp_deflection]^T
        """
        # Expected measurement including feedforward actuator force on accelerometer
        h_x = self.H @ self.x + np.array([[u_force / self.ms], [0.0]])
        y_residual = z_meas - h_x

        S = self.H @ self.P @ self.H.T + self.R
        K_gain = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K_gain @ y_residual
        I_KH = np.eye(4) - K_gain @ self.H
        # Joseph form for numerical positive semi-definiteness
        self.P = I_KH @ self.P @ I_KH.T + K_gain @ self.R @ K_gain.T

        return self.x
