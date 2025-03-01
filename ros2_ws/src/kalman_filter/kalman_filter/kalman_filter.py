#!/usr/bin/env python3
import numpy as np

class KalmanFilter:
    def __init__(self, dt=0.01, sigma_a=0.1, sigma_flow=0.01, sigma_b=0.1):
        """
        Initialize a simple Kalman Filter for a 1D system with state:
        [position, velocity, bias]^T.
        """
        self.dt_default = dt
        self.sigma_a = sigma_a # Noise in acceleration measurements
        self.sigma_flow = sigma_flow # Noise in optical flow measurements
        self.sigma_b = sigma_b # Noise in the bias of the acceleration 
        

        # Initial state and covariance
        self.x_hat = np.zeros((3, 1))
        # self.P = np.eye(3) * 0.01
        self.P = np.eye(3) * 0.1

        # Continuous system matrix A
        self.A = np.array([[0, 1, 0],
                           [0, 0, -1],
                           [0, 0, 0]])
        self.I = np.eye(3)

        # Continuous control input matrix B
        self.B = np.array([[0],
                           [1],
                           [0]])

        # Process noise input matrix E and noise covariance for [acceleration, bias]
        self.E = np.array([[0, 0],
                           [-1, 0],
                           [0, 1]])
        self.Q_cont = np.array([[sigma_a**2, 0],
                                [0, sigma_b**2]])

        # Observation model: we measure the velocity (state index 1)
        self.C = np.array([[0, 1, 0]])
        self.R = np.array([[sigma_flow**2]])

    def predict(self, u, dt=None):
        """
        Propagate the filter state using the measured acceleration u.
        Optionally, use a custom time step dt.
        """
        if dt is None:
            dt = self.dt_default
        # Discrete-time state transition (first-order Taylor expansion)
        Ad = self.I + self.A * dt # Change to second order Taylor or use Runge_Kutta method?
        Bd = self.B * dt
        Ed = self.E * dt
        Qd = Ed @ self.Q_cont @ Ed.T

        self.x_hat = Ad @ self.x_hat + Bd @ np.array([[u]]) # Implements the mathematical model
        self.P = Ad @ self.P @ Ad.T + Qd # Updates the covariance

    def update(self, z):
        """
        Correct the state with an optical flow velocity measurement z.
        """
        z_vec = np.array([[z]])
        y_tilde = z_vec - self.C @ self.x_hat  # innovation
        S = self.C @ self.P @ self.C.T + self.R
        K = self.P @ self.C.T @ np.linalg.inv(S)
        self.x_hat = self.x_hat + K @ y_tilde
        self.P = (self.I - K @ self.C) @ self.P

    def get_state(self):
        """Return the current state estimate as a flat array."""
        return self.x_hat.flatten()
    
    