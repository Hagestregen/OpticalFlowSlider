#!/usr/bin/env python3
import numpy as np

class KalmanFilter:
    def __init__(self, dt=0.01, sigma_a=0.1, sigma_flow=0.001, sigma_b=0.1):
        """
        Initialize a simple Kalman Filter for a 1D system with state:
        [position, velocity, bias]^T.
        """
        self.dt_default = dt
        self.sigma_a = sigma_a
        self.sigma_b = sigma_b
        self.sigma_flow = sigma_flow

        # Initial state and covariance
        self.x_hat = np.zeros((3, 1))
        self.P = np.eye(3) * 0.01

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
        Ad = self.I + self.A * dt
        Bd = self.B * dt
        Ed = self.E * dt
        Qd = Ed @ self.Q_cont @ Ed.T

        self.x_hat = Ad @ self.x_hat + Bd @ np.array([[u]])
        self.P = Ad @ self.P @ Ad.T + Qd

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


# import numpy as np

# class KalmanFilter:
#     def __init__(self, dt=0.01, sigma_a=0.1, sigma_flow=0.001, sigma_b=0.1):
#         """
#         Initialize Kalman Filter with 2-step Taylor expansion.
        
#         Args:
#             dt (float): Time step (seconds)
#             sigma_a (float): Std dev of IMU acceleration noise (m/s^2)
#             sigma_flow (float): Std dev of optical flow velocity noise (m/s)
#             sigma_b (float): Std dev of acceleration bias noise (m/s^2)
#         """
#         # State vector: [p_x, v_x, b_accx]^T (3x1)
#         self.x_hat = np.zeros((3, 1))
        
#         # Covariance matrix: P_k|k (3x3)
#         self.P = np.eye(3) * 0.01
        
#         # Continuous-time system matrix: A
#         A = np.array([[0, 1, 0],
#                       [0, 0, -1],
#                       [0, 0, 0]])
        
#         # Discrete state transition matrix: Ad (2-step Taylor)
#         self.Ad = np.eye(3) + A * dt
        
#         # Continuous control input matrix: B
#         B = np.array([[0, 1, 0]]).transpose()
        
#         # Discrete control input matrix: Bd (2-step Taylor)
#         self.Bd = B * dt
        
#         # Observation matrix: Cd
#         C = np.array([[0, 1, 0]])
#         self.Cd = C
        
#         # Process noise input matrix: Ed
#         E = np.array([[0, 0],
#                       [-1, 0],
#                       [0, 1]])
#         self.Ed = E * dt
        
#         # Process noise covariance: Qd (corrected)
#         Q_cont = np.array([[sigma_a ** 2, 0],
#                            [0, sigma_b ** 2]])
#         self.Qd = self.Ed @ Q_cont @ self.Ed.T
        
#         # Measurement noise covariance: Rd (1x1)
#         self.Rd = np.array([[sigma_flow ** 2]])
        
#         self.dt = dt
        
    

#     def predict(self, u):
#         """
#         Prediction step using IMU acceleration.
        
#         Args:
#             u (float): IMU acceleration (m/s^2)
#         """
#         u_vec = np.array([[u]])
#         # x_k|k-1 = Ad * x_k-1|k-1 + Bd * u_k-1
#         self.x_hat = self.Ad @ self.x_hat + self.Bd @ u_vec # Predict next state
#         # P_k|k-1 = Ad * P_k-1|k-1 * Ad^T + Qd
#         self.P = self.Ad @ self.P @ self.Ad.T + self.Qd # 

#     def update(self, z):
#         """
#         Update step using optical flow velocity measurement.
        
#         Args:
#             z (float): Optical flow velocity (m/s)
#         """
#         z_vec = np.array([[z]])
#         # Innovation: y_tilde_k = z_k - Cd * x_k|k-1
#         y_tilde = z_vec - self.Cd @ self.x_hat # Difference between measurement and prediction
#         # Innovation covariance: S_k = Cd * P_k|k-1 * Cd^T + Rd
#         S = self.Cd @ self.P @ self.Cd.T + self.Rd # How much we trust camera vs guess
#         # Kalman gain: K_k = P_k|k-1 * Cd^T * S_k^-1
#         K = self.P @ self.Cd.T @ np.linalg.inv(S) # How much we ajust the guess based on camera
#         # Update state: x_k|k = x_k|k-1 + K_k * y_tilde_k
#         self.x_hat = self.x_hat + K @ y_tilde # Update guess (state estimate) based on camera
#         print("x_hat: ", self.x_hat)
#         # Update covariance: P_k|k = (I - K_k * Cd) * P_k|k-1
#         I = np.eye(3)
#         self.P = (I - K @ self.Cd) @ self.P 

#     def get_state(self):
#         """Return state estimate [p_x, v_x, b_accx]."""
#         return self.x_hat.flatten()