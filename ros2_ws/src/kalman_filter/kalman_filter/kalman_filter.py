import numpy as np

class KalmanFilter:
    def __init__(self, dt=0.01, sigma_a=0.3, sigma_flow=0.05, sigma_b=0.01):
        self.dt_default = dt            # Default time step
        self.sigma_a = sigma_a          # Process noise (acceleration)
        self.sigma_flow = sigma_flow    # Measurement noise (optical flow)
        self.sigma_b = sigma_b          # Bias noise
        
        self.K_1 = None  # Kalman gain for debugging

        # State vector: [position, velocity, bias]
        self.x_hat = np.zeros((3, 1))  # Initial state
        # self.P = np.eye(3) * 0.01  # Initial covariance
        self.P = np.eye(3) * 0.1  # Initial covariance

        
        # State transition matrix (discrete time)
        self.A = np.array([[1, dt, 0],   # position = position + velocity * dt
                           [0, 1, -dt],  # velocity = velocity - bias * dt
                           [0, 0, 1]])   # bias remains constant

        self.I = np.eye(3)  # Identity matrix

        # Control input matrix (acceleration input)
        self.B = np.array([[0],   # No direct effect on position
                           [dt],  # velocity = velocity + acceleration * dt
                           [0]])  # No effect on bias

        # Process noise input matrix
        # self.E = np.array([[0, 0],
        #                    [0, 0],
        #                    [0, 1]])  # Bias noise

        # self.E_cont = np.array([
        #     [0.5*self.dt_default**2,   0],
        #     [    self.dt_default,      0],
        #     [0,           self.dt_default]
        # ])  

        self.Q_cont = np.array([[sigma_a**2, 0],
                                [0, sigma_b**2]])  # Process noise covariance

        # Observation model: measure velocity
        self.C = np.array([[0, 1, 0]])  # Observes velocity only
        self.R = np.array([[sigma_flow**2]])  # Measurement noise covariance

    def predict(self, u, dt=None):
        if dt is None:
            dt = self.dt_default
        # Update A and B with current dt
        Ad = np.array([[1, dt, 0],
                       [0, 1, -dt],
                       [0, 0, 1]])

        Bd = np.array([[0],
                       [dt],
                       [0]])
        
        # Ed = np.array([[0, 0],
        #                [0, 0],
        #                [0, dt]])
        
        Ed = np.array([
            [0.5*dt**2,   0     ],   # accel‐noise → position
            [   dt,       0     ],   # accel‐noise → velocity
            [   0,      dt     ]    # bias‐noise → bias
            ])
        
        Qd = Ed @ self.Q_cont @ Ed.T

        # Predict next state
        self.x_hat = Ad @ self.x_hat + Bd * u
        self.P = Ad @ self.P @ Ad.T + Qd

    def update(self, z):
        if abs(z) > 8.0:  # Example threshold (m/s), adjust based on system
            print("Warning: Unrealistic velocity measurement, skipping update")
            return
        z_vec = np.array([[z]])
        y_tilde = z_vec - self.C @ self.x_hat  # Measurement residual
        S = self.C @ self.P @ self.C.T + self.R  # Innovation covariance
        if np.abs(S) < 1e-6:  # Prevent division by near-zero
            S += 1e-6
        K = self.P @ self.C.T @ np.linalg.inv(S)  # Kalman gain
        self.K_1 = K  # Store Kalman gain for debugging
        self.x_hat = self.x_hat + K @ y_tilde  # Update state
        self.P = (self.I - K @ self.C) @ self.P  # Update covariance

    def get_state(self):
        return self.x_hat.flatten()  # Returns [position, velocity, bias]

