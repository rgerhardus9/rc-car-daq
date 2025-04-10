'''
Kalman Filter - Get velocity from GPS velocity data and IMU acceleration data
TODO:
    - Interrupts for when we call the function?
    - process_var = 0.1
    - gps_var = 2.0
    - Make test data
    - Read IMU and GPS
'''


import numpy as np

class KalmanFilter:

    # Initilization - dt = time b/t samples (IMU sample period)
    def __init__(self, dt, process_var, gps_var):
        self.dt = dt

        # State vector: [velocity, acceleration]
        self.x = np.zeros((2, 1))  # [v, a]

        # State covariance matrix
        self.P = np.eye(2)

        # State transition matrix
        self.A = np.array([[1, dt],
                           [0, 1]])

        # Measurement matrix (GPS measures velocity)
        self.H = np.array([[1, 0]])

        # Process noise covariance
        self.Q = process_var * np.eye(2)

        # Measurement noise covariance
        self.R = np.array([[gps_var]])

    # Call at frequency of IMU samples
    def predict(self, accel_measurement):
        # Update state transition with new acceleration
        self.x[1, 0] = accel_measurement
        self.x = self.A @ self.x

        self.P = self.A @ self.P @ self.A.T + self.Q
    
    # Call at GPS frequency
    def update(self, gps_velocity):
        z = np.array([[gps_velocity]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(2) - K @ self.H) @ self.P

    def get_velocity(self):
        return self.x[0, 0]