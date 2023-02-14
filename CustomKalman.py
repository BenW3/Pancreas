#based on: https://machinelearningspace.com/2d-object-tracking-using-kalman-filter/

import numpy as np

class TwoDKalman():
    def __init__(self, initial_state, dt, stdX, stdY, stdVX, stdVY):
        self.dt = dt
        self.state = initial_state
        self.F = np.array([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        self.H = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        self.Q = np.eye(4)
        self.R = np.array([[stdX**2, 0, 0, 0],
                            [0, stdY**2, 0, 0],
                            [0, 0, stdVX**2, 0],
                            [0, 0, 0, stdVY**2]])
        self.P = np.eye(self.F.shape[1])
        self.I = np.eye(self.H.shape[1])
        # pass
    def update(self, measured_state):
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        kalman_gain = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.state = self.state + np.dot(kalman_gain, (measured_state - np.dot(self.H, self.state)))
        # print(self.state.shape)
        self.P = (self.I - (np.dot(kalman_gain,self.H)))*self.P
        return self.state[0:2]
        # pass
    def predict(self):
        self.state = np.dot(self.F, self.state)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        # pass
    def filter(self, measured_state):
        pass
# F = np.eye(5)