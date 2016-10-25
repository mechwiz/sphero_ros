import numpy as np
from numpy import pi
from scipy.stats import multivariate_normal
from odeint2 import monte_carlo

def wrap2Pi(x):
    ''' helper function '''
    x = np.mod(x+pi, 2*pi)
    if x < 0:
        x += 2*pi
    return x-pi

# def wrap2Pi(x):
#     ''' helper function '''
#     x = np.mod(x, 2*pi)
#     if x < 0:
#         x += 2*pi
#     return x


class EKF:
    ''' Class for making the EKF '''
    def __init__(self, mean, cov):
        ''' ekf init parameters '''
        self.mean = mean
        self.n_param = len(self.mean)
        self.n_meas = 1
        self.cov = cov
        # self.Q = np.zeros([self.n_param]*2)
        self.Q = np.diag([1.0/20.0 ** 2]*2)
        self.__Q = [1.0/20.0 ]*2
        self.R = np.diag([0.1 ** 2]*self.n_meas)
        self.invR = np.linalg.inv(self.R)
        self.belief = multivariate_normal(self.mean, self.cov)

    def f(self, param):
        return param + 0*np.random.normal(0, self.__Q)

    def fdx(self, param):
        return np.eye(self.n_param) + 0*np.random.normal(0, self.__Q)

    def h(self):
        return None

    def hdx(self):
        return None

    def update(self, sensor_state, measurement):

        ###################
        # prediction steps
        ###################
        xk = self.f(self.mean) # predict the measurement you expect
        A = self.fdx(sensor_state) # linearize parameter est
        H = self.hdx(sensor_state, xk) # linearize observation est
        P = A.dot(self.cov).dot(A.T) + self.Q # predicted covariance
        ###################
        # update steps
        ###################
        if measurement is not None:
            y = np.array([wrap2Pi(measurement - self.h(sensor_state, xk))]) # innovation
            S = H.dot(P).dot(H.T) + self.R
            K = P.dot(H.T).dot(np.linalg.inv(S)) # kalman gain
            # assign the new mean and cov
            self.mean = xk + K.dot(y)
            self.cov = (np.eye(self.n_param) - K.dot(H)).dot(P)
            self.belief = multivariate_normal(self.mean, self.cov)
        else:
            self.mean = xk
            self.cov = P
            self.belief = multivariate_normal(self.mean, self.cov)
