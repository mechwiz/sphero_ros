import numpy as np
from numpy import pi
from scipy.stats import multivariate_normal
from odeint2 import monte_carlo

from ekf import EKF, wrap2Pi
from eid import EID

class BearingOnly(EKF, EID):
    ''' sensor class that inherits the EKF object'''
    def __init__(self, param, mean, cov):
        # initialize the EKF
        EKF.__init__(self, mean, cov)
        EID.__init__(self)
        self.param = param
        self.n_param = len(self.param)
        self._hdx = np.zeros((self.n_meas, self.n_param))
        self.__drop_off = 0.3
        self.__Rv = 0.1 ** 2

    def h(self, sensor_state, param=None):
        ''' measurement model '''

        if param is None:
            if np.sqrt((self.param[1] - sensor_state[1])**2 + (self.param[0] - sensor_state[0])**2) > self.__drop_off:
                return None
            else:
                return wrap2Pi(np.arctan2( self.param[1] - sensor_state[1] , self.param[0] - sensor_state[0] ) + np.random.normal(0, self.__Rv))
        else:
            return wrap2Pi(np.arctan2( param[1] - sensor_state[1] , param[0] - sensor_state[0] ))

    def hdx(self, sensor_state, param=None):
        ''' measurement model wrt to parameter '''
        eps = 1e-5
        if param is None:
            h0 = self.h(sensor_state, self.param)
            for i in range(self.n_param):
                param_temp = self.param
                param_temp[i] += eps
                self._hdx[:,i] = (self.h(sensor_state, param_temp) - h0) / eps
            return self._hdx
        else:
            h0 = self.h(sensor_state, param)
            for i in range(self.n_param):
                param_temp = param
                param_temp[i] += eps
                self._hdx[:,i] = (self.h(sensor_state, param_temp) - h0) / eps
            return self._hdx
    '''
    Explicit model proves so be slightly faster,
    but I am having trouble getting the right equation written for it
    '''
    # def hdx(self, sensor_state, param=None):
    #     ''' measurement model wrt to parameter '''
    #     # hdx = np.zeros((self.n_meas, self.n_param))
    #     # eps = 1e-5
    #     x = sensor_state
    #
    #     # if param is None:
    #     #     xt = self.param
    #     #     vdx1 = -(x[1]-xt[1])/((x[1]-xt[1])**2 + (x[0]-xt[0])**2)
    #     #     vdx2 = (x[0]-xt[0])/((x[1]-xt[1])**2 + (x[0]-xt[0])**2)
    #     #     hdx = np.array([[vdx1, vdx2]])
    #     #     return hdx
    #     # else:
    #     xt = param
    #     vdx1 = -(x[1]-xt[1])/((x[1]-xt[1])**2 + (x[0]-xt[0])**2)
    #     vdx2 = (x[0]-xt[0])/((x[1]-xt[1])**2 + (x[0]-xt[0])**2)
    #     hdx = np.array([[vdx1, vdx2]])
    #     return hdx
