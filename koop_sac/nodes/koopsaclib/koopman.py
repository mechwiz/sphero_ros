import numpy as np
import scipy.io as sio
from koop_fun import phi, dphi
class Koopman(object):
    def __init__(self):
        k = sio.loadmat('/home/anon/ros_ws/src/sphero_ros/koop_sac/nodes/Data_Export.mat')
        self.K = k['K']

    def step(self, x, u):
        # return self.K.dot(self.phi(np.hstack((x,u))))
        return self.K[:,0:6].dot(np.hstack((x,u)))

    def phi(self, x):
        return phi(x)

    def dphi(self, x):
        return dphi(x)
