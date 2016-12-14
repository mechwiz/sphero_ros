import numpy as np
import scipy.io as sio
from koop_fun import phi, dphi
class Koopman(object):
    def __init__(self):
        k = sio.loadmat('/home/anon/ros_ws/src/sphero_ros/koop_sac/nodes/Data_Export.mat')
        # K = np.load('/home/anon/ros_ws/src/sphero_ros/koop_sac/nodes/kmat.npy')
        self.K = k['K']
        # self.K = K
        self.K = self.K[0:4,0:6]
    def step(self, x, u):
        phipo =  self.K.dot(np.hstack((x,u)))
        return phipo

    def dK(self, x, u):
        self.K.dot(self.dphi(np.hstack((x,u))))
        return None
    def phi(self, x):
        return phi(x)

    def dphi(self, x):
        return dphi(x)
