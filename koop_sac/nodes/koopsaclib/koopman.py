import numpy as np
import scipy.io as sio
from koop_fun3affine import phi, dphidx, dphidu
class Koopman(object):
    def __init__(self):
        # k = sio.loadmat('/home/anon/ros_ws/src/sphero_ros/koop_sac/nodes/Data_Export.mat')
        # k = sio.loadmat('/home/anon/ros_ws/src/sphero_ros/koop_sac/nodes/K3rdupdate.mat')
        # k = sio.loadmat('/home/anon/ros_ws/src/sphero_ros/koop_sac/nodes/K3rd.mat')
        k = sio.loadmat('/home/anon/ros_ws/src/sphero_ros/koop_sac/nodes/Kaffine.mat')
        # K = np.load('/home/anon/ros_ws/src/sphero_ros/koop_sac/nodes/kmat.npy')
        self.K = k['K']
        # self.K = K
        # self.K = self.K[0:4,0:6]
    def step(self, x, u):
        # phipo =  self.K.dot(np.hstack((x,u)))
        phipo = self.K.dot(self.phi(x,u))
        # phipo = self.K[0:4,0:6].dot(np.hstack((x,u)))
        return phipo

    def phi(self, x,u):
        return phi(np.hstack((x,u)))

    def dphidx(self, x, u):
        return dphidx(np.hstack((x,u)))

    def dphidu(self, x, u):
        return dphidu(np.hstack((x,u)))
