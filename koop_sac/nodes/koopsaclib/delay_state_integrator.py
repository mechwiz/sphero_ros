import numpy as np
from scipy.integrate import odeint
from odeint2 import rk4
from scipy import interpolate

from koopman import Koopman

class Delay_State(object):
    '''
    default class for a cart pendulum
    This makes code more generalizable for different integration schemes
    like VI's and what not
    '''
    def __init__(self,dt):
        # physical parameters of the cart pendulum

        self._nX = 4
        self._nu = 2
        self.dt = dt
        self.A = np.eye(self._nX) +\
            np.array([
                        [1.8246,-0.0297,-0.8486, 0.0306],
                        [0.0358, 1.8611, -0.0445, -0.8676],
                        [1.,0.,0.,0.],
                        [0.,1.,0.,0.],
                        ])
        self.B = np.array([
                        [0.0602,0.0060],
                        [0.0008,0.1160],
                        [0.,0.],
                        [0.,0.]
        ])

        self.bias = np.array([0.0128,0.0108,0.,0.])

        self.kop = Koopman()
        self._use_koop = False

    def f(self, x, u, *args):
        '''
        x[0] = x
        x[1] = y
        x[2] = xdot
        x[3] = ydot
        '''
        xkpo = self.A.dot(x) + self.B.dot(u) + self.bias#+ self.kop.step(x,u)
        # return self.kop.step(x,u)
        return xkpo
        # return self.kop.step(x,u)
    def fdx(self, x, u):
        '''
        df/dx linearization
        '''
        L = self.kop.K.dot(self.kop.dphidx(x,u))
        # np.set_printoptions(precision=3)
        # np.set_printoptions(suppress=True)
        return self.A #+ L
        # return self.A + self.kop.K[0:4,0:4]
        # return L[0:6,0:6]

    def fdu(self, x, u):
        '''
        df/du linearization
        '''
        # print self.kop.K.shape
        L = self.kop.K.dot(self.kop.dphidu(x,u))
        # print self.B + L
        # print L.shape
        # print L[0:6,0:6]
        #print L.shape
        return self.B #+ L
        # return self.B + self.kop.K[0:4,4:6]


    def simulate(self, x0, u0, t0, tf, dt=0.1, args=(None,)):
        '''
        forward simualte the cart pendulum dynamics.
        x0: state at time t0
        u0: control as a function of time u0(t), passing a function

        function should return a time vector t and a interpolating function x
        '''
        # t = np.linspace(t0, tf, (tf-t0)/dt, endpoint=True)
        # xsol = odeint(self.f, x0, t, args=(u0,)+args)
        # xsol = rk4(self.f, x0, t, dt, *(u0,))
        # return (t, interpolate.interp1d(t, xsol.T, fill_value="extrapolate")) # return an interpolating function
        N = int((tf - t0)/self.dt)
        X = [x0]
        for i in range(N):
            x0 = self.f(x0, u0[i])
            X.append(x0)
        return X
