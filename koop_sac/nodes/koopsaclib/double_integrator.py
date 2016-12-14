import numpy as np
from scipy.integrate import odeint
from odeint2 import rk4
from scipy import interpolate

from koopman import Koopman

class DoubleIntegrator(object):
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
                        [0.,0.,dt,0.],
                        [0.,0.,0.,dt],
                        [0.,0.,-0.25,0.],
                        [0.,0.,0.,-0.25],
                        ])
        self.B = np.array([
                        [0.,0.],
                        [0.,0.],
                        [0.4,0.],
                        [0.,0.4]
        ])

        self.kop = Koopman()
        self._use_koop = False

    def f(self, x, u, *args):
        '''
        x[0] = x
        x[1] = y
        x[2] = xdot
        x[3] = ydot
        '''
        xkpo = self.A.dot(x) + self.B.dot(u) #+ self.kop.step(x,u)
        # return self.kop.step(x,u)
        return xkpo
        # return self.kop.step(x,u)
    def fdx(self, x, u):
        '''
        df/dx linearization
        '''
        L = self.kop.K#.dot(self.kop.dphi(np.hstack((x,u))))
        # np.set_printoptions(precision=3)
        # np.set_printoptions(suppress=True)
        # print L[0:6,0:6]
        return self.A #+ L[0:4,0:4]
        # return L[0:6,0:6]

    def fdu(self, x, u):
        '''
        df/du linearization
        '''
        # print self.kop.K.shape
        L = self.kop.K#.T.dot(self.kop.dphi(np.hstack((x,u))))
        # print L.shape
        # print L[0:6,0:6]
        #print L.shape
        return self.B #+ L[0:4, 3:5]
        # return L[0:4,3:5]


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
