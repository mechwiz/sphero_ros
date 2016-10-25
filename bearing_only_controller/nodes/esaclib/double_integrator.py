import numpy as np
from scipy.integrate import odeint
from odeint2 import rk4
from scipy import interpolate

class DoubleIntegrator:
    '''
    default class for a cart pendulum
    This makes code more generalizable for different integration schemes
    like VI's and what not
    '''
    def __init__(self,forcing=True):
        # physical parameters of the cart pendulum
        self._nX = 4
        self._nu = 2
    def f(self, x, t, u, *args):
        '''
        x[0] = x
        x[1] = y
        x[2] = xdot
        x[3] = ydot
        '''
        dxdt = np.array([x[2],x[3],u(t)[0],u(t)[1]])
        return dxdt
    def fdx(self, x, t, u):
        '''
        df/dx linearization
        '''
        dfdx = np.array([
                            [0.,0.,1.,0.],
                            [0.,0.,0.,1.],
                            [0.,0.,0.,0.],
                            [0.,0.,0.,0.]])
        return dfdx
    def fdu(self, x, t, u):
        '''
        df/du linearization
        '''
        # x = x(t)
        # u = u(t)
        dfdu = np.array([
                        [0.,0.],
                        [0.,0.],
                        [1.,0.],
                        [0.,1.]])
        return dfdu

    def simulate(self, x0, u0, t0, tf, dt=0.1, args=(None,)):
        '''
        forward simualte the cart pendulum dynamics.
        x0: state at time t0
        u0: control as a function of time u0(t), passing a function

        function should return a time vector t and a interpolating function x
        '''
        t = np.linspace(t0, tf, (tf-t0)/dt, endpoint=True)
        # xsol = odeint(self.f, x0, t, args=(u0,)+args)
        xsol = rk4(self.f, x0, t, dt, *(u0,))
        return (t, interpolate.interp1d(t, xsol.T, fill_value="extrapolate")) # return an interpolating function
