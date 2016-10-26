import numpy as np
# from scipy.integrate import odeint
import trep
from trep import tx, ty
# import collisions as col
# from collisions import detect
from odeint2 import rk4
from scipy import interpolate

class Single_Integrator:
    '''
    default class for a single integrator
    Used for the Sphero SAC code so no need for collision detection or trep
    '''
    def __init__(self):
        '''
        class constructor for the SIngle integrator,
        '''

        self._nX = 2
        self._nU = 2


    def f(self, x, t, u, *args):
        '''
        x[0] = u1
        x[1] = u2
        '''
        dxdt = np.array([u(t)[0],u(t)[1]])
        return dxdt
    def fdx(self, x, t, u):
        '''
        df/dx linearization
        '''
        dfdx = np.array([
                            [0.,0.],
                            [0.,0.]
                            ])
        return dfdx
    def fdu(self, x, t, u):
        '''
        df/du linearization
        '''
        x = x(t)
        u = u(t)
        dfdu = np.array([
                        [1.,0.],
                        [0.,1.]])
        return dfdu

    def simulate(self, x0, u0, t0, tf, dt=0.01, args=(None,)):
        '''
        forward simualte the double integrator dynamics.
        x0: state at time t0
        u0: control as a function of time u0(t), passing a function

        function should return a time vector t and a interpolating function x
        '''
        t = np.linspace(t0, tf, (tf-t0)/dt, endpoint=True)
        xsol = rk4(self.f, x0, t, dt, *(u0,))
        # xsol = odeint(self.f, x0, t, args=(u0,)+args)
        return (t, interpolate.interp1d(t, xsol.T, fill_value="extrapolate")) # return an interpolating function
