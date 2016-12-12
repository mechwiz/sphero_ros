import numpy as np
from numpy import pi
from numpy.linalg import norm
from scipy.integrate import nquad, trapz
from odeint2 import monte_carlo

class CostFunction:
    '''
    Class for defining the ergodic cost function
    '''
    def __init__(self, dt):

        self.dt = dt
        self.Q = np.diag([100.0,100.0,0.1,0.1])
        self.R = np.diag([0.001]*2)
        self.xd = lambda k : np.array([0.3*np.cos(k)+0.5,0.3*np.sin(2*k)+0.5,0.0,0.0])
        # self.xd = lambda k : np.array([k,0.5*np.sin(k),0.0,0.0])
    def l(self, x, u, k):
        '''
        running cost
        '''

        return 0.5 * (x-self.xd(k)).dot(self.Q).dot(x-self.xd(k)) + 0.5*u.dot(self.R).dot(u)

    def ldx(self, x, u,k):
        '''
        derivative of running cost wrt to the states
        '''
        drunning_cost = self.Q.dot(x-self.xd(k))
        return drunning_cost

    def get_cost(self, x, u, t0):
        '''
        J = integral l(x,u) + m(x(T))
        '''
        kf = len(x)
        J = 0.0
        for k in range(kf-1):
            J += self.l(x[k], u[k], t0+k*self.dt )
        return J
