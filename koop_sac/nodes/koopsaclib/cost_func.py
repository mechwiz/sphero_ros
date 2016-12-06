import numpy as np
from numpy import pi
from numpy.linalg import norm
from scipy.integrate import nquad, trapz
from odeint2 import monte_carlo

class CostFunction:
    '''
    Class for defining the ergodic cost function
    '''
    def __init__(self, basis, phi, coef, xlim):

        self.Q = np.diag([10.0]*4)
        self.R = np.diag([0.01]*2)

    def l(self, x, u):
        '''
        running cost
        '''
        return 0.5 * x.dot(self.Q).dot(x) + 0.5*u.dot(self.R).dot(u)

    def ldx(self, x, u):
        '''
        derivative of running cost wrt to the states
        '''
        drunning_cost = self.Q.dot(x) + self.R.dot(u)
        return drunning_cost

    def get_cost(self, x, u):
        '''
        J = integral l(x,u) + m(x(T))
        '''
        kf = len(x)
        for k in range(kf-1):
            J += self.l(x[k], u[k])
        return J
