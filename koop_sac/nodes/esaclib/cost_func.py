import numpy as np
from numpy import pi
from numpy.linalg import norm
from scipy.integrate import nquad, trapz
from odeint2 import monte_carlo

class CostFunction:
    '''
    Class for defining the ergodic cost function
    '''
    def __init__(self):

        self.basis = basis # load up the basis function
        self.coef = coef # load up the num of coefficients

        self.Q = np.diag([10.0]*4)
        self.R = np.diag([0.01]*2)


    def l(self, x, t, u):
        '''
        running cost
        '''
        return 0.5 * x.dot(self.Q).dot(x) + 0.5*u(t).dot(self.R).dot(u(t))

    def ldx(self, x, ck, u, t):
        '''
        derivative of running cost wrt to the states
        '''
        drunning_cost = self.Q.dot(x) + self.R.dot(u)
        return drunning_cost

    def get_cost(self, x, ck, u, t):
        '''
        J = integral l(x,u) + m(x(T))
        '''
        kf = len(t)
        for k in range(kf-1):
            J += self.l(x, t[k], u)*(t[k+1]-t[k])
        return J
