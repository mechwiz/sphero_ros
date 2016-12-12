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

        self.basis = basis # load up the basis function
        self.coef = coef # load up the num of coefficients

        self.q = 30
        self.R = np.diag([0.01,0.01])

        # get the norm factor
        self.lamk = np.zeros(self.coef+1)
        for i in range(self.coef[0]+1):
            for j in range(self.coef[1]+1):
                self.lamk[i,j] = (1+norm([i,j]))**(-4.0/2.0)

        # calculate initial phik and ck
        self.phik = np.zeros(self.coef+1)
        self.ck = np.zeros(self.coef+1)
        for i in range(self.coef[0]+1):
            for j in range(self.coef[1]+1):
                temp_fun = lambda x,y: phi(x,y)*self.basis.fk([i,j], x, y)
                # self.phik[i,j] = nquad(temp_fun, xlim)[0]
                self.phik[i,j] = monte_carlo(temp_fun, xlim, n=500)

        self.__ck_temp =  np.zeros(self.coef+1)

    def update_phik(self, phi, state):
        n = len(phi)
        for i in range(self.coef[0]+1):
            for j in range(self.coef[1]+1):
                temp = phi*self.basis.fk([i,j], state[0], state[1])
                self.phik[i,j] = np.sum(temp)/float(len(state[0]))

    def calc_ck(self, x, t):
        ck_temp = np.zeros(self.coef+1)

        x = x(t).T
        for i in range(self.coef[0]+1):
            for j in range(self.coef[1]+1):
                _fk = self.basis.fk([i,j], x[:,0], x[:,1])
                ck_temp[i,j] = trapz(_fk, x=t)

        return ck_temp

    def l(self, x, t, u):
        '''
        running cost
        '''
        return 0.5*u(t).dot(self.R).dot(u(t)) + self.barr(x(t))

    def ldx(self, x, ck, u, t):
        '''
        derivative of running cost wrt to the states
        '''
        x = x(t)
        drunning_cost = np.zeros(x.shape[0])
        for i in range(self.coef[0]+1):
            for j in range(self.coef[1]+1):
                drunning_cost += self.lamk[i,j] * (ck[i,j] - self.phik[i,j])*self.basis.dfk([i,j], x)

        drunning_cost = 2*self.q*drunning_cost + self.dbarr(x)
        return drunning_cost

    def get_cost(self, x, ck, u, t):
        '''
        J = integral l(x,u) + m(x(T))
        '''
        J = np.sum(self.lamk*(ck-self.phik)**2)
        kf = len(t)
        for k in range(kf-1):
            J += self.l(x, t[k], u)*(t[k+1]-t[k])
        return J

    def barr(self, x):

        '''
        Optional barrier function
        '''
        barr_temp = 0.
        if x[0] >= 0.99:
            barr_temp += 10000.*(x[0] - 0.99)**2
        elif x[0] <= 0.01:
            barr_temp += 10000.*(x[0] - 0.01)**2

        if x[1] >= 0.99:
            barr_temp += 10000.*(x[1] - 0.99)**2
        elif x[1] <= 0.01:
            barr_temp += 10000.*(x[1] - 0.01)**2
        return barr_temp

    def dbarr(self, x):
        '''
        derivative of the barrier function
        '''
        dbarr_temp = np.zeros(len(x))
        if x[0] >= 0.99:
            dbarr_temp[0] += 2*10000.*(x[0] - 0.99)
        elif x[0] <= 0.01:
            dbarr_temp[0] += 2*10000.*(x[0] - 0.01)

        if x[1] >= 0.99:
            dbarr_temp[1] += 2*10000.*(x[1] - 0.99)
        elif x[1] <= 0.01:
            dbarr_temp[1] += 2*10000.*(x[1] - 0.01)

        return dbarr_temp
