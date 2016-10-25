import numpy as np
from numpy import pi
from numpy.linalg import norm
from scipy.integrate import nquad, trapz

class cost_functional:
    '''
    Class for defining the ergodic cost function
    '''
    def __init__(self, basis, phi, coef, xlim, T, div):

        self.basis = basis # load up the basis function
        self.coef = coef # load up the num of coefficients

        self.q = 10
        self.R = np.diag([0.01,0.01])

        # get the norm factor
        self.lamk = np.zeros(self.coef+1)
        for i in range(self.coef[0]+1):
            for j in range(self.coef[1]+1):
                self.lamk[i,j] = (1+norm([i,j]))**(-1/2)

        # calculate initial phik and ck
        self.div = div
        self.xarr = [np.arange(i[0],i[1],1./self.div) for i in xlim]
        self.mesh = np.meshgrid(self.xarr[0],self.xarr[1])
        self.xlim = xlim
        self.barr_lim = 0.01
        self.phi = phi
        self.T = T
        self.phik = np.zeros(self.coef+1)
        self.ck = np.zeros(self.coef+1)
        for i in range(self.coef[0]+1):
            for j in range(self.coef[1]+1):
                self.phik[i,j] = sum(sum(phi*self.basis.fk([i,j], self.mesh[0], self.mesh[1])))/self.div**2
        # self.phik = self.phik # this works for information that is not time varying

    def update_phik(self, phi, T):
        '''
        update phik based on EID update
        '''
        for i in range(self.coef[0]+1):
            for j in range(self.coef[1]+1):
                self.phik[i,j] = sum(sum(phi*self.basis.fk([i,j], self.mesh[0], self.mesh[1])))/self.div**2
        # self.phik = self.phik
        # self.phik = self.phik

    def calc_ck(self, x, t):
        '''
        Method to calculte robot egodicity
        '''
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
        J = sum(sum(self.lamk*(ck-self.phik)**2))
        kf = len(t)
        for k in range(kf-1):
            J += self.l(x, t[k], u)*(t[k+1]-t[k])
        return J

    def barr(self, x):

        '''
        Optional barrier function
        '''
        barr_temp = 0.
        if x[0] >= self.xlim[0][1]-self.barr_lim:
            barr_temp += 10000.*(x[0] - (self.xlim[0][1]-self.barr_lim))**2
        elif x[0] <= (self.xlim[0][0]+self.barr_lim):
            barr_temp += 10000.*(x[0] - (self.xlim[0][0]+self.barr_lim))**2

        if x[1] >= (self.xlim[1][1]-self.barr_lim):
            barr_temp += 10000.*(x[1] - (self.xlim[1][1]-self.barr_lim))**2
        elif x[1] <= (self.xlim[1][0]+self.barr_lim):
            barr_temp += 10000.*(x[1] - (self.xlim[1][0]+self.barr_lim))**2
        return barr_temp

    def dbarr(self, x):
        '''
        derivative of the barrier function
        '''
        dbarr_temp = np.zeros(len(x))
        if x[0] >= self.xlim[0][1]-self.barr_lim:
            dbarr_temp[0] += 2*10000.*(x[0] - (self.xlim[0][1]-self.barr_lim))
        elif x[0] <= (self.xlim[0][0]+self.barr_lim):
            dbarr_temp[0] += 2*10000.*(x[0] - (self.xlim[0][0]+self.barr_lim))

        if x[1] >= (self.xlim[1][1]-self.barr_lim):
            dbarr_temp[1] += 2*10000.*(x[1] - (self.xlim[1][1]-self.barr_lim))
        elif x[1] <= (self.xlim[1][0]+self.barr_lim):
            dbarr_temp[1] += 2*10000.*(x[1] - (self.xlim[1][0]+self.barr_lim))

        return dbarr_temp
