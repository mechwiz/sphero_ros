import numpy as np
from numpy import exp, sqrt, pi, cos, sin
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt
from scipy import interpolate
from odeint2 import monte_carlo

def rot(theta):
    return np.array([[cos(theta), -sin(theta)],[sin(theta), cos(theta)]])


class EID_solver:

    def __init__(self):

        #self.p = Pool(2)

        self.sigma = 0.2
        self.Sigma = np.linalg.inv(np.diag([self.sigma**2, self.sigma**2]))
        self.mean = np.array([0.5,0.5])
        self.ndiv = 20.
        self.param = np.meshgrid(np.arange(0,1,1./self.ndiv),
                        np.arange(0,1,1./self.ndiv))
        self.state_space = np.meshgrid(np.arange(0,1,1./self.ndiv),
                        np.arange(0,1,1./self.ndiv))

        self.param_lim = [[0,1],[0,1]]
        self.x_lim = [[0,1],[0,1]]
        belief_temp = lambda xc, yc: exp(- 80.0 * (xc - 0.5)**2) * exp(- 80.0 * (yc - 0.5)**2)
        normfact = monte_carlo(belief_temp, self.param_lim)
        self.belief = lambda xc, yc: belief_temp(xc,yc)/normfact
        # self.belief = lambda xc,yc: np.ones(xc.shape)
        # self.belief = np.ones(self.state_space[0].shape)

        self.R = 0.2


        self.rot = rot(pi/4)
        # self._phi = lambda x,y,xc,yc: -(xc-x)**4 - (yc-y)**4 + self.R**4
        # self._phi = lambda x,y,xc,yc: -abs(xc-x) - abs(yc-y) + self.R
        # self.phi = lambda x,y,xc,yc: exp(- 10. * ( self._phi(x,y,xc,yc)) )
        # self.phi = lambda x,y,xc,yc: (( self._phi(x,y,xc,yc)))
        self.phi = lambda x,y,xc,yc: 1.0/(1.0 + exp(-( self._phi(x,y,xc,yc))/0.01 ))

        self.n = 500

    def _phi(self, x,y,xc,yc):
        (xtilde, ytilde) = self.rot.dot([x,y])
        (xct, yct) = self.rot.dot([xc,yc])
        return -(xct-xtilde)**4 - (yct-ytilde)**4 + self.R**4

    def update_belief(self, x, beta):
        self.belief *= exp(-((self.phi(x[0],x[1],self.param[0],self.param[1])-beta)**2)/(2*self.sigma**2))
        self.belief = self.belief/(np.sum(self.belief)/self.ndiv**2)

        # belief_temp = lambda xc,yc: exp(-((self.phi(x[0],x[1],xc,yc)-beta)**2)/(2*self.sigma**2)) * self.belief(xc,yc)

        print 'Updating EID...'
        self._eid()
        print 'Done.'


    def _fim(self, x, y, xc, yc):
        fim = np.zeros((2,2))
        eps = 0.001
        up0 = self.phi(x, y, xc, yc)
        up_temp1 = (self.phi(x, y, xc+eps, yc) - up0)/eps
        up_temp2 = (self.phi(x, y, xc, yc+eps) - up0)/eps

        fim[0,0] = up_temp1 * up_temp1 / self.sigma**2
        fim[1,1] = up_temp2 * up_temp2 / self.sigma**2
        fim[0,1] = up_temp1 * up_temp2 / self.sigma**2
        fim[1,0] = up_temp2 * up_temp1 / self.sigma**2
        return fim

    def _eim(self, x, y):
        # psamp = [np.random.uniform(low=i[0], high=i[1], size=self.n) for i in self.param_lim]
        integrand = map(lambda xc, yc: self._fim(x,y,xc,yc)*self.belief(xc,yc), self.psamp[0], self.psamp[1])
        eim = np.sum(integrand, axis=0)/float(self.n)
        return eim

    def _eid(self):
        import time

        start_time = time.time()

        xsamp = [np.random.uniform(low=i[0], high=i[1], size=self.n) for i in self.x_lim]
        self.psamp = [np.random.uniform(low=i[0], high=i[1], size=self.n) for i in self.param_lim]

        # normal ditribution
        # xsamp = [np.random.normal(loc=(i[0]+i[1])/2, size=self.n) for i in self.x_lim]
        # self.psamp = [np.random.normal(loc=(i[0]+i[1])/2, size=self.n) for i in self.param_lim]
        # eim = map(self._eim, xsamp[0], xsamp[1])
        print 'getting EIM'
        eim = map(self._eim, self.state_space[0].ravel(), self.state_space[1].ravel())
        print 'done!!!'
        eid = map(np.linalg.det, eim)
        # normalize
        # eid = np.array(eid)/(np.sum(eid)/self.ndiv**2)
        # self.eid = eid.reshape(self.state_space[0].shape)
        eid /= np.sum(eid)/float(self.n)
        print 'EID'
        self.eid = eid.reshape(self.state_space[0].shape)
        elapsed_time = time.time() - start_time
        print elapsed_time, 'elapsed time'
        plt.imshow(self.eid, extent=(0,1,0,1), origin='lower')
        plt.colorbar()
        plt.show()


if __name__ == '__main__':
    EID = EID_solver()
    EID._eid()
