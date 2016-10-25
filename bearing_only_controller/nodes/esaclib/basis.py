import numpy as np
from numpy import pi, sin, cos
from math import sqrt
from scipy.integrate import nquad

class Basis:

    def __init__(self, coef, xlim):

        self.xlim = xlim
        self.dl = [i[1]-i[0] for i in self.xlim]
        self.hk = np.zeros(coef+1)
        for i in range(coef[0]+1):
            for j in range(coef[1]+1):
                self.hk[i,j] = sqrt(nquad(lambda y, x: (cos(pi*i*x)*cos(pi*j*y))**2,xlim)[0])

    def fk(self, k, x, y):
        return cos(pi*k[0]*x/self.dl[0])*cos(pi*k[1]*y/self.dl[1])/self.hk[k[0],k[1]]

    def dfk(self, k, x):
        dfk_temp = np.array([
                -k[0]*pi*sin(pi*k[0]*x[0]/self.dl[0])*cos(pi*k[1]*x[1]/self.dl[1])/self.hk[k[0],k[1]],
                -k[1]*pi*sin(pi*k[1]*x[1]/self.dl[1])*cos(pi*k[0]*x[0]/self.dl[0])/self.hk[k[0],k[1]],
                0,
                0
        ])
        return dfk_temp
