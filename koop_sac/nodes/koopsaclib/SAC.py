from scipy.integrate import odeint
from odeint2 import rk4, euler
import numpy as np
from numpy import cos, sin, pi, dot, outer
from scipy import interpolate
import time

class SAC:
    '''
    A class for SAC controller.
    Takes in two other classes, system and cost which should have specific
    functions associated with them
    '''
    def __init__(self, system, cost):
        '''
        inheret the subclasses, I think there may be a better way of doing this, not sure at the moment
        this works though
        '''
        self.system = system
        self.A = self.system.A
        self.B = self.system.B
        self.dt = self.system.dt
        self.cost = cost
        self.R = self.cost.R
        self.umax = [1.,1.]
        '''
        Need to add the bunches of parameters that go along with SAC, yay.

        '''
        self.gamma = -555.
        self.omega = 0.25 # 0.55
        self.dtinit = 0.3

        '''
        Storage stuff to make things run faster..ish
        '''
        self.rho0 = np.zeros(self.system._nX)

    def rhok(self, rho, x, u, *args):
        '''
        adjoint differential equation
        x, u: both are functions of time (t)
        '''
        return -self.cost.ldx(x, u) - self.A.T.dot(rho)

    def back_sim(self, rho0, x, u, t0, tf, dt=0.1, args=(None,)):
        '''
        Solve for the backwards integrated adjoint differential equation
        '''
        # t = np.linspace(tf, t0, (tf-t0)/dt, endpoint=True)
        # rhosol = rk4(self.rhodot, rho0, t, -dt, *(x, ck, u))
        # rhosol = euler(self.rhodot, rho0, t, -dt, *(x, ck, u))
        # rhosol = odeint(self.rhodot, rho0, t, args=(x,ck,u)+args)
        # return (t, interpolate.interp1d(t, rhosol.T, fill_value="extrapolate"))
        N = len(x)
        rho = [None]*N
        rho[N] = rho0
        for i in reversed(range(1,N)):
            rho[i-1] = self.rhok(rho[i], x[i], u[i])

        return rho

    def calc_ustar(self, rho, x, u, alpha_d):
        '''
        calculate the optimal controller
        '''
        kf = len(x)-1
        ustar = [None]*kf
        for k in range(kf):
            B = self.B # might not be constant with koop
            lam = B.T.dot(outer(rho[k], rho[k]).dot(B))
            ustar[k] = np.linalg.inv(lam + self.R.T).dot( lam.dot(u[k]) + \
                            B.T.dot(rho[k])*alpha_d)
        return ustar

    def calc_dJdlam(self, ustar, rho, x, u):
        '''
        calculate the change in cost wrt infinitesimal application time
        '''
        kf = len(ustar)
        dJdlam = [None]*kf
        for k in xrange(kf):
            f1 = self.system.f(x[k], u[k]) # check if I want to make u def. a list
            f2 = self.system.f(x(t[k]), ustar[k])
            dJdlam[k] = rho(t[k]).dot(f2-f1)

        return dJdlam

    def sat(self, u):
        '''
        control saturation:
        I feel this should probably be a user def function...
        '''
        utemp = []
        u_unit = u/np.linalg.norm(u) # direction vector

        for i,ui in enumerate(u):
            if abs(ui) > self.umax[i]:
                # utemp.append(np.sign(u[i])*self.umax[i])
                u = np.array(self.umax)*u_unit
                break
            else:
                utemp.append(ui)
                # utemp.append(0)
        # return np.array(utemp)
        return u

    def control(self, x0, ck0, u0, tcurr, T, Jprev=None):
        '''
        Inputs:
        x0: current system state
        u0: default control, would an LQR make this better?
        tcurr: current controller time
        T: horizon time
        '''

        (t, xsol) = self.system.simulate(x0, u0, tcurr, tcurr+T) # simulate forward dynamics

        (_, rhosol) = self.back_sim(self.rho0, xsol, u0, tcurr, tcurr+T) # back sim adjoint

        Jinit = self.cost.get_cost(xsol, u0) # get the initial cost
        alpha_d = self.gamma*Jinit # regulate it
        ustar = self.calc_ustar(rhosol, xsol, u0, alpha_d) # get optimal control sequence
        dJdlam = self.calc_dJdlam(ustar, rhosol, xsol, u0) # calc change in cost wrt to application time
        tau = dJdlam.index(min(dJdlam)) # get the most optimal application time
        u2 = self.sat(ustar[tau]) # get the control at tau and saturate it
        u0[tau] = u2
        return u0
