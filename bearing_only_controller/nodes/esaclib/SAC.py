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
        self.A = self.system.fdx
        self.B = self.system.fdu
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

    def rhodot(self, rho, t, x, ck, u, *args):
        '''
        adjoint differential equation
        x, u: both are functions of time (t)
        '''
        return -self.cost.ldx(x, ck, u, t) - self.A(x, t, u).T.dot(rho)

    def back_sim(self, rho0, x, ck, u, t0, tf, dt=0.1, args=(None,)):
        '''
        Solve for the backwards integrated adjoint differential equation
        '''
        t = np.linspace(tf, t0, (tf-t0)/dt, endpoint=True)
        rhosol = rk4(self.rhodot, rho0, t, -dt, *(x, ck, u))
        # rhosol = euler(self.rhodot, rho0, t, -dt, *(x, ck, u))
        # rhosol = odeint(self.rhodot, rho0, t, args=(x,ck,u)+args)
        return (t, interpolate.interp1d(t, rhosol.T, fill_value="extrapolate"))

    def calc_ustar(self, rho, x, u, t, alpha_d):
        '''
        calculate the optimal controller
        '''
        kf = len(t)
        ustar = [None]*kf
        for k in range(kf):
            B = self.B(x,t[k],u)
            lam = B.T.dot(outer(rho(t[k]), rho(t[k])).dot(B))
            ustar[k] = np.linalg.inv(lam + self.R.T).dot( lam.dot(u(t[k])) + \
                            B.T.dot(rho(t[k]))*alpha_d)

        return interpolate.interp1d(t, np.array(ustar).T, fill_value="extrapolate")

    def calc_dJdlam(self, ustar, rho, x, u, t):
        '''
        calculate the change in cost wrt infinitesimal application time
        '''
        kf = len(t)
        dJdlam = [None]*kf
        for k in xrange(kf):
            f1 = self.system.f(x(t[k]), t[k], u)
            f2 = self.system.f(x(t[k]), t[k], ustar)
            dJdlam[k] = rho(t[k]).dot(f2-f1)

        return dJdlam

    def sat(self, u):
        '''
        control saturation:
        I feel this should probably be a user def function...
        '''
        utemp = []
        for i in xrange(len(u)):
            if abs(u[i]) > self.umax[i]:
                utemp.append(np.sign(u[i])*self.umax[i])
            else:
                utemp.append(u[i])
        return np.array(utemp)

    def uinterval(self, t, tau):
        '''
        Nice interval code written by Katie originally in mathematica that restricts the
        line search control window to only the simulated horizon t0->t0+T
        '''
        if tau[0] < t[0]:
            tau[0] = t[0]
        else:
            tau[0] = tau[0]
        if tau[1] > t[1]:
            tau[1] = t[1]
        else:
            tau[1] = tau[1]
        return tau


    def line_search(self, tau, u2_val, Jinit, x0, ck0, tcurr, T, dJmin=0., kmax=10):
        '''
        Now it works. Should look into making this faster, second derivative??
        '''
        Jnew = np.inf
        omega = 0.55
        k = 0
        '''
        what if I did not want to do this line search mechanism??
        '''
        lam = self.dtinit*self.omega**k
        # generate the control interval and make sure that it is within bounds
        # (tau0, tauf) = (tau-lam/2 , tau+lam/2)
        # (tau0, tauf) = self.uinterval([tcurr, tcurr+T], [tau0, tauf])
        # def u2_temp(t): # create a piecewise function that will be used for control
        #     if tau0 <= t <= tauf:
        #         return u2_val
        #     else:
        #         return np.array([0.,0.])

        while Jnew-Jinit > dJmin:
            lam = self.dtinit*self.omega**k
            # generate the control interval and make sure that it is within bounds
            (tau0, tauf) = (tau-lam/2 , tau+lam/2)
            (tau0, tauf) = self.uinterval([tcurr, tcurr+T], [tau0, tauf])

            def u2_temp(t): # create a piecewise function that will be used for control
                if tau0 <= t <= tauf:
                    return u2_val
                else:
                    return np.array([0.,0.])

            (t, xsol) = self.system.simulate(x0, u2_temp, tcurr, tcurr+T)
            ck2 = self.cost.calc_ck(xsol, t)
            Jnew = self.cost.get_cost(xsol, (ck0*tcurr+ck2)/t[-1], u2_temp, t)
            k += 1
            if k > kmax:
                break
        return (u2_temp, [tau0, tauf])

    def control(self, x0, ck0, u0, tcurr, T, Jprev=None):
        '''
        Inputs:
        x0: current system state
        u0: default control, would an LQR make this better?
        tcurr: current controller time
        T: horizon time
        '''

        (t, xsol) = self.system.simulate(x0, u0, tcurr, tcurr+T) # simulate forward dynamics
        ck1 = self.cost.calc_ck(xsol, t)

        (_, rhosol) = self.back_sim(self.rho0, xsol, (ck0*tcurr+ck1)/t[-1], u0, tcurr, tcurr+T) # back sim adjoint

        Jinit = self.cost.get_cost(xsol, (ck0*tcurr+ck1)/t[-1], u0, t) # get the initial cost
        alpha_d = self.gamma*Jinit # regulate it
        ustar = self.calc_ustar(rhosol, xsol, u0, t, alpha_d) # get optimal control sequence
        dJdlam = self.calc_dJdlam(ustar, rhosol, xsol, u0, t) # calc change in cost wrt to application time
        tau = t[dJdlam.index(min(dJdlam))] # get the most optimal application time
        u2 = self.sat(ustar(tau)) # get the control at tau and saturate it
        (u2, tspan) = self.line_search(tau, u2, Jinit, x0, ck0, tcurr, T) # do line search to get the application window

        return (tspan, u2)
