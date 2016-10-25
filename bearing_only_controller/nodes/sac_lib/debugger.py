'''
Debugging script just to make sure that this sytem works
'''

from single_integrator import Single_Integrator
from sensor import Tactile_Sensor
from erg_cost_func import cost_functional
from basis import basis
from SAC import SAC
import numpy as np
from scipy.integrate import nquad
from numpy import exp

# initial conditions
q0 = np.array([0.4,0.7])
x0 = np.hstack((q0))
u0 = lambda t: np.array([0.,0.])

# instantiate an system object, cost object, and the SAC object
tcurr = 0.
ts = 1./10.
T =0.8
tf = 20

si = Single_Integrator() # create the single integrator
sens = Tactile_Sensor() # generate the sensor class

# put u pthe coefficients
coef = np.array([10,10]) # this has to be a numpy array
xlim= [[0,1],[0,1]]
basis = basis(coef, xlim)

phi_temp = lambda x,y: exp( -50*(x - 0.5)**2)*exp( -50*(y - 0.5)**2) + \
            exp( -50*(x - 0.3)**2)*exp( -50*(y - 0.7)**2)
normfact = nquad(lambda x,y: phi_temp(x,y), xlim)[0]
phi = lambda x,y: phi_temp(x,y)/normfact

div = 40.0
X,Y = np.meshgrid(np.arange(0,1,1.0/div), np.arange(0,1,1.0/div))
phi_num = phi(X, Y)
# phi_num = np.ones(X.shape)

cost = cost_functional(basis, phi_num, coef, xlim, tf+T, div=div)
# cost = cost_functional(basis, phi_num, coef, xlim, 1.0, div=div)
sac = SAC(si, cost)

xx = np.array(x0)
tt = np.array(tcurr)
ck0 = np.zeros(coef+1)

# let's set up some plotting stuff
import matplotlib.pyplot as plt
from pylab import *
plt.ion()
plt.figure(1, facecolor='white')
while tcurr < tf:
    # calculate control
    (tau, u2) = sac.control(x0, ck0, u0, tcurr, T)
    #forward simulate
    (t, xsol) = si.simulate(x0, u2, tcurr, tcurr+ts)
    ck0 += cost.calc_ck(xsol, t)
    # ck0 = cost.calc_ck(xsol, t)
    x0 = xsol(t[-1])
    xx = np.vstack((xx, x0))
    tcurr += ts
    #if ck0[0,0] >= 1.0:
    #    ck0 = np.zeros(coef+1)

    # if collide:
    #     # ck0 = np.zeros(coef+1) # reset
    #     sens.update_belief(x0, True)
    #     cost.update_phik(sens.belief, tf+T-tcurr)
    #     #cost.update_phik(sens.belief, 1.0)
    # else:
    #     # ck0 = np.zeros(coef+1) # reset
    #     sens.update_belief(x0, False)
    #     cost.update_phik(sens.belief, tf+T-tcurr)
    #     #cost.update_phik(sens.belief, 1.0)
    print 'Time: ', tcurr, ' Ergodic Metric: ', cost.get_cost(xsol, ck0, u2, t)
    tt = np.hstack((tt, tcurr))

    plt.clf()
    plt.subplot(121)
    plt.imshow(phi_num,interpolation='gaussian', origin='lower',extent=(0,1,0,1))
    plt.plot(xx[:,0],xx[:,1],'g')

    # plt.subplot(122)
    # plt.clf()
    # plt.imshow(sens.boundary_func, origin='lower',extent=(0,1,0,1))
    # plt.contour(sens.param[0], sens.param[1],sens.boundary_func,colors=['k', 'k', 'k'],
                # linestyles=['--','-','--'],
                # levels=[-0.3,0.,0.3])
    # circ1 = plt.Circle((0.6,0.6), 0.1, color='g', fill=False)
    # circ2 = plt.Circle((0.4,0.3), 0.1, color='g', fill=False)
    # plt.gca().add_artist(circ1)
    # plt.gca().add_artist(circ2)
    # xs = np.linspace(0,1)
    # plt.plot(xs, 0.2*np.sin(2.0*np.pi*4.0*xs + np.pi/2)+0.3,'g')
    plt.draw()
