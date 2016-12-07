from koopsaclib import *
import numpy as np
dt = 0.1
N = 5
di = DoubleIntegrator(dt)
cost = CostFunction(dt)
sac = SAC(di, cost)

unom = np.array([0.0,0.0])
u0 = [np.array([0.0,0.0])]*N
x0 = np.array([0.2,0.3,0.0,0.0])
tcurr = 0
T = N*dt

import matplotlib.pyplot as plt

xarr = x0
import time
plt.ion()

while tcurr < 50:
    ta = time.clock()
    u0 = sac.control(x0, u0, tcurr, T, dt)
    tb = time.clock()
    print 'time: ', tb-ta
    x0 = di.f(x0, u0[0])
    xarr = np.vstack((xarr,x0))
    print u0[0]
    u0.pop(0)
    u0.append(unom)
    plt.clf()
    plt.step([tcurr+i*dt for i in range(len(u0))],u0)
    plt.ylim(-sac.umax[0],sac.umax[0])
    plt.draw()
    plt.pause(0.01)

    tcurr += dt
    print x0

plt.plot(xarr[:,0], xarr[:,1])
# plt.plot(xarr[:,1])
plt.show()
