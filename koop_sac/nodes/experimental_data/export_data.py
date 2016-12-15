import numpy as np
from scipy.interpolate import interp1d
xdat = np.loadtxt('data20161214-105420/robot.csv')
udat = np.loadtxt('data20161214-105420/cmd.csv')

t0 = udat[0,0]
tx = xdat[:,0] - t0
tu = udat[:,0] - t0

xvel = np.array([0.,0.])
for i in range(len(xdat)-1):
    dt = xdat[i+1,0] - xdat[i,0]
    dx = xdat[i+1,1:3] - xdat[i,1:3]
    dxdt = dx/dt
    xvel = np.vstack((xvel, dxdt))

xdat = np.hstack((xdat, xvel))
print tu[0]
dt = 0.05
ti = np.arange(tu[0],tx[-1], dt)
print tx[-1], tu[-1]
xinterp = interp1d(tx, xdat[:,1:5].T)
xarr = xinterp(ti).T

uinterp = interp1d(tu, udat[:,1:3].T/255., fill_value='extrapolate')
uarr = uinterp(ti).T

np.save('xdat', xarr)
np.save('cmd', uarr)

# import matplotlib.pyplot as plt
# plt.plot(xarr[:,2])
# plt.plot(xarr[:,3])
# plt.show()
