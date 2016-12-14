import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

kxdat = np.loadtxt('data20161214-121012/robot.csv')
kudat = np.loadtxt('data20161214-121012/cmd.csv')

xdat = np.loadtxt('data20161214-121113/robot.csv')
udat = np.loadtxt('data20161214-121113/cmd.csv')

vel = 1.0
xd = lambda k : np.array([0.4*np.cos(vel*k)+0.5, 0.4*np.sin(2*vel*k)+0.5])

tk = np.arange(0,30,0.1)
xtarg = np.array(map(xd, tk))

N = len(xdat)
tf = int(0.75*N)
plt.plot(xtarg[:,0], xtarg[:,1], 'k', ls='-.', label='Target Trajectory')
plt.plot(xdat[0:tf,1], xdat[0:tf,2],'b',label='Nominal Model')
plt.plot(kxdat[0:tf,1], kxdat[0:tf,2],'r',label='Koopman ')
plt.legend()
plt.show()
