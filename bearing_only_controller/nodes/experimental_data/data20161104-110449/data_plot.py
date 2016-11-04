import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
import warnings
import matplotlib.cbook
warnings.filterwarnings("ignore",category=matplotlib.cbook.mplDeprecation)

mean = np.loadtxt('mean.csv')
target = np.loadtxt('target.csv')
cov = np.loadtxt('cov.csv')
phik = np.loadtxt('phik.csv')
robot = np.loadtxt('robot.csv')


# mesh for the pdf
div = 60.0
xarr = np.arange(0,1,1./div)
yarr = np.arange(0,1,1./div)
x,y = np.meshgrid(xarr, yarr)


plt.ion()
for i in range(len(mean[:,0])):
    try:
        cov[i,2] = 0
        cov[i,3] = 0
        print cov[i,1:].reshape((2,2))
        p=multivariate_normal(mean[i,1:3], cov[i,1:].reshape((2,2)))
        bel = p.pdf(np.dstack([x,y]))
        plt.clf()
        plt.plot(target[i,1], target[i,2],)
        robot_drawing = plt.Circle((robot[i,1], robot[i,2]), 0.01, color='m', linewidth='3.0', fill=False)
        target_loc = plt.Circle((target[i,1],target[i,2]), 0.01, color='g', linewidth='3.0', fill=False)

        plt.imshow(bel, extent=(0,1,0,1), origin='lower')
        plt.gca().add_artist(robot_drawing)
        plt.gca().add_artist(target_loc)
        plt.draw()
        plt.pause(0.1)
    except:
        pass
