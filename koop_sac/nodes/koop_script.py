import numpy as np
from koopsaclib.koop_fun import phi, dphi
import matplotlib.pyplot as plt
xdat = np.load('xdat.npy')

udat = np.load('cmd.npy')
print udat
nk = phi(np.hstack((xdat[0,:],udat[0,:]))).shape[0]
G = np.zeros((nk,nk))
A = np.zeros((nk,nk))

N = len(udat)
for i in range(N-1):
    phix = phi(np.hstack((xdat[i], udat[i])))
    phixpo = phi(np.hstack((xdat[i+1], udat[i+1])))
    G += np.outer(phix, phix)
    A += np.outer(phix, phixpo)
K = np.linalg.pinv(G).dot(A)

xnew = xdat[0]
xarr = xnew
for i in range(N-1):
    phipo = phi(np.hstack((xnew, udat[i]))).dot(K)
    # phipo = K.T.dot(phi(np.hstack((xnew, udat[i,:]))))
    xnew = phipo[0:4]
    # print np.linalg.norm(phipo[4:6]- udat[i+1,:])
    # xnew = K[0:6,0:4].T.dot(np.hstack((xnew,udat[i,:])))
    xarr = np.vstack((xarr, xnew))
np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)
np.save('kmat', K)
import matplotlib.pyplot as plt
nf = 30

plt.plot(xarr[0:nf,0], xarr[0:nf,1],'g')
plt.plot(xdat[0:nf,0], xdat[0:nf,1],'b')
plt.show()
