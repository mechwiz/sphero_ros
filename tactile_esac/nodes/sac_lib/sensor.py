import numpy as np
from numpy import exp
from scipy.stats import multivariate_normal
import matplotlib.pyplot as plt
import random

# let's add a SVM using a RBF kernal for shits in giggles
from sklearn.svm import SVC

class Tactile_Sensor:
    '''
    Class for tactile sensor with RBF SVM
    '''

    def __init__(self, xlim, div):

        # belief
        self.ndiv = div # discretize to speed things up
        self.xlim = xlim
        _param = [np.arange(i[0],i[1],1./self.ndiv) for i in xlim]
        self.param = np.meshgrid(_param[0], _param[1])
        self.belief = np.ones(self.param[0].shape)
        self.boundary_func = np.ones(self.param[0].shape)

        # prediction parameters
        self._nTh = 2
        self.has_collided_once = False

        # revamp
        self.contact_points = []
        self.dydx = [] # local slope... assumping I don;t have that info at first contact
        self.kappa = [] # curvature
        self.predicted_collision = None

        # SVM stays as a object of this sensor
        self.sampled_points = []
        self.has_collided = []
        self.weights = []
        # gamma = 0.3, C = 1000
        # self.clf = SVC(C=100.0,gamma=10, probability=True) # cool, now I just need data
        self.clf = SVC(C=1000.0,gamma=50.3, probability=True)


    def v(self, x):
        '''
        v = [ x, y[x] ]
        For the point mass, V returns a x,y measurement if in collision
        Else, not sure what to do...
        '''
        return x

    def update_belief(self, x, touching):
        '''
        SVM based EID
        '''

        if len(self.sampled_points) > 50:
            self.sampled_points.pop(0)
            self.has_collided.pop(0)

        vk = self.v(x) #+ np.random.normal(0, self.sigma)

        # let's store stuff into the SVM data
        self.sampled_points.append(vk)

        # do some data storage on the sensors for the svm
        if touching:
            self.has_collided.append(1.0)
        else:
            self.has_collided.append(0.0)

        print 'Collected data: ', self.has_collided[-1], self.sampled_points[-1]

        # once the collision flag has been switched to true
        try:
            # fit the data
            self.clf.fit(np.array(self.sampled_points), np.array(self.has_collided))
            # generate a dummy variable for the Platt Scaling prediction of P(y == 1 | x) points
            Z = self.clf.predict_proba(np.c_[self.param[0].ravel(), self.param[1].ravel()])[:,1]
            Z = Z.reshape(self.param[0].shape)

            # update the belief of where information may be stored
            self.belief = Z/(sum(sum(Z))/self.ndiv**2)

            # use the dummy variable to generate the boundary function phi(x) = 0
            Zn = self.clf.decision_function(np.c_[self.param[0].ravel(), self.param[1].ravel()])
            Zn = Zn.reshape(self.param[0].shape)

            # store the level sets in this class member variable
            self.boundary_func = Zn
        except:
            print 'Need more data points. Keep Searching.'

    # def _ekf(self, x, vk, P):
    #
    #     # prediction
    #     xk = self.f(x)
    #     A = self.fdx()
    #     H = np.array(self.vdx(x)).T
    #     P = A.dot(P).dot(A.T) + self.Q
    #
    #     # update
    #     y = vk - self.v(x)
    #     S = H.dot(P).dot(H.T) + self.R
    #     K = P.dot(H.T).dot(np.linalg.inv(S))
    #     mean = xk + K.dot(y)
    #     P = (np.eye(self._nTh) - K.dot(H)).dot(P)
    #     return (mean, P)
