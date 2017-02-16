#!/usr/bin/python

'''
Ian Abraham
SAC control class for the sphero ROS package

SAC can either be in ergodic mode or regular quadratic control mode
This is dependent on how one initializes the cost function

Since the sphero node class has its own publisher and subscribers, this class
will have two SAC timer which controls the rate at which SAC reads IMU/Odometry
data as well as collision data to update the EID

SUBSCRIBERS:
     I should put a node here for a ps3 controller... later
     - odom -- sphero odometry
     - imu -- sphero imu data
     - collision -- sphero collision data (custom msg class that should be initialized and installed as a python import)
PUBLISHERS:
    Probably going to combine some stuff with the ps3 controller class... I have a custom one made but maybe the ROs one will work just fine
    - cmd_vel -- Twist
    - set_color -- ColorRGBA (make some cool colors when I collide or some shit like that)
    - set_back_led -- Float32 -- this will be used for setting up the heading with the ps3 controller
    For now that is all, but the sphero class does subscribe to a few more inputs that may be useful later on

'''
####
# Ros Imports
####
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from sphero_node.msg import SpheroCollision
from std_msgs.msg import ColorRGBA, Float32, Bool
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

####
# SAC imports... going to make an include folder for this... maybe even a struct
####
from esaclib import *
import numpy as np
from scipy.integrate import nquad
from numpy import exp

# random system import
import sys
from scipy import interpolate


PAST = 20# original

class SpheroController:

    def __init__(self, param=None):
        ''' Initialize the sphero controller class (I should actually inherit the actual controller class which makes the setup simple)'''

        if not param:
            self.param = {
                'sac_timer'         : 0.1,
                'sensor_timer'      : 0.1,
                'eid_timer'         : PAST*0.1, # twice the sac rate
            }

        rospy.loginfo("Creating Sphero Controller Class")
        rospy.init_node('sphero_controller') # init node at class init

        self.__t0 = rospy.get_time() # return init time in seconds
        self._init_sac() # init sac
        print "Initializing SAC"
        self._init_pubsub() # init the publishers and subscribers
        print "Initializing pub sub"
        self._init_sac_timers() # initialize the timers
        print "Init SAC timer"

    def _init_sac(self):

        self.x0 = np.array([0.,0.])
        coef = np.array([5]*2) # number of coefficients
        xlim = [[0,1],[0,1]] # search space limit
        basis = Basis0(coef, xlim) # basis function (should be inherited by the cost)
        self.xlim = xlim

        # phi_temp = lambda x,y: exp( -50*(x - 1.2)**2)*exp( -50*(y - 1.0)**2) + \
        #             exp( -50*(x - 1.3)**2)*exp( -50*(y - 0.7)**2)
        # phi_temp = lambda x,y: np.ones(x.shape)*np.ones(y.shape)
        # normfact = nquad(lambda x,y: phi_temp(x,y), xlim)[0]
        # phi = lambda x,y: phi_temp(x,y)/normfact
        phi = lambda x,y: np.ones(x.shape)*np.ones(y.shape)

        self.cost = CostFunction(basis, phi, coef, xlim)
        self.u0 = lambda t: np.array([0.,0.])
        self.system = Single_Integrator() # init model

        self.T = 0.5
        self.ts = self.param['sac_timer']

        self.sac = SAC(self.system, self.cost)

        self.ck0 = np.zeros(coef+1)
        self.ck0[0,0] = 1
        self.n_past = 10#self.T/self.ts
        self.xpast = [np.array([0.,0.])]
        self.tcurr = 0
        self.tpast = [0.0]

        # setup initial EID
        div = 40.0
        xarr = np.linspace(0,1,div)
        yarr = np.linspace(0,1,div)
        x,y = np.meshgrid(xarr, yarr)
        self.__x = x
        self.__y = y
        X,Y = np.meshgrid(np.arange(0,xlim[0][1],1.0/div), np.arange(0,xlim[1][1],1.0/div))
        self.phi_num = phi(X, Y)

        param1 = np.array([0.5,0.5])
        # mean1 =  np.array([0.5,0.5])
        mean1 = np.random.uniform(0.25,0.75,2)
        cov1 = np.diag([0.5]*2)
        param2 = np.array([0.5,0.5])
        # mean2 = np.array([0.5,0.5])#np.random.uniform(0.1,0.9,2)
        mean2 = np.random.uniform(0.25,0.75,2)
        cov2 = np.diag([0.5]*2)

        param3 = np.array([0.5,0.5])
        # mean2 = np.array([0.5,0.5])#np.random.uniform(0.1,0.9,2)
        mean3 = np.random.uniform(0.25,0.75,2)
        cov3 = np.diag([0.5]*2)

        self.sensor1 = BearingOnly(param1, mean1, cov1)
        self.sensor2 = BearingOnly(param2, mean2, cov2)
        self.sensor3 = BearingOnly(param3, mean3, cov3)

    def _init_pubsub(self):
        # setup publishers + subribers + timers:

        # subscribers
        # use this subsriber when working with the webcam and not the kinect
        self.odom_sub = rospy.Subscriber('odomRobot', Pose, self._get_odometry ,queue_size=2)
        # self.odom_sub = rospy.Subscriber('k_odom', Pose, self._get_odometry ,queue_size=2)
        self.target_sub1 = rospy.Subscriber('odomTarget1', Pose, self._get_target1, queue_size=1)
        self.target_sub2 = rospy.Subscriber('odomTarget2', Pose, self._get_target2, queue_size=1)
        self.target_sub3 = rospy.Subscriber('odomTarget3', Pose, self._get_target3, queue_size=1)

        # publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)# queue_size=2 originally
        self.angular_velocity_pub = rospy.Publisher('set_angular_velocity', Float32, queue_size=1)
        self.heading_pub = rospy.Publisher('set_heading', Float32, queue_size=1)
        self.vk_pub1 = rospy.Publisher('vk1', Float32, queue_size=1)
        self.vk_pub2 = rospy.Publisher('vk2', Float32, queue_size=1)
        self.vk_pub3 = rospy.Publisher('vk3', Float32, queue_size=1)
        self.dir_heading_pub = rospy.Publisher('dir_heading', Twist, queue_size=1)
        self.reset_loc_pub = rospy.Publisher('reset_loc', Float32, queue_size=1)

        # publishers for recording data
        self.phik_pub = rospy.Publisher('phik', numpy_msg(Floats), queue_size=1)
        self.mean_pub1 = rospy.Publisher('mean1', numpy_msg(Floats), queue_size=1)
        self.cov_pub1 = rospy.Publisher('cov1', numpy_msg(Floats), queue_size=1)
        self.mean_pub2 = rospy.Publisher('mean2', numpy_msg(Floats), queue_size=1)
        self.cov_pub2 = rospy.Publisher('cov2', numpy_msg(Floats), queue_size=1)
        self.mean_pub3 = rospy.Publisher('mean3', numpy_msg(Floats), queue_size=1)
        self.cov_pub3 = rospy.Publisher('cov3', numpy_msg(Floats), queue_size=1)
        # mode indicators for the sphero
        self.back_led_pub = rospy.Publisher('set_back_led', Float32, queue_size=1)
        self.color_pub = rospy.Publisher('set_color', ColorRGBA, queue_size=1)

    def _init_sac_timers(self):
        self.sac_timer = rospy.Timer(rospy.Duration(self.param['sac_timer']), self._get_control)
        # self.sensor_timer = rospy.Timer(rospy.Duration(self.param['sensor_timer']), self._sense)
        self.EID_timer = rospy.Timer(rospy.Duration(self.param['eid_timer']), self._update_EID)

    def _update_EID(self, data):
        '''
        Method that will keep the sensor class + the ergodic metric class
        Updates EID depending on whether collision is detected or not
        '''
        self.sensor1._eid() # update the eid
        self.sensor2._eid() # update the eid
        self.sensor3._eid() # update the eid
        c1 = np.linalg.det(self.sensor1.cov)
        c2 = np.linalg.det(self.sensor2.cov)
        c3 = np.linalg.det(self.sensor3.cov)
        c = c1/c2
        eeid1 = self.sensor1.eid
        eeid2 = self.sensor2.eid
        eeid3 = self.sensor3.eid

        eeid1 /= eeid1.max()
        eeid2 /= eeid2.max()
        eeid3 /= eeid3.max()
        # if c1 > 0.5:
        #     eeid1 = np.ones(eeid1.shape)
        # if c2 > 0.5:
        #     eeid2 = np.ones(eeid2.shape)
        eeid = np.maximum(eeid1,eeid2)
        eeid = np.maximum(eeid, eeid3)
        # eeid = eeid1
        # eeid = eeid1 + eeid2 + eeid3
        # eeid /= eeid.max()
        eeid /= np.sum(eeid)/np.product(eeid.shape)

        self.cost.update_phik(eeid.ravel(), [self.sensor1.state_space[0].ravel(), self.sensor1.state_space[1].ravel()]) # update the phik
        # c1 = np.linalg.det(self.sensor1.cov)
        # c2 = np.linalg.det(self.sensor2.cov)
        # # print 'det cov: ', c1, c2
        # # if c1 < 0.9 and c2 < 0.9:
        # bel1 = self.sensor1.belief.pdf(np.dstack([self.__x,self.__y]))
        # bel1 = bel1/bel1.max()
        # bel2 = self.sensor2.belief.pdf(np.dstack([self.__x,self.__y]))
        # bel2 = bel2/bel2.max()
        # bel = bel1 + bel2
        # bel /= np.sum(bel)/np.product(bel.shape)
        # self.cost.update_phik(bel.ravel(), [self.__x.ravel(), self.__y.ravel()])


        self.phik_pub.publish(self.cost.phik.ravel().astype(np.float32)) # publish the unwraveled phik

    def _get_target1(self, data):
        ''' update the target's position in the world '''
        temp_target = np.array([data.position.x, 1-data.position.y])# target location
        # if np.linalg.norm(temp_target-self.x0) > 0.2:
        self.sensor1.param = temp_target
        # print 'target loc: ',self.sensor.param
        vk = self.sensor1.h(self.x0[0:2])# get the robot's state and the targets state and find the angle between
        if vk is not None:
            vk += 0*np.random.normal(0,0.1) # add noise
        if vk is None:
            self.vk_pub1.publish(Float32(10))
        else:
            self.vk_pub1.publish(Float32(vk))
        try:
            self.sensor1.update(self.x0, vk) # update the ekf
        except:
            pass
        self.mean_pub1.publish(self.sensor1.mean.astype(np.float32)) # publish the mean
        self.cov_pub1.publish(self.sensor1.cov.ravel().astype(np.float32)) # publish the covariance
        # print 'Updating sensor1 measurements: ', vk, self.sensor1.mean
        # print 'mean: ', self.sensor1.mean
    def _get_target2(self, data):
        ''' update the target's position in the world '''
        temp_target = np.array([data.position.x, 1-data.position.y])# target location
        # if np.linalg.norm(temp_target-self.x0) > 0.2:
        self.sensor2.param = temp_target
        # print 'target loc: ',self.sensor.param
        vk = self.sensor2.h(self.x0[0:2])# get the robot's state and the targets state and find the angle between
        if vk is not None:
            vk += 0*np.random.normal(0,0.1) # add noise
        if vk is None:
            self.vk_pub2.publish(Float32(10))
        else:
            self.vk_pub2.publish(Float32(vk))
        try:
            self.sensor2.update(self.x0, vk) # update the ekf
        except:
            pass
        self.mean_pub2.publish(self.sensor2.mean.astype(np.float32)) # publish the mean
        self.cov_pub2.publish(self.sensor2.cov.ravel().astype(np.float32)) # publish the covariance
        # print 'Updating sensor2 measurements: ', vk, self. sensor2.mean
        # print 'mean: ', self.sensor2.mean
    def _get_target3(self, data):
        ''' update the target's position in the world '''
        temp_target = np.array([data.position.x, 1-data.position.y])# target location
        # if np.linalg.norm(temp_target-self.x0) > 0.2:
        self.sensor3.param = temp_target
        # print 'target loc: ',self.sensor.param
        vk = self.sensor3.h(self.x0[0:2])# get the robot's state and the targets state and find the angle between
        if vk is not None:
            vk += 0*np.random.normal(0,0.1) # add noise
        if vk is None:
            self.vk_pub3.publish(Float32(10))
        else:
            self.vk_pub3.publish(Float32(vk))
        try:
            self.sensor3.update(self.x0, vk) # update the ekf
        except:
            pass
        self.mean_pub3.publish(self.sensor3.mean.astype(np.float32)) # publish the mean
        self.cov_pub3.publish(self.sensor3.cov.ravel().astype(np.float32)) # publish the covariance
        # print 'Updating sensor2 measurements: ', vk, self. sensor2.mean
        # print 'mean: ', self.sensor2.mean
    def _get_odometry(self, data):
        ''' update the robot's position in the world '''
        self.x0 = np.array([data.position.x,1-data.position.y])
        print "here"
        if len(self.xpast) < self.n_past:
            self.xpast.append(self.x0)
            self.tpast.append(rospy.get_time() - self.__t0) # add the new time
            x_hist = interpolate.interp1d(self.tpast, np.asarray(self.xpast).T, fill_value="extrapolate")
            self.ck0 = self.cost.calc_ck(x_hist, self.tpast)/(self.tpast[-1] - self.tpast[0])
        else:
            self.xpast.pop(0)
            self.tpast.pop(0)
            self.xpast.append(self.x0)
            self.tpast.append(rospy.get_time() - self.__t0)
            x_hist = interpolate.interp1d(self.tpast, np.asarray(self.xpast).T, fill_value="extrapolate")
            self.ck0 = self.cost.calc_ck(x_hist, self.tpast)/(self.tpast[-1] - self.tpast[0])
        # print 'rob odom: ',self.x0, ' ck0: ', self.ck0[0,0]

    def _get_control(self, data):
        ''' Get the SAC controller and control the sphero'''
        self.color_pub.publish(ColorRGBA(0.5,0,0.5,0))
        t1 = np.linalg.norm(self.sensor1.param-self.sensor1.mean)
        t2 = np.linalg.norm(self.sensor2.param-self.sensor2.mean)
        t3 = np.linalg.norm(self.sensor3.param-self.sensor3.mean)
        if self.tpast[-1] > 2.5:
            print t1,t2,t3
            (_, u2) = self.sac.control(self.x0, self.ck0, self.u0, self.tcurr, self.T)
            # self.u0 = u2
            #self.u = u2(self.tcurr)*0.2 #for the other runs
            self.u = u2(self.tcurr)*0.2
            self.cmd_vel_pub.publish(Twist(Vector3(int(self.u[0]*255),int(self.u[1]*255),0.0), Vector3(0.0,0.0,0.0)))
            if t1 < 2e-2 and t2 < 2e-2 and t2<2e-2 and self.tpast[-1] > 50:
                print "ALL DONE"


if __name__ == '__main__':
    sac = SpheroController()
    rospy.spin()
