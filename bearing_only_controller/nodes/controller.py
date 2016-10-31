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

class SpheroController:

    def __init__(self, param=None):
        ''' Initialize the sphero controller class (I should actually inherit the actual controller class which makes the setup simple)'''

        if not param:
            self.param = {
                'sac_timer'         : 0.1,
                'sensor_timer'      : 0.01,
                'eid_timer'         : 0.5,
            }

        rospy.loginfo("Creating Sphero Controller Class")
        rospy.init_node('sphero_controller') # init node at class init

        self._init_sac() # init sac
        self._init_pubsub() # init the publishers and subscribers
        self._init_sac_timers() # initialize the timers

    def _init_sac(self):

        self.x0 = np.array([0.,0.])
        coef = np.array([5,5]) # number of coefficients
        xlim = [[0,1],[0,1]] # search space limit
        basis = Basis0(coef, xlim) # basis function (should be inherited by the cost)
        self.xlim = xlim

        phi_temp = lambda x,y: exp( -50*(x - 1.2)**2)*exp( -50*(y - 1.0)**2) + \
                    exp( -50*(x - 1.3)**2)*exp( -50*(y - 0.7)**2)
        normfact = nquad(lambda x,y: phi_temp(x,y), xlim)[0]
        phi = lambda x,y: phi_temp(x,y)/normfact

        self.cost = CostFunction(basis, phi, coef, xlim)
        self.u0 = lambda t: np.array([0.,0.])
        self.system = Single_Integrator() # init model

        self.T = 0.8
        self.ts = 1./10.

        self.sac = SAC(self.system, self.cost)

        self.ck0 = np.zeros(coef+1)
        self.ck0[0,0] = 1
        self.n_past = 1.0/0.1
        self.xpast = [np.array([0.,0.])]
        self.tcurr = 0
        self.tpast = [0.0]

        # setup initial EID
        div = 40.0
        X,Y = np.meshgrid(np.arange(0,xlim[0][1],1.0/div), np.arange(0,xlim[1][1],1.0/div))
        self.phi_num = phi(X, Y)

        param = np.array([0.5,0.5])
        mean = np.array([0.5,0.5])
        cov = np.diag([0.5]*2)

        self.sensor = BearingOnly(param, mean, cov)

    def _init_pubsub(self):
        # setup publishers + subribers + timers:

        # subscribers
        self.odom_sub = rospy.Subscriber('odomRobot', Pose, self._get_odometry ,queue_size=1)
        self.target_sub = rospy.Subscriber('odomTennisBall', Pose, self._get_target, queue_size=1)

        # publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)
        self.angular_velocity_pub = rospy.Publisher('set_angular_velocity', Float32, queue_size=1)
        self.heading_pub = rospy.Publisher('set_heading', Float32, queue_size=1)
        self.dir_heading_pub = rospy.Publisher('dir_heading', Twist, queue_size=1)
        self.reset_loc_pub = rospy.Publisher('reset_loc', Float32, queue_size=1)
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
        self.sensor._eid() # update the eid
        self.cost.update_phik(self.sensor.eid.ravel(), [self.sensor.state_space[0].ravel(), self.sensor.state_space[1].ravel()]) # update the phik

    def _get_target(self, data):
        ''' update the target's position in the world '''
        self.sensor.param = np.array([data.position.x, 1-data.position.y])# target location
        print 'target loc: ',self.sensor.param
        vk = self.sensor.h(self.x0[0:2]) # get the robot's state and the targets state and find the angle between
        self.sensor.update(self.x0, vk) # update the ekf

    def _get_odometry(self, data):
        ''' update the robot's position in the world '''
        self.x0 = np.array([data.position.x,1-data.position.y])
    # def _sense(self, data):
    #     '''
    #     Update the EKF estimate of the target
    #     '''
    #     vk = self.sensor.h(self.x0[0:2]) # get the robot's state and the targets state and find the angle between
    #     print 'Sensor val: ', vk
    #     self.sensor.update(self.x0, vk) # update the ekf


    def _get_control(self, data):
        ''' Get the SAC controller and control the sphero'''
        (_, u2) = self.sac.control(self.x0, self.ck0, self.u0, self.tcurr, self.T)
        self.u = u2(self.tcurr)*0.1
        print self.u
        self.cmd_vel_pub.publish(Twist(Vector3(int(self.u[0]*255),int(self.u[1]*255),0.0), Vector3(0.0,0.0,0.0)))

if __name__ == '__main__':
    sac = SpheroController()
    rospy.spin()
