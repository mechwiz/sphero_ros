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
from koopsaclib import *
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
                'sac_timer'         : 0.05,
            }

        rospy.loginfo("Creating Sphero Controller Class")
        rospy.init_node('sphero_controller') # init node at class init

        self.__t0 = rospy.get_time() # return init time in seconds
        self._init_sac() # init sac
        self._init_pubsub() # init the publishers and subscribers
        # self.reset_loc_pub.publish(Float32(1))
        self._init_sac_timers() # initialize the timers
        self.__txdat = rospy.get_time()

    def _init_sac(self):

        self.x0 = np.array([0.,0.,0.,0.])
        self.cost = CostFunction(self.param['sac_timer'])

        self.system = DoubleIntegrator(self.param['sac_timer']) # init model

        self.T = 0.5
        self.ts = self.param['sac_timer']
        N = int(self.T/self.ts)
        self.unom = np.array([0.,0.])
        self.u0 = [self.unom]*N

        self.sac = SAC(self.system, self.cost)

        self.tcurr = 0

    def _init_pubsub(self):
        # setup publishers + subribers + timers:

        # subscribers
        # self.odom_sub = rospy.Subscriber('odomRobot', Pose, self._get_odometry ,queue_size=1)
        # publishers
        self.odom_sub = rospy.Subscriber('odom', Odometry, self._get_odometry ,queue_size=1)
        # publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)# queue_size=2 originally
        self.angular_velocity_pub = rospy.Publisher('set_angular_velocity', Float32, queue_size=1)
        self.heading_pub = rospy.Publisher('set_heading', Float32, queue_size=1)
        self.target_pub = rospy.Publisher('target', numpy_msg(Floats), queue_size=1)
        self.dir_heading_pub = rospy.Publisher('dir_heading', Twist, queue_size=1)
        self.reset_loc_pub = rospy.Publisher('reset_loc', Float32, queue_size=1)

        # mode indicators for the sphero
        self.back_led_pub = rospy.Publisher('set_back_led', Float32, queue_size=1)
        self.color_pub = rospy.Publisher('set_color', ColorRGBA, queue_size=1)

    def _init_sac_timers(self):
        self.sac_timer = rospy.Timer(rospy.Duration(self.param['sac_timer']), self._get_control)

    # def _get_odometry(self, data):
    #     ''' update the robot's position in the world '''
    #     xcurr = np.array([data.position.x,1-data.position.y])
    #     tcurr = rospy.get_time()
    #     dt = tcurr - self.__txdat
    #     dxdt = (xcurr - self.x0[0:2])/dt
    #     self.__txdat = tcurr
    #     self.x0 = np.hstack((xcurr, dxdt))
    #     print dxdt, dt
    #     # print 'rob odom: ',self.x0, ' ck0: ', self.ck0[0,0]
    def _get_odometry(self, data):
        ''' update the robot's position in the world '''
        xcurr = np.array([data.pose.pose.position.x,data.pose.pose.position.y])
        dxdt = np.array([data.twist.twist.linear.x,data.twist.twist.linear.y])
        self.x0 = np.hstack((xcurr, dxdt))
        print self.x0

    def _get_control(self, data):
        ''' Get the SAC controller and control the sphero'''
        # self.color_pub.publish(ColorRGBA(0.5,0,0.5,0))
        if self.tcurr > 2.5:
            self.target_pub.publish(self.cost.xd(self.tcurr).astype(np.float32))
            self.u0 = self.sac.control(self.x0, self.u0, self.tcurr, self.T)
            # self.u0 = u2
            #self.u = u2(self.tcurr)*0.2 #for the other runs
            # print self.u0[0]
            self.u = self.u0[0]
            self.cmd_vel_pub.publish(Twist(Vector3(int(self.u[0]*255),int(self.u[1]*255),0.0), Vector3(0.0,0.0,0.0)))
            self.u0.pop(0)
            self.u0.append(self.unom)
            if self.tcurr > 20:
                print 'DONE!!!'
        self.tcurr += self.ts


if __name__ == '__main__':
    sac = SpheroController()
    rospy.spin()
