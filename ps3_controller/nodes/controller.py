#!/usr/bin/python

'''
Ian Abraham
Sphero Controller class to initialize the robot and control it

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

# random system import
import sys
import numpy as np

class PSController(object):

    def __init__(self):

        rospy.init_node('Sphero_PS3_Controller') # init node at class init

        # should define some runtime flags here
        self.__manual_control = True
        self.__calibration_control = False

        self.__joy = {'axes' : [0,0,0,0]}
        self.__button = {
            'start'     : 0,
            'select'    : 0,
            'triangle'  : 0,
            'square'    : 0,
            'circle'    : 0,
            'cross'     : 0
        }
        self.__trigger = { 'L2':0, 'R2':0 }

        # init the publishers and subscribers
        self.__init_pubsub()

    def __init_pubsub(self):
        ''' Intialize the publishers '''

        # the one lonely subscriber to the joystick node
        self.__controller_sub = rospy.Subscriber('joy', Joy, self.__parse_controller, queue_size=1)

        #################################
        # publishers
        #################################
        self.__cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)
        self.__angular_velocity_pub = rospy.Publisher('set_angular_velocity', Float32, queue_size=1)
        self.__heading_pub = rospy.Publisher('set_heading', Float32, queue_size=1)
        self.__dir_heading_pub = rospy.Publisher('dir_heading', Twist, queue_size=1)
        self.__reset_loc_pub = rospy.Publisher('reset_loc', Float32, queue_size=1)

        #################################
        # mode indicators for the sphero
        #################################
        self.__back_led_pub = rospy.Publisher('set_back_led', Float32, queue_size=1)
        self.__color_pub = rospy.Publisher('set_color', ColorRGBA, queue_size=1)

    def __init_calibration_timer(self):
        self.__calibration_control = True
        self.__calibration_timer = rospy.Timer(rospy.Duration(self.param['calibration_timer']), self._calibration)

    def __shutdown_calibration_timer(self):
        self.__calibration_control = False
        self.__back_led_pub.publish(Float32(0))
        self.__calibration_timer.shutdown()

    def __reset_odom(self):
        ''' reset the odometry '''
        self.reset_loc_pub.publish(Float32(1))

    def spin(self, rate=60):
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():

            if self.__manual_control: # check to see if manual control is on
                self.__control() # send the control to the sphero
            if self.__calibration_control:
                self.__calibration() # send calibration controls to the sphero
            if self.__button['select'] and self.__button['start'] and self.__calibration_control is False:
                self.__back_led_pub.publish(Float32(255))
                self.__color_pub.publish(ColorRGBA(0,1,1,0))
                self.__calibration_control = True # set the calibration control on
                self.__manual_control = False
                rospy.sleep(0.5)
            if self.__button['select'] and self.__button['start'] and self.__calibration_control is True:
                self.__back_led_pub.publish(Float32(0))
                self.__color_pub.publish(ColorRGBA(1,0,1,0))
                self.__calibration_control = False
                self.__manual_control = True
                rospy.sleep(0.5)
            r.sleep()

    def __calibration(self):
        ''' Method to calibrate the sphero '''
        if np.linalg.norm(self.__joy['axes'][0:2]) > 0:
            self.__cmd_vel_pub.publish(Twist(Vector3(int(self.__joy['axes'][0]*100),int(self.__joy['axes'][1]*100),0.0), Vector3(0.0,0.0,0.0)))
        if np.linalg.norm(self.__joy['axes'][3:4]) > 0:
            self.__dir_heading_pub.publish(Twist(Vector3(int(self.__joy['axes'][2]*255),int(self.__joy['axes'][3]*255),0.0), Vector3(0.0,0.0,0.0)))
        if self.__trigger['R2'] == 1:
            self.__heading_pub.publish(Float32(0))
            rospy.sleep(0.5)

    def __parse_controller(self, data):
        ''' Method to parse the ps3 controller node '''
        _joy = np.array(data.axes[0:4])
        # mirror the axis to that of the joy stick
        _joy[0] = -1.0*_joy[0]
        _joy[2] = -1.0*_joy[2]
        self.__joy.update({'axes':_joy})
        self.__button.update({'start': data.buttons[3]})
        self.__button.update({'select': data.buttons[0]})
        self.__button.update({'triangle': data.buttons[12]})
        self.__button.update({'circle': data.buttons[13]})
        self.__button.update({'cross': data.buttons[14]})
        self.__button.update({'square': data.buttons[15]})

        self.__trigger.update({'L2': data.buttons[8], 'R2': data.buttons[9]})

    def __control(self):
        ''' Method that fully controls the sphero '''
        u = self.__joy['axes'][0:2]
        self.__cmd_vel_pub.publish(Twist(Vector3(int(u[0]*255),int(u[1]*255),0.0), Vector3(0.0,0.0,0.0)))

if __name__ == '__main__':
    sac = PSController()
    sac.spin()
