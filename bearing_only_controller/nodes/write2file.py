#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from sphero_node.msg import SpheroCollision
from std_msgs.msg import ColorRGBA, Float32, Bool
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import time
import os
class Write2File(object):

    def __init__(self):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        direc = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + '/experimental_data/data' + timestr
        rospy.init_node('write2file')


    def _init_pubsub(self):

        ################
        # Subscribe to the data from the controller
        ################
        self.__phik_sub = rospy.Subscriber('phik', numpy_msg(Floats), self.__get_phik)
        self.__mean_sub = rospy.Subscriber('mean', numpy_msg(Floats), self.__get_mean)
        self.__cov_sub = rospy.Subscriber('cov', numpy_msg(Floats), self.__get_cov)

        self.__rob_sub = rospy.Subscriber('odomRobot', Pose, self.__get_odom)
        self.__target_sub = rospy.Subscriber('odomTarget', Pose, self.__get_target)
        self.__vk_sub = rospy.Subscriber('vk', Float32, self.__get_vk)

    def __write2file(self, filename):
