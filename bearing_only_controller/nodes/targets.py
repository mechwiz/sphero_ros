#!/usr/bin/python
""" class that publishes the position of the targets """
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

import numpy as np
from scipy.integrate import nquad
from numpy import exp

# random system import
import sys
from scipy import interpolate

class Targets(object):

    def __init__(self):

        rospy.init_node("Target_Publisher")
        self.__target_publishers = []
        self.__target_publishers.append(rospy.Publisher('odomTarget1'), Pose, queue_size=1)
        self.__target_publishers.append(rospy.Publisher('odomTarget2'), Pose, queue_size=1)
        self.__target_publishers.append(rospy.Publisher('odomTarget3'), Pose, queue_size=1)

        vel = 0.06 # target velocity
        self.__stat_1 = lambda t: np.array([0.15*np.cos(vel*t)+0.5, 0.15*np.sin(vel*t)+0.5])
        self.__stat_2 = lambda t: self.__stat_1(t+np.pi/vel)
        self.__stat_3 = lambda t: self.__stat_1(t+(np.pi/2.0)/vel)

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            x,y = self.__stat_1(rospy.get_time() - self.__t0)
            pose = Pose(Point(x,y,0.0), Quaternion(0,0,0,1))
            self.__target_publishers[0].publish(pose)
            x,y = self.__stat_2(rospy.get_time() - self.__t0)
            pose = Pose(Point(x,y,0.0), Quaternion(0,0,0,1))
            self.__target_publishers[1].publish(pose)
            x,y = self.__stat_3(rospy.get_time() - self.__t0)
            pose = Pose(Point(x,y,0.0), Quaternion(0,0,0,1))
            self.__target_publishers[2].publish(pose)
            r.sleep()

if __name__ == '__main__':
    try:
        Targets()
    except KeyboardInterrupt:
        pass
