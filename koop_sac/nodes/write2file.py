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
import inspect
class Write2File(object):

    def __init__(self):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        direc = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + '/experimental_data/data' + timestr
        if not os.path.exists(direc):
            os.makedirs(direc)
        rospy.init_node('write2file')
        now = rospy.Time()
        # self.__write2file(now) # write the current time to the first element of the file
        ################
        # Paths to file names
        ###############

        self.__robot_path = direc+'/robot.csv'
        self.__cmd_path = direc+'/cmd.csv'
        self._init_pubsub()

    def _init_pubsub(self):

        ################
        # Subscribe to the data from the controller
        ################

        self.__rob_sub = rospy.Subscriber('odomRobot', Pose, self.__get_odom)

        self.__cmd_sub = rospy.Subscriber('cmd_vel', Twist, self.__get_cmd)

    def __write2file(self, filename, data):
        with open(filename, 'a') as f:
            np.savetxt(f, [data])
        f.close()

    # def close(self):
    #     self.__phik_path.close()
    #     self.__mean_path.close()
    #     self.__cov_path.close()
    #
    #     self.__robot_path.close()
    #     self.__target_path.close()
    #     self.__vk_path.close()
    def __get_cmd(self, data):
        self.__cmd = np.array([data.linear.x,data.linear.y])
        self.__cmd = np.hstack((rospy.get_time(), self.__cmd))
        self.__write2file(self.__cmd_path, self.__cmd)

    def __get_odom(self, data):
        self.__rob = np.array([data.position.x, 1-data.position.y])
        self.__rob = np.hstack((rospy.get_time(), self.__rob))
        self.__write2file(self.__robot_path, self.__rob)



if __name__ == '__main__':
    wf = Write2File()
    # rospy.spin()
    # wf.close()
    try:
        rospy.spin()
        # wf.close()
    except KeyboardInterrupt:
        pass
        # wf.close()
