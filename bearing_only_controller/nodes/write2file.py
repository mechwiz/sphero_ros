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
        ################
        self.__phik_path = direc+'/phik.csv'
        self.__mean_path = direc+'/mean.csv'
        self.__cov_path = direc+'/cov.csv'

        self.__robot_path = direc+'/robot.csv'
        self.__target_path = direc+'/target.csv'
        self.__vk_path = direc+'/vk.csv'

        self._init_pubsub()

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

    def __write2file(self, filename, data):
        print 'SAVING'
        with open(filename, 'a') as f:
            print data
            np.savetxt(f, [data])
        f.close()
        print 'IT IS SAVED'

    # def close(self):
    #     self.__phik_path.close()
    #     self.__mean_path.close()
    #     self.__cov_path.close()
    #
    #     self.__robot_path.close()
    #     self.__target_path.close()
    #     self.__vk_path.close()
    def __get_phik(self, data):
        self.__phik = np.hstack((rospy.get_time(), data.data))
        self.__write2file(self.__phik_path, self.__phik)
    def __get_mean(self, data):
        self.__mean = np.hstack((rospy.get_time(), data.data))
        self.__write2file(self.__mean_path, self.__mean)
    def __get_cov(self, data):
        self.__cov = np.hstack((rospy.get_time(), data.data))
        self.__write2file(self.__cov_path, self.__cov)
    def __get_odom(self, data):
        self.__rob = np.array([data.position.x, 1-data.position.y])
        self.__rob = np.hstack((rospy.get_time(), self.__rob))
        self.__write2file(self.__robot_path, self.__rob)
    def __get_target(self, data):
        self.__target = np.array([data.position.x, 1-data.position.y])
        self.__target = np.hstack((rospy.get_time(), self.__target))
        self.__write2file(self.__target_path, self.__target)
    def __get_vk(self, data):
        self.__vk = data.data
        self.__vk = np.hstack((rospy.get_time(), self.__vk))
        self.__write2file(self.__vk_path, self.__vk)

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
