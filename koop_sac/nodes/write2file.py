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
        self.__mean_path1 = direc+'/mean1.csv'
        self.__cov_path1 = direc+'/cov1.csv'
        self.__mean_path2 = direc+'/mean2.csv'
        self.__cov_path2 = direc+'/cov2.csv'
        self.__mean_path3 = direc+'/mean3.csv'
        self.__cov_path3 = direc+'/cov3.csv'

        self.__robot_path = direc+'/robot.csv'
        self.__target_path1 = direc+'/target1.csv'
        self.__target_path2 = direc+'/target2.csv'
        self.__target_path3 = direc+'/target3.csv'
        self.__vk_path1 = direc+'/vk1.csv'
        self.__vk_path2 = direc+'/vk2.csv'
        self.__vk_path3 = direc+'/vk3.csv'
        self.__cmd_path = direc+'/cmd.csv'
        self._init_pubsub()

    def _init_pubsub(self):

        ################
        # Subscribe to the data from the controller
        ################
        self.__phik_sub = rospy.Subscriber('phik', numpy_msg(Floats), self.__get_phik)
        self.__mean_sub1 = rospy.Subscriber('mean1', numpy_msg(Floats), self.__get_mean1)
        self.__cov_sub1 = rospy.Subscriber('cov1', numpy_msg(Floats), self.__get_cov1)
        self.__mean_sub2 = rospy.Subscriber('mean2', numpy_msg(Floats), self.__get_mean2)
        self.__cov_sub2 = rospy.Subscriber('cov2', numpy_msg(Floats), self.__get_cov2)
        self.__mean_sub3 = rospy.Subscriber('mean3', numpy_msg(Floats), self.__get_mean3)
        self.__cov_sub3 = rospy.Subscriber('cov3', numpy_msg(Floats), self.__get_cov3)
        self.__rob_sub = rospy.Subscriber('odomRobot', Pose, self.__get_odom)
        self.__target_sub1 = rospy.Subscriber('odomTarget1', Pose, self.__get_target1)
        self.__target_sub2 = rospy.Subscriber('odomTarget2', Pose, self.__get_target2)
        self.__target_sub3 = rospy.Subscriber('odomTarget3', Pose, self.__get_target3)
        self.__vk_sub1 = rospy.Subscriber('vk1', Float32, self.__get_vk1)
        self.__vk_sub2 = rospy.Subscriber('vk2', Float32, self.__get_vk2)
        self.__vk_sub3 = rospy.Subscriber('vk3', Float32, self.__get_vk3)

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
    def __get_phik(self, data):
        self.__phik = np.hstack((rospy.get_time(), data.data))
        self.__write2file(self.__phik_path, self.__phik)

    def __get_mean1(self, data):
        self.__mean1 = np.hstack((rospy.get_time(), data.data))
        self.__write2file(self.__mean_path1, self.__mean1)
    def __get_cov1(self, data):
        self.__cov1 = np.hstack((rospy.get_time(), data.data))
        self.__write2file(self.__cov_path1, self.__cov1)

    def __get_mean2(self, data):
        self.__mean2 = np.hstack((rospy.get_time(), data.data))
        self.__write2file(self.__mean_path2, self.__mean2)
    def __get_cov2(self, data):
        self.__cov2 = np.hstack((rospy.get_time(), data.data))
        self.__write2file(self.__cov_path2, self.__cov2)

    def __get_mean3(self, data):
        self.__mean3 = np.hstack((rospy.get_time(), data.data))
        self.__write2file(self.__mean_path3, self.__mean3)
    def __get_cov3(self, data):
        self.__cov3 = np.hstack((rospy.get_time(), data.data))
        self.__write2file(self.__cov_path3, self.__cov3)

    def __get_odom(self, data):
        self.__rob = np.array([data.position.x, 1-data.position.y])
        self.__rob = np.hstack((rospy.get_time(), self.__rob))
        self.__write2file(self.__robot_path, self.__rob)

    def __get_target1(self, data):
        self.__target1 = np.array([data.position.x, 1-data.position.y])
        self.__target1 = np.hstack((rospy.get_time(), self.__target1))
        self.__write2file(self.__target_path1, self.__target1)
    def __get_vk1(self, data):
        self.__vk1 = data.data
        self.__vk1 = np.hstack((rospy.get_time(), self.__vk1))
        self.__write2file(self.__vk_path1, self.__vk1)

    def __get_target2(self, data):
        self.__target2 = np.array([data.position.x, 1-data.position.y])
        self.__target2 = np.hstack((rospy.get_time(), self.__target2))
        self.__write2file(self.__target_path2, self.__target2)
    def __get_vk2(self, data):
        self.__vk2 = data.data
        self.__vk2 = np.hstack((rospy.get_time(), self.__vk2))
        self.__write2file(self.__vk_path2, self.__vk2)

    def __get_target3(self, data):
        self.__target3 = np.array([data.position.x, 1-data.position.y])
        self.__target3 = np.hstack((rospy.get_time(), self.__target3))
        self.__write2file(self.__target_path3, self.__target3)
    def __get_vk3(self, data):
        self.__vk3 = data.data
        self.__vk3 = np.hstack((rospy.get_time(), self.__vk3))
        self.__write2file(self.__vk_path3, self.__vk3)

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
