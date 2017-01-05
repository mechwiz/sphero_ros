#!/usr/bin/python
'''
Ian Abraham:

Class for tracking circular objects (basically the spheros)

SUBSCRIBERS:
            None?
PUBLISHERS:
            Location for each object (and estimated velocity?)
'''

###########
# ROS Imports
###########
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion, Twist
from std_msgs.msg import Float32
###########
# Imports for opencv
###########
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import freenect

class KinectTracker(object):
    ''' Ball tracking class using opencv'''
    def __init__(self):
        ''' Constructor to initialize the mode of operation '''

        self.__threshold = [0,600]
        rospy.init_node('kinect_tracker')
        self.__init_pubsub()
        self.__t0 = rospy.get_time()

    def __init_pubsub(self):
        """ Only publishing the location of the robot
        Going to have to calibrate the x,y position to match that of the window
        + height of the kinect relative to the ground
        """
        self.odom_pub = rospy.Publisher('odom', Pose, queue_size=2)

    def stop_track(self):
        ''' safety camera release '''
        cv2.destroyAllWindows()

    def drawObject(self, center,x,y,radius,name=None):
        ''' draw the object that is being tracked '''
        cv2.circle(self.frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        cv2.circle(self.frame, center, 5, (0,0,255), -1)
        if name is None:
            cv2.putText(self.frame, str((x/320.0, y/320.0)), (int(x), int(y)) ,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255))
        else:
            cv2.putText(self.frame, name , (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255))

    def __get_video(self):
        img, _ = freenect.sync_get_video()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.__video_frame = img.astype(np.uint8)

    def __get_depth(self, lower, upper):
        depth,_ = freenect.sync_get_depth()
        depth = 255 * np.logical_and( depth > lower, depth < upper)
        self.__depth_frame = depth.astype(np.uint8)

    def track_object(self):
        """
        Track the object with the kinet
        """


        self.__get_video()
        self.__get_depth(*self.__threshold)

        cnts, hierarchy = cv2.findContours(self.__depth_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            M = cv2.moments(c)
            ((x,y), radius) = cv2.minEnclosingCircle(c)
            print x,y
            cv2.circle(self.__video_frame, (int(x), int(y)), int(radius), (0,255,255),-1)
            pose = Pose(Point(x/320.0,y/320.0,0.0), Quaternion(0,0,0,1))
            self.odom_pub.publish(pose) # publish the data
        cv2.imshow('RGB image', self.__video_frame)
        cv2.imshow('Depth image', self.__depth_frame)

        print 'current time : ', rospy.get_time() - self.__t0
        self.__t0 = rospy.get_time()


    def drawTime(self):
        cv2.putText(self.frame, 'ros time: ' + str(rospy.get_time()) , (int(320*0.01), int(320*0.05)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,255))

    def spin(self):
        ''' Actual code that infinitely loops'''
        r = rospy.Rate(60)
        while not rospy.is_shutdown(): # infinite loop that will stop when KeyboardInterrupt is raised
            self.track_object() # if there was an explicit list passed

            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
            r.sleep()

if __name__ == '__main__':
    bt = BallTracker()
    try:
        bt.spin()
    except KeyboardInterrupt:
        bt.stop_track()
