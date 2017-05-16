#!/usr/bin/python
'''
Ian Abraham:

Class for tracking circular objects (basically the spheros)

SUBSCRIBERS:
            None?
PUBLISHERS:
            Location (and estimated velocity)
'''

###########
# ROS Imports
###########
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion, Twist
from std_msgs.msg import Float32
from rospy_tutorials.msg import Floats

###########
# Imports for opencv and kinect
###########
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import freenect
import os

METERS_PER_PIXEL = 0.00342708

def nothing(x):
    ''' Random function that literally does nothing. Needed for opencv. '''
    pass

class KinectTracker(object):
    ''' Ball tracking class using opencv'''
    def __init__(self):
        ''' Constructor to initialize the mode of operation '''
        # cwd = os.getcwd()
        # print "THIS DIRECTORY", cwd
        # cv2.namedWindow('BGR')
        # # create trackbars
        # cv2.createTrackbar('Rmax','BGR',0,255,nothing)
        # cv2.createTrackbar('Gmax','BGR',0,255,nothing)
        # cv2.createTrackbar('Bmax','BGR',0,255,nothing)
        # cv2.createTrackbar('Rmin','BGR',0,255,nothing)
        # cv2.createTrackbar('Gmin','BGR',0,255,nothing)
        # cv2.createTrackbar('Bmin','BGR',0,255,nothing)

        self.upper = (255, 255, 255)
        self.lower = (125, 149, 253)
        self.__offset = np.array([1.0,0.75]) - np.array([0.5,0.5])
        self.__offset *= 0
        self.__threshold = [800,901]
        rospy.init_node('kinect_tracker')
        self.__init_pubsub()
        self.__xmo = None
        self.__mean = np.array([0.,0.]) + self.__offset
        self.__img_arr = None
        self.__t0 = rospy.get_time()

    def __init_pubsub(self):
        """ Only publishing the location of the robot
        Going to have to calibrate the x,y position to match that of the window
        + height of the kinect relative to the ground
        """
        self.__odom_pub = rospy.Publisher('k_odom', Odometry, queue_size=2)
        self.__mean_sub = rospy.Subscriber('terrain_bel', Floats, self.__get_mean, queue_size = 2)
        # self.__particles_sub = rospy.Subscriber('particles', Floats, self.__get_particles, queue_size=1)
    def __get_mean(self, data):
        self.__mean = data.data[1:] + self.__offset

    def stop_track(self):
        """ safety camera release """
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
        # if self.__img_arr is None:
        #     # self.__img_arr.append(img.copy().ravel())
        #     self.__img_arr = img.copy()
        self.__video_frame = img.astype(np.uint8)
    def save(self):
        print "Saving image"
        direc = '/home/anon/Desktop/'
        filename = direc + 'img.png'
        cv2.imwrite('FINDME.jpg', self.__img_arr)
        # with open(filename, 'a') as f:
        #     np.savetxt(f, self.__img_arr)
        # f.close()
        print "Saved"
    def __get_depth(self, lower, upper):
        depth,_ = freenect.sync_get_depth()
        depth = 255 * np.logical_and( depth > lower, depth < upper)
        self.__depth_frame = depth.astype(np.uint8)

    def __track_object(self):
        """
        Track the object with the kinet
        """

        r = cv2.getTrackbarPos('Rmax','BGR')
        g = cv2.getTrackbarPos('Gmax','BGR')
        b = cv2.getTrackbarPos('Bmax','BGR')
        # self.upper = (int(b), int(g), int(r))
        r = cv2.getTrackbarPos('Rmin','BGR')
        g = cv2.getTrackbarPos('Gmin','BGR')
        b = cv2.getTrackbarPos('Bmin','BGR')
        # self.lower = (int(b), int(g), int(r))


        self.__get_video()
        self.__get_depth(*self.__threshold)
        rgbThreshold = cv2.inRange(self.__video_frame, self.lower, self.upper)
        # cnts, hierarchy = cv2.findContours(self.__depth_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts, hierarchy = cv2.findContours(rgbThreshold.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        tcurr = rospy.get_time()
        if len(cnts) > 0:
            tcurr = rospy.get_time()
            c = max(cnts, key=cv2.contourArea)
            M = cv2.moments(c)
            ((x,y), radius) = cv2.minEnclosingCircle(c)
            # xcurr = np.array([x/480.0, 1.0-y/480.0])
            xcurr = np.array([x*METERS_PER_PIXEL, 1.645-y*METERS_PER_PIXEL])
            mean = np.array([self.__mean[0]/METERS_PER_PIXEL, (1.645-self.__mean[1])/METERS_PER_PIXEL])
            cv2.circle(self.__video_frame, (int(mean[0]), int(mean[1])), int(radius), (255,255,0), 2)
            cv2.circle(self.__video_frame, (int(x), int(y)), int(radius), (0,255,255),2)
            orig = np.array([0.5,0.5]) + self.__offset
            center = np.array([orig[0]/METERS_PER_PIXEL, (1.645-orig[1])/METERS_PER_PIXEL])
            cv2.circle(self.__video_frame, (int(center[0]), int(center[1])), int(radius), (0,0,255),2)

            # cv2.circle(self.__depth_frame, (int(x), int(y)), int(radius), (255,255,255),2)
            # odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_footprint')
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose = Pose(Point(xcurr[0]-self.__offset[0],xcurr[1]-self.__offset[1],0.0), Quaternion(0,0,0,1))
            if self.__xmo is None:
                odom.twist.twist = Twist(Vector3(0.,0.,0.), Vector3(0.,0.,0.))
                self.__xmo = xcurr
            else:
                dxdt = (xcurr - self.__xmo)/(tcurr - self.__t0)
                odom.twist.twist = Twist(Vector3(dxdt[0], dxdt[1], 0.0), Vector3(0.,0.,0.))
                self.__xmo = xcurr
            # print odom
            self.__odom_pub.publish(odom) # publish the data
            cv2.putText(self.__video_frame, str((xcurr[0], xcurr[1])), (int(x), int(y)) ,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255))
        # print 'Frequency : ', 1.0/(tcurr - self.__t0)
        self.__t0 = tcurr
        # cv2.imshow('BGR', rgbThreshold)
        cv2.imshow('RGB image', self.__video_frame)
        # cv2.imshow('Depth image', self.__depth_frame)


    def drawTime(self):
        cv2.putText(self.frame, 'ros time: ' + str(rospy.get_time()) , (int(320*0.01), int(320*0.05)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,255))

    def spin(self):
        ''' Actual code that infinitely loops'''
        r = rospy.Rate(20)
        while not rospy.is_shutdown(): # infinite loop that will stop when KeyboardInterrupt is raised
            self.__track_object() # if there was an explicit list passed

            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
            r.sleep()
        # self.save()

if __name__ == '__main__':

    tracker = KinectTracker()
    try:
        tracker.spin()
    except KeyboardInterrupt:
        tracker.stop_track()
