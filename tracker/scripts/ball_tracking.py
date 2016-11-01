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
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from std_msgs.msg import Float32
###########
# Imports for opencv
###########
from collections import deque
import numpy as np
import argparse
import imutils
import cv2

def nothing(x):
    ''' Random ass function that literally does nothing. Needed for opencv. '''
    pass

class BallTracker:
    ''' Ball tracking class using opencv'''
    def __init__(self, objects_to_track=None):
        ''' Constructor to initialize the mode of operation '''
        # argument parser
        ap = argparse.ArgumentParser()
        ap.add_argument("-v", "--video", help="path to the (optional) video file")
        ap.add_argument("-camera", "--cam_number", type=int, default=1, help="camera path")
        ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
        ap.add_argument("-c", "--config", type=int, default=1, help="configure settings")
        self.args = vars(ap.parse_args(rospy.myargv()[1:]))

        cv2.namedWindow('Image') # make a window named image
        # if video not supplied, get camera
        if not self.args.get("video", False):
            self.camera = cv2.VideoCapture(self.args["cam_number"])
        else:
            self.camera = cv2.VideoCapture(self.args["video"])

        if objects_to_track is None: # see if any objects are passed
            # define lower and upper boundaries of color
            self.lower = (29, 86, 6)
            self.upper = (64, 255, 255)

            self.pts = deque(maxlen=self.args["buffer"])

            self.camera.set(3, 320)
            self.camera.set(4, 320)

            cv2.namedWindow('BGR')
            # create trackbars
            cv2.createTrackbar('Rmax','BGR',0,255,nothing)
            cv2.createTrackbar('Gmax','BGR',0,255,nothing)
            cv2.createTrackbar('Bmax','BGR',0,255,nothing)
            cv2.createTrackbar('Rmin','BGR',0,255,nothing)
            cv2.createTrackbar('Gmin','BGR',0,255,nothing)
            cv2.createTrackbar('Bmin','BGR',0,255,nothing)

            self._objects_to_track = None
            self.__n_objects = self.args["config"]
        else:
            self.camera.set(3,320)
            self.camera.set(4,320)
            self._objects_to_track = objects_to_track
            self.__n_objects = len(objects_to_track) # number of objects to track

        rospy.init_node('tracker')
        self._init_pubsub()

    def _init_pubsub(self):

        if self._objects_to_track is None:
            self.odom_pub = [rospy.Publisher('odom%d' %i, Pose, queue_size=2) for i in range(self.__n_objects)]
        else:
            self.odom_pub = [rospy.Publisher('odom'+obj.name, Pose, queue_size=2) for obj in self._objects_to_track]
            self.__vk = 10.0
            self.vk_sub = rospy.Subscriber('vk', Float32, self._get_vk)

    def _get_vk(self, data):
        self.__vk = data.data

    def stop_track(self):
        ''' safety camera release '''
        self.camera.release()
        cv2.destroyAllWindows()

    def drawObject(self, center,x,y,radius,name=None):
        ''' draw the object that is being tracked '''
        cv2.circle(self.frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        cv2.circle(self.frame, center, 5, (0,0,255), -1)
        if name is None:
            cv2.putText(self.frame, str((x/320.0, y/320.0)), (int(x), int(y)) ,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255))
        else:
            cv2.putText(self.frame, name , (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255))

    def track_filtered_objects(self):
        '''
        Loop through the object to track list and find each object
        '''
        l1 = []
        for i, thing in enumerate(self._objects_to_track):
            self.mask = cv2.inRange(self.hsv, thing.lower, thing.upper) # yes I used thing ... don't want to use object
            self.mask = cv2.erode(self.mask, None, iterations=2)
            self.mask = cv2.dilate(self.mask, None, iterations=2)

            try:
                cnts, hierarchy = cv2.findContours(self.mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            except:
                print "I changed the mask input to mask.copy() (just FYI)"
            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                M = cv2.moments(c)
                ((x,y), radius) = cv2.minEnclosingCircle(c)
                pose = Pose(Point(x/320.0,y/320.0,0.0), Quaternion(0,0,0,1))

                self.odom_pub[i].publish(pose) # publish the data

                center = (int(M["m10"] / M["m00"]), int( M["m01"] / M["m00"] ))
                thing.x0 = np.array([x,y])
                thing.center = center
                thing.radius = radius

                if hasattr(thing, 'draw_me'):
                    thing.draw_me(self.frame)
                else:
                    self.drawObject(center, x, y, radius, name=thing.name)
                l1.append((int(thing.x0[0]), int(thing.x0[1])))
        if self.__vk < 5 and len(l1)>=2:
            cv2.line(self.frame, l1[0], l1[1], (255,0,0), 5)

    def configure_tracking(self):
        '''
        Configure the tracking thresholds on opencv
        '''
        # get the trackbar threshold values
        r = cv2.getTrackbarPos('Rmax','BGR')
        g = cv2.getTrackbarPos('Gmax','BGR')
        b = cv2.getTrackbarPos('Bmax','BGR')
        self.upper = (int(b), int(g), int(r))
        r = cv2.getTrackbarPos('Rmin','BGR')
        g = cv2.getTrackbarPos('Gmin','BGR')
        b = cv2.getTrackbarPos('Bmin','BGR')
        self.lower = (int(b), int(g), int(r))

        self.mask = cv2.inRange(self.hsv, self.lower, self.upper) # threshold the frame
        self.mask = cv2.erode(self.mask, None, iterations=2) # erode it
        self.mask = cv2.dilate(self.mask, None, iterations=2) # dilate it to find the contours

        # find the contours
        cnts, hierarchy = cv2.findContours(self.mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        if hierarchy is not None: # check to see if there even is an object detected
            if len(hierarchy[0]) > self.__n_objects: # ensure that you're not tracking other random noise
                print "Too Many objects, noisy data: ", len(hierarchy[0]), len(cnts)
            else:
                for i in range(len(hierarchy[0])): # loop through the objects
                    M = cv2.moments(cnts[i])
                    ((x,y), radius) = cv2.minEnclosingCircle(cnts[i])
                    pose = Pose(Point(x/320.0,y/320.0,0.0), Quaternion(0,0,0,1))
                    self.odom_pub[i].publish(pose) # publish the data
                    print "Area: ,", M["m00"]
                    center = (int(M["m10"] / M["m00"]), int( M["m01"] / M["m00"] ))
                    self.drawObject(center, x, y, radius)

        cv2.imshow('BGR', self.mask) # just to check on the mask

    def spin(self):
        ''' Actual code that infinitely loops'''
        r = rospy.Rate(20.0)
        # actual ball tracking loop
        while not rospy.is_shutdown(): # infinite loop that will stop when KeyboardInterrupt is raised
            (grabbed, self.frame) = self.camera.read() # grab the frame
            # self.frame = cv2.resize(self.frame, (500,500))
            if self.args.get("video") and not grabbed:
                break # if at end of video, break out of loop
            self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV) # extract the hsv space
            if self._objects_to_track is None: # check to see which mode I am
                self.configure_tracking() # if there was not an explicit list passed
            else:
                self.track_filtered_objects() # if there was an explicit list passed
            cv2.imshow('Image', self.frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            r.sleep()

if __name__ == '__main__':
    bt = BallTracker()
    try:
        bt.spin()
    except KeyboardInterrupt:
        bt.stop_track()
