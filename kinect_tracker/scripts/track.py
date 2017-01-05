#!/usr/bin/python

'''
Ian Abraham
Tracker script for tracking multiple circular objects

SUBSCRIBERS:
     None
PUBLISHERS (from the BallTracker class):
    Location of each object that we are tracking
'''
from kinect_tracking import KinectTracker

if __name__ == '__main__':

    tracker = KinectTracker()
    try:
        tracker.spin()
    except KeyboardInterrupt:
        tracker.stop_track()
