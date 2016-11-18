#!/usr/bin/python

'''
Ian Abraham
Tracker script for tracking multiple circular objects

SUBSCRIBERS:
     None
PUBLISHERS (from the BallTracker class):
    Location of each object that we are tracking
'''
from ball_tracking import BallTracker
from filtered_objects import Sphero, Light1, Light2
from filtered_objects import TennisBall
if __name__ == '__main__':
    rb = Light1()
    rb2 = Light2()
    sb = Sphero()
    tracker = BallTracker(objects_to_track=[sb,rb,rb2])
    # tracker = BallTracker()
    try:
        tracker.spin()
    except KeyboardInterrupt:
        tracker.stop_track()
