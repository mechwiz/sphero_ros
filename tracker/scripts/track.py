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

if __name__ == '__main__':
    tracker = BallTracker()
    try:
        tracker.spin()
    except KeyboardInterrupt:
        tracker.stop_track()
