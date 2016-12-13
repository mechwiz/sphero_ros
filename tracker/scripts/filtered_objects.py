'''
List of Filtered objects that were experimentally obtains (with various lighting conditions)
May not work if the lighting conditions change.
'''
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
import cv2
import numpy as np

class Sphero(object):

    def __init__(self):
        self.name = "Robot"
        # self.upper = (116, 84, 255)#(95, 217, 255)
        ''' Parameters in the marionette room
        self.upper = (255,66,255)
        self.lower = (0,0,208)
        '''
        # self.lower = (39, 2, 219)
        # BGR
        # self.upper = (252, 50, 255)
        # self.lower = (0,0,123)
        self.upper = (255, 255, 255)
        self.lower = (0,0,245)
        self.x0 = np.array([0,0])
        self.center = None
        self.radius = None
        self.sensor_range = int(0.15*320)

    def draw_me(self, img, vk=None):
        cv2.circle(img, (int(self.x0[0]), int(self.x0[1])), int(self.radius), (0, 255, 0), 2)
        cv2.circle(img, self.center, 5, (0,255,0), -1)
        cv2.circle(img, self.center, self.sensor_range, (255,0,0), 2)
        cv2.putText(img, self.name , (int(self.x0[0]), int(self.x0[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255))

class TennisBall(object):

    def __init__(self):
        self.name = "Target"
        self.upper = (72, 255, 255)
        self.lower = (19, 69, 96)
        self.x0 = None
        self.center = None
        self.radius = None

    def draw_me(self, img, vk=None):
        cv2.circle(img, (int(self.x0[0]), int(self.x0[1])), int(self.radius), (0, 0, 255), 2)
        cv2.circle(img, self.center, 5, (0,0,255), -1)
        cv2.putText(img, self.name , (int(self.x0[0]), int(self.x0[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255))

class Light1(object):

    def __init__(self):
        self.name = "Target1"
        ''' marionette room parameters
        self.upper = (32, 246, 255)
        self.lower = (0,39, 76)
        '''
        self.upper = (42, 115, 255)
        self.lower = (0, 40, 91)
        self.x0 = None
        self.center = None
        self.radius = None

    def draw_me(self, img, vk=None):
        cv2.circle(img, (int(self.x0[0]), int(self.x0[1])), int(self.radius), (0, 0, 255), 2)
        cv2.circle(img, self.center, 5, (0,0,255), -1)
        cv2.putText(img, self.name , (int(self.x0[0]), int(self.x0[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255))
class Light2(object):

    def __init__(self):
        self.name = "Target2"
        ''' marionette room parameters
        self.upper = (32, 246, 255)
        self.lower = (0,39, 76)
        '''
        self.upper = (112, 255, 255)
        self.lower = (0, 80, 101)
        self.x0 = None
        self.center = None
        self.radius = None

    def draw_me(self, img, vk=None):
        cv2.circle(img, (int(self.x0[0]), int(self.x0[1])), int(self.radius), (0, 0, 255), 2)
        cv2.circle(img, self.center, 5, (0,0,255), -1)
        cv2.putText(img, self.name , (int(self.x0[0]), int(self.x0[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255))
