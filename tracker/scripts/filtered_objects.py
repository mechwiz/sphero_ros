'''
List of Filtered objects that were experimentally obtains (with various lighting conditions)
May not work if the lighting conditions change.
'''
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3

class Sphero(object):

    def __init__(self):
        self.name = "Robot"
        self.upper = (116, 84, 255)#(95, 217, 255)
        self.lower = (0,0,39)#(39, 2, 219)

class TennisBall(object):

    def __init__(self):
        self.name = "TennisBall"
        self.upper = (72, 255, 255)
        self.lower = (19, 69, 96)

class Light(object):

    def __init__(self):
        self.name = "Light"
        self.upper = (116, 255, 255)
        self.lower = (0,127, 94)
