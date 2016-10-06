#!/usr/bin/python

'''
Ian Abraham
SAC control class for the sphero ROS package

SAC can either be in ergodic mode or regular quadratic control mode
This is dependent on how one initializes the cost function

Since the sphero node class has its own publisher and subscribers, this class
will have two SAC timer which controls the rate at which SAC reads IMU/Odometry
data as well as collision data to update the EID

SUBSCRIBERS:
     I should put a node here for a ps3 controller... later
     - odom -- sphero odometry
     - imu -- sphero imu data
     - collision -- sphero collision data (custom msg class that should be initialized and installed as a python import)
PUBLISHERS:
    Probably going to combine some stuff with the ps3 controller class... I have a custom one made but maybe the ROs one will work just fine
    - cmd_vel -- Twist
    - set_color -- ColorRGBA (make some cool colors when I collide or some shit like that)
    - set_back_led -- Float32 -- this will be used for setting up the heading with the ps3 controller
    For now that is all, but the sphero class does subscribe to a few more inputs that may be useful later on

'''
####
# Ros Imports
####
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from sphero_node.msg import SpheroCollision
from std_msgs.msg import ColorRGBA, Float32, Bool

####
# SAC imports... going to make an include folder for this... maybe even a struct
####
from sac_lib import *
import numpy as np
from scipy.integrate import nquad
from numpy import exp

####
# matplotlib imports
####
import matplotlib.pyplot as plt
from plotwindow import PlotWindow
import sys, random
from PyQt4.QtCore import *
from PyQt4.QtGui import *

# random system import
import sys
from scipy import interpolate

class SpheroController:

    def __init__(self, param=None):

        if not param:
            self.param = {
                'sac_timer'         : 0.2,
                'EID_timer'         : 0.4,
                'plot_timer'        : 0.1,
                'calibration_timer' : 0.1,
                'main_timer'        : 0.1
            }

        rospy.loginfo("Creating Sphero Controller Class")
        rospy.init_node('sphero_controller') # init node at class init

        # should define some runtime flags here
        self.sac_control = False # coinsides with the ps3 controller
        self.manual_control = True
        self.calibration_control = False

        self.joy = {'axes' : [0,0,0,0]}
        self.button = {
            'start'     : 0,
            'select'    : 0,
            'triangle'  : 0,
            'square'    : 0,
            'circle'    : 0,
            'cross'     : 0
        }
        self.trigger = { 'L2':0, 'R2':0 }

        # some class properties that may be useful for storing data from different threads
        self._last_impact_time = None
        self.impact = {'timestamp':None, 'loc':None, 'axis':None, 'vec':None, 'speed':None, 'odom_meas':None}
        self.odom0 = None # shifts back the odometry measurements
        self._nsamp = 3
        self._usamp = 3
        self._odom_dump = {'x': [0.0]*self._nsamp, 'y': [0.0]*self._nsamp}
        self._u_dump =  {'x': [0.0]*self._usamp, 'y': [0.0]*self._usamp}
        self.u = [0.0,0.0]
        self.odom = None
        self.imu = None
        self.control_weight = 1.0/6.0
        self.collision_threshold = 0.0 # no threshold

        # init sac
        self._init_sac()

        self._init_plot()

        # init the publishers and subscribers
        self._init_pubsub()

        # setup the heading
        # self.angular_velocity_pub.publish(Float32(.1))
        # self.angular_velocity_pub.publish(Float32(0.01))


    def _init_plot(self):
        self.plot_window = PlotWindow()
        self.plot_window.show()

    def _init_sac(self):
        self.u0 = lambda t: np.array([0.,0.])
        self.system = Single_Integrator() # init model

        coef = np.array([3,3])
        xlim = [[0,2],[0,2]]
        b_fun = basis(coef, xlim)

        self.xlim = xlim

        phi_temp = lambda x,y: exp( -50*(x - 1.2)**2)*exp( -50*(y - 1.0)**2) + \
                    exp( -50*(x - 1.3)**2)*exp( -50*(y - 0.7)**2)
        normfact = nquad(lambda x,y: phi_temp(x,y), xlim)[0]
        phi = lambda x,y: phi_temp(x,y)/normfact

        self.ck0 = np.zeros(coef+1)
        self.t_dump = [0.0,0.0]
        self.x_dump = [[0.0,0.0], [0.0,0.0]]

        # setup initial EID
        div = 40.0
        X,Y = np.meshgrid(np.arange(0,xlim[0][1],1.0/div), np.arange(0,xlim[1][1],1.0/div))
        self.phi_num = phi(X, Y)

        self.sensor = Tactile_Sensor(xlim, div) # init sensor, holds EID
        self.cost = cost_functional(b_fun, self.phi_num, coef, xlim, 20.0, div)
        self.tcurr = 0.
        self.T = 0.5
        self.sac = SAC(self.system, self.cost)


    def _init_pubsub(self):
        # setup publishers + subribers + timers:

        # subscribers
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.get_odometry )
        self.imu_sub = rospy.Subscriber('imu', Imu, self.get_imu)
        self.collision_sub = rospy.Subscriber('collision', SpheroCollision, self._get_collision)
        self.controller_sub = rospy.Subscriber('joy', Joy, self._parse_controller, queue_size=1)

        # publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)
        self.angular_velocity_pub = rospy.Publisher('set_angular_velocity', Float32, queue_size=1)
        self.heading_pub = rospy.Publisher('set_heading', Float32, queue_size=1)
        self.dir_heading_pub = rospy.Publisher('dir_heading', Twist, queue_size=1)
        self.reset_loc_pub = rospy.Publisher('reset_loc', Float32, queue_size=1)
        # mode indicators for the sphero
        self.back_led_pub = rospy.Publisher('set_back_led', Float32, queue_size=1)
        self.color_pub = rospy.Publisher('set_color', ColorRGBA, queue_size=1)

        # necessary timers to make sac run as efficiently as possible
        self.main_timer = rospy.Timer(rospy.Duration(self.param['main_timer']), self._main_loop)

    def _init_sac_timers(self):
        self.sac_control = True
        self.sac_timer = rospy.Timer(rospy.Duration(self.param['sac_timer']), self._get_control)
        self.EID_timer = rospy.Timer(rospy.Duration(self.param['EID_timer']), self._update_EID)
        self.plot_timer = rospy.Timer(rospy.Duration(self.param['plot_timer']), self._update_plots)

    def _shutdown_sac_timers(self):
        try:
            self.sac_control = False
            self.sac_timer.shutdown()
            self.EID_timer.shutdown()
            self.plot_timer.shutdown()
            self.plot_window.axes.clear()
        except:
            pass

    def _init_calibration_timer(self):
        self.calibration_control = True
        self.calibration_timer = rospy.Timer(rospy.Duration(self.param['calibration_timer']), self._calibration)

    def _shutdown_calibration_timer(self):
        self.calibration_control = False
        self.back_led_pub.publish(Float32(0))
        self.calibration_timer.shutdown()

    def _reset_odom(self):
        ''' reset the odometry '''
        self.reset_loc_pub.publish(Float32(1))

    def _reset_dat_collect(self):
        self.sensor.sampled_points = []
        self.sensor.has_collided = []

    def _main_loop(self, data):
        # print self.joy
        # print self.button
        # print self.trigger
        print self.odom
        sys.stdout.write('Current Mode: \t')
        if self.calibration_control == True:
            sys.stdout.write('calibration \t \r')
        elif self.sac_control == True:
            sys.stdout.write('sac control \t \r')
        else:
            sys.stdout.write('idle           \r')

        # calibration conditional
        if self.button['select'] and self.button['start'] and self.calibration_control == False:
            self._init_calibration_timer()
            rospy.sleep(0.1)
        elif self.button['select'] and self.button['start'] and self.calibration_control == True:
            self._shutdown_calibration_timer()
            rospy.sleep(0.1)

        # sac conditional
        if self.button['cross'] and self.sac_control == False:
            self._init_sac_timers()
            rospy.sleep(0.1)
        elif self.button['cross'] and self.sac_control == True:
            self._shutdown_sac_timers()
            rospy.sleep(0.1)
        # reset odometry
        if self.button['triangle']:
            self._reset_odom()
            self._reset_dat_collect()
            rospy.sleep(0.1)

        sys.stdout.flush()

    def _calibration(self, data):
        self.back_led_pub.publish(Float32(255))
        self.color_pub.publish(ColorRGBA(0,1,1,0))
        if np.linalg.norm(self.joy['axes'][0:2]) > 0:
            self.cmd_vel_pub.publish(Twist(Vector3(int(self.joy['axes'][0]*100),int(self.joy['axes'][1]*100),0.0), Vector3(0.0,0.0,0.0)))
        if np.linalg.norm(self.joy['axes'][3:4]) > 0:
            self.dir_heading_pub.publish(Twist(Vector3(int(self.joy['axes'][2]*255),int(self.joy['axes'][3]*255),0.0), Vector3(0.0,0.0,0.0)))
        if self.trigger['R2'] == 1:
            self.heading_pub.publish(Float32(0))

    def _parse_controller(self, data):
        _joy = np.array(data.axes[0:4])
        # mirror the axis to that of the joy stick
        _joy[0] = -1.0*_joy[0]
        _joy[2] = -1.0*_joy[2]
        self.joy.update({'axes':_joy})
        self.button.update({'start': data.buttons[3]})
        self.button.update({'select': data.buttons[0]})
        self.button.update({'triangle': data.buttons[12]})
        self.button.update({'circle': data.buttons[13]})
        self.button.update({'cross': data.buttons[14]})
        self.button.update({'square': data.buttons[15]})

        self.trigger.update({'L2': data.buttons[8], 'R2': data.buttons[9]})

    def _update_EID(self, data):
        '''
        Method that will keep the sensor class + the ergodic metric class
        Updates EID depending on whether collision is detected or not
        '''
        if (np.linalg.norm(self.velocity) < 1e-2) and np.linalg.norm(self.u) > 0.0:
            self.sensor.update_belief(self.odom, True)
            self.cost.update_phik(self.sensor.belief, self.T)
        else:
            self.sensor.update_belief(self.odom, False)
            self.cost.update_phik(self.sensor.belief, self.T)

    def _update_plots(self, data):
        # update plots
        try:
            self.plot_window.axes.clear()
            self.plot_window.axes.imshow(self.sensor.belief, origin='lower', extent=(0,self.xlim[0][1],0,self.xlim[1][1]))
            # self.plot_window.axes.imshow(self.cost.phi, origin='lower', extent=(0,self.xlim[0][1],0,self.xlim[1][1]))
            data_points = np.array(self.sensor.sampled_points)
            self.plot_window.axes.scatter(data_points[:,0],data_points[:,1], c=self.sensor.has_collided ,alpha=0.5)
            self.plot_window.axes.contour(self.sensor.param[0], self.sensor.param[1], self.sensor.boundary_func, colors=['k', 'k', 'k'],
                        linestyles=['--','-','--'],
                        levels=[-0.3,0.,0.3])
            circ1 = plt.Circle(self.odom, 0.02, color='g', fill=False)
            self.plot_window.axes.add_artist(circ1)
            self.plot_window.canvas.draw()
        except:
            print 'Need more data points.'


    def _get_control(self, data):
        ''' method that will have all the SAC stuff'''
        # sac will probably send only 2 control imputs for x and y
        if self.sac_control:
            (_, u2) = self.sac.control(self.odom, self.ck0, self.u0, self.tcurr, self.T)
            u_temp = u2(self.tcurr)*self.control_weight
            self._u_dump['x'].pop(0)
            self._u_dump['x'].append(u_temp[0])
            self._u_dump['y'].pop(0)
            self._u_dump['y'].append(u_temp[1])
            self.u = np.array([np.mean(self._u_dump['x']), np.mean(self._u_dump['y'])]) # save the current control
            try:
                self.x_dump.pop(0)
                self.x_dump.append(self.odom)
                self.t_dump.pop(0)
                self.t_dump.append(self.tcurr+self.param['sac_timer'])
                x_int = interpolate.interp1d(self.t_dump, self.x_dump, fill_value="extrapolate")
                ck1 = self.cost.calc_ck(x_int, self.t_dump)
                self.ck0 = (self.ck0*self.tcurr + ck1)/self.t_dump[-1]
                self.tcurr += self.param['sac_timer']
            except:
                self.tcurr += self.param['sac_timer']
        else:
            self.u = self.joy['axes'][0:2]
        self.cmd_vel_pub.publish(Twist(Vector3(int(self.u[0]*255),int(self.u[1]*255),0.0), Vector3(0.0,0.0,0.0)))

    def _get_collision(self, data):
        ''' Method to update the collsion state '''

        if data.speed > self.collision_threshold:
            print 'Detected collsion... storing data...'
            self._last_impact_time = self.impact['timestamp'] # keep track of the last impact

            # update the new impact
            self.impact.update({'timestamp' : data.timestamp})
            self.impact.update({'odom_meas' : self.odom})
            self.impact.update({'loc' : [data.x, data.y]})
            self.impact.update({'speed' : data.speed})
            # self.impact['odom_meas'] = self.odom # where the measurement was located
            # self.impact['loc'] = [data.x, data.y] # dont give a shit about the z axis as the sphero is not flying...not yet at least
            # self.impact['axis'] = data.axis
            # self.impact['vec'] = [data.x_magnitude, data.y_magnitude]
            # self.impact['speed'] = data.speed

            '''
            going to try to update the EID here
            '''
            rospy.sleep(2.0) # chill out for a bit
            self.sensor.update_belief(self.impact['odom_meas'], True)
            self.cost.update_phik(self.sensor.belief, self.T)

    def get_odometry(self, data):
        ''' method to update the odometry '''
        #if self.odom0 == None:
        #    self.odom0 = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        self._odom_dump['x'].pop(0)
        self._odom_dump['y'].pop(0)
        self._odom_dump['x'].append(data.pose.pose.position.x)
        self._odom_dump['y'].append(data.pose.pose.position.y) #-self.odom0
        self.odom = np.array([np.mean(self._odom_dump['x']), np.mean(self._odom_dump['y'])])
        self.velocity = np.array([data.twist.twist.linear.x,data.twist.twist.linear.y])

    def get_imu(self, data):
        ''' method to update the imu '''
        self.linear_acceleration = [data.linear_acceleration.x, data.linear_acceleration.y]
        self.angular_velocity = [data.angular_velocity.x, data.angular_velocity.y]


if __name__ == '__main__':
    app = QApplication(sys.argv) # starts up matplotlib
    sac = SpheroController()
    app.exec_() # execute Qt so I can plot things
    rospy.spin()
