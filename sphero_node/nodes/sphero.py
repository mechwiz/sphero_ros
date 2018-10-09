#!/usr/bin/python

import rospy

import math
import sys
import tf
import PyKDL

from sphero_driver import sphero_driver
import dynamic_reconfigure.server

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from sphero_node.msg import SpheroCollision
from std_msgs.msg import ColorRGBA, Float32, Bool, Float32MultiArray, Int32MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sphero_node.cfg import ReconfigConfig

import numpy as np

class SpheroNode(object):
    battery_state =  {1:"Battery Charging",
                      2:"Battery OK",
                      3:"Battery Low",
                      4:"Battery Critical"}


    ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                            0, 1e-3, 0, 0, 0, 0,
                            0, 0, 1e6, 0, 0, 0,
                            0, 0, 0, 1e6, 0, 0,
                            0, 0, 0, 0, 1e6, 0,
                            0, 0, 0, 0, 0, 1e3]


    ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                             0, 1e-3, 0, 0, 0, 0,
                             0, 0, 1e6, 0, 0, 0,
                             0, 0, 0, 1e6, 0, 0,
                             0, 0, 0, 0, 1e6, 0,
                             0, 0, 0, 0, 0, 1e3]

    def __init__(self, default_update_rate=50.0):
        rospy.init_node('sphero')
        self.update_rate = default_update_rate
        self.sampling_divisor = int(400/self.update_rate)

        self.is_connected = False
        self._init_pubsub()
        self._init_params()
        #self.robot = sphero_driver.Sphero()
        self.robot_name = "sphero"
        self.robot_bt_addr = None
        if rospy.has_param('/predator/sphero/bt_addr'):
            self.robot_bt_addr = rospy.get_param('/predator/sphero/bt_addr')
        if rospy.has_param('/prey/sphero/bt_addr'):
            self.robot_bt_addr = rospy.get_param('/prey/sphero/bt_addr')

        print "connect bt_addr " + str(self.robot_bt_addr)
        if self.robot_bt_addr != None:
            self.robot = sphero_driver.Sphero(self.robot_name, self.robot_bt_addr)
        else:
            self.robot = sphero_driver.Sphero()
        self.imu = Imu()
        self.imu.orientation_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.imu.angular_velocity_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.imu.linear_acceleration_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        self.last_cmd_vel_time = rospy.Time.now()
        self.last_diagnostics_time = rospy.Time.now()
        self.cmd_heading = 0
        self.cmd_speed = 0
        self.power_state_msg = "No Battery Info"
        self.power_state = 0

    def _init_pubsub(self):
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size = 5)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size = 5)
        self.collision_pub = rospy.Publisher('collision', SpheroCollision, queue_size = 5)
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size = 5)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel, queue_size = 1)
        self.raw_motor_cmd_sub = rospy.Subscriber('raw_motor_cmd', Float32MultiArray, self.raw_motor_cmd, queue_size=1)
        self.color_sub = rospy.Subscriber('set_color', ColorRGBA, self.set_color, queue_size = 1)
        self.back_led_sub = rospy.Subscriber('set_back_led', Float32, self.set_back_led, queue_size = 1)
        self.stabilization_sub = rospy.Subscriber('disable_stabilization', Bool, self.set_stabilization, queue_size = 1)
        self.heading_sub = rospy.Subscriber('set_heading', Float32, self.set_heading, queue_size = 1)
        self.angular_velocity_sub = rospy.Subscriber('set_angular_velocity', Float32, self.set_angular_velocity, queue_size = 1)
        self.dir_heading_sub = rospy.Subscriber('dir_heading', Twist, self.dir_heading, queue_size=1)
        self.reset_loc_sub = rospy.Subscriber('reset_loc', Float32, self.reset_loc, queue_size=1)
        self.config_collision_detect_sub = rospy.Subscriber('config_collision_detect', Int32MultiArray, self.config_collision_detect, queue_size=1)
        self.reconfigure_srv = dynamic_reconfigure.server.Server(ReconfigConfig, self.reconfigure)
        self.transform_broadcaster = tf.TransformBroadcaster()

    def _init_params(self):
        self.connect_color_red = rospy.get_param('~connect_red',255)
        self.connect_color_blue = rospy.get_param('~connect_blue',255)
        self.connect_color_green = rospy.get_param('~connect_green',0)
        self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))
        self.diag_update_rate = rospy.Duration(rospy.get_param('~diag_update_rate', 1.0))

    def normalize_angle_positive(self, angle):
        return math.fmod(math.fmod(angle, 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi);

    def start(self):
        try:
            self.is_connected = self.robot.connect()
            rospy.loginfo("Connect to Sphero with address: %s" % self.robot.bt.target_address)
        except:
            rospy.logerr("Failed to connect to Sphero.")
            sys.exit(1)
        #setup streaming
        self.robot.set_filtered_data_strm(self.sampling_divisor, 1 , 0, True)
        self.robot.add_async_callback(sphero_driver.IDCODE['DATA_STRM'], self.parse_data_strm)
        #setup power notification
        self.robot.set_power_notify(True, False)
        self.robot.add_async_callback(sphero_driver.IDCODE['PWR_NOTIFY'], self.parse_power_notify)
        #setup collision detection
        self.robot.config_collision_detect(1, 45, 110, 45, 110, 100, False)
        self.robot.add_async_callback(sphero_driver.IDCODE['COLLISION'], self.parse_collision)
        #set the ball to connection color
        self.robot.set_rgb_led(self.connect_color_red,self.connect_color_green,self.connect_color_blue,0,False)
        #now start receiving packets
        self.robot.start()

    def spin(self):
        r = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if  (now - self.last_cmd_vel_time) > self.cmd_vel_timeout:
                if self.cmd_heading != 0 or self.cmd_speed != 0:
                    # self.cmd_heading = 0
                    self.cmd_speed = 0
                    self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), 1, False)
            if (now - self.last_diagnostics_time) > self.diag_update_rate:
                self.last_diagnostics_time = now
                self.publish_diagnostics(now)
            r.sleep()

    def stop(self):
        #tell the ball to stop moving before quiting
        self.robot.roll(int(0), int(0), 1, False)
        self.robot.shutdown = True
        rospy.sleep(1.0)
        self.is_connected = self.robot.disconnect()
        self.robot.join()

    def publish_diagnostics(self, time):
        diag = DiagnosticArray()
        diag.header.stamp = time

        stat = DiagnosticStatus(name="Battery Status", level=DiagnosticStatus.OK, message=self.power_state_msg)
        if self.power_state == 3:
            stat.level=DiagnosticStatus.WARN
        if self.power_state == 4:
            stat.level=DiagnosticStatus.ERROR
        diag.status.append(stat)

        self.diag_pub.publish(diag)


    def parse_collision(self, data):
        if self.is_connected:
            now = rospy.Time.now()
            collision = SpheroCollision()
            collision.header.stamp = now
            collision.x = data["X"]
            collision.y = data["Y"]
            collision.z = data["Z"]
            collision.axis = int(data["Axis"])
            collision.x_magnitude = data["xMagnitude"]
            collision.y_magnitude = data["yMagnitude"]
            collision.speed = data["Speed"]
            collision.timestamp = data["Timestamp"]

            self.collision = collision
            self.collision_pub.publish(self.collision)


    def parse_power_notify(self, data):
        if self.is_connected:
            self.power_state = data
            self.power_state_msg = self.battery_state[data]

    def parse_data_strm(self, data):
        if self.is_connected:
            now = rospy.Time.now()
            imu = Imu(header=rospy.Header(frame_id="imu_link"))
            imu.header.stamp = now
            imu.orientation.x = data["QUATERNION_Q0"]
            imu.orientation.y = data["QUATERNION_Q1"]
            imu.orientation.z = data["QUATERNION_Q2"]
            imu.orientation.w = data["QUATERNION_Q3"]
            imu.linear_acceleration.x = data["ACCEL_X_FILTERED"]/4096.0*9.8
            imu.linear_acceleration.y = data["ACCEL_Y_FILTERED"]/4096.0*9.8
            imu.linear_acceleration.z = data["ACCEL_Z_FILTERED"]/4096.0*9.8
            imu.angular_velocity.x = data["GYRO_X_FILTERED"]*10*math.pi/180
            imu.angular_velocity.y = data["GYRO_Y_FILTERED"]*10*math.pi/180
            imu.angular_velocity.z = data["GYRO_Z_FILTERED"]*10*math.pi/180

            self.imu = imu
            self.imu_pub.publish(self.imu)

            odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_footprint')
            odom.header.stamp = now
            odom.pose.pose = Pose(Point(data["ODOM_X"]/100.0,data["ODOM_Y"]/100.0,0.0), Quaternion(0.0,0.0,0.0,1.0))
            odom.twist.twist = Twist(Vector3(data["VELOCITY_X"]/1000.0, data["VELOCITY_Y"]/1000.0, 0),
                                    Vector3(data["GYRO_X_FILTERED"]*10.0*math.pi/180.0, data["GYRO_Y_FILTERED"]*10.0*math.pi/180.0, data["GYRO_Z_FILTERED"]*10.0*math.pi/180.0))
            odom.pose.covariance =self.ODOM_POSE_COVARIANCE
            odom.twist.covariance =self.ODOM_TWIST_COVARIANCE
            self.odom_pub.publish(odom)

            #need to publish this trasform to show the roll, pitch, and yaw properly
            self.transform_broadcaster.sendTransform((0.0, 0.0, 0.038 ),
                (data["QUATERNION_Q0"], data["QUATERNION_Q1"], data["QUATERNION_Q2"], data["QUATERNION_Q3"]),
                odom.header.stamp, "base_link", "base_footprint")

    def config_collision_detect(self, msg):
        if len(msg.data) == 5:
            print "configuring collision with :", msg.data
            self.robot.config_collision_detect(0x01, msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], False)


    def reset_loc(self, msg):
        if self.is_connected:
            if int(msg.data) > 0:
                self.robot.cfg_loc(False)

    def raw_motor_cmd(self, msg):
        if self.is_connected:
            self.last_cmd_vel_time = rospy.Time.now()
            if msg.data[0] > 0:
                l_mode = 0x01
            elif msg.data[0] < 0:
                l_mode = 0x02
            else:
                l_mode = 0x00
            if msg.data[1] > 0:
                r_mode = 0x01
            elif msg.data[1] < 0:
                r_mode = 0x02
            else:
                r_mode = 0x00
            self.robot.set_raw_motor_values(l_mode, int(abs(msg.data[0])*255), r_mode, int(abs(msg.data[1])*255), False)

    def cmd_vel(self, msg):
        if self.is_connected:
            self.last_cmd_vel_time = rospy.Time.now()
            if np.linalg.norm([msg.linear.x, msg.linear.y]) > 1:
                self.cmd_heading = self.normalize_angle_positive(math.atan2(msg.linear.x,msg.linear.y))*180/math.pi
            self.cmd_speed = math.sqrt(math.pow(msg.linear.x,2)+math.pow(msg.linear.y,2))
            self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), 1, False)

    def dir_heading(self, msg):
        if self.is_connected:
            self.last_cmd_vel_time = rospy.Time.now()
            # if np.linalg.norm([msg.linear.x, msg.linear.y]) > 1:
            #     self.cmd_heading = self.normalize_angle_positive(math.atan2(msg.linear.x,msg.linear.y))*180/math.pi
            self.cmd_heading = self.normalize_angle_positive(math.atan2(msg.linear.x,msg.linear.y))*180/math.pi
            self.cmd_speed = 0
            self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), 1, False)

    def set_color(self, msg):
        if self.is_connected:
            self.robot.set_rgb_led(int(msg.r*255),int(msg.g*255),int(msg.b*255),0,False)

    def set_back_led(self, msg):
        if self.is_connected:
            self.robot.set_back_led(int(msg.data), False)

    def set_stabilization(self, msg):
        if self.is_connected:
            if not msg.data:
                self.robot.set_stablization(1, False)
            else:
                self.robot.set_stablization(0, False)

    def set_heading(self, msg):
        if self.is_connected:
            heading_deg = int(self.normalize_angle_positive(msg.data)*180.0/math.pi)
            self.robot.set_heading(heading_deg, False)

    def set_angular_velocity(self, msg):
        if self.is_connected:
            rate = int((msg.data*180/math.pi)/0.784)
            self.robot.set_rotation_rate(rate, False)

    def configure_collision_detect(self, msg):
        pass

    def reconfigure(self, config, level):
        if self.is_connected:
            self.robot.set_rgb_led(int(config['red']*255),int(config['green']*255),int(config['blue']*255),0,False)
        return config


if __name__ == '__main__':
    try:
        s = SpheroNode()
        s.start()
        s.spin()
        s.stop()
    except rospy.ROSInterruptException:
        s.stop()
