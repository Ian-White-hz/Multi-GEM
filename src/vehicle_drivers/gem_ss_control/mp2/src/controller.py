import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler

from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose,Twist
from waypoint_list import WayPoints
from matplotlib import pyplot as plt
import pygame
from carlo.highbay import HighwayWorld 
import time

import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal

# ROS Headers
import alvinxy.alvinxy as axy # Import AlvinXY transformation module
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva
from sensor_msgs.msg import NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt


class MPCcontroller():
    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        # self.controlPube4 = rospy.Publisher("/e4/ackermann_cmd", AckermannDrive, queue_size=1)
        
        self.max_steering_angle = 0.78  # 45 degrees, adjust as needed
        self.max_speed = 8
        self.world = HighwayWorld()     
        self.world.setup_world()
        self.world.init_car()

        self.rate       = rospy.Rate(10)

        # self.look_ahead = 4
        self.look_ahead = 10
        self.wheelbase  = 2.57 # meters
        self.offset     = 1.26 # meters

        # self.gnss_sub   = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)

        self.gnss_sub   = rospy.Subscriber("/septentrio_gnss/navsatfix", NavSatFix, self.gnss_callback)
        self.ins_sub    = rospy.Subscriber("/septentrio_gnss/insnavgeod", INSNavGeod, self.ins_callback)

        self.gnss_sub_e2 = rospy.Subscriber("/e2/septentrio_gnss/navsatfix", NavSatFix, self.gnss_e2_callback)
        self.ins_sub_e2  = rospy.Subscriber("/e2/septentrio_gnss/insnavgeod", INSNavGeod, self.ins_e2_callback)

        self.lat        = 0.0
        self.lon        = 0.0
        self.heading    = 0.0

        self.enable_sub = rospy.Subscriber("/pacmod/as_tx/enable", Bool, self.enable_callback)

        self.speed_sub  = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        # initialize e2 state variables
        self.speed      = 0.0
        self.heading_e2 = 0.0
        self.lat_e2     = 0.0
        self.lon_e2     = 0.0
        self.speed_e2   = 0.0

        self.olat       = 40.0927
   
        self.olon       = -88.2354


        # read waypoints into the system 
        self.goal       = 0          

        # self.desired_speed = 1.5  # m/s, reference speed
        # self.max_accel     = 0.5 # % of acceleration
        # self.pid_speed     = PID(0.5, 0.0, 0.1, wg=20)

        self.desired_speed = 2  # m/s, reference speed
        self.max_accel     = 0.4 # % of acceleration
        # self.pid_speed     = PID(0.2, 0.2, 0.0, wg=20)
        # self.speed_filter  = OnlineFilter(1.2, 30, 4)


        # -------------------- PACMod setup --------------------

        self.gem_enable    = False
        self.pacmod_enable = False

        # GEM vehicle enable, publish once
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.enable_cmd = Bool()
        self.enable_cmd.data = False

        # GEM vehicle gear control, neutral, forward and reverse, publish once
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.ui16_cmd = 2 # SHIFT_NEUTRAL

        # GEM vehilce brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = False
        self.brake_cmd.clear  = True
        self.brake_cmd.ignore = True

        # GEM vechile forward motion control
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = False
        self.accel_cmd.clear  = True
        self.accel_cmd.ignore = True

        # GEM vechile turn signal control
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # None

        # GEM vechile steering wheel control
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 3.5 # radians/second

    def ins_callback(self, msg):
        self.heading = round(msg.heading, 6)

    def gnss_callback(self, msg):
        self.lat = round(msg.latitude, 6)
        self.lon = round(msg.longitude, 6)

    def ins_e2_callback(self, msg):
        self.heading_e2 = round(msg.heading, 6)
        vn = msg.vn
        ve = msg.ve
        # Calculate forward velocity in m/s
        self.speed_e2 = round(math.sqrt(vn**2 + ve**2), 3)  # forward velocity in m/s

    def gnss_e2_callback(self, msg):
        self.lat_e2 = round(msg.latitude, 6)
        self.lon_e2 = round(msg.longitude, 6)

    def speed_callback(self, msg):
        self.speed = round(msg.vehicle_speed, 3) # forward velocity in m/s

    def enable_callback(self, msg):
        self.pacmod_enable = msg.data

    def heading_to_yaw(self, heading_curr):
        if (heading_curr >= 270 and heading_curr < 360):
            yaw_curr = np.radians(450 - heading_curr)
        else:
            yaw_curr = np.radians(90 - heading_curr)
        return yaw_curr

    def front2steer(self, f_angle):
        if(f_angle > 35):
            f_angle = 35
        if (f_angle < -35):
            f_angle = -35
        if (f_angle > 0):
            steer_angle = round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
        elif (f_angle < 0):
            f_angle = -f_angle
            steer_angle = -round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
        else:
            steer_angle = 0.0
        return steer_angle

    def read_waypoints(self):
        # read recorded GPS lat, lon, heading
        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/xyhead_demo_pp.csv')
        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]
        # x towards East and y towards North
        self.path_points_lon_x   = [float(point[0]) for point in path_points] # longitude
        self.path_points_lat_y   = [float(point[1]) for point in path_points] # latitude
        self.path_points_heading = [float(point[2]) for point in path_points] # heading

        self.wp_size             = len(self.path_points_lon_x)
        self.dist_arr            = np.zeros(self.wp_size)

    def wps_to_local_xy(self, lon_wp, lat_wp):
        # convert GNSS waypoints into local fixed frame reprented in x and y
        lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, self.olat, self.olon)
        return lon_wp_x, lat_wp_y   
    
    def get_gem_state_e2(self):
        # vehicle gnss heading (yaw) in degrees
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon_e2, self.lat_e2)

        # heading to yaw (degrees to radians)
        # heading is calculated from two GNSS antennas
        curr_yaw = self.heading_to_yaw(self.heading_e2) 

        # reference point is located at the center of rear axle
        curr_x = local_x_curr - self.offset * np.cos(curr_yaw)
        curr_y = local_y_curr - self.offset * np.sin(curr_yaw)

        return round(curr_x, 3), round(curr_y, 3), round(self.speed_e2, 3), round(curr_yaw, 4)

    def get_gem_state_e4(self):

        # vehicle gnss heading (yaw) in degrees
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)
        # heading to yaw (degrees to radians)
        # heading is calculated from two GNSS antennas
        curr_yaw = self.heading_to_yaw(self.heading) 

        # reference point is located at the center of rear axle
        curr_x = local_x_curr - self.offset * np.cos(curr_yaw)
        curr_y = local_y_curr - self.offset * np.sin(curr_yaw)

        return round(curr_x, 3), round(curr_y, 3), round(self.speed, 3), round(curr_yaw, 4)

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    # def getModelState2(self):
    #     rospy.wait_for_service('/gazebo/get_model_state')
    #     try:
    #         serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    #         resp = serviceResponse(model_name='gem_e4')
    #     except rospy.ServiceException as exc:
    #         rospy.loginfo("Service did not process request: "+str(exc))
    #         resp = GetModelStateResponse()
    #         resp.success = False
    #     return resp

    # def extract_vehicle_info(self, currentPose):

    #     pos_x, pos_y, vel, yaw = 0, 0, 0, 0
    #     pos_x = currentPose.pose.position.x
    #     pos_y = currentPose.pose.position.y
    #     # find velocity from x and y components
    #     # rospy.loginfo(f"Current speed: {self.speed}")
    #     # rospy.loginfo(f"Velocity components - x: {currentPose.twist.linear.x}, y: {currentPose.twist.linear.y}")
    #     vel = math.sqrt(currentPose.twist.linear.x ** 2 + currentPose.twist.linear.y ** 2)
    #     # rospy.loginfo(f"Calculated velocity magnitude: {vel}")
    #     roll_pitch_yaw = quaternion_to_euler(currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w)
    #     yaw = roll_pitch_yaw[2]

    #     return pos_x, pos_y, vel, yaw # note that yaw is in radian

    def mpc_controller(self):
        # Extract state for MPC
        # Extract vehicle states using helper function
        e4_x, e4_y, e4_vel, e4_yaw = self.get_gem_state_e4()
        # print("_____________________")
        # print("e4_vel", e4_vel)
        # print("e4_x", e4_x)
        # print("e4_y", e4_y)
        # print("e4_yaw", e4_yaw)
        self.speed = e4_vel

        # e2_x, e2_y, e2_vel, e2_yaw = self.get_gem_state_e2()
        e2_x = 10
        e2_y = 4
        e2_vel = 0.0
        e2_yaw = 0.0
        # print("e4_yaw", e4_yaw)

        # Update ego and other vehicle states
        ego = {
            'x': e4_x,
            'y': e4_y, 
            'heading': e4_yaw,
            'speed': e4_vel
        }
        other = {
            'x': e2_x,
            'y': e2_y,
            'heading': e2_yaw, 
            'speed': e2_vel
        }
        current_state = {'ego': ego, 'others': [other]}
       
        # Call your MPC policy
        steering, acceleration = self.world.mpc_highway_policy(current_state)
        # print("steering", steering)
        
       
        # Clip to vehicle limits
        steering = max(min(steering, self.max_steering_angle), -self.max_steering_angle)
        acceleration = max(min(acceleration, self.max_accel), -self.max_accel)
        print("acceleration", acceleration)
        return steering, acceleration
    
    def execute_e4(self):
        steering, acceleration = self.mpc_controller()
        print("steering", steering)
        self.turn_cmd.ui16_cmd = 1
        self.accel_cmd.f64_cmd = acceleration
        self.steer_cmd.angular_position = steering  # Convert steering angle to radians
        self.accel_pub.publish(self.accel_cmd)
        self.steer_pub.publish(self.steer_cmd)
        self.turn_pub.publish(self.turn_cmd)

        self.rate.sleep()