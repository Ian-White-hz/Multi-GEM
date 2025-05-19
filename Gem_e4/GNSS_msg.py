from __future__ import print_function

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal
from geometry import Point, Rectangle, Circle, Ring

# ROS Headers
import alvinxy.alvinxy as axy # Import AlvinXY transformation module
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
try:
    from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva
    from sensor_msgs.msg import NavSatFix
    from septentrio_gnss_driver.msg import INSNavGeod
except ImportError:
    print("GNSS messages not found. Please install the required packages.")
    pass


# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt

class Gem_e2_gnss_msg:
    def __init__(self):

        self.rate       = rospy.Rate(10)


        # self.gnss_sub   = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)

        self.gnss_sub   = rospy.Subscriber("/e2/septentrio_gnss/navsatfix", NavSatFix, self.gnss_callback)
        self.ins_sub    = rospy.Subscriber("/e2/septentrio_gnss/insnavgeod", INSNavGeod, self.ins_callback)

    def ins_callback(self, msg):
        self.heading = round(msg.heading, 6)

    def gnss_callback(self, msg):
        self.lat = round(msg.latitude, 6)
        self.lon = round(msg.longitude, 6)

class Gem_e4_gnss_msg:
    def __init__(self):

        self.rate       = rospy.Rate(10)

        self.look_ahead = 4
        self.wheelbase  = 2.57 # meters
        self.offset     = 1.26 # meters

        # self.gnss_sub   = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)

        self.gnss_sub   = rospy.Subscriber("/septentrio_gnss/navsatfix", NavSatFix, self.gnss_callback)
        self.ins_sub    = rospy.Subscriber("/septentrio_gnss/insnavgeod", INSNavGeod, self.ins_callback)

        self.lat        = 0.0
        self.lon        = 0.0
        self.heading    = 0.0

        self.enable_sub = rospy.Subscriber("/pacmod/as_tx/enable", Bool, self.enable_callback)

        self.speed_sub  = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        self.speed      = 0.0

        self.olat       = 40.092855    
        self.olon       = -88.235981 

        # read waypoints into the system 
        self.goal       = 0            

        self.desired_speed = 1.5  # m/s, reference speed
        self.max_accel     = 0.5 # % of acceleration

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

    def speed_callback(self, msg):
        self.speed = round(msg.vehicle_speed, 3) # forward velocity in m/s

    def enable_callback(self, msg):
        self.pacmod_enable = msg.data

    def wps_to_local_xy(self, lon_wp, lat_wp):
            # convert GNSS waypoints into local fixed frame reprented in x and y
            lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, self.olat, self.olon)
            return lon_wp_x, lat_wp_y   
    
    def publish_cmd(self,output_accel, steering_angle):
        self.accel_cmd.f64_cmd = output_accel
        self.steer_cmd.angular_position = np.radians(steering_angle)
        self.accel_pub.publish(self.accel_cmd)
        self.steer_pub.publish(self.steer_cmd)
        self.turn_pub.publish(self.turn_cmd)

        self.rate.sleep()
    def heading_to_yaw(self, heading_curr):
        if (heading_curr >= 270 and heading_curr < 360):
            yaw_curr = np.radians(450 - heading_curr)
        else:
            yaw_curr = np.radians(90 - heading_curr)
        return yaw_curr
    def get_gem_state(self):

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

        # use bicycle model to calculate the kinematics

        self.heading = curr_yaw
        speed = self.speed
        heading = self.heading
        # Kinematic bicycle model dynamics based on
        # "Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design" by
        # Jason Kong, Mark Pfeiffer, Georg Schildbach, Francesco Borrelli
        lr = self.rear_dist
        lf = lr # we assume the center of mass is the same as the geometric center of the entity
        beta = np.arctan(lr / (lf + lr) * np.tan(self.inputSteering))

        new_angular_velocity = speed * self.inputSteering # this is not needed and used for this model, but let's keep it for consistency (and to avoid if-else statements)
        new_acceleration = self.inputAcceleration - self.friction
        new_speed = np.clip(speed + new_acceleration * dt, self.min_speed, self.max_speed)
        new_heading = heading + ((speed + new_speed)/lr)*np.sin(beta)*dt/2.
        angle = (heading + new_heading)/2. + beta
        new_center = self.center + (speed + new_speed)*Point(np.cos(angle), np.sin(angle))*dt / 2.
        new_velocity = Point(new_speed * np.cos(new_heading), new_speed * np.sin(new_heading))
        return round(curr_x, 3), round(curr_y, 3), round(curr_yaw, 4), round(self.speed, 3) ,new_acceleration, new_angular_velocity

        