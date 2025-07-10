#!/usr/bin/env python3

#================================================================
# File name: gem_rviz_text_both.py                                                                  
# Description: show sensor info in Rviz for multiple GEMs                                                            
# Author: Tianhao Ji
# Email: ji21@illinois.edu                                                                     
# Date created: 07/07/2025                                                                
# Date last modified: 07/07/2025                                                         
# Version: 0.2                                                                  
# Usage: rosrun gem_gnss gem_rviz_text.py                                                                     
# Python version: 3.8                                                             
#================================================================

import rospy
import math
import random
import numpy as np

from geometry_msgs.msg import Twist
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA, Float32, Float64
from sensor_msgs.msg import NavSatFix
from septentrio_gnss_driver.msg import INSNavGeod, PVTGeodetic

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt


class GEMOverlay(object):

    def __init__(self):
          
        self.text_pub     = rospy.Publisher("/e2/gem_rviz_text", OverlayText, queue_size=5)
        self.text_pub_e4   = rospy.Publisher("/gem_rviz_text", OverlayText, queue_size=5)
        
        self.gps_sub      = rospy.Subscriber("/e2/septentrio_gnss/navsatfix", NavSatFix, self.gps_callback)
        self.ins_sub      = rospy.Subscriber("/e2/septentrio_gnss/insnavgeod", INSNavGeod, self.ins_callback)
        self.rtk_sub      = rospy.Subscriber("/e2/septentrio_gnss/pvtgeodetic", PVTGeodetic, self.rtk_callback)
        self.speed_sub    = rospy.Subscriber("/e2/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        self.steer_sub    = rospy.Subscriber("/e2/pacmod/parsed_tx/steer_rpt", SystemRptFloat, self.steer_callback)
        


        self.gps_sub_e4      = rospy.Subscriber("/septentrio_gnss/navsatfix", NavSatFix, self.gps_callback_e4)
        self.ins_sub_e4     = rospy.Subscriber("/septentrio_gnss/insnavgeod", INSNavGeod, self.ins_callback_e4)
        self.rtk_sub_e4     = rospy.Subscriber("/septentrio_gnss/pvtgeodetic", PVTGeodetic, self.rtk_callback_e4)
        self.speed_sub_e4    = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback_e4)
        self.steer_sub_e4    = rospy.Subscriber("/pacmod/parsed_tx/steer_rpt", SystemRptFloat, self.steer_callback_e4)
        

        self.overlaytext  = self.update_overlaytext()
        self.overlaytext_e4  = self.update_overlaytext()


        self.gps_update   = False
        self.lat          = 0.0
        self.lon          = 0.0
        self.yaw          = 0.0
        self.speed        = 0.0 # m/s
        self.steer        = 0.0 # degrees
        self.rtk          = "Disabled"
        self.stopSign     = 0.0

        self.lat_e4          = 0.0
        self.lon_e4          = 0.0
        self.yaw_e4          = 0.0
        self.speed_e4        = 0.0 # m/s
        self.steer_e4        = 0.0 # degrees
        self.rtk_e4          = "Disabled"
        self.stopSign     = 0.0

        self.rate         = rospy.Rate(20)

    def speed_callback(self, msg):
        self.speed = round(msg.vehicle_speed, 2)
    def speed_callback_e4(self, msg):
        self.speed_e4 = round(msg.vehicle_speed, 2)

    def steer_callback(self, msg):
        self.steer = round(np.degrees(msg.output),1)
    def steer_callback_e4(self, msg):
        self.steer_e4 = round(np.degrees(msg.output),1)

    def ins_callback(self, msg):
        self.yaw = round(msg.heading, 6)
    def ins_callback_e4(self, msg):
        self.yaw_e4 = round(msg.heading, 6)

    def gps_callback(self, msg):
        self.lat = round(msg.latitude, 6)
        self.lon = round(msg.longitude, 6)
    def gps_callback_e4(self, msg):
        self.lat_e4 = round(msg.latitude, 6)
        self.lon_e4 = round(msg.longitude, 6)
        

    def rtk_callback(self, msg):
        if msg.mode == 4:
            self.rtk = "Enabled Fixed"
        elif msg.mode == 5:
            self.rtk = "Enabled Float"
        else:
            self.rtk = "Disabled"
    def rtk_callback_e4(self, msg):
        if msg.mode == 4:
            self.rtk_e4 = "Enabled Fixed"
        elif msg.mode == 5:
            self.rtk_e4 = "Enabled Float"
        else:
            self.rtk_e4 = "Disabled"

    def update_overlaytext(self, rtk="Disabled", lat=0.0, lon=0.0, yaw=0.0, speed=0.0, steer=0.0, rtk_e4="Disabled", lat_e4=0.0, lon_e4=0.0, yaw_e4=0.0, speed_e4=0.0, steer_e4=0.0):
        text            = OverlayText()
        text.width      = 400
        text.height     = 350
        text.left       = 10
        text.top        = 10
        text.text_size  = 12
        text.line_width = 2
        text.font       = "DejaVu Sans Mono"
        
        text.text       = """E2
                             RTK      = %s
                             Lat      = %s
                             Lon      = %s
                             Yaw      = %s
                             Speed [m/s] = %s 
                             Steer [deg] = %s 
                            
                             E4
                                RTK      = %s
                                Lat      = %s
                                Lon      = %s
                                Yaw      = %s
                                Speed [m/s] = %s
                                Steer [deg] = %s
                          """ % (rtk, str(lat), str(lon), str(yaw), str(speed), str(steer),rtk_e4, str(lat_e4), str(lon_e4), str(yaw_e4), str(speed_e4), str(steer_e4))
        text.fg_color   = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        text.bg_color   = ColorRGBA(0.0, 0.0, 0.0, 0.2)
        return text
    
    def update_overlay_textonly(self, new_text):
        self.overlaytext.text = new_text
 
    
    def start_demo(self):
        
        while not rospy.is_shutdown():

            if(self.gps_update):
                gps_text = """E2
                              RTK      = %s
                              Lat      = %s
                              Lon      = %s
                              Yaw      = %s
                              Speed [m/s] = %s 
                              Steer [deg] = %s

                              E4
                              RTK      = %s
                              Lat      = %s
                              Lon      = %s
                              Yaw      = %s
                              Speed [m/s] = %s
                              Steer [deg] = %s
                           """ % (self.rtk, str(self.lat), str(self.lon), str(self.yaw), str(self.speed), str(self.steer),str(self.rtk_e4), str(self.lat_e4), str(self.lon_e4), str(self.yaw_e4), str(self.speed_e4), str(self.steer_e4))
                self.update_overlay_textonly(gps_text)


            else:
                self.overlaytext = self.update_overlaytext()
                self.gps_update  = True

            self.text_pub.publish(self.overlaytext)
            self.rate.sleep()

  
def gem_overlay():

    rospy.init_node('gem_rviz_markers', anonymous=True)

    gem_overlay_object = GEMOverlay()

    try:
        gem_overlay_object.start_demo()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    gem_overlay()