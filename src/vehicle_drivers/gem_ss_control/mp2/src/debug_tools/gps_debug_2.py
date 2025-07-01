#!/usr/bin/env python     

import rospy
from math import *
from septentrio_gnss_driver.msg import INSNavGeod
import utm
import rosbag
from rosgraph_msgs.msg import Clock
import csv
import numpy as np

#load bag - to get time info
bag = rosbag.Bag('/home/sridhar/Downloads/rosbags/ag1.bag')
start_time = bag.get_start_time()
end_time   = bag.get_end_time()

current_time = None
percent_done = 0.0
e2_pos = None
e4_pos = None
e2_once_logged = False
csv_writer = None
dist_file = None
dist_writer = None
e2_positions_list = [] 

######################
#GPS MODULE
#Transform from Lat Lon to (x,y)

olat = 40.092855
olon = -88.235981
offset = 1.26 # meters

def heading_to_yaw(heading_curr):
    if (heading_curr >= 270 and heading_curr < 360):
        yaw_curr = np.radians(450 - heading_curr)
    else:
        yaw_curr = np.radians(90 - heading_curr)
    return yaw_curr

#Using UTM package
def get_xy_UTM(lat, lon):
    """Convert GPS lat/lon to UTM x, y."""
    x, y, _, _ = utm.from_latlon(lat, lon)
    return x, y

#Using AlvinXY package

def  mdeglat(lat):
    '''
    Provides meters-per-degree latitude at a given latitude
    Args:
      lat (float): latitude
    Returns:
      float: meters-per-degree value
    '''
    latrad = lat*2.0*pi/360.0
    dy = 111132.09 - 566.05 * cos(2.0*latrad) + 1.20 * cos(4.0*latrad) - 0.002 * cos(6.0*latrad)
    return dy

def mdeglon(lat):
    '''
    Provides meters-per-degree longitude at a given latitude
    Args:
      lat (float): latitude in decimal degrees
    Returns:
      float: meters per degree longitude
    '''
    latrad = lat*2.0*pi/360.0
    dx = 111415.13 * cos(latrad) - 94.55 * cos(3.0*latrad) + 0.12 * cos(5.0*latrad)
    return dx

def ll2xy(lat, lon, orglat, orglon):
    '''
    AlvinXY: Lat/Long to X/Y
    Converts Lat/Lon (WGS84) to Alvin XYs using a Mercator projection.
    Args:
      lat (float): Latitude of location
      lon (float): Longitude of location
      orglat (float): Latitude of origin location
      orglon (float): Longitude of origin location
    Returns:
      tuple: (x,y) where...
        x is Easting in m (Alvin local grid)
        y is Northing in m (Alvin local grid)
    '''
    x = (lon - orglon) * mdeglon(orglat)
    y = (lat - orglat) * mdeglat(orglat)
    return (x,y)

def get_xy_AlvinXY(lat,lon):
    # reference point is located at the center of GNSS antennas
    local_x, local_y=ll2xy(lat,lon,olat,olon)
    return local_x,local_y

######################
# END OF GPS MODULE


def clock_callback(msg):
    global current_time, percent_done
    current_time = msg.clock.to_sec()
    elapsed = current_time - start_time
    percent_done = (elapsed / (end_time - start_time)) * 100.0

def get_distance(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def gps_callback_e2(msg):
    global e2_pos, percent_done
    
    try:
        #convert GPS input lat/lon to degrees
        lat_deg = degrees(msg.latitude) if abs(msg.latitude) < 2 * pi else msg.latitude
        lon_deg = degrees(msg.longitude) if abs(msg.longitude) < 2 * pi else msg.longitude

        ### UTM ###
        # e2x, e2y = get_xy_UTM(lat_deg, lon_deg)
        # e2x = e2x - offset * np.cos(heading_to_yaw(msg.heading))
        # e2y = e2y - offset * np.sin(heading_to_yaw(msg.heading))
        # print(f"Lat: {lat_deg},39.0338 Lon: {lon_deg}, X: {e2x}, Y: {e2y}")

        ### AlvinXY ###
        e2x,e2y = get_xy_AlvinXY(lat_deg,lon_deg)
        # e2x = e2x - offset * np.cos(heading_to_yaw(msg.heading))
        # e2y = e2y - offset * np.sin(heading_to_yaw(msg.heading))
        # print(f"Lat: {lat_deg}, Lon: {lon_deg}, X: {e2x}, Y: {e2y}")

        e2_pos = (e2x, e2y)
        # e2_positions_list.append(e2_pos)
        # print("% done = {:.4f}%, E2: x={:.4f}, y={:.4f}".format(percent_done, e2x, e2y))
    
    except Exception as e:
        rospy.logwarn("e2 GPS conversion failed: {}".format(e))

def gps_callback_e4(msg):
    global e4_pos, percent_done, csv_writer,dist_writer
    
    try:
        #convert GPS input lat/lon to degrees
        lat_deg = degrees(msg.latitude) if abs(msg.latitude) < 2 * pi else msg.latitude
        lon_deg = degrees(msg.longitude) if abs(msg.longitude) < 2 * pi else msg.longitude

        ### UTM ###
        # e4x, e4y = get_xy_UTM(lat_deg, lon_deg)
        # e4x = e4x - offset * np.cos(heading_to_yaw(msg.heading))
        # e4y = e4y - offset * np.sin(heading_to_yaw(msg.heading))
        # print(f"Lat: {lat_deg}, Lon: {lon_deg}, X: {e4x}, Y: {e4y}")
        
        ### AlvinXY ###
        e4x,e4y = get_xy_AlvinXY(lat_deg, lon_deg)
        # e4x = e4x - offset * np.cos(heading_to_yaw(msg.heading))
        # e4y = e4y - offset * np.sin(heading_to_yaw(msg.heading))
        # print(f"X: {e4x}, Y: {e4y}")
        
        e4_pos = (e4x,e4y)
        # print("% done = {:.4f}%, E4: x={:.4f}, y={:.4f}".format(percent_done, e4x, e4y))
        
        ###### writing E4 x,y to CSV ##########
        timestamp = msg.header.stamp.to_sec()
        csv_writer.writerow([f"{timestamp:.3f}", f"{e4x:.4f}", f"{e4y:.4f}"])
        
        ####### Distance Calculation & Write to CSV #########
        # if e2_pos is not None:
        #     dist = get_distance(e2_pos, e4_pos)
        #     timestamp = msg.header.stamp.to_sec()

        #     # Compute E2's position relative to E4 (in sensor frame)
        #     rel_x = e2_pos[0] - e4_pos[0]
        #     rel_y = e2_pos[1] - e4_pos[1]

        #     #Get the velocity of E4
        #     vel = np.sqrt(msg.ve**2 + msg.vn**2)

        #     # print("% done = {:.4f}%, Distance = {:.4f} m, Timestamp = {:.3f}, e2_x = {:.2f}, e2_y = {:.2f}".format(
        #     #     percent_done, dist, timestamp, rel_x, rel_y))

        #     # Write timestamp, distance, relative x, y of E2 in E4 frame
        #     dist_writer.writerow([f"{timestamp:.3f}", f"{dist:.4f}", f"{rel_x:.3f}", f"{rel_y:.3f}", f"{vel:.3f}"])

    
    except Exception as e:
        rospy.logwarn("e4 GPS conversion failed: {}".format(e))

def main():
    global csv_writer, output_file,dist_file, dist_writer

    #use simulated time - to get rosbag time
    rospy.set_param('/use_sim_time', True)
    
    #node for GPS
    rospy.init_node("septentrio_debug", anonymous=True)
    
    ##### PLOT ##########
    #for plotting e4 - x,y
    output_file = open("e4_coords_new.csv", "w", newline='')
    csv_writer = csv.writer(output_file)
    csv_writer.writerow(["timestamp", "x", "y"])
    # rospy.on_shutdown(lambda: output_file.close())

    #PLOT - distance vs timestamp
    # dist_file = open("dGPS.csv", "w", newline='')
    # dist_writer = csv.writer(dist_file)
    # dist_writer.writerow(["timestamp", "distance", "rel_x", "rel_y", "vel"])
    
    def on_shutdown():
        if dist_file:
            dist_file.close()
        if 'output_file' in globals() and output_file:
            output_file.close()
        
        # Compute average E2 position
        # if e2_positions_list:
        #     avg_x = sum(p[0] for p in e2_positions_list) / len(e2_positions_list)
        #     avg_y = sum(p[1] for p in e2_positions_list) / len(e2_positions_list)
        #     print(f"Avg E2 X: {avg_x:.4f} meters,   Avg E2 Y: {avg_y:.4f} meters")

    rospy.on_shutdown(on_shutdown)
    ####################

    #subscribers
    rospy.Subscriber("/clock", Clock,clock_callback)
    rospy.Subscriber("/e2/septentrio_gnss/insnavgeod", INSNavGeod, gps_callback_e2)
    rospy.Subscriber("/septentrio_gnss/insnavgeod",   INSNavGeod, gps_callback_e4)
    
    rospy.spin()

if __name__ == "__main__":
    main()
