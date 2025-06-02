#!/usr/bin/env python     

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
import rosbag

count = 0

def pcl_callback(msg):
    '''
    header:
        seq, timestamp
    height, width
    fields:
        x,y,z,intensity,t, reflectivity, ring, ambient, range
    is_bigendian
    point_step,row_step
    data
    is_dense
    '''
    global count
    n_points = msg.width*msg.height
    if count==0:
        print(msg[0])
        count+=1

    # print("Height =  {} | Width = {} | N.Points = {}".format(msg.height,msg.width,n_points))

def scan_callback(msg):
    '''
    header:
        seq, timestamp
    angle_min,max, range_min,max
    ranges(0-255): 2048 vals.
    intensities(0-255): 2048 vals.
    '''
    global count
    if count==0:
        print(msg)
        count+=1
        # print("Num ranges: {} | Num intensities: {}".format(len(msg.ranges),len(msg.intensities)))

def main():
    rospy.init_node('ouster_debug',anonymous=True)
    rospy.Subscriber('/ouster/points', PointCloud2,pcl_callback)
    # rospy.Subscriber('/ouster/scan', LaserScan, scan_callback)
    rospy.spin()
    

if __name__=="__main__":
    main()
