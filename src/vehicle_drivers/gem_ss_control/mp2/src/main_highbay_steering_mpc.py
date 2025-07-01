## Imports
import rospy
import numpy as np
import argparse
import sys
import os

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from gazebo_msgs.msg import  ModelState
import time
from waypoint_list import WayPoints
from util  import euler_to_quaternion, quaternion_to_euler
from controller import MPCcontroller

#checks gps distance(euclidean) between e2-e4
def checkdistance(currState_e2,currState_e4):
    dist = np.sqrt((currState_e2.pose.position.x - currState_e4.pose.position.x)**2 + (currState_e2.pose.position.y - currState_e4.pose.position.y)**2)
    return dist<1.25

#checks distance(abs) between target and current pos
def checkdistanceToTarget(distToTargetX_e4,distToTargetY_e4):
    dist = np.sqrt(distToTargetX_e4**2 + distToTargetY_e4**2)
    return dist< 5


#entry point to main
def run_model():
    
    rospy.init_node("model_dynamics")
    start_time = rospy.Time.now()
    #e4 - ego/autonomous car controller+waypoint setup
    controller_e4 = MPCcontroller()
    waypoints_e4 = WayPoints()
    pos_list_e4 = waypoints_e4.getWayPoints2_highbay()

    #initialize safety_flag
    safety_flag=False
    e2_flag=False
    start_merging = False
    end_merging = False

    pos_index_e4 = 1

    #set initial speed
    e4_max_speed = 5

    #get target positions
    target_x_e4,target_y_e4 = pos_list_e4[pos_index_e4]
    
    # rospy.on_shutdown(shutdown)

    rate = rospy.Rate(10)  #100 Hz
    rospy.sleep(0.0)

    #main control loop
    while not rospy.is_shutdown() :
        cur_time = rospy.Time.now()
        
        rate.sleep()
        controller_e4.execute_e4()
        
if __name__ == "__main__":
    try:
        status, num_waypoints, time_taken = run_model()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("****Shutting Down****")
