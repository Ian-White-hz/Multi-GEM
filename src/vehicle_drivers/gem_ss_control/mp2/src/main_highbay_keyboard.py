## Imports
import rospy
import numpy as np
import argparse

from gazebo_msgs.msg import  ModelState
from controller import vehicleController
import time
from waypoint_list import WayPoints
from util import euler_to_quaternion, quaternion_to_euler

#checks gps distance(euclidean) between e2-e4
def checkdistance(currState_e2,currState_e4):
    dist = np.sqrt((currState_e2.pose.position.x - currState_e4.pose.position.x)**2 + (currState_e2.pose.position.y - currState_e4.pose.position.y)**2)
    return dist<1.25

#checks distance(abs) between target and current pos
def checkdistanceToTarget(distToTargetX_e4,distToTargetY_e4):
    return distToTargetX_e4<2 and distToTargetY_e4<3

#checks if current distance < curr relative velocity
def checkcollisionwarning(currState_e2,currState_e4):
    dist = np.sqrt((currState_e2.pose.position.x - currState_e4.pose.position.x)**2 + (currState_e2.pose.position.y - currState_e4.pose.position.y)**2)
    relative_vel = np.sqrt((currState_e2.twist.linear.x - currState_e4.twist.linear.x)**2 + (currState_e2.twist.linear.y - currState_e4.twist.linear.y)**2)
    return dist<relative_vel

#entry point to main
def run_model():
    rospy.init_node("model_dynamics")

    #e4 - ego/autonomous car controller+waypoint setup
    controller_e4 = vehicleController(keyboardControl=False)
    waypoints_e4 = WayPoints()
    pos_list_e4 = waypoints_e4.getWayPoints2_highbay()

    #e2 - human driven car controller
    controller_e2 = vehicleController(keyboardControl=True)
    
    #initialize safety_flag
    safety_flag=False

    pos_index_e4 = 1

    #get target positions
    target_x_e4,target_y_e4 = pos_list_e4[pos_index_e4]

    #Shutdown routine
    def shutdown():
        """Stop the cars when this ROS node shuts down"""
        controller_e4.stop()
        controller_e2.stop()
        rospy.loginfo("Stop the car")
    
    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(100)  #100 Hz
    rospy.sleep(0.0)
    start_time = rospy.Time.now()
    prev_wp_time = start_time

    #main control loop
    while not rospy.is_shutdown() and safety_flag is False:
        rate.sleep()
        
        #Get the current position and orientation of the vehicle
        currState_e2 = controller_e2.getModelState1()
        currState_e4 = controller_e4.getModelState2()

        #Check safety violations --- as of now it checks only distance between e2-e4
        if(checkdistance(currState_e2,currState_e4)):
            rospy.loginfo("******COLLISION******")
            controller_e4.stop()
            controller_e2.stop()
            return False,pos_index_e4,(rospy.Time.now()-start_time).to_sec()

        if(checkcollisionwarning(currState_e2,currState_e4)):
            rospy.loginfo("******COLLISION WARNING*******")

            #control e4 to avoid collision
            #lane checking:
            #if same lane - change lane
            if((15.0 < currState_e2.pose.position.y < 17.5 and 15.0 < currState_e4.pose.position.y < 17.5) or (17.5 < currState_e2.pose.position.y < 20.0 and 17.5 < currState_e4.pose.position.y < 20.0)):
                rospy.loginfo("***CHANGE LANE***")

            else:   #different lane:
                #if y-gap is good enough
                if(abs(currState_e2.pose.position.y-currState_e4.pose.position.y) >= 1.35):
                    rospy.loginfo("***MAINTAIN***")
                
                else: #y-gap is tight -- check which is behind in x. and brake/speed accordingly
                    if(currState_e2.pose.position.x < currState_e4.pose.position.x):
                        rospy.loginfo("***INCREASE SPEED***")
                    elif(currState_e2.pose.position.x < currState_e4.pose.position.x):
                        rospy.loginfo("***SLOW DOWN***")
                

        if not currState_e2.success or not currState_e4.success:
            print("No model state")
            continue
        
        #Get distance between target-current pos - e4
        distToTargetX_e4 = abs(target_x_e4-currState_e4.pose.position.x)
        distToTargetY_e4 = abs(target_y_e4-currState_e4.pose.position.y)
        
        cur_time = rospy.Time.now()
        #check if reached within time
        if (cur_time - prev_wp_time).to_sec() > 4:  #might change time based on dist btwn waypoints
            print(f"failure to reach {pos_index_e4}-th waypoint in time")
            return False, pos_index_e4, (cur_time - start_time).to_sec()

        if(checkdistanceToTarget(distToTargetX_e4,distToTargetY_e4)):   #might change time based on dist btwn waypoints
            #e4 is close to target-move to next target
            prev_pos_index_e4=pos_index_e4
            pos_index_e4 = pos_index_e4+1

            if pos_index_e4==len(pos_list_e4):
                total_time = (cur_time - start_time).to_sec()
                print("**SAFE** | Reached all the waypoints | Total Time: ",total_time)
                return True,pos_index_e4,total_time

            target_x_e4,target_y_e4 = pos_list_e4[pos_index_e4]

            time_taken=(cur_time-prev_wp_time).to_sec()
            prev_wp_time=cur_time
            print("**SAFE** | Time Taken: {} , Last Reached(e4)= [{},{}] , Next(e4)= [{},{}]".format(round(time_taken,2), pos_list_e4[prev_pos_index_e4][0],pos_list_e4[prev_pos_index_e4][1], pos_list_e4[pos_index_e4][0],pos_list_e4[pos_index_e4][1]))

        controller_e4.execute_e4(currState_e4, [target_x_e4, target_y_e4], pos_list_e4[pos_index_e4:])
        
        #just for debug:
        e2vel = np.sqrt(currState_e2.twist.linear.x **2 + currState_e2.twist.linear.y **2)
        e4vel = np.sqrt(currState_e4.twist.linear.x **2 + currState_e4.twist.linear.y **2)
        print("********** e2 VEL={}, e4 VEL={} ".format(e2vel,e4vel))

if __name__ == "__main__":
    try:
        status, num_waypoints, time_taken = run_model()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("****Shutting Down****")
