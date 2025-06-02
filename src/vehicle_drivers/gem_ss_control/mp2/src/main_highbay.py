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
    dist = np.sqrt(distToTargetX_e4**2 + distToTargetY_e4**2)
    return dist<3


#entry point to main
def run_model():
    rospy.init_node("model_dynamics")
    
    #e2 - human driven car controller+waypoint setup
    controller_e2 = vehicleController(keyboardControl=False)
    waypoints_e2 = WayPoints()
    pos_list_e2 = waypoints_e2.getWayPoints1_highbay()

    #e4 - ego/autonomous car controller+waypoint setup
    controller_e4 = vehicleController(keyboardControl=False)
    waypoints_e4 = WayPoints()
    pos_list_e4 = waypoints_e4.getWayPoints2_highbay()
    
    #initialize safety_flag
    safety_flag=False
    e2_flag=False
    merging_flag=False

    pos_index_e2, pos_index_e4 = 1,1

    #set initial speed
    e2_max_speed = 1
    e4_max_speed = 3

    #get target positions
    target_x_e2,target_y_e2 = pos_list_e2[pos_index_e2]
    target_x_e4,target_y_e4 = pos_list_e4[pos_index_e4]

    #Shutdown routine
    def shutdown():
        """Stop the car when this ROS node shuts down"""
        controller_e4.stop(vehicle="e4")
        rospy.loginfo("Stop the car")
    
    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(100)  #100 Hz
    rospy.sleep(0.0)
    start_time = rospy.Time.now()
    prev_wp_time = start_time

    #main control loop
    while not rospy.is_shutdown() :
        rate.sleep()
        
        #Get the current position and orientation of the vehicle
        currState_e2 = controller_e2.getModelState1()
        currState_e4 = controller_e4.getModelState2()

        if not currState_e2.success or not currState_e4.success:
            print("No model state")
            continue

        # #Check safety violations --- as of now it checks only distance between e2-e4
        # if(checkdistance(currState_e2,currState_e4)):
        #     rospy.loginfo("******COLLISION******")
        #     controller_e4.stop()
        #     controller_e2.stop()
        #     return False,pos_index_e4,(rospy.Time.now()-start_time).to_sec()

        # current state safe condition

        dist1 = np.sqrt((currState_e2.pose.position.x - currState_e4.pose.position.x)**2 + (currState_e2.pose.position.y - currState_e4.pose.position.y)**2)
        dist2 = np.sqrt(((currState_e2.pose.position.x+0.5) - currState_e4.pose.position.x)**2 + (currState_e2.pose.position.y - currState_e4.pose.position.y)**2)
        dist3 = np.sqrt(((currState_e2.pose.position.x-0.5) - currState_e4.pose.position.x)**2 + (currState_e2.pose.position.y - currState_e4.pose.position.y)**2)
        dist4 = np.sqrt(((currState_e2.pose.position.x+0.5) - (currState_e4.pose.position.x+0.5))**2 + (currState_e2.pose.position.y - currState_e4.pose.position.y)**2)
        dist5 = np.sqrt(((currState_e2.pose.position.x-0.5) - (currState_e4.pose.position.x-0.5))**2 + (currState_e2.pose.position.y - currState_e4.pose.position.y)**2)
        dist6 = np.sqrt((currState_e2.pose.position.x - (currState_e4.pose.position.x+0.5))**2 + (currState_e2.pose.position.y - currState_e4.pose.position.y)**2)
        dist7 = np.sqrt((currState_e2.pose.position.x - (currState_e4.pose.position.x-0.5))**2 + (currState_e2.pose.position.y - currState_e4.pose.position.y)**2)
        dist8 = np.sqrt(((currState_e2.pose.position.x+0.5) - (currState_e4.pose.position.x-0.5))**2 + (currState_e2.pose.position.y - currState_e4.pose.position.y)**2)
        dist9 = np.sqrt(((currState_e2.pose.position.x-0.5) - (currState_e4.pose.position.x+0.5))**2 + (currState_e2.pose.position.y - currState_e4.pose.position.y)**2)
        
        if dist1 < 2.5 or dist2 < 2.5 or dist3 < 2.5 or dist4 < 2.5 or dist5 < 2.5 or dist6 < 2.5 or dist7 < 2.5 or dist8 < 2.5 or dist9 < 2.5:
            safety_flag=False
            # rospy.loginfo("******UNSAFE_TO_TURN******")
        else:
            safety_flag=True
            # rospy.loginfo("******SAFE_TO_TURN******")
        # calculate TTC
        # Extract positions and velocities of both vehicles
        e2_pos_x, e2_pos_y, e2_vel, e2_yaw = controller_e2.extract_vehicle_info(currState_e2)
        e4_pos_x, e4_pos_y, e4_vel, e4_yaw = controller_e4.extract_vehicle_info(currState_e4)
        

        ttc_mid_mid = controller_e4.calculate_time_to_collision(e2_pos_x, e2_pos_y, e2_vel, e2_yaw,
                                                        e4_pos_x, e4_pos_y, e4_vel, e4_yaw)

        ttc_front_mid = controller_e4.calculate_time_to_collision(e2_pos_x+0.5, e2_pos_y, e2_vel, e2_yaw,
                                                        e4_pos_x, e4_pos_y, e4_vel, e4_yaw)                                            

        ttc_rear_mid = controller_e4.calculate_time_to_collision(e2_pos_x-0.5, e2_pos_y, e2_vel, e2_yaw,
                                                        e4_pos_x, e4_pos_y, e4_vel, e4_yaw)                         
        ttc_mid_front = controller_e4.calculate_time_to_collision(e2_pos_x, e2_pos_y, e2_vel, e2_yaw,
                                                        e4_pos_x+0.5, e4_pos_y, e4_vel, e4_yaw)    
        ttc_mid_rear = controller_e4.calculate_time_to_collision(e2_pos_x, e2_pos_y, e2_vel, e2_yaw,
                                                        e4_pos_x-0.5, e4_pos_y, e4_vel, e4_yaw)
        ttc_front_front = controller_e4.calculate_time_to_collision(e2_pos_x+0.5, e2_pos_y, e2_vel, e2_yaw,
                                                        e4_pos_x+0.5, e4_pos_y, e4_vel, e4_yaw)
        ttc_front_rear = controller_e4.calculate_time_to_collision(e2_pos_x+0.5, e2_pos_y, e2_vel, e2_yaw,
                                                        e4_pos_x-0.5, e4_pos_y, e4_vel, e4_yaw)
        ttc_rear_front = controller_e4.calculate_time_to_collision(e2_pos_x-0.5, e2_pos_y, e2_vel, e2_yaw,
                                                        e4_pos_x+0.5, e4_pos_y, e4_vel, e4_yaw)
        ttc_rear_rear = controller_e4.calculate_time_to_collision(e2_pos_x-0.5, e2_pos_y, e2_vel, e2_yaw,
                                                        e4_pos_x-0.5, e4_pos_y, e4_vel, e4_yaw)                                                      

        # rospy.loginfo(f"Time to collision: {ttc}")            
        # rospy.loginfo("v_e2: " + str(np.sqrt(currState_e2.twist.linear.x**2 + currState_e2.twist.linear.y**2)))
        # rospy.loginfo("v_e4: " + str(np.sqrt(currState_e4.twist.linear.x**2 + currState_e4.twist.linear.y**2)))



        # if (
        #     (ttc_mid_mid < 5 and ttc_mid_mid != -1) or
        #     (ttc_front_mid < 5 and ttc_front_mid != -1) or
        #     (ttc_rear_mid < 5 and ttc_rear_mid != -1) or
        #     (ttc_mid_front < 5 and ttc_mid_front != -1) or
        #     (ttc_mid_rear < 5 and ttc_mid_rear != -1) or
        #     (ttc_front_front < 5 and ttc_front_front != -1) or
        #     (ttc_front_rear < 5 and ttc_front_rear != -1) or
        #     (ttc_rear_front < 5 and ttc_rear_front != -1) or
        #     (ttc_rear_rear < 5 and ttc_rear_rear != -1)
        # ):

        #     rospy.loginfo("******COLLISION******")
        #     # e2_max_speed = -3
        #     e4_max_speed = -4

        if (currState_e2.pose.position.x < currState_e4.pose.position.x - 2) and merging_flag == False:
            merging_flag = True
            
        if merging_flag:
            target_y_e4 = -17.5
            # e4_max_speed = 5
            rospy.loginfo("******MERGING******")
            if ttc_mid_mid < 3 and ttc_mid_mid != -1:
                rospy.loginfo("******ttc_mid_mid COLLISION******")
                rospy.loginfo(f"Time to collision: {ttc_mid_mid}")  
                merging_flag = False
            elif ttc_front_mid < 3 and ttc_front_mid != -1:
                rospy.loginfo("******ttc_front_mid COLLISION******")
                rospy.loginfo(f"Time to collision: {ttc_front_mid}")
                merging_flag = False
            elif ttc_rear_mid < 3 and ttc_rear_mid != -1:
                rospy.loginfo("******ttc_rear_mid COLLISION******")
                rospy.loginfo(f"Time to collision: {ttc_rear_mid}")
                merging_flag = False
            elif ttc_mid_front < 3 and ttc_mid_front != -1:
                rospy.loginfo("******ttc_mid_front COLLISION******")
                rospy.loginfo(f"Time to collision: {ttc_mid_front}")
                merging_flag = False
            elif ttc_mid_rear < 3 and ttc_mid_rear != -1:
                rospy.loginfo("******ttc_mid_rear COLLISION******")
                rospy.loginfo(f"Time to collision: {ttc_mid_rear}")
                merging_flag = False
            elif ttc_front_front < 3 and ttc_front_front != -1:
                rospy.loginfo("******ttc_front_front COLLISION******")
                rospy.loginfo(f"Time to collision: {ttc_front_front}")
                merging_flag = False
            elif ttc_front_rear < 3 and ttc_front_rear != -1:
                rospy.loginfo("******ttc_front_rear COLLISION******")
                rospy.loginfo(f"Time to collision: {ttc_front_rear}")
                merging_flag = False
            elif ttc_rear_front < 3 and ttc_rear_front != -1:   
                rospy.loginfo("******ttc_rear_front COLLISION******")
                rospy.loginfo(f"Time to collision: {ttc_rear_front}")
                merging_flag = False
            elif ttc_rear_rear < 3 and ttc_rear_rear != -1:
                rospy.loginfo("******ttc_rear_rear COLLISION******")
                rospy.loginfo(f"Time to collision: {ttc_rear_rear}")
                merging_flag = False
        
            # return False,pos_index_e4,(rospy.Time.now()-start_time).to_sec()
        #Get distance between target-current pos
        distToTargetX_e4 = abs(target_x_e4-currState_e4.pose.position.x)
        distToTargetY_e4 = abs(target_y_e4-currState_e4.pose.position.y)
        
        cur_time = rospy.Time.now()
        #check if reached within time
        if (cur_time - prev_wp_time).to_sec() > 5:  #might change time based on dist btwn waypoints
            print(f"failure to reach {pos_index_e4}-th waypoint in time")
            return False, pos_index_e4, (cur_time - start_time).to_sec()

        if(checkdistanceToTarget(distToTargetX_e4,distToTargetY_e4)):   #might change time based on dist btwn waypoints
            rospy.loginfo("******E4 REACHED WAYPOINT{}******".format(pos_index_e4))
            #e4 is close to target-move to next target
            prev_pos_index_e2=pos_index_e2
            pos_index_e2 = pos_index_e2+1
            pos_index_e4 = pos_index_e4+1

            if pos_index_e4==len(pos_list_e4):
                total_time = (cur_time - start_time).to_sec()
                print("**SAFE** | Reached all the waypoints | Total Time: ",total_time)
                return True,pos_index_e4,total_time

            # planer

            if pos_index_e2 <= len(pos_list_e2) - 1:
                target_x_e2,target_y_e2 = pos_list_e2[pos_index_e2]
            else:
                e2_flag = True
                rospy.loginfo("******E2 REACHED ALL WAYPOINTS******")
            
            target_x_e4,target_y_e4 = pos_list_e4[pos_index_e4]
            
            # if not merging_flag:
            #     target_x_e4,target_y_e4 = pos_list_e4[pos_index_e4]
            # else:
            #     target_x_e4,target_y_e4 = pos_list_e4[pos_index_e4]
            #     target_y_e4 = target_y_e4 + 5

            time_taken=(cur_time-prev_wp_time).to_sec()
            prev_wp_time=cur_time
            # print("**SAFE** | Time Taken: {} , Last Reached(e2)= [{},{}] , Next(e2)= [{},{}]".format(round(time_taken,2), pos_list_e2[prev_pos_index_e2][0],pos_list_e2[prev_pos_index_e2][1], pos_list_e2[pos_index_e2][0],pos_list_e2[pos_index_e2][1]))

        if not e2_flag:
            controller_e2.execute_e2(currState_e2, [target_x_e2, target_y_e2], pos_list_e2[pos_index_e2:], e2_max_speed)
        rospy.loginfo(f"e4 current position: {currState_e4.pose.position.x}, {currState_e4.pose.position.y}")
        rospy.loginfo(f"e4 heading target: {target_x_e4}, {target_y_e4}")
        rospy.loginfo("**************************************************")
        controller_e4.execute_e4(currState_e4, [target_x_e4, target_y_e4], pos_list_e4[pos_index_e4:], e4_max_speed)


if __name__ == "__main__":
    try:
        status, num_waypoints, time_taken = run_model()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("****Shutting Down****")
