import rospy
import numpy as np
import argparse

from gazebo_msgs.msg import  ModelState
from controller import vehicleController
import time
from waypoint_list import WayPoints
from util import euler_to_quaternion, quaternion_to_euler
from visualization_msgs.msg import Marker






def run_model():
    rospy.init_node("model_dynamics")
    controller1 = vehicleController(keyboardControl=False)
    controller2 = vehicleController(keyboardControl=False)
    
    # e2
    waypoints1 = WayPoints()
    pos_list1 = waypoints1.getWayPoints1_highbay()
    # e4

    waypoints2 = WayPoints()
    pos_list2 = waypoints2.getWayPoints2_highbay()

    #safety flag
    s_flag = False

    pos_idx1 = 1
    pos_idx2 = 1
    target_x1, target_y1 = pos_list1[pos_idx1]
    target_x2, target_y2 = pos_list2[pos_idx2]
    def shutdown():
        """Stop the car when this ROS node shuts down"""
        controller2.stop()
        rospy.loginfo("Stop the car")

    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(100)  # 100 Hz
    rospy.sleep(0.0)
    start_time = rospy.Time.now()
    prev_wp_time = start_time

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        # Get the current position and orientation of the vehicle
        currState1 =  controller1.getModelState1()
        # e4
        currState2 =  controller2.getModelState2()
        
        cur_time = rospy.Time.now()

        dist = np.sqrt((currState1.pose.position.x - currState2.pose.position.x)**2 + (currState1.pose.position.y - currState2.pose.position.y)**2)
        # if dist < 5:
        #     print("collision")
        #     # controller 2 as e4
        #     print("Unsafe!!")
            

        #     return False, pos_idx1, (cur_time - start_time).to_sec()


        
        
        if not currState1.success or not currState2.success:
            print("no model state")
            continue

        # Compute relative position between vehicle and waypoints
        distToTargetX1 = abs(target_x1 - currState1.pose.position.x)
        distToTargetY1 = abs(target_y1-  currState1.pose.position.y)

        distToTargetX2 = abs(target_x2 - currState2.pose.position.x)
        distToTargetY2 = abs(target_y2 - currState2.pose.position.y)

            

        if (cur_time - prev_wp_time).to_sec() > 4:
            print(f"failure to reach {pos_idx1}-th waypoint in time")
            return False, pos_idx1, (cur_time - start_time).to_sec()

        # e4 is auto car, e2 is human drive, so we focus on e4            
        if (distToTargetX2 < 2 and distToTargetY2 < 2): 
            # If the vehicle is close to the waypoint, move to the next waypoint
            prev_pos_idx1 = pos_idx1
            pos_idx1 = pos_idx1+1

            pos_idx2 = pos_idx2+1

            if pos_idx2 == len(pos_list2): #Reached all the waypoints for e4
                controller2.plot_log()
                print("Reached all the waypoints")
                print("safe!!") 
                total_time = (cur_time - start_time).to_sec()
                print(total_time)
                return True, pos_idx1, total_time

            # Check if (x, y) is inside the ellipse
            if dist < 3:
                print("⚠️ Unsafe! Vehicle entered elliptical zone.")
            
                # print("collision")
                # controller 2 as e4
                print("unsafeeeeeeeeeeeeeeeeeeeeeeeeeeeee!!")
                
                target_x1, target_y1 = pos_list1[pos_idx1]
                target_x2, target_y2 = pos_list2[pos_idx2]
                
            else:

                
                print("safe!!")
                
                # planer lane merge
                
                target_x1, target_y1 = pos_list1[pos_idx1]
                target_x2, target_y2 = pos_list1[pos_idx1]


                

            time_taken = (cur_time- prev_wp_time).to_sec()
            prev_wp_time = cur_time
        

            print(f"Time Taken: {round(time_taken, 2)}", "reached",pos_list1[prev_pos_idx1][0],pos_list1[prev_pos_idx1][1],"next",pos_list1[pos_idx1][0],pos_list1[pos_idx1][1])

        controller1.execute_e2(currState1, [target_x1, target_y1], pos_list1[pos_idx1:])
        controller2.execute_e4(currState2, [target_x2, target_y2], pos_list2[pos_idx2:])

if __name__ == "__main__":
    try:
        status, num_waypoints, time_taken = run_model()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down")
