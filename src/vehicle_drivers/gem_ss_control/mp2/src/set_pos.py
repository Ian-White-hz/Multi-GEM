import sys
import os
import argparse

import numpy as np

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState

from util import euler_to_quaternion

def getModelState_e2():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        modelState = serviceResponse(model_name='gem_e2')
    except rospy.ServiceException as exc:
        rospy.loginfo("Service did not process request: "+str(exc))
    return modelState

def getModelState_e4():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        modelState = serviceResponse(model_name='gem_e4')
    except rospy.ServiceException as exc:
        rospy.loginfo("Service did not process request: "+str(exc))
    return modelState

def setModelState(model_state):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(model_state)
    except rospy.ServiceException as e:
        rospy.loginfo("Service did not process request: "+str(e))

def set_position_e2(x = 0,y = 0, yaw=0):
    
    rospy.init_node("set_pos")

    curr_state = getModelState_e2()
    new_state = ModelState()
    new_state.model_name = 'gem_e2'
    new_state.twist.linear.x = 0
    new_state.twist.linear.y = 0
    new_state.twist.linear.z = 0
    new_state.pose.position.x = x
    new_state.pose.position.y = y
    new_state.pose.position.z = 1
    q = euler_to_quaternion([0,0,yaw])
    new_state.pose.orientation.x = q[0]
    new_state.pose.orientation.y = q[1]
    new_state.pose.orientation.z = q[2]
    new_state.pose.orientation.w = q[3]
    new_state.twist.angular.x = 0
    new_state.twist.angular.y = 0
    new_state.twist.angular.z = 0
    setModelState(new_state)

def set_position_e4(x = 0,y = 0, yaw=0):
    
    rospy.init_node("set_pos")

    curr_state = getModelState_e4()
    new_state = ModelState()
    new_state.model_name = 'gem_e4'
    new_state.twist.linear.x = 0
    new_state.twist.linear.y = 0
    new_state.twist.linear.z = 0
    new_state.pose.position.x = x
    new_state.pose.position.y = y
    new_state.pose.position.z = 1
    q = euler_to_quaternion([0,0,yaw])
    new_state.pose.orientation.x = q[0]
    new_state.pose.orientation.y = q[1]
    new_state.pose.orientation.z = q[2]
    new_state.pose.orientation.w = q[3]
    new_state.twist.angular.x = 0
    new_state.twist.angular.y = 0
    new_state.twist.angular.z = 0
    setModelState(new_state)

if __name__ == "__main__":
    parser1 = argparse.ArgumentParser(description = 'Set the x, y position of the vehicle')

    # x_default = -20
    # y_default = -103
    # yaw_default = 0
    x_default = -40
    y_default = -17.50
    z_default = 0
    yaw_default = 0


    


    parser1.add_argument('--x', type = float, help = 'x position of the vehicle.', default = x_default)
    parser1.add_argument('--y', type = float, help = 'y position of the vehicle.', default = y_default)
    parser1.add_argument('--z', type = float, help = 'z position of the vehicle.', default = z_default)
    parser1.add_argument('--yaw', type = float, help = 'yaw of the vehicle.', default = yaw_default)

    argv = parser1.parse_args()

    x = argv.x
    y = argv.y
    yaw = argv.yaw

    set_position_e2(x = x, y = y, yaw = yaw)

    # x_default = 0
    # y_default = -98
    # yaw_default = 0

    # x_default = -25
    # y_default = -16.25
    # yaw_default = 0

    x_default = -30
    y_default = -22.50
    z_default = -0.5
    yaw_default = 0


    
    parser = argparse.ArgumentParser(description = 'Set the x, y position of the vehicle')

    parser.add_argument('--x', type = float, help = 'x position of the vehicle.', default = x_default)
    parser.add_argument('--y', type = float, help = 'y position of the vehicle.', default = y_default)
    parser.add_argument('--z', type = float, help = 'z position of the vehicle.', default = z_default)
    parser.add_argument('--yaw', type = float, help = 'yaw of the vehicle.', default = yaw_default)

    argv = parser.parse_args()

    x = argv.x
    y = argv.y
    yaw = argv.yaw

    set_position_e4(x = x, y = y, yaw = yaw)
