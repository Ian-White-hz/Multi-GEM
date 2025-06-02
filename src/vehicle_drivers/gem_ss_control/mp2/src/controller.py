import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose,Twist
from waypoint_list import WayPoints
from matplotlib import pyplot as plt
import pygame
from src.carlo.highbay import HighwayWorld 
import time

class vehicleController():

    def __init__(self,keyboardControl):
        # Publisher to publish the control input to the vehicle model
        self.controlPube2 = rospy.Publisher("/e2/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.controlPube4 = rospy.Publisher("/e4/ackermann_cmd", AckermannDrive, queue_size = 1)
        
        if(keyboardControl):
            #sets subscriber and params for keyboard control for e2
            self.max_speed = 10.0  # Max linear velocity (m/s)
            self.max_steering_angle = 0.78  # Max steering angle (radians)
            self.controlSube2_keyboard = rospy.Subscriber("/cmd_vel", Twist, self.execute_keyboard)

        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = True

        self.acc_log = []
        self.traj_log_x = []
        self.traj_log_y = []

        self.friction = 0.0       #might have to change
        self.max_speed = 8
        self.max_steering_angle = 0.78     #45 degrees
        self.speed = 1e-4
        self.heading = 0
        self.angular_vel = 0
        self.acceleration = 1e-4
        self.prev_steering_angle = 0
        self.prev_time = rospy.Time.now().to_sec()
        self.steering_controller = steeringWheelController()

    def execute_keyboard(self, twist_msg):
        #Subscriber callback for teleop_keyboard control
        #Calculates speed, steering angle from keyboard inputs and publishes to ackermann_cmd
        
        keyboardAckermannCmd = AckermannDrive()

        keyboard_vel_e2 = np.sqrt(twist_msg.linear.x **2 + twist_msg.linear.y **2) + 7.0    #start speed in teleop is 1. Adding more to cope with e4
        keyboardAckermannCmd.speed = max(min(keyboard_vel_e2,self.max_speed),-self.max_speed)
        print("******** e2 VELOCITY = ",keyboardAckermannCmd.speed)
        keyboardAckermannCmd.steering_angle = max(min(twist_msg.angular.z, self.max_steering_angle),-self.max_steering_angle)

        #publish to ackermann_cmd topic to move
        self.controlPube2.publish(keyboardAckermannCmd)

    def set_steering_control(self):
        #just make sure steering params r received dynamiclally.
        self.execute_steering(self.steering_controller.steering, self.steering_controller.throttle, self.steering_controller.brake)
        
    def execute_steering(self,input_steering,input_throttle, input_brake):
        keyboardAckermannCmd = AckermannDrive()
        
        #define speed,heading,angular_vel,dt,acc,prev_steering_angle
        
        #how its used in influence-main: 
        '''
        speed - stored,used prev
        heading - stored,used prev
        angular_vel - stored,used prev
        dt - based on loop time
        acceleration - stored,used prev
        prev_steering_angle - stored,used prev
        '''
        #Raw input normalization
        input_steering *= self.max_steering_angle   #check if prev_steering_angle shd be changedin constructor
        input_throttle = (1+input_throttle) / 2.0
        input_brake = (1+input_brake) / 2.0
        # input_steering = 0.2 * self.prev_steering_angle + (1-0.2) * (input_steering)
        print("********",input_throttle,input_brake)
        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        #damp factor for vehicle ang velocity - but ang velo not used in ackermann at the moment
        speed_damp_factor =  self.speed/(self.speed + 1e-2)
        speed_damp_factor = 1
        new_angular_vel = speed_damp_factor * self.speed * input_steering
        new_acceleration = input_throttle - input_brake * 100
        new_acceleration*=1.25
        new_heading = self.heading + (self.angular_vel + new_angular_vel) * dt / 2.
        new_speed = np.clip(self.speed + (self.acceleration + new_acceleration) * dt / 2., 0,self.max_speed)

        keyboardAckermannCmd.speed = new_speed
        keyboardAckermannCmd.acceleration = new_acceleration
        keyboardAckermannCmd.steering_angle = input_steering
        keyboardAckermannCmd.steering_angle_velocity = (self.prev_steering_angle - input_steering) / (dt+1e-8)

        #storing for the next loop
        self.speed = new_speed
        self.heading = np.mod(new_heading, 2*np.pi)
        self.angular_vel = new_angular_vel
        self.acceleration = new_acceleration
        self.prev_steering_angle = input_steering

        self.controlPube2.publish(keyboardAckermannCmd)

    def getModelState1(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem_e2')
            
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp

    def getModelState2(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem_e4')
            
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp

    def extract_vehicle_info(self, currentPose):

        pos_x, pos_y, vel, yaw = 0, 0, 0, 0
        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y
        # find velocity from x and y components
        rospy.loginfo(f"Velocity components - x: {currentPose.twist.linear.x}, y: {currentPose.twist.linear.y}")
        vel = math.sqrt(currentPose.twist.linear.x ** 2 + currentPose.twist.linear.y ** 2)
        rospy.loginfo(f"Calculated velocity magnitude: {vel}")
        roll_pitch_yaw = quaternion_to_euler(currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w)
        yaw = roll_pitch_yaw[2]

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints, max_velocity = 0):
        # Compute Angle Tolerance in radian(degree/180*pi)
        tolerance = 7.5/180*math.pi
        target_velocity = max_velocity

        # See if the vehicle is far from curvature
        prev_waypoint = [curr_x,curr_y]
        prev_yaw = curr_yaw
        num_straight = 0
        for i in range(len(future_unreached_waypoints)):
            next_point = future_unreached_waypoints[i]
            # Compute target yaw
            next_yaw = np.arctan2((next_point[1]-prev_waypoint[1]), (next_point[0]-prev_waypoint[0]))
            if prev_yaw==next_yaw:
                num_straight+=1
            else:
                if i==0 and abs(next_yaw-prev_yaw)<=tolerance:
                    num_straight+=1
                break
            prev_waypoint = next_point
            prev_yaw = next_yaw
        
        # If current car is not pointing toward the waypoint, it means we have to turn (target velocity = 8); 
        # otherwise, go straight and target velocity = 10
        # if num_straight>=5:
        #     target_velocity = 5
        # elif num_straight>=3:
        #     target_velocity = 4
        # elif num_straight>=1:
        #     target_velocity = 3
        # else:
        #     target_velocity = 2

        return target_velocity




    # Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):
        distance = math.sqrt((target_point[0]-curr_x)**2+(target_point[1]-curr_y)**2)

        # if the next point is too close, skip it
        if distance < 3 and len(future_unreached_waypoints) >= 2:
            target_point[0] = future_unreached_waypoints[1][0]
            target_point[1] = future_unreached_waypoints[1][1]

        # Compute target yaw        
        target_yaw = np.arctan2((target_point[1]-curr_y), (target_point[0]-curr_x))
        # Compute alpha (angle between vehicle's heading and look-ahead line)
        alpha = target_yaw - curr_yaw
        # Compute lookahead distance ld (distance between vehicle's position and the lookahead point)
        ld = math.sqrt((target_point[0]-curr_x)**2+(target_point[1]-curr_y)**2)
        # Compute delta (= 2L*sin(alpha)/ld)
        target_steering = 2*self.L*math.sin(alpha)/ld

        return target_steering



    def execute_e2(self, currentPose, target_point, future_unreached_waypoints, max_velocity):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

        # Acceleration Profile
        # if self.log_acceleration:
        #     acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz



        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints, max_velocity)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)

        # rospy.loginfo(f"e2 target velocity: {target_velocity}")
        # rospy.loginfo(f"e2 target steering: {target_steering}")

        #Pack computed velocity and steering angle into Ackermann command 
        # e2
        newAckermannCmd1 = AckermannDrive()
        newAckermannCmd1.speed = target_velocity
        newAckermannCmd1.steering_angle = target_steering

        self.controlPube2.publish(newAckermannCmd1)


    def execute_e4(self, currentPose, target_point, future_unreached_waypoints, max_velocity):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)
        self.traj_log_x.append(curr_x)
        self.traj_log_y.append(curr_y)


        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz
            self.prev_vel = curr_vel
            # print('acc',acceleration)
            self.acc_log.append(acceleration)


        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints, max_velocity)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)


        #Pack computed velocity and steering angle into Ackermann command 

        # Publish the computed control input to vehicle model
        # e4

        newAckermannCmd2 = AckermannDrive()
        newAckermannCmd2.speed = target_velocity
        newAckermannCmd2.steering_angle = target_steering

        self.controlPube4.publish(newAckermannCmd2)

    def plot_log(self):
        # Check if there is any data to plot
        if not self.acc_log:
            print("No acceleration data to plot.")
            return
        
        # Create a time array based on the length of the logged data
        time_array = np.arange(len(self.acc_log)) / 100.0 
        waypoint = WayPoints()
        waypoint_array = np.array(waypoint.getWayPoints2())
        print(waypoint_array)

        
        plt.figure(figsize=(10, 6))
        plt.subplot(121)
        #plt.title('Trajectories and Way Points')
        plt.scatter(self.traj_log_x, self.traj_log_y, c = 'b', label = 'trajectories', s= 5)
        plt.scatter(waypoint_array[:,0],waypoint_array[:,1], c = 'r', label = 'way points', s= 5)
        plt.grid(True)
        plt.legend()
        plt.subplot(122)
        plt.plot(time_array, self.acc_log, label="Acc (m^2/s)")
        plt.title('Vehicle Acc Over Time')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Acc (m/s)')
        plt.grid(True)
        plt.legend()
        plt.show()

    def calculate_time_to_collision(self, e2_pos_x, e2_pos_y, e2_vel, e2_yaw, e4_pos_x, e4_pos_y, e4_vel, e4_yaw):
        # Calculate the time to collision between two vehicles
        # Input:
        #   currState_e2: ModelState, the current state of the first vehicle
        #   currState_e4: ModelState, the current state of the second vehicle
        # Output: float, the time to collision between the two vehicles
        #   return -1 if the time to collision is negative or imaginary

        
        
        # Convert to vectors for easier calculation
        # Position vectors
        p1 = np.array([e2_pos_x, e2_pos_y])
        p2 = np.array([e4_pos_x, e4_pos_y])
        
        # Velocity vectors (magnitude * direction)
        v1 = np.array([e2_vel * np.cos(e2_yaw), e2_vel * np.sin(e2_yaw)])
        v2 = np.array([e4_vel * np.cos(e4_yaw), e4_vel * np.sin(e4_yaw)])
        
        # Calculate relative position and velocity
        p_rel = p2 - p1  # Relative position vector
        v_rel = v2 - v1  # Relative velocity vector
        
        # Minimum acceptable distance (vehicle safety buffer)
        d_min = 2.5
        
        # Calculate the magnitude of the relative velocity
        v_rel_mag = np.linalg.norm(v_rel)
        
        # If vehicles are not moving relative to each other, no collision will occur
        if v_rel_mag < 0.001:  # Small threshold to avoid division by zero
            return -1
        
        # Calculate the dot product of relative velocity and relative position
        dot_product = np.dot(v_rel, p_rel)
        
        # Calculate the discriminant under the square root
        discriminant = (dot_product)**2 - (v_rel_mag**2) * (np.linalg.norm(p_rel)**2 - d_min**2)
        
        # If discriminant is negative, the vehicles will not collide
        if discriminant < 0:
            return -1
        
        # Calculate TTC using the formula from the image
        # TTC = (-dot_product ± sqrt(discriminant)) / |v_rel|^2
        # We take the smaller positive value
        ttc1 = (-dot_product + np.sqrt(discriminant)) / (v_rel_mag**2)
        ttc2 = (-dot_product - np.sqrt(discriminant)) / (v_rel_mag**2)
        
        # Choose the smaller positive TTC
        if ttc1 > 0 and ttc2 > 0:
            time_to_collision = ttc2
        else:
            # No future collision (vehicles are moving away from each other)
            time_to_collision = -1
            
        return time_to_collision


    def stop(self, vehicle):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        if vehicle == 'e2':
            self.controlPube2.publish(newAckermannCmd)
        else:
            self.controlPube4.publish(newAckermannCmd)


class steeringWheelController():
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        else:
            print("******** No SteeringWheel detected *********")
            self.joystick = None
    
    @property
    def steering(self):
        if self.joystick:
            events = pygame.event.get()
            return -self.joystick.get_axis(0) * 0.5
        else:
            return 0     # No steering if no steeringwheel

    @property
    def throttle(self):
        if self.joystick:
            events = pygame.event.get()
            return -self.joystick.get_axis(2)
        else:
            return 0     # No throttle if no steeringwheel
        
    @property
    def brake(self):
        if self.joystick:
            events = pygame.event.get()
            return -self.joystick.get_axis(1)
        else:
            return 0     # No brake if no steeringwheel

class MPCcontroller():
    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPube4 = rospy.Publisher("/e4/ackermann_cmd", AckermannDrive, queue_size=1)
        self.max_steering_angle = 0.78  # 45 degrees, adjust as needed
        self.max_speed = 8
        self.world = HighwayWorld()     
        self.world.setup_world()
        self.world.init_car()
        self.speed = 1000

    def getModelState2(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem_e4')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp

    def extract_vehicle_info(self, currentPose):

        pos_x, pos_y, vel, yaw = 0, 0, 0, 0
        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y
        # find velocity from x and y components
        # rospy.loginfo(f"Current speed: {self.speed}")
        # rospy.loginfo(f"Velocity components - x: {currentPose.twist.linear.x}, y: {currentPose.twist.linear.y}")
        vel = math.sqrt(currentPose.twist.linear.x ** 2 + currentPose.twist.linear.y ** 2)
        # rospy.loginfo(f"Calculated velocity magnitude: {vel}")
        roll_pitch_yaw = quaternion_to_euler(currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w)
        yaw = roll_pitch_yaw[2]

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    def mpc_controller(self, currState_e4, currState_e2):
        # Extract state for MPC
        # Extract vehicle states using helper function
        e4_x, e4_y, e4_vel, e4_yaw = self.extract_vehicle_info(currState_e4)
        print("_____________________")
        print("e4_vel", e4_vel)
        self.speed = e4_vel

        e2_x, e2_y, e2_vel, e2_yaw = self.extract_vehicle_info(currState_e2)
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
        print("acceleration", acceleration)
       
        # Clip to vehicle limits
        steering = max(min(steering, self.max_steering_angle), -self.max_steering_angle)
        # acceleration = max(min(acceleration, self.max_speed), -self.max_speed)
        
        return steering, acceleration
    
    def execute_e4(self, currState_e4, currState_e2):
        steering, acceleration = self.mpc_controller(currState_e4, currState_e2)
        cmd = AckermannDrive()
        cmd.steering_angle = steering
        # cmd.acceleration = acceleration 
        cmd.speed = self.speed + acceleration * 0.5
        # cmd.jerk = acceleration

        self.controlPube4.publish(cmd)