from djitellopy import Tello
import pygame
import numpy as np
import time
import pickle
import argparse



#options are 0 for baseline and 1 for ours
parser = argparse.ArgumentParser()
parser.add_argument('--algo', type=int, default=0)
parser.add_argument('--user', type=int, default=0)
args = parser.parse_args()


def belief_update(belief, human, robot):
    distance = np.linalg.norm(human - robot)
    P_crash = np.exp(-0.5*distance)
    P_safe = 1 - P_crash
    belief[0] *= P_crash
    belief[1] *= P_safe
    belief /= np.sum(belief)
    if belief[0] < 0.1:
        belief = np.array([0.1, 0.9])
    if belief[0] > 0.9:
        belief = np.array([0.9, 0.1])
    return belief


class JoystickControl(object):

    def __init__(self):
        pygame.init()
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        self.deadband = 0.1
        self.timeband = 0.5
        self.lastpress = time.time()

    def getInput(self):
        pygame.event.get()
        curr_time = time.time()
        A_pressed = self.gamepad.get_button(0) and (curr_time - self.lastpress > self.timeband)
        B_pressed = self.gamepad.get_button(1) and (curr_time - self.lastpress > self.timeband)
        X_pressed = self.gamepad.get_button(2) and (curr_time - self.lastpress > self.timeband)
        START_pressed = self.gamepad.get_button(7) and (curr_time - self.lastpress > self.timeband)
        dyaw = self.gamepad.get_axis(3)
        if abs(dyaw) < self.deadband:
            dyaw = 0.0
        if A_pressed or START_pressed or B_pressed or X_pressed:
            self.lastpress = curr_time
        return A_pressed, B_pressed, X_pressed, START_pressed, dyaw


tello = Tello()
tello.connect()
joystick = JoystickControl()
tello.send_rc_control(0, 0, 0, 0)

takeoff = False
start_run = False
scale_actions = 60

robot = None
human = None
belief = np.array([0.5, 0.5])
actions = None
direction = -1.0
switch = False
on_left = False
on_right = True

savename = "data/user" + str(args.user) + "/algo" + str(args.algo)
outfile = open(savename, 'wb')
save_data = []


while True:
    A_pressed, B_pressed, X_pressed, START_pressed, dyaw = joystick.getInput()

    if A_pressed and not takeoff:
        takeoff = True
        tello.send_rc_control(0, 0, 0, 0)
        tello.takeoff()
    if B_pressed and takeoff:
        takeoff = False
        tello.send_rc_control(0, 0, 0, 0)
        tello.land()
    if X_pressed and not takeoff:
        tello.send_rc_control(0, 0, 0, 0)
        tello.end()
        break
    if START_pressed and takeoff:
        start_run = True
        start_time = time.time()


    # get robot position
    try:
        robot = np.array(pickle.load(open('drone_position.pkl', 'rb')))
    except EOFError:
       True

    # get human position
    try:
        human = np.array(pickle.load(open('vive_position.pkl', 'rb')))
    except EOFError:
        True

    # get robot actions
    parameters = np.array([robot[0], robot[1], human[0], human[1], belief[0], belief[1], direction, switch])
    pickle.dump(parameters, open('optimization_parameters.pkl', 'wb'))
    try:
        actions = np.array(pickle.load(open('actions.pkl', 'rb')))
    except EOFError:
        True

    belief = belief_update(belief, human, robot)
    print("robot: ", np.round(robot, 2), "human: ", np.round(human, 2), "switch: ", switch)

    # if start_run:
    if start_run:

        # SAVE THE DATA
        # [0] = timestamp; [1] = robot pos x; [2] = robot pos y; [3] = human pos x;
        # [4] = human pos y; [5] = mpc action; [6] = switch; [7] = direction
        elapsed_time = time.time() - start_time
        current_data = [elapsed_time] + robot.tolist() + human.tolist() + list(actions[0]) + [switch] + [direction]
        save_data.append(current_data)

        # KEEP THE DRONE FROM DRIFTING LEFT / RIGHT
        if robot[0] > 1.05:
            dx = -10
        elif robot[0] < 0.85:
            dx = +10
        else:
            dx = 0

        # STOP THE DRONE AT EDGES OF THE WORKSPACE
        dy = actions[0]
        dy = int(dy * scale_actions * 7)
        if robot[1] > 1.9 and dy > 0:
            dy = 0
        if robot[1] > 2.0 and dy > 0:
            dy = -30
        if robot[1] < 0.8 and dy < 0:
            dy = 0
        if robot[1] < 0.7 and dy < 0:
            dy = +30

        # SWITCH SIDES ON HUMAN MOVEMENT
        if human[0] < -1.0:
            on_right = True
        if on_right and human[0] > -0.9:
            on_right = False
            direction = +1.0
            if args.algo == 1:
                if np.random.rand() > 0.25:
                    switch = True
                else:
                    switch = False
        if human[0] > 2.8:
            on_left = True
        if on_left and human[0] < 2.7:
            on_left = False
            direction = -1.0
            if args.algo == 1:
                if np.random.rand() > 0.25:
                    switch = True
                else:
                    switch = False

        # SWITCH SIDES RANDOMLY
        # if robot[1] > 2.1:
        #     direction = -1.0
        #     on_left = True
        #     if on_left and on_right and args.algo == 1:
        #         if np.random.rand() > 0.25:
        #             switch = True
        #         else:
        #             switch = False
        #     on_right = False
        # elif robot[1] < 0.8:
        #     direction = 1.0
        #     on_right = True
        #     if on_left and on_right and args.algo == 1:
        #         if np.random.rand() > 0.25:
        #             switch = True
        #         else:
        #             switch = False
        #     on_left = False


        # assigns the robot's yaw speed
        dyaw = int(dyaw * 60)

    else:

        dx = 0
        dy = 0
        dyaw = 0

    # send the robot move command
    tello.send_rc_control(dx, dy, 0, dyaw)

pickle.dump(save_data, outfile)
print(len(save_data))
print("Data saved and program ended.")
