import numpy as np
from world import World
from agents import Car, RectangleBuilding, Painting, RingBuilding, CircleBuilding
from interactive_controllers import KeyboardController, SteeringWheelController
from geometry import Point
import time
import copy
from mpc_roundabout import MPC
import pickle
from utils import R_score, Flag

# User ID
user_id = 12 # Manually change 2 to 3, etc. for each user

# initilize GUI
flag = Flag()
score = R_score()

total_score = 0 # Cumulative score across all scenes and iterations

dt = 1.0 # time steps in terms of seconds. In other words, 1/dt is the FPS.
w = World(dt, width = 120, height = 120, ppm = 5) # The world is 120 meters by 120 meters. ppm is the pixels per meter.

# Roads
yr = RectangleBuilding(Point(60, 60), Point(40, 160), 'gray26')
w.add(yr)
xr = RectangleBuilding(Point(60, 60), Point(200, 40), 'gray26')
w.add(xr)

# To make roundabout we make CircleBuilding and then RingBuilding
cb = CircleBuilding(Point(60, 60), 15, 'gray80')
w.add(cb)
cr = RingBuilding(Point(60, 60), 15, 45, 'gray26')
w.add(cr)

# Lane markings
for idx in range(0, 30, 10):
    w.add(Painting(Point(60, idx), Point(0.7, 3), 'white'))
for idx in range(100, 121, 10):
    w.add(Painting(Point(60, idx), Point(0.7, 3), 'white'))
for idx in range(0, 30, 10):
    w.add(Painting(Point(idx, 60), Point(3, 0.7), 'white'))
for idx in range(100, 121, 10):
    w.add(Painting(Point(idx, 60), Point(3, 0.7), 'white'))


# A Car object is a dynamic object -- it can move.
c1 = Car(Point(120,70), np.pi, 'orange')
c1.velocity = Point(0, 0)
c1.max_speed = +0.5
c1.min_speed = -0.5
w.add(c1)
c2 = Car(Point(70,20), np.pi/2, 'blue')
c2.velocity = Point(0, 3.0)
c2.max_speed = +1.0
c2.min_speed = -1.0
w.add(c2)

w.render() # This visualizes the world we just constructed.
mpc = MPC(horizon=4, max_iter=3) # Create the controller for our robt

# Mechanics of the flag screen to display and close
flag.flag_text.config (bg = '#B81D13', text = flag.flag[0])
flag.root.update()
time.sleep(0.8)
flag.flag_text.config (bg = '#EFB700', text = flag.flag[1])
flag.root.update()
time.sleep(0.8)
flag.flag_text.config (bg = '#008450', text = flag.flag[2])
flag.root.update()
time.sleep(0.8)
flag.root.destroy()

# open the score file. First time is handled with try/except
try:
    total_score = pickle.load(open("runs/total_score_u" + str(user_id) + ".pkl", "rb"))
except:
    pass

dataset = []
controller = SteeringWheelController(w)
roundab_score = 0 # running score
for k in range(125):
    start_time = time.time()
    human_car_y0 = c2.y
    mpc.update_world(w)
    mpc.optimize_r()
    UR = mpc.robot_actions
    UH = mpc.human_actions
    c1.set_control(-0.02, UR[0])
    c2.set_control(controller.steering, controller.throttle)
    state = [k, c1.x, c1.y, c1.heading, c2.x, c2.y, c2.heading, c1.collidesWith(c2), roundab_score, total_score]
    action = [-0.02, UR[0], controller.steering, controller.throttle]
    dataset.append(state + action)
    w.tick()
    human_car_y1 = c2.y
    w.render()
    # print("step number: ", k, "step time: ", time.time() - start_time)
    delta_human_car_y = abs(human_car_y1 - human_car_y0)
    # Giving positive scores for staying on the road. Negative for off road
    if (42 <= c2.x <= 100) and (c2.collidesWith(cb) == False): # upper was 78 users are not driving off the road much. so increase the x bound for better scoring
        roundab_score += delta_human_car_y
        score.textbox5.delete(0, 'end')
        score.textbox5.insert(0, str(int(roundab_score)))
        score.root.update()
    else:
        roundab_score -= 10 # out of the road
        score.textbox5.delete(0, 'end')
        score.textbox5.insert(0, str(int(roundab_score)))
        score.root.update()
    if c1.collidesWith(c2): # collision with the other car
        print('Collision!')
        roundab_score -= 100
        score.textbox5.delete(0, 'end')
        score.textbox5.insert(0, str(int(roundab_score)))
        score.root.update()
    # Handling the first time when a pickle does not exist
    try:
        score.textbox6.delete(0, 'end')
        score.textbox6.insert(0, str(int(total_score)))
        score.root.update()
    except:
        pass
w.close()
pickle.dump(dataset, open("runs/roundabout_u" + str(user_id) + "_" + time.strftime("%a; %b %d %Y; %H.%M.%S") + ".pkl", "wb")) # Manually change u1 to u2, etc. for each user
try:
    pickle.dump(int(total_score + roundab_score), open("runs/total_score_u" + str(user_id) + ".pkl", "wb")) # Manually change u1 to u2, u3, etc.
except:
    pickle.dump(int(roundab_score), open("runs/roundabout_score_u" + str(user_id) + ".pkl", "wb"))
