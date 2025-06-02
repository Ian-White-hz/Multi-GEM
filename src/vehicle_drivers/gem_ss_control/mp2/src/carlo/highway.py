import numpy as np
from world import World
from agents import Car, RectangleBuilding, Pedestrian, Painting
from interactive_controllers import KeyboardController, SteeringWheelController
from geometry import Point
import time
import copy
from mpc_highway import MPC
import pickle
from utils import H_score, Flag

# User ID
user_id = 12 # Manually change 2 to 3, etc. for each user

# initilize GUI
flag = Flag()
score = H_score()

total_score = 0 # Cumulative score across all scenes and iterations

dt = 1.0 # time steps in terms of seconds. In other words, 1/dt is the FPS.
w = World(dt, width = 120, height = 120, ppm = 5) # The world is 120 meters by 120 meters. ppm is the pixels per meter.

# Let's add some sidewalks
w.add(Painting(Point(25, 60), Point(50, 150), 'gray80'))
w.add(Painting(Point(95, 60), Point(50, 150), 'gray80'))

# Then we add some rectangle buildings on top
w.add(RectangleBuilding(Point(20, 60), Point(48, 150)))
w.add(RectangleBuilding(Point(100, 60), Point(48, 150)))

# # Let's also add some lane markings
for idx in range(0, 121, 10):
    w.add(Painting(Point(60, idx), Point(0.5, 3), 'white'))


# A Car object is a dynamic object -- it can move.
c1 = Car(Point(65,30), np.pi/2, 'orange')
c1.velocity = Point(0, 1.5)
c1.max_speed = 1.5
c1.min_speed = 0.75
w.add(c1)
c2 = Car(Point(55,10), np.pi/2, 'blue')
c2.velocity = Point(0, 1.5)
c2.max_speed = 1.75
c2.min_speed = 0
w.add(c2)

w.render() # This visualizes the world we just constructed.
mpc = MPC(horizon=3, max_iter=2) # Create the controller for our robt

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
# controller = SteeringWheelController(w)
controller = KeyboardController(w) # Use keyboard controller for the human car
highway_score = 0 # running score
for k in range(75):
    start_time = time.time()
    human_car_y0 = c2.y
    mpc.update_world(w)
    mpc.optimize_r()
    UR = mpc.robot_actions
    UH = mpc.human_actions
    c1.set_control(UR[0, 0], UR[0, 1])
    c2.set_control(controller.steering, controller.throttle)
    state = [k, c1.x, c1.y, c1.heading, c2.x, c2.y, c2.heading, c1.collidesWith(c2), highway_score, total_score]
    action = [UR[0, 0], UR[0, 1], controller.steering, controller.throttle]
    dataset.append(state + action)
    w.tick()
    human_car_y1 = c2.y
    w.render()
    # print("step number: ", k, "step time: ", time.time() - start_time)
    delta_human_car = (human_car_y1 - human_car_y0)
    # Giving positive scores for staying on the road. Negative for off road
    if 52 <= c2.x <= 68:
        highway_score += delta_human_car
        score.textbox1.delete(0, 'end')
        score.textbox1.insert(0, str(int(highway_score)))
        score.root.update()
    else:
        highway_score -= 10
        score.textbox1.delete(0, 'end')
        score.textbox1.insert(0, str(int(highway_score)))
        score.root.update()
    if c1.collidesWith(c2):
        print('Collision!')
        highway_score -= 100
        score.textbox1.delete(0, 'end')
        score.textbox1.insert(0, str(int(highway_score)))
        score.root.update()
    try: # Handling the first time when a pickle does not exist
        score.textbox2.delete(0, 'end')
        score.textbox2.insert(0, str(int(total_score)))
        score.root.update()
    except:
        pass
w.close()
pickle.dump(dataset, open("runs/highway_u" + str(user_id) + "_" + time.strftime("%a; %b %d %Y; %H.%M.%S") + ".pkl", "wb")) # Manually change u1 to u2, etc. for each user
try:
    pickle.dump(int(total_score + highway_score), open("runs/total_score_u" + str(user_id) + ".pkl", "wb")) # Manually change u1 to u2, u3, etc.
except:
    pickle.dump(int(highway_score), open("runs/highway_score_u" + str(user_id) + ".pkl", "wb"))
