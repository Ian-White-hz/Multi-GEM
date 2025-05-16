import numpy as np
from world import World
from agents import Car, RectangleBuilding, Painting
from interactive_controllers import KeyboardController, SteeringWheelController
from geometry import Point
import time
from utils import Flag

# initilize GUI
flag = Flag()

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
c2 = Car(Point(55,10), np.pi/2, 'blue')
c2.velocity = Point(0, 1.5)
c2.max_speed = 1.75
c2.min_speed = 0
w.add(c2)

w.render() # This visualizes the world we just constructed.

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

controller = SteeringWheelController(w)
for k in range(75):
    start_time = time.time()
    c2.set_control(controller.steering, controller.throttle)
    w.tick()
    w.render()
    time.sleep(0.15)
w.close()
