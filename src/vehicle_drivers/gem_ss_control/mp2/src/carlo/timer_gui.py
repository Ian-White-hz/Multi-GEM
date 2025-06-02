from utils import Timer
import time

# initilize timer GUI
timer = Timer()

# Mechanics of the timer screen to display and close
timer.timer_text.config (bg = '#B81D13', text = timer.timer[0])
timer.root.update()
time.sleep(0.3)
timer.timer_text.config (bg = '#EFB700', text = timer.timer[1])
timer.root.update()
time.sleep(0.3)
timer.timer_text.config (bg = '#008450', text = timer.timer[2])
timer.root.update()
time.sleep(0.3)
timer.root.destroy()