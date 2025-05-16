# stackelberg_trust

Repeated stackelberg games where the robot tries to manipulate the human, potentially breaking trust.

## INSTRUCTIONS

We base our implementation on CARLO. https://github.com/Stanford-ILIAD/CARLO

There are 3 different driving scenarios: an intersection, a roundabout, and a highway scenario. You will find two .py codes for each. Below is a list of them:

- highway.py && mpc_highway.py
- intersection && mpc_intersection.py
- roundabout.py && mpc_roundabout.py

--> Run the code from the left column of the above list to interact with each scene. We call them "main" codes here.

### TO MAKE CHANGES
- If you want to change size of the simulation change ppm in "w = World(dt, width = 120, height = 120, ppm = 5 " at the begining of each main code. 
- Higher ppm == bigger objects

### TO INTERACT WITH THE SCENES
- Plug in a joystick. Tested with Logitech Gamepad F310, Logitech Rumblepad 2, PS4 controllers. The code does not work with Xbox controllers.
- You can use Logitech G29 Steering wheel to interact with the scenes as well.

--> You can make changes to the joystick/steering wheel axes in lines 73 & 77 of the interactive_controllers.py

Note: There is a keyboard controller, but controlling the scenes with it is very laggy.

## SETTING UP SCENES FOR USER STUDY
All the scenes are automated using bash. 

On Linux and MacOs:
1. Run `sh pract_bash.sh` to run the practice scenes
2. Run `sh block1_bash.sh` to run the first set of scenes
3. Increase the numerical value of `block1` to run the rest of the blocks

On Windows:
Modify the bash files to be `py <name of file>.py`

