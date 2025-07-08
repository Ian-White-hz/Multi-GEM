$ source devel/setup.bash 
$ roslaunch basic_launch sensor_init.launch 

$ source devel/setup.bash 
$ bash src/utility/radar_start.sh 

# --------------------------------------------

$ source devel/setup.bash 
$ roslaunch basic_launch visualization.launch 

$ source devel/setup.bash
$ roslaunch basic_launch dbw_joystick.launch

/e2/septentrio_gnss/imu /e2/septentrio_gnss/insnavgeod /e2/septentrio_gnss/navsatfix /septentrio_gnss/imu /septentrio_gnss/insnavgeod /septentrio_gnss/navsatfix
