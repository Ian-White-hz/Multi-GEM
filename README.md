# GEM

## Overview

GEM is a robotics platform integrating multiple sensors and drivers for autonomous vehicle research and development. This workspace includes ROS packages and utilities for GNSS, LiDAR, radar, and vehicle control.

## Main Topics

The following ROS topics are commonly used in this workspace:
- `/livox/lidar`
- `/e2/septentrio_gnss/insnavgeod`
- `/e2/septentrio_gnss/navsatfix`
- `/septentrio_gnss/insnavgeod`
- `/ouster/points`
- `/ouster/scan`
- `/tf`
- `/tf_static`
 rosbag record /e2/septentrio_gnss/insnavgeod /e2/septentrio_gnss/navsatfix /septentrio_gnss/insnavgeod /septentrio_gnss/navsatfix

## Quick Start

To run the GNSS tracker script:
```sh
python3 src/vehicle_drivers/gem_gnss_control/scripts/gem_gnss_tracker_pp.py
```

To run the ss control script:
```sh
python3 /src/vehicle_drivers/gem_ss_control/mp2/src/main_highbay_steering_mpc.py
```

## Directory Structure

- `src/`: Source code for drivers, utilities, and launch files.
- `stackelberg_trust/`: Research code for trust and influence in autonomous driving.
- `results/`: Output plots and results from experiments.
- `ag1_a_RTK*/`: Example datasets and logs.

## Building the Workspace

1. Initialize and configure your catkin workspace:
    ```sh
    cd ~/GEM
    catkin init
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    ```
2. Build the workspace:
    ```sh
    catkin build
    ```
3. Source the setup file:
    ```sh
    source devel/setup.bash
    ```

## Launching Sensors and Visualization

Example launch commands:
```sh
roslaunch basic_launch sensor_init.launch
roslaunch basic_launch visualization.launch
roslaunch basic_launch dbw_joystick.launch
```

## Documentation

- See [src/readme.txt](src/readme.txt) for more launch and usage examples.
- Refer to each package's README for specific instructions.


