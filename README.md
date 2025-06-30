# GEM

## Overview

GEM is a robotics platform integrating multiple sensors and drivers for autonomous vehicle research and development. This workspace includes ROS packages and utilities for GNSS, LiDAR, radar, and vehicle control.

# GEMstack: software for Towards Robots that Influence Humans over Long-Term Interaction

[Original paper](https://ieeexplore.ieee.org/abstract/document/10160321)

[Online documentation](https://gemstack.readthedocs.org) 
[About the GEM e2 vehicle](https://publish.illinois.edu/robotics-autonomy-resources/gem/)

<div align="center">
  <a href="https://www.youtube.com/watch?v=ePmhrkKGKno">
    <img src="https://img.youtube.com/vi/VIDEO_ID/maxresdefault.jpg" 
         alt="视频标题" 
         style="width:60%;">
  </a>
  <p><em>点击上方图片观看视频</em></p>
</div>


## Dependencies
GEMstack uses **Python 3.7+** and **ROS Noetic**.  
(It is possible to do some offline and simulation work without ROS, but it is highly recommended to install it if you are working on any onboard behavior or training for rosbag files.)


## 📂 Directory layout
Please pay attention to MAIN ****
```text
.

├── results/                   # Generated figures / plots
│
├── rosbags/                   # Data capture & analysis tools
│
├── src/                       # ROS catkin workspace (Noetic)
│   ├── CMakeLists.txt → /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
│   ├── basic_launch/          # Launch files for quick testing
│   ├── hardware_drivers/      # Low-level CAN / PACMod / sensor nodes
│   ├── readme.txt
│   ├── utility/               # Generic ROS utilities (tf, loggers…)
│   └── vehicle_drivers/       # GEM-specific high-level drivers
|        ├── gem_gnss_control
        ├── gem_ss_control
        │   ├── actor_collision
        │   ├── carlo   --------------------------  #  ← SUPPORT MAIN on-vehicle MPC
                ├── agents.py
                ├── entities.py  ------------------ #  ← MAIN MPC VEHICLE MODEL
                ├── geometry.py
                ├── graphics.py
                ├── highbay.py   ------------------ #  ← MAIN MPC POLICY
                ├── highway.py
                ├── interactive_controllers.py
                ├── intersection.py
                ├── mpc_highway.py  --------------- #  ← MAIN MPC COST FUNCTION
                ├── mpc_intersection.py
                ├── mpc_roundabout.py
                ├── pkl_to_csv_converter.py
                └── world.py
        │   ├── notebooks
        │   └── velodyne_simulator 
        └── gem_visualization
```
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


