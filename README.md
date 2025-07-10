# Multi-GEM: A multi-agent control and coordination framework for GEM vehicles

## Overview

Multi-GEM is a modular framework for research and development with multiple GEM autonomous vehicles, with a special focus on game-theoretic approaches to multi-agent coordination and control. It provides tools, controllers, and utilities for experimenting with and deploying advanced strategies—including those based on game theory—in scenarios involving multiple GEM vehicles, both in simulation and on real hardware.

The framework supports a variety of driving environments—including highways, intersections, and roundabouts—and enables the integration of both classical and advanced control strategies such as Model Predictive Control (MPC) and game-theoretic decision making. Multi-GEM is designed to facilitate rapid prototyping, testing, and deployment of multi-vehicle algorithms, making it ideal for academic research, education, and real-world experimentation.

Key features include:

- Support for multi-agent coordination and control using game-theoretic and classical methods
- Ready-to-use modules for common driving scenarios
- Integration with ROS and Gazebo for seamless simulation and real-vehicle interfacing
- Extensible architecture for custom controllers and environments
- Utilities for data logging, visualization, and analysis

Whether you are developing new game-theoretic control algorithms, testing cooperative driving strategies, or deploying on real GEM vehicles, Multi-GEM provides a robust foundation for your work.

> Note: For the simulator, please visit https://github.com/yejimun/GEM_simulator.

## Related Works
### Towards Robots that Influence Humans over Long-Term Interaction
**Authors.** Conference/Journal, Year.
[Original paper](https://ieeexplore.ieee.org/abstract/document/10160321)


## Demo
[![Demo Video](https://img.youtube.com/vi/ePmhrkKGKno/0.jpg)](https://www.youtube.com/watch?v=ePmhrkKGKno)

## Dependencies
GEMstack uses **Python 3.7+** and **ROS Noetic**.  
(It is possible to do some offline and simulation work without ROS, but it is highly recommended to install it if you are working on any onboard behavior or training for rosbag files.)


## Directory layout
Please pay attention to MAIN ****
### Legend

- 🟥 **TODO**
- 🟧 **early development** (usable, but many features not complete or tested)
- 🟨 **in development** (usable but to be tuned)
- 🟩 **stable** (most features complete and tested)
- 🟦 **mature**

```text
.

├── src/                       # ROS catkin workspace (Noetic)
│   ├── CMakeLists.txt → /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
│   ├── basic_launch/          # Launch files for quick testing
│   ├── hardware_drivers/      # Low-level CAN / PACMod / sensor nodes
│   ├── readme.txt
│   ├── utility/               # Generic ROS utilities (tf, loggers…)
│   └── vehicle_drivers/       # GEM-specific high-level drivers
|       ├── gem_gnss_control
        ├── gem_ss_control
        │   ├── actor_collision # simulator support
        |   ├── src
                ├── carlo                ------------------ 🟨  ← MAIN MPC VEHICLE MODEL FUNCTIONS
                        ├── entities.py  ------------------ 🟨  ← MAIN MPC VEHICLE KINEMATIC MODEL
                        ├── highbay.py   ------------------ 🟩  ← MAIN MPC POLICY API
                        ├── mpc_highway.py  --------------- 🟨  ← MAIN MPC COST FUNCTION
                        ├── agents.py    ------------------ 🟩  ← MAIN IMAGINARY CAR CLASSES
                        ├── world.py     ------------------ 🟩  ← MAIN IMAGINARY CARLO WORLD
                ├── main_highbay_steering_mpc.py ---------- 🟧  ← MAIN MPC RUN FILE
                ├── controller.py        ------------------ 🟧  ← MAIN MPC CLASS
        │   ├── notebooks
        │   └── velodyne_simulator # simulator support
        └── gem_visualization      # simulator support
```
### MAIN IDEA: 

We build a MPC for GEMSTACK in real world from a relatively sophisticated simulation platform carlo, the pipline of making this work is to pass real world data ( lon to x, lat to y, absolute yaw) to simulation carlo world(Imaginary). After small horizon iteration, we would have output data (acceleration in m/s^2, heading) from carlo. In order to make these output data align to ackermn cmd, we make calibration with respect to 

- GEM_e4 as autonomous vehicle with inertial frame (0,0), GEM_e2 as human driver.

- Alignment of all units as meter
        
- Alignment of different coordinate system of GNSS sensor yaw, Imaginary world simulation heading and real world steering wheel.  
        
- Alignment between ros update rate and ros communication delay.
        
After all, ackermn cmd control gas pedal and steering wheel (radians).

## Setup

### 1. Initialize and configure your catkin workspace:
    ```sh
    cd ~/Multi-GEM
    ```
### 2. Build the workspace:
    ```sh
    catkin_make
    ```
### 3. Source the setup file:
    ```sh
    source devel/setup.bash
    ```
## Usage
### 1. Launching Sensors and Visualization

Example launch commands:
```sh
roslaunch basic_launch sensor_init.launch
roslaunch basic_launch visualization.launch
roslaunch basic_launch dbw_joystick.launch
```

### 2. Quick Start

To run the GNSS tracker script:
```sh
python3 src/vehicle_drivers/gem_gnss_control/scripts/gem_gnss_tracker_pp.py
```

To run the ss control script:
```sh
python3 src/vehicle_drivers/gem_ss_control/mp2/src/main_highbay_steering_mpc.py
```

## Documentation
[Online documentation](https://gemstack.readthedocs.org) 
[About the GEM e2 vehicle](https://publish.illinois.edu/robotics-autonomy-resources/gem/)
- See [src/readme.txt](src/readme.txt) for more launch and usage examples.
- Refer to each package's README for specific instructions.

## Team

**Team Members:**
- Yan Bai - [yanb2@illinois.edu]
- Tianhao Ji - [email]
- Sridharan Subramanian - [ss233@illinois.edu]

**Advisors:**
- Ye-Ji Mun - [yejimun2@illinois.edu]
- Mahsa Golchoubian - [mahsa.golchoubian@gmail.com]
- Katie Driggs-Campbell - [krdc.illinois.edu]

## Acknowledgments

This work builds upon several open-source projects and research contributions:

- **GEMstack**: [https://github.com/krishauser/GEMstack](https://github.com/krishauser/GEMstack)
- **POLARIS_GEM_e2_Real**: [https://github.com/SafeRoboticsLab/KLGame](https://github.com/SafeRoboticsLab/KLGame)
- **GEM Simulator**: [https://github.com/yejimun/GEM_simulator](https://github.com/yejimun/GEM_simulator)

