# ros_marsarm
Code for the mars arm particle filter and touch planning

## Dependencies - be sure to get the following:
ROS indigo

Eigen

[Catkin tools](https://catkin-tools.readthedocs.io/en/latest/) if you want to follow the syntax below

## Compilation and usage
Clone this project and run
```
catkin build
source devel/setup.bash
roslaunch touch_optimization setup_real_boeing.launch 
```
RViz should open showing the semi-transparent parts at poses specified by the particles

In a new terminal window
```
source devel/setup.bash
roslaunch touch_optimization simulate_localization.launch
```


## Connection to robot
If connected to the CMU marsarm robot run
```
roslaunch touch_optimization run_full_boeing.launch
```
Ros talks to the robot over the ipc_ros_bridge. For new robots a new form of communication will be needed.


## Quick Overview
* particle_filter: Our implementation of a particle filter specialized for accurate measurements
* touch_optimization: Chooses best touch point based on Information Gain
* using_markers: Visualization of particles and rays in RViz
* custom_ray_trace: Casts rays on particles and calculates information gain
* ipc_ros_bridge: Bridge to talk to IPC which controls our robot
