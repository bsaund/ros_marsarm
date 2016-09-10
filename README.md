# ros_marsarm
Code for the mars arm particle filter and touch planning

## Dependencies - be sure to get the following:
ROS indigo

Eigen 
```sudo apt-get install eigen3```

*Note - you may have to create a symlink to eigen in eigen3
```cd /usr/include && sudo ln -sf eigen3/Eigen Eigen``

PCL
```sudo apt-get install libpcl1```

## Compilation and usage
Clone this project and run
```
catkin make
source devel/setup.bash
roslaunch touch_optimization localize_real_wood.launch
```
RViz should open showing the particles

In a new terminal window
```
source devel/setup.bash
rosrun custom_ray_trace custom_rayTrace_test 30 .9 0 0 0 0
```
A line of rays should appear, showing the expected information gain.


## Connection to robot
If connected to the CMU marsarm robot run
```
rosrun touch_optimization run_full_wood
```
Ros talks to the robot over the ipc_ros_bridge. For new robots a new form of communication will be needed.


## Quick Overview
* particle_filter: Our implementation of a particle filter specialized for accurate measurements
* touch_optimization: Chooses best touch point based on Information Gain
* using_markers: Visualization of particles and rays in RViz
* custom_ray_trace: Casts rays on particles and calculates information gain
* ipc_ros_bridge: Bridge to talk to IPC which controls our robot
