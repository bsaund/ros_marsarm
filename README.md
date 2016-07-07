# ros_marsarm
Code for the mars arm particle filter and touch planning

## Dependencies - be sure to get the following:
ROS indigo
Eigen

## Compilation and usage
From the directory with these files run 
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
Ros talks to the robot over the ipc_ros_bridge. For new robots a new form of communication will be needed
