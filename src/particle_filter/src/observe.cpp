#include "ros/ros.h"
#include "particle_filter/AddObservation.h"
#include "custom_ray_trace/rayTracePlotter.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "observe");
  if (argc < 7 || argc > 9){
    ROS_INFO("usage: observe: (start) x y z (dir) x y z [name]");
    return 1;
  }


}
