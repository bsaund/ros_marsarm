#include <iostream>
#include <gazebo_ray_trace/calcEntropy.h>
#include <gazebo_ray_trace/plotRayUtils.h>
#include <vector>
#include <tf/tf.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ray_trace_test");
  PlotRayUtils plt;

  plt.plotEntropyRay(tf::Point(1.5, 2, 3.5),
		     tf::Point(1.5, 2, 2.5),
		     false);
  plt.plotEntropyRay(tf::Point(1.5, 3, 3.5),
		     tf::Point(1.5, 1, 2.5),
		     false);

  std::cout << "Hello World" << std::endl;
  return 0;
  
}
