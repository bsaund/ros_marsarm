/**
 *  Plots a ray and the intersections of that ray with obstacles 
 */

#include "ros/ros.h"
#include "gazebo_ray_trace/RayTrace.h"
#include "gazebo_ray_trace/RayTraceEachParticle.h"
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "calcEntropy.h"
#include "plotRayUtils.h"









int main(int argc, char **argv){
  if (argc != 7){
    ROS_INFO("usage: x y z x y z");
    return 1;
  }
  ros::init(argc, argv, "ray_trace_test");
  // ros::NodeHandle n;




  //Start and end vectors of the ray
  tf::Point start(atof(argv[1]),
		  atof(argv[2]),
		  atof(argv[3]));
		  
  tf::Point end(atof(argv[4]),
		atof(argv[5]),
		atof(argv[6]));

  PlotRayUtils plt;

  plt.plotRay(start, end);
  std::vector<double> dist = plt.getDistToParticles(start, end);

  plt.plotIntersections(dist, start, end);

  ROS_INFO("Entropy is %f", CalcEntropy::calcEntropy(dist));

  return 0;
}
  












