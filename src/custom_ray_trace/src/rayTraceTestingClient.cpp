#include "ros/ros.h"
#include "rayTracer.h"


int main(int argc, char **argv){
  ros::init(argc, argv, "ray_trace_test");
  if (argc != 7){
    ROS_INFO("usage: currently, adds the norm of two vectors x y z x y z");
    return 1;
  }

  RayTracer rayTracer;


  tf::Point start = tf::Point(atof(argv[1]),
			      atof(argv[2]),
			      atof(argv[3]));

  tf::Point end = tf::Point(atof(argv[4]),
			    atof(argv[5]),
			    atof(argv[6]));
  double dist;
  Ray ray(start, end);
  std::vector<double> dists;

  rayTracer.traceRay(ray, dist);

  ROS_INFO("dist: %f", dist);

  // for(int i=0; i<1000; i++){
  //   rayTracer.traceAllParticles(ray, dists);
  // }

  // rayTracer.traceAllParticles(ray, dists);
  // ROS_INFO("numParticles: %d", dists.size());

  // for(int i=0; i<dists.size(); i++){
  //   ROS_INFO("Distance: %f", dists[i]);
  // }

  return 0;
}
  












