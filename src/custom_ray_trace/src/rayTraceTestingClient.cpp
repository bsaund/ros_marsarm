#include "ros/ros.h"
#include "rayTracer.h"
#include "plotRayUtils.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "ray_trace_test");
  if (argc != 7){
    ROS_INFO("usage: currently, adds the norm of two vectors x y z x y z");
    return 1;
  }

  RayTracer rayTracer;
  PlotRayUtils plt;


  tf::Point start = tf::Point(atof(argv[1]),
			      atof(argv[2]),
			      atof(argv[3]));

  tf::Point end = tf::Point(atof(argv[4]),
			    atof(argv[5]),
			    atof(argv[6]));
  double dist;
  Ray ray(start, end);
  std::vector<double> distsQuick;
  std::vector<double> dists;

  // rayTracer.traceRay(ray, dist);

  // ROS_INFO("dist: %f", dist);
  // ros::Time s = ros::Time::now();

  // for(int i=0; i<1000; i++){
  //   rayTracer.traceAllParticles(ray, distsQuick, true);
  // }
  // ROS_INFO("Quick time: %f", (ros::Time::now() - s).toSec());
  // s = ros::Time::now();
  // for(int i=0; i<1000; i++){
  //   rayTracer.traceAllParticles(ray, dists, false);
  // }

  // ROS_INFO("Slow time: %f", (ros::Time::now() - s).toSec());


  // rayTracer.traceAllParticles(ray, dists, false);

  // if(dists == distsQuick)
  //   ROS_INFO("Distances equal");
  // else
  //   ROS_INFO("Distances Not Equal");
  // rayTracer.traceAllParticles(ray, dists);
  // ROS_INFO("numParticles: %d", dists.size());

  // for(int i=0; i<dists.size(); i++){
  //   ROS_INFO("Distance: %f", dists[i]);
  // }

  // ROS_INFO("IG: %f", rayTracer.getIG(ray, 0.01, 0.01));
  Ray ray_base(tf::Point(.9, 0, 1), tf::Point(.9, 0, 0));
  plt.plotRay(ray_base, false);
  plt.labelRay(ray_base, rayTracer.getIG(ray_base, 0.01, 0.01));
  plt.plotRay(ray, false);
  plt.labelRay(ray, rayTracer.getIG(ray, 0.01, 0.01));

  ROS_INFO("IG of base ray: %f", rayTracer.getIG(ray_base, 0.01, 0.01));
  ROS_INFO("IG of input ray: %f", rayTracer.getIG(ray, 0.01, 0.01));
  ROS_INFO("IG of both: %f", rayTracer.getIG(ray_base, ray, 0.01, 0.01));
  


  return 0;
}
  












