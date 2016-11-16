/**
 *  This is a rosnode that randomly chooses touch points, 
 *    performs simulated measurements, and updates the particle filter.
 *   This node is made to work with a particle filter node, a node that 
 *    publishes visualization messages, and RViz.
 */
#include <ros/ros.h>
#include <ros/console.h>
#include "particle_filter/PFilterInit.h"
#include "particle_filter/AddObservation.h"
#include "custom_ray_trace/rayTracePlotter.h"
#include "custom_ray_trace/rayTracer.h"
#include "simulateMeasurement.h"

#include <Eigen/Dense>

#define NUM_TOUCHES 20

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulating_measurement");
  ros::NodeHandle n;
  RayTracePlotter plt;

  std::random_device rd;
  tf::Point start(.5,-.02,.3);
  tf::Point end(.5,-.02,.1);
  Ray measurementRay(start,end);

  ROS_INFO("Running...");

  ros::ServiceClient srv_add = 
    n.serviceClient<particle_filter::AddObservation>("particle_filter_add");
 
  ros::Duration(1).sleep();

  plt.plotRay(measurementRay); 
  int hitPart = simulateMeasurement(measurementRay, plt, srv_add, 0.001);
  ros::Duration(1).sleep();
  if(hitPart < 0) {
    ROS_INFO("NO INTERSECTION, Skipping");
    return -1;
  }

  return 0;
}
