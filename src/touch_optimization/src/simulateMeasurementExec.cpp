/**
 *  This is a rosnode that randomly chooses touch points, 
 *    performs simulated measurements, and updates the particle filter.
 *   This node is made to work with a particle filter node, a node that 
 *    publishes visualization messages, and RViz.
 */
#include <ros/ros.h>
#include <ros/console.h>
#include "particle_filter/AddObservation.h"
#include "custom_ray_trace/rayTracePlotter.h"
#include "custom_ray_trace/rayTracer.h"
#include "simulateMeasurement.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulating_measurement");
  ros::NodeHandle n;
  RayTracePlotter plt;

  std::random_device rd;
  std::vector<double> measurement;



  if(!n.getParam("measurement", measurement)){
    ROS_INFO("Failed to get param: measurement");
    return -1;
  }

  tf::Point start(measurement[0], measurement[1], measurement[2]);
  tf::Point end(measurement[3], measurement[4], measurement[5]);
  Ray measurementRay(start,end);

  plt.deleteAll();
  plt.plotRay(measurementRay); 

  ros::ServiceClient srv_add = 
    n.serviceClient<particle_filter::AddObservation>("/observation_distributor");


  ROS_INFO("Running...");


  int hitPart = simulateMeasurement(measurementRay, plt, srv_add, 0.001);

  if(hitPart < 0) {
    ROS_INFO("NO INTERSECTION, Skipping");
    ros::Duration(2).sleep();
    return -1;
  }

  return 0;
}
