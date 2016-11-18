/**
 *  This is a rosnode that randomly chooses touch points, 
 *    performs simulated measurements, and updates the particle filter.
 *   This node is made to work with a particle filter node, a node that 
 *    publishes visualization messages, and RViz.
 */
#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include "particle_filter/AddObservation.h"

std::vector<ros::ServiceClient> pfilterSrvs;

bool distributeObservationData(particle_filter::AddObservation::Request &req,
			       particle_filter::AddObservation::Response &resp){
  ROS_INFO("Received Observation");
  particle_filter::AddObservation pfilter_obs;
  pfilter_obs.request = req;
  for(auto srv : pfilterSrvs){
    srv.call(pfilter_obs);
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "observation_distributor");
  ros::NodeHandle n;
  ros::ServiceServer obsHandle = 
    n.advertiseService("/observation_distributor", distributeObservationData);

  ros::ServiceClient top_add = 
    n.serviceClient<particle_filter::AddObservation>("/top_datum/particle_filter_add");

  ros::ServiceClient right_add = 
    n.serviceClient<particle_filter::AddObservation>("/right_datum/particle_filter_add");

  pfilterSrvs.push_back(top_add);
  pfilterSrvs.push_back(right_add);

  

  ros::spin();

  return 0;
}
