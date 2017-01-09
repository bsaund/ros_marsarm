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

  ros::ServiceClient srv_add = 
    n.serviceClient<particle_filter::AddObservation>("/observation_distributor");

  std::vector<std::string> pieces = {"top_datum", "right_datum", "J1_section",
				    "bottom_section", "back_datum"};
  std::vector<RayTracer*> rayts;
  for(std::string &piece:pieces){
    RayTracer* rayt = new RayTracer(piece);
    rayts.push_back(rayt);
  }


  int i = 1;
  while(true){
    std::stringstream ss;
    ss << "measurement" << i;
    if(!n.getParam(ss.str(), measurement)){
      if(i < 2){
	ROS_ERROR("Failed to get any measurement params");
	return -1;
      }
      return 0;
    }

    tf::Point start(measurement[0], measurement[1], measurement[2]);
    tf::Point end(measurement[3], measurement[4], measurement[5]);
    Ray measurementRay(start,end);

    plt.deleteAll();
    plt.plotRay(measurementRay); 

    ROS_INFO("Running...");

    int hitPart = simOnAllParts(measurementRay, rayts, srv_add, 0.001);

    if(hitPart < 0) {
      ROS_INFO("NO INTERSECTION, Skipping");
      ros::Duration(2).sleep();
      return -1;
    }
    i++;
  }
  return 0;
}
