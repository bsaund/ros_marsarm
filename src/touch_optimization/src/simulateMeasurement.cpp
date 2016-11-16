#include "simulateMeasurement.h"
#include <ros/console.h>
#include "particle_filter/AddObservation.h"

int simulateMeasurement(Ray measurementAction, RayTracer &rayt,
			 ros::ServiceClient pfilterAdd, double noiseStdDev) 
{
  std::random_device rd;
  std::normal_distribution<double> randn(0.0, noiseStdDev);

  tf::Point start = measurementAction.start;
  tf::Point end = measurementAction.end;
  tf::Point intersection;

  geometry_msgs::Point obs;
  geometry_msgs::Point dir;


  double distToPart;

  if(!rayt.traceRay(measurementAction, distToPart)){
    return -1;
  }

  intersection = start + (end-start).normalize() * (distToPart);
  std::cout << "Intersection at: " << intersection.getX() << "  " << intersection.getY() << "   " << intersection.getZ() << std::endl;
  tf::Point ray_dir(end.x()-start.x(),end.y()-start.y(),end.z()-start.z());
  ray_dir = ray_dir.normalize();

  obs.x=intersection.getX() + randn(rd); 
  obs.y=intersection.getY() + randn(rd); 
  obs.z=intersection.getZ() + randn(rd);
  dir.x=ray_dir.x();
  dir.y=ray_dir.y();
  dir.z=ray_dir.z();

  particle_filter::AddObservation pfilter_obs;
  pfilter_obs.request.p = obs;
  pfilter_obs.request.dir = dir;
  if(!pfilterAdd.call(pfilter_obs)){
    ROS_INFO("Failed to call add observation");
  }
  return 1;
}
