#include "ros/ros.h"
#include "gazebo_ray_trace/RayTrace.h"
#include <math.h>

bool rayTrace(gazebo_ray_trace::RayTrace::Request &req,
	      gazebo_ray_trace::RayTrace::Response &resp){
  resp.dist = sqrt(
		   pow(req.start.x - req.end.x, 2) + 
		   pow(req.start.y - req.end.y, 2) + 
		   pow(req.start.z - req.end.z, 2));

  ROS_INFO("Sending Response: %ld", (long int)resp.dist);
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "ray_trace_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("ray_trace", rayTrace);
  ROS_INFO("Ready to ray trace");
  ros::spin();

  return 0;
}
	      

