#include "ros/ros.h"
#include "gazebo_ray_trace/RayTrace.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "ray_trace_test");
  if (argc != 7){
    ROS_INFO("usage: currently, adds the norm of two vectors x y z x y z");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_ray_trace::RayTrace>("ray_trace");


  gazebo_ray_trace::RayTrace srv;
  srv.request.start.x = atoll(argv[1]);
  srv.request.start.y = atoll(argv[2]);
  srv.request.start.z = atoll(argv[3]);

  srv.request.end.x = atoll(argv[4]);
  srv.request.end.y = atoll(argv[5]);
  srv.request.end.z = atoll(argv[6]);

  if(client.call(srv)){
    ROS_INFO("Distance  %ld", (long int)srv.response.dist);
  }else{
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
  












