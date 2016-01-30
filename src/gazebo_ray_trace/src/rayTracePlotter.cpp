#include "ros/ros.h"
#include "gazebo_ray_trace/RayTrace.h"
#include <visualization_msgs/MarkerArray.h>

visualization_msgs::Marker createMarker()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
 
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "ray";
  marker.id = 0;
 
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, anLINDER
  marker.type = visualization_msgs::Marker::ARROW;
 
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
 
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified he header
  // marker.pose.position.x = 0;
  // marker.pose.position.y = 0;
  // marker.pose.position.z = 0;
  // marker.pose.orientation.x = 0.0;
  // marker.pose.orientation.y = 0.0;
  // marker.pose.orientation.z = 0.0;
  // marker.pose.orientation.w = 1.0;
  ROS_INFO("About to set points");

  
  // geometry_msgs::Point p;
  // p.x = 0;
  // p.y = 0;
  // p.z = 0;
  // marker.points.push_back(p);
  // p.x = 1;
  // p.y = 0;
  // p.z = 1;
  // marker.points.push_back(p);

  marker.points.resize(2);
  marker.points[0].x = 0;
  marker.points[0].y = 0;
  marker.points[0].z = 0;
  marker.points[1].x = 1;
  marker.points[1].y = 1;
  marker.points[1].z = 2;
 
  ROS_INFO("Set points");

  marker.scale.x = 0.05;
  marker.scale.y = 0.1;
  // marker.scale.z = 1.0;
 
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
 
  marker.lifetime = ros::Duration();

  return marker;
}



int main(int argc, char **argv){
  ros::init(argc, argv, "ray_trace_test");
  // if (argc != 7){
  //   ROS_INFO("usage: currently, adds the norm of two vectors x y z x y z");
  //   return 1;
  // }


  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_ray_trace::RayTrace>("/gazebo_simulation/ray_trace");
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("ray_trace_markers", 10);

  visualization_msgs::Marker marker = createMarker();

  //wait until subscribed
  ros::Rate poll_rate(100);
  while(marker_pub.getNumSubscribers() == 0){
    poll_rate.sleep();
  }


  marker_pub.publish(marker);
  return 1;

  gazebo_ray_trace::RayTrace srv;
  srv.request.start.x = atof(argv[1]);
  srv.request.start.y = atof(argv[2]);
  srv.request.start.z = atof(argv[3]);

  srv.request.end.x = atof(argv[4]);
  srv.request.end.y = atof(argv[5]);
  srv.request.end.z = atof(argv[6]);





  if(client.call(srv)){
    ROS_INFO("Distance  %f", srv.response.dist);
  }else{
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
  












