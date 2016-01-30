/**
 *  Plots a ray and the intersections of that ray with obstacles 
 */

#include "ros/ros.h"
#include "gazebo_ray_trace/RayTrace.h"
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

visualization_msgs::Marker createMarker(tf::Point start, tf::Point end)
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
 
  ROS_INFO("About to set points");

  
  marker.points.resize(2);
  tf::pointTFToMsg(start, marker.points[0]);
  tf::pointTFToMsg(end, marker.points[1]);
 
  ROS_INFO("Set points");

  marker.scale.x = 0.001;
  marker.scale.y = 0.1;
  // marker.scale.z = 1.0;
 
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.6;
 
  marker.lifetime = ros::Duration();

  return marker;
}


void plotRay(tf::Point start, tf::Point end, ros::Publisher marker_pub){


  visualization_msgs::Marker marker = createMarker(start, end);

  //wait until subscribed
  ros::Rate poll_rate(100);
  int i = 0;
  while(marker_pub.getNumSubscribers() == 0 && i < 100){
    poll_rate.sleep();
    i++;
  }
  marker_pub.publish(marker);
}

double getDistToPart(tf::Point start, tf::Point end, ros::NodeHandle n){
  ros::ServiceClient client = n.serviceClient<gazebo_ray_trace::RayTrace>("/gazebo_simulation/ray_trace");

  //Do Ray Trace
  tf::TransformListener tf_listener;
  tf::StampedTransform trans;

  tf_listener.waitForTransform("/my_frame", "/particle_frame", ros::Time(0), ros::Duration(10.0));
  tf_listener.lookupTransform("/particle_frame", "/my_frame", ros::Time(0), trans);


  gazebo_ray_trace::RayTrace srv;
  tf::pointTFToMsg(trans * start, srv.request.start);
  tf::pointTFToMsg(trans * end,   srv.request.end);


  if(client.call(srv)){
    ROS_INFO("Distance  %f", srv.response.dist);
  }else{
    ROS_ERROR("Ray Trace Failed");
  }
  return srv.response.dist;
}


void plotIntersections(tf::Point intersection, ros::Publisher marker_pub){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
 
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "ray_intersection";
  marker.id = 1;
 
  marker.type = visualization_msgs::Marker::SPHERE;
 
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
 
  tf::pointTFToMsg(intersection, marker.pose.position);

  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
 
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
 
  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
}





int main(int argc, char **argv){
  if (argc != 7){
    ROS_INFO("usage: x y z x y z");
    return 1;
  }
  ros::init(argc, argv, "ray_trace_test");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("ray_trace_markers", 10);
  ros::Subscriber particle_sub = n.subscribe("transform_particles");

  //Start and end vectors of the ray
  tf::Point start(atof(argv[1]),
		  atof(argv[2]),
		  atof(argv[3]));
		  
  tf::Point end(atof(argv[4]),
		atof(argv[5]),
		atof(argv[6]));


  plotRay(start, end, marker_pub);

  double dist = getDistToPart(start, end, n);
 

  plotIntersections(start + dist * (end-start)/(end-start).length(), marker_pub);

  return 0;
}
  












