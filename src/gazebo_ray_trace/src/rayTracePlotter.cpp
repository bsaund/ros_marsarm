/**
 *  Plots a ray and the intersections of that ray with obstacles 
 */

#include "ros/ros.h"
#include "gazebo_ray_trace/RayTrace.h"
#include "gazebo_ray_trace/RayTraceEachParticle.h"
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "calcEntropy.h"

/**
 * Creates and returns the ray marker
 */
visualization_msgs::Marker createMarker(tf::Point start, tf::Point end)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
 
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "ray";
  marker.id = 0;
 

  marker.type = visualization_msgs::Marker::ARROW;
 

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
  
  ros::Time begin = ros::Time::now();

  if(client.call(srv)){
    ROS_INFO("Distance  %f", srv.response.dist);
  }else{
    ROS_ERROR("Ray Trace Failed");
  }
  ROS_INFO("Time for ray trace: %f", (ros::Time::now() - begin).toSec());
  return srv.response.dist;
}

/**
 * Calls the ros service provided by ray_trace_pluggin.
 *  This service accepts a ray and returns a list of points for where the ray 
 *  intersected each obstacle
 */
std::vector<double> getDistToParticles(tf::Point start, tf::Point end, ros::NodeHandle n){

  ros::ServiceClient client = n.serviceClient<gazebo_ray_trace::RayTraceEachParticle>("/gazebo_simulation/ray_trace_each_particle");


  //Transform the ray into the particle frame to pass correct ray to gazebo for ray casting
  tf::TransformListener tf_listener;
  tf::StampedTransform trans;

  tf_listener.waitForTransform("/my_frame", "/particle_frame", ros::Time(0), ros::Duration(10.0));
  tf_listener.lookupTransform("/particle_frame", "/my_frame", ros::Time(0), trans);


  gazebo_ray_trace::RayTraceEachParticle srv;
  tf::pointTFToMsg(trans * start, srv.request.start);
  tf::pointTFToMsg(trans * end,   srv.request.end);
  
  ros::Time begin = ros::Time::now();

  if(client.call(srv)){
    // ROS_INFO("Distance  %f", srv.response.dist[0]);
  }else{
    ROS_ERROR("Ray Trace Failed");
  }
  ROS_INFO("Time for ray trace: %f", (ros::Time::now() - begin).toSec());
  return srv.response.dist;
}

/**
 *  Plots intersections of a ray with all particles as red dots
 */
void plotIntersections(tf::Point intersection, ros::Publisher marker_pub, int index){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
 
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "ray_intersection";
  marker.id = index;
 
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


  //Start and end vectors of the ray
  tf::Point start(atof(argv[1]),
		  atof(argv[2]),
		  atof(argv[3]));
		  
  tf::Point end(atof(argv[4]),
		atof(argv[5]),
		atof(argv[6]));


  plotRay(start, end, marker_pub);

  
  // double dist = getDistToPart(start, end, n);
  // plotIntersections(start + dist * (end-start)/(end-start).length(), marker_pub, 0);

  std::vector<double> dist = getDistToParticles(start, end, n);
  for(int i = 1; i < dist.size(); i++){
    ROS_INFO("Dist is %f", dist[i]);
    plotIntersections(start + dist[i] * (end-start)/(end-start).length(), marker_pub, i);
  }

  ROS_INFO("Entropy is %f", CalcEntropy::calcEntropy(dist));

  return 0;
}
  












