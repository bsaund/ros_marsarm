#include "plotRayUtils.h"

#include "gazebo_ray_trace/RayTrace.h"
#include "gazebo_ray_trace/RayTraceEachParticle.h"
#include "calcEntropy.h"

PlotRayUtils::PlotRayUtils()
{
  marker_pub_ = 
    n_.advertise<visualization_msgs::Marker>("ray_trace_markers", 1000);
  client_ray_trace_ = 
    n_.serviceClient<gazebo_ray_trace::RayTrace>("/gazebo_simulation/ray_trace");
  client_ray_trace_particles_ = 
    n_.serviceClient<gazebo_ray_trace::RayTraceEachParticle>
    ("/gazebo_simulation/ray_trace_each_particle");

  intersect_index_ = 0;
  ray_index_ = 0;
}

/** 
 * Plots a point as a red dot
 */
void PlotRayUtils::plotIntersection(tf::Point intersection, int index){
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
  marker_pub_.publish(marker);
}

/**
 *  Plots intersections of a ray with all particles as red dots
 */
void PlotRayUtils::plotIntersections(std::vector<double> dist, 
				     tf::Point rayStart, tf::Point rayEnd,
				     bool overwrite)
{

  if(!overwrite){
    intersect_index_ += dist.size();
  }

  int point_id = intersect_index_;
  for(int i = 0; i < dist.size(); i++){
    plotIntersection(rayStart + dist[i] * (rayEnd-rayStart)/(rayEnd-rayStart).length(), 
		     point_id);
    point_id++;
  }
}

void PlotRayUtils::plotIntersections(tf::Point rayStart, tf::Point rayEnd, bool overwrite)
{
  plotIntersections(getDistToParticles(rayStart, rayEnd), rayStart, rayEnd, overwrite);
}



/**
 * Creates and returns the ray marker
 */
visualization_msgs::Marker PlotRayUtils::createRayMarker(tf::Point start, tf::Point end, 
							 int index)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
 
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "ray";
  marker.id = index;

  marker.type = visualization_msgs::Marker::ARROW;
 
  marker.action = visualization_msgs::Marker::ADD;
 
  ROS_INFO("About to set points");
  
  marker.points.resize(2);
  tf::pointTFToMsg(start, marker.points[0]);
  tf::pointTFToMsg(end, marker.points[1]);
 
  ROS_INFO("Set points");

  marker.scale.x = 0.005;
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


/*
 * publishes a message with the ray.
 *  By default overwrites the previous array
 */
void PlotRayUtils::plotRay(tf::Point start, tf::Point end, bool overwrite)
{
  if(!overwrite){
    ray_index_++;
  }
  visualization_msgs::Marker marker = createRayMarker(start, end, ray_index_);

  //wait until subscribed
  ros::Rate poll_rate(100);
  int i = 0;
  while(marker_pub_.getNumSubscribers() == 0 && i < 100){
    poll_rate.sleep();
    i++;
  }
  marker_pub_.publish(marker);
}

void PlotRayUtils::labelRay(tf::Point start, std::string text){
 visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
 
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "ray_label";
  marker.id = ray_index_;

  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
 
  marker.action = visualization_msgs::Marker::ADD;
 
  ROS_INFO("About to set label");
  
    
  tf::pointTFToMsg(start, marker.pose.position);
 
  ROS_INFO("Set points");


  marker.scale.z = 0.05;
 
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.text = text;
 
  marker.lifetime = ros::Duration();

  marker_pub_.publish(marker);
}



/**
 *  Plots ray, intersections, and entropy (text)
 */
void PlotRayUtils::plotEntropyRay(tf::Point start, tf::Point end, bool overwrite)
{
  plotRay(start, end, overwrite);
  plotIntersections(start, end, overwrite);
  std::stringstream s;
  s << CalcEntropy::calcEntropy(getDistToParticles(start,end));
  labelRay(start, s.str());
}


/**
 * Call the ros service providedby ray_trace_plugging
 * This servce accepts a ray and returns the transform to the true part
 * particles are not considered
 */
double PlotRayUtils::getDistToPart(tf::Point start, tf::Point end)
{
  // ros::ServiceClient client = n_.serviceClient<gazebo_ray_trace::RayTrace>("/gazebo_simulation/ray_trace");

  //Do Ray Trace
  tf::StampedTransform trans;

  tf_listener_.waitForTransform("/my_frame", "/particle_frame", ros::Time(0), ros::Duration(10.0));
  tf_listener_.lookupTransform("/particle_frame", "/my_frame", ros::Time(0), trans);


  gazebo_ray_trace::RayTrace srv;
  tf::pointTFToMsg(trans * start, srv.request.start);
  tf::pointTFToMsg(trans * end,   srv.request.end);
  
  ros::Time begin = ros::Time::now();

  if(client_ray_trace_.call(srv)){
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
std::vector<double> PlotRayUtils::getDistToParticles(tf::Point start, tf::Point end){

  // ros::ServiceClient client = n_.serviceClient<gazebo_ray_trace::RayTraceEachParticle>("/gazebo_simulation/ray_trace_each_particle");

  //Transform the ray into the particle frame to pass correct ray to gazebo for ray casting

  tf::StampedTransform trans;

  tf_listener_.waitForTransform("/my_frame", "/particle_frame", ros::Time(0), ros::Duration(10.0));
  tf_listener_.lookupTransform("/particle_frame", "/my_frame", ros::Time(0), trans);


  gazebo_ray_trace::RayTraceEachParticle srv;
  tf::pointTFToMsg(trans * start, srv.request.start);
  tf::pointTFToMsg(trans * end,   srv.request.end);
  
  ros::Time begin = ros::Time::now();

  if(!client_ray_trace_particles_.call(srv)){
    ROS_ERROR("Ray Trace Failed");
  }

  ROS_INFO("Time for ray trace: %f", (ros::Time::now() - begin).toSec());
  return srv.response.dist;
}
