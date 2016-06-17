#include "plotRayUtils.h"
#include "calcEntropy.h"



PlotRayUtils::PlotRayUtils()
{
  marker_pub_ = 
    n_.advertise<visualization_msgs::Marker>("ray_trace_markers", 1000);
  marker_pub_array_ = 
    n_.advertise<visualization_msgs::MarkerArray>("ray_trace_markers_array", 10);

  intersect_index_ = 0;
  ray_index_ = 0;
}


/** 
 * Plots a point as a red dot
 */
void PlotRayUtils::plotIntersection(tf::Point intersection, int index){
  marker_pub_.publish(getIntersectionMarker(intersection, index));
}

/**
 *  Plots intersections of a ray with all particles as red dots
 */
void PlotRayUtils::plotIntersections(std::vector<double> dist, 
				     tf::Point rayStart, tf::Point rayEnd,
				     bool overwrite)
{
  visualization_msgs::MarkerArray m;
  if(!overwrite){
    intersect_index_ += dist.size();
  }

  int point_id = intersect_index_;
  for(int i = 0; i < dist.size(); i++){
    tf::Point intersection = rayStart + dist[i] * (rayEnd-rayStart).normalized();
    m.markers.push_back(getIntersectionMarker(intersection, point_id));
    point_id++;
  }
  marker_pub_array_.publish(m);
}

void PlotRayUtils::plotIntersections(tf::Point rayStart, tf::Point rayEnd, bool overwrite)
{
  // plotIntersections(getDistToParticles(rayStart, rayEnd), rayStart, rayEnd, overwrite);
}

visualization_msgs::Marker PlotRayUtils::getIntersectionMarker(tf::Point intersection, int index){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
 
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "ray_intersection";
  marker.id = index;
 
  marker.type = visualization_msgs::Marker::SPHERE;
 
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
  return marker;
}

/*
 * publishes a visualization message with the ray.
 *  By default overwrites the previous array
 */
void PlotRayUtils::plotRay(Ray ray, bool overwrite)
{
  if(!overwrite){
    ray_index_++;
  }

  visualization_msgs::Marker marker = createRayMarker(ray, ray_index_);

  //wait until subscribed
  ros::Rate poll_rate(100);
  int i = 0;
  while(marker_pub_.getNumSubscribers() == 0 && i < 100){
    poll_rate.sleep();
    i++;
  }
  marker_pub_.publish(marker);
}


void PlotRayUtils::labelRay(Ray ray, double d){
  std::stringstream s;
  //Note: IG should always be positive, but I am using fabs here to see errors if IG is negative
  s << d;
  labelRay(ray.start, s.str());
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
 
  tf::pointTFToMsg(start, marker.pose.position);
 
  marker.scale.z = 0.05;
 
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.7f;
  marker.color.a = 1.0;

  marker.text = text;
 
  marker.lifetime = ros::Duration();

  marker_pub_.publish(marker);
}

/**
 * Creates and returns the ray marker
 */
visualization_msgs::Marker PlotRayUtils::createRayMarker(Ray ray, int index)
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
 
  
  marker.points.resize(2);
  tf::pointTFToMsg(ray.start, marker.points[0]);
  tf::pointTFToMsg(ray.end, marker.points[1]);
 
  marker.scale.x = 0.005;
  marker.scale.y = 0.04; //Head Width
  // marker.scale.z = 1.0;
 
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.6;
 
  marker.lifetime = ros::Duration();

  return marker;
}


/**
 * Plots a cylinder of rays and labels the ray with the information gain
 *  radial_err determines the radius of the cylinder
 *  dist_err determines the bin size for calculating the entropy for calculating Information Gain
 */


void PlotRayUtils::plotCylinder(tf::Point start, tf::Point end, double radial_err, double dist_err, bool overwrite)
{

  // if(overwrite){
  //   ray_index_ = intersect_index_ = 0;
  // }
    
  // gazebo_ray_trace::RayTraceCylinder srv = getIGFullResponse(start, end, radial_err, dist_err);

  // tf::Point start_tmp;
  // tf::Point end_tmp;

  // //Plot all rays used, transforming to world coordinates
  // for(int i=0; i<srv.response.rays.size(); i++){
  //   transformRayToBaseFrame(srv.response.rays[i].start,
  // 			    srv.response.rays[i].end,
  // 			    start_tmp, end_tmp);
		     
  //   plotRay(start_tmp, end_tmp, overwrite);
  //   plotIntersections(srv.response.rays[i].dist, start_tmp, end_tmp, overwrite);
  //   // ros::Duration(0.02).sleep();
  // }
  // //Plot and label center ray
  // plotRay(start, end, false);
  // std::stringstream s;
  // //Note: IG should always be positive, but I am using fabs here to see errors if IG is negative
  // s << (fabs(srv.response.IG) < .0001 ? 0 : srv.response.IG); 
  // labelRay(start, s.str());
}


/**
 * Returns the point of intersection with the part along the ray
 *   rays are given in the world frame, 
 *   intersection point is returned in the world frame
 */
bool PlotRayUtils::getIntersectionWithPart(tf::Point start, tf::Point end, tf::Point &intersection)
{
  // double dist = getDistToPart(start, end);
  // intersection= start + (end-start).normalize() * dist;
  // return dist < 999;
}


