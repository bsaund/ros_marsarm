#include "rayTracePlotter.h"

  // void plotCylinder(Ray ray, double radius);
  // void plotIG(Ray ray);
  // void plotRayWithIntersections(Ray ray);
  // void plotCylinderWithIntersections(Ray ray, double radius);


RayTracePlotter::RayTracePlotter(){
  marker_pub = 
    n.advertise<visualization_msgs::Marker>("ray_trace_markers", 1000);
  marker_pub_array = 
    n.advertise<visualization_msgs::MarkerArray>("ray_trace_markers_array", 10);

}



void RayTracePlotter::plotRay(Ray ray, int index){
  //wait until subscribed
  ros::Rate poll_rate(100);
  int i = 0;
  while(marker_pub.getNumSubscribers() == 0 && i < 100){
    poll_rate.sleep();
    i++;
  }

  visualization_msgs::Marker marker = createRayMarker(ray, index);
  marker_pub.publish(marker);
}

int RayTracePlotter::plotRays(std::vector<Ray> rays, int id){
 for(Ray cylinderRay:rays){
    plotRay(cylinderRay, id++);
  }
  return id; 
}

int RayTracePlotter::plotCylinder(Ray ray, double radius, int id){
  std::vector<Ray> rays;
  getCylinderRays(ray, radius, rays);
  return plotRays(rays);
}


int RayTracePlotter::plotIntersections(Ray ray, int id){
  std::vector<double> dist;
  traceAllParticles(ray, dist);
  return plotIntersections(dist, ray, id);
}

/**
 *  Plots intersections of a ray with all particles as red dots
 */
int RayTracePlotter::plotIntersections(const std::vector<double> &dist, 
					Ray ray, int id) {
  std::vector<tf::Point> p;
  for(int i = 0; i < dist.size(); i++){
    tf::Point intersection = ray.start + dist[i] * (ray.end-ray.start).normalized();
    p.push_back(intersection);
  }
  return plotIntersections(p, id);
}

int RayTracePlotter::plotIntersections(const std::vector<tf::Point> intersections,
					int id){
  visualization_msgs::MarkerArray m;
  for(int i = 0; i < intersections.size(); i++){
    m.markers.push_back(getIntersectionMarker(intersections[i], id));
    id++;
  }
  marker_pub_array.publish(m);
  return id;
}


void RayTracePlotter::deleteAll(){
  // visualization_msgs::Marker marker = getDeleteMarker("ray", 1);
  // marker_pub.publish(marker);

  visualization_msgs::MarkerArray m;
  for(int i = 0; i < 500; i++){
    m.markers.push_back(getDeleteMarker("ray",i));
  }
  marker_pub_array.publish(m);

}


// LABELING



void RayTracePlotter::labelRay(Ray ray, double d, int id){
  std::stringstream s;
  s << d;
  labelRay(ray.start, s.str(), id);
}

void RayTracePlotter::labelRay(tf::Point start, std::string text, int id){
  label(start, id, text);
}


void RayTracePlotter::label(tf::Point start, int id, std::string text){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
 
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "ray_label";
  marker.id = id;
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

  marker_pub.publish(marker);
}




visualization_msgs::Marker RayTracePlotter::createRayMarker(Ray ray, int index)
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


visualization_msgs::Marker RayTracePlotter::getIntersectionMarker(tf::Point intersection, 
								  int id){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();
 
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "ray_intersection";
  marker.id = id;
 
  marker.type = visualization_msgs::Marker::SPHERE;
 
  marker.action = visualization_msgs::Marker::ADD;
 
  tf::pointTFToMsg(intersection, marker.pose.position);

  marker.scale.x = 0.008;
  marker.scale.y = 0.008;
  marker.scale.z = 0.008;
 
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
 
  marker.lifetime = ros::Duration();
  return marker;
}

visualization_msgs::Marker RayTracePlotter::getDeleteMarker(string str, int id){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();

  marker.ns = str;
  marker.id = id;
  marker.type = visualization_msgs::Marker::DELETE;
  return marker;
}
