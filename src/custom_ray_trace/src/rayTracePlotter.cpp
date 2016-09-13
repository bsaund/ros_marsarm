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

  visualization_msgs::Marker marker = createRayMarker(ray, index);
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
