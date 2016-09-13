#ifndef RAY_TRACE_PLOTTER_H
#define RAY_TRACE_PLOTTER_H

#include "rayTracer.h"
#include <visualization_msgs/MarkerArray.h>

class RayTracePlotter: public RayTracer
{
 private:
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Publisher marker_pub_array;

  visualization_msgs::Marker createRayMarker(Ray ray, int index);

 public:
  RayTracePlotter();
  void plotRay(Ray ray, int index = 0);
  void plotCylinder(Ray ray, double radius);
  void plotIG(Ray ray);
  void plotRayWithIntersections(Ray ray);
  void plotCylinderWithIntersections(Ray ray, double radius);


};



#endif



