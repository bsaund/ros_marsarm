#ifndef PLOTSHAPE_H
#define PLOTSHAPE_H

#include "simpleShape.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>

class ShapePlotter
{
 private:
  ros::NodeHandle n;
  ros::Publisher marker_pub;

  tf::TransformBroadcaster br;
  tf::Transform transform;


 public:
  ShapePlotter();
  void plotShape(const Cube shape, geometry_msgs::TransformStamped trans);
  void plotShape(const Cube shape, tf::StampedTransform trans);

};
#endif
