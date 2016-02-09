#ifndef PLOT_RAY_UTILS_H
#define PLOT_RAY_UTILS_H

#include "ros/ros.h"
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>


class PlotRayUtils{
 private:
  ros::NodeHandle n_;
  ros::Publisher marker_pub_;

 public:
  PlotRayUtils();
  void plotIntersections(std::vector<double> dist, tf::Point rayStart, tf::Point rayEnd);
  void plotIntersection(tf::Point intersection, int index);
			

  visualization_msgs::Marker createRayMarker(tf::Point start, tf::Point end);
  
  void plotRay(tf::Point start, tf::Point end);

  double getDistToPart(tf::Point start, tf::Point end);
  std::vector<double> getDistToParticles(tf::Point start, tf::Point end);

};


#endif
