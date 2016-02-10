#ifndef PLOT_RAY_UTILS_H
#define PLOT_RAY_UTILS_H

#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>


class PlotRayUtils{
 private:
  ros::NodeHandle n_;
  ros::Publisher marker_pub_;
  ros::ServiceClient client_ray_trace_;
  ros::ServiceClient client_ray_trace_particles_;
  tf::TransformListener tf_listener_;

  int intersect_index_;
  int ray_index_;

 public:
  PlotRayUtils();
  void plotIntersections(tf::Point rayStart, tf::Point rayEnd, bool overwrite = true);
  void plotIntersections(std::vector<double> dist, tf::Point rayStart, tf::Point rayEnd,
			 bool overwrite = true);
  void plotIntersection(tf::Point intersection, int index);
			

  visualization_msgs::Marker createRayMarker(tf::Point start, tf::Point end, int index);
  
  void plotRay(tf::Point start, tf::Point end, bool overwrite = true);
  void labelRay(tf::Point start, std::string text);
  void plotEntropyRay(tf::Point start, tf::Point end, bool overwrite);

  double getDistToPart(tf::Point start, tf::Point end);
  std::vector<double> getDistToParticles(tf::Point start, tf::Point end);

  void listDistances(std::vector<double> dist){
    for(int i=0; i < dist.size(); i++){
      ROS_INFO("Dist is %f", dist[i]);
    }
  };

};


#endif
