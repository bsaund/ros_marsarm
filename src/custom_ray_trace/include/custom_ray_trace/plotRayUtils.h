#ifndef PLOT_RAY_UTILS_H
#define PLOT_RAY_UTILS_H

#include "ros/ros.h"
#include <tf/tf.h>

#include <visualization_msgs/MarkerArray.h>

#include "rayTracer.h"

class PlotRayUtils{
 private:
  ros::NodeHandle n_;
  ros::Publisher marker_pub_;
  ros::Publisher marker_pub_array_;


  int intersect_index_;
  int ray_index_;



 public:
  PlotRayUtils();

  /* RayTracer rayTracer; */
  

  void plotIntersections(tf::Point rayStart, tf::Point rayEnd, bool overwrite = true);
  void plotIntersections(std::vector<double> dist, tf::Point rayStart, tf::Point rayEnd,
			 bool overwrite = true);
  void plotIntersection(tf::Point intersection, int index);
			

  visualization_msgs::Marker createRayMarker(tf::Point start, tf::Point end, int index);
  
  void plotRay(tf::Point start, tf::Point end, bool overwrite = true);
  void labelRay(tf::Point start, std::string text);
  void plotEntropyRay(tf::Point start, tf::Point end, bool overwrite);
  void plotCylinder(tf::Point start, tf::Point end, double radial_err, double dist_err, bool overwrite = false);

  double getDistToPart(tf::Point start, tf::Point end);
  bool getIntersectionWithPart(tf::Point start, tf::Point end, tf::Point &intersection);
  std::vector<double> getDistToParticles(tf::Point start, tf::Point end);
  void getIGFullResponse(
		 tf::Point start, tf::Point end, double radial_err, double dist_err);
  double getIG(tf::Point start, tf::Point end, double radial_err, double dist_err);

  void listDistances(std::vector<double> dist){
    for(int i=0; i < dist.size(); i++){
      ROS_INFO("Dist is %f", dist[i]);
    }
  };



 private:
  visualization_msgs::Marker getIntersectionMarker(tf::Point intersection, int index);

};


#endif
