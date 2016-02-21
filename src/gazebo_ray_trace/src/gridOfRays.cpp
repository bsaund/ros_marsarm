/**
 *  Plots a grid of rays and the intersections of that ray with obstacles 
 */

#include "ros/ros.h"

#include "calcEntropy.h"
#include "plotRayUtils.h"
#include <sstream>
#include <math.h>


int main(int argc, char **argv){

  ros::init(argc, argv, "ray_trace_grid");

  PlotRayUtils plt;
  // for(int i = -2; i < 3; i++){
  //   for(int j = -2; j < 3; j++){
  //     double x = 1 + 0.2 * (i);
  //     double y = 2 + 0.2 * (j);
  //     tf::Point start(x, y, 3.3);
  //     tf::Point end(x, y, 2.0);

  //     plt.plotCylinder(start, end, 0.01, 0.002);
  //     ros::Duration(0.1).sleep();
  //   } 
  // }
  double n = 5;
  for(int i = 0; i <= n; i++){
    double theta = 3.1415*i/n/2;
    double x1 = .9 - cos(theta);
    double x2 = .9 + cos(theta);
    double y1 = .75 - sin(theta);
    double y2 = .75 + sin(theta);
    tf::Point start(x1, y1, 1.1);
    tf::Point end(x2, y2, 1.1);

    plt.plotCylinder(start, end, 0.01, 0.002);
    ros::Duration(0.1).sleep();

  }


  return 0;
}
