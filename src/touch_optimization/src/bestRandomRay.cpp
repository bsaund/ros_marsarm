/**
 *  Plots a ray and the intersections of that ray with obstacles 
 */

#include "ros/ros.h"

#include "gazebo_ray_trace/calcEntropy.h"
#include "gazebo_ray_trace/plotRayUtils.h"
#include <sstream>



int main(int argc, char **argv){
  ros::init(argc, argv, "best_random_ray");

  tf::Point best_start, best_end;
  double bestIG;
  bestIG = 0;

  PlotRayUtils plt;
  for(int i = -20; i <= 20; i++){
    for(int j = -20; j <= 20; j++){
      double x = 1 + 0.02 * (i);
      double y = 2 + 0.02 * (j);
      tf::Point start(x, y, 3.3);
      tf::Point end(x, y, 2.0);

      // plt.plotCylinder(start, end, 0.01, 0.002);
      double IG = plt.getIG(start, end, 0.01, 0.002);
      if (IG > bestIG){
	bestIG = IG;
	best_start = start;
	best_end = end;
      }

      // ros::Duration(0.1).sleep();
    } 
  }

  plt.plotCylinder(best_start, best_end, 0.01, 0.002);
  
  

  return 0;
}
