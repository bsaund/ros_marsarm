/**
 *  Plots a ray and the intersections of that ray with obstacles 
 */

#include "ros/ros.h"

#include "calcEntropy.h"
#include "plotRayUtils.h"
#include <sstream>



int main(int argc, char **argv){

  ros::init(argc, argv, "ray_trace_grid");

  PlotRayUtils plt;
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      double x = 1 + 0.2 * (i-1);
      double y = 2 + 0.2 * (j-1);
      tf::Point start(x, y, 4);
      tf::Point end(x, y, 2);



      // plt.plotRay(start, end, false);

 
      // std::vector<double> dist = plt.getDistToParticles(start, end);

      // plt.plotIntersections(start, end, false);
      // double entropy = CalcEntropy::calcDifferentialEntropy(dist);


      // std::stringstream s;
      // std::string entropy_string;
      // // std::sprintf(entropy_string, "%f", entropy);
      // s  << std::fixed << std::setprecision(2) << entropy;
      // plt.labelRay(start, s.str());
      plt.plotCylinder(start, end, 0.01, 0.002);
      ros::Duration(0.2).sleep();
    } 
  }


  return 0;
}
