/**
 *  Plots a ray and the intersections of that ray with obstacles 
 */

#include "ros/ros.h"
#include <tf/tf.h>
#include "calcEntropy.h"
#include "plotRayUtils.h"
#include <sstream>
#include <math.h>


int main(int argc, char **argv){
  ros::init(argc, argv, "ray_trace_test");
  PlotRayUtils plt;



  //Start and end vectors of the ray
  // tf::Point start(1.5, 2, 3.5);
  // tf::Point end(1.5, 2, 2.5);

  // //This was a quick script for casting rays in a circle
  // for(int i = 0; i < 3; i++){
  //   double radius = .2 * i;
  //   for(int j = 0; j < 8; j++){
  //     double theta = (2*3.1415 * j)/8;
  //     double x = 1 + radius * cos(theta);
  //     double y = 2 + radius * sin(theta);
      
  //     plt.plotEntropyRay(tf::Point(x,y, 3.5),
  // 			 tf::Point(x,y, 2.5),
  // 			 false);
  //     ros::Duration(.2).sleep();
  //   }
  // }


  plt.plotEntropyRay(tf::Point(1.5, 2, 3.5),
		     tf::Point(1.5, 2, 2.5),
		     false);
  plt.plotEntropyRay(tf::Point(1.5, 3, 3.5),
		     tf::Point(1.5, 1, 2.5),
		     false);


  

  return 0;
}
