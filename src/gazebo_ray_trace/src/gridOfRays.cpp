/**
 *  Plots a grid of rays and the intersections of that ray with obstacles 
 */

#include "ros/ros.h"

#include "calcEntropy.h"
#include "plotRayUtils.h"
#include <sstream>
#include <math.h>
#include <iostream>
#include <fstream>



int main(int argc, char **argv){

  ros::init(argc, argv, "ray_trace_grid");

  std::ofstream myfile;
  myfile.open("example.txt");



  PlotRayUtils plt;
  double y, z;
  y = -.9;
  while( y <= .9){
    z = 0.0;
    while( z <= .6){
      tf::Point start(0, y, z);
      tf::Point end(2, y, z);

      // plt.plotCylinder(start, end, 0.01, 0.002);
      // ros::Duration(0.4).sleep();

      myfile << y << ", " << z << ", " << plt.getIG(start, end, 0.01, 0.002) << std::endl;
      z += .01;
    }
    std::cout << "x, z: " << y << ", " << z << std::endl;
    y += .01;
  }

  // double x, z;
  // x = -.3;
  // while( x <= 1.3){
  //   z = 0.0;
  //   while( z <= .6){
  //     tf::Point start(x, -1, z);
  //     tf::Point end(x, 1, z);

  //     // plt.plotCylinder(start, end, 0.01, 0.002);
  //     // ros::Duration(0.4).sleep();

  //     myfile << x << ", " << z << ", " << plt.getIG(start, end, 0.01, 0.002) << std::endl;
  //     z += .01;
  //   }
  //   std::cout << "x, z: " << x << ", " << z << std::endl;
  //   x += .01;
  // }

  // double x, y;
  // x = -.3;
  // while( x <= 1.3){
  //   y = -.9;
  //   while( y <= .9){
  //     tf::Point start(x, y, 1.0);
  //     tf::Point end(x, y, 0.0);

  //     // plt.plotCylinder(start, end, 0.01, 0.002);
  //     // ros::Duration(0.4).sleep();

  //     myfile << x << ", " << y << ", " << plt.getIG(start, end, 0.01, 0.002) << std::endl;
  //     y += .01;
  //   }
  //   std::cout << "x, y: " << x << ", " << y << std::endl;
  //   x += .01;
  // }

  myfile.close();
  // for(int i = -2; i < 3; i++){
  //   for(int j = -2; j < 3; j++){
  //     double x = .5 + 0.2 * (i);
  //     double y = 0.2 * (j);
  //     tf::Point start(x, y, 1.0);
  //     tf::Point end(x, y, 0.0);

  //     plt.plotCylinder(start, end, 0.01, 0.002);
  //     ros::Duration(0.1).sleep();
  //   } 
  // }
  // double n = 10;
  // for(int i = -2; i <= n+2; i++){
  //   double theta = 3.1415*i/n/2;
  //   double x1 = .9 - cos(theta);
  //   double x2 = .9 + cos(theta);
  //   double y1 = .75 - sin(theta);
  //   double y2 = .75 + sin(theta);
  //   tf::Point start(x1, y1, 1.1);
  //   tf::Point end(x2, y2, 1.1);

  //   plt.plotCylinder(start, end, 0.01, 0.002);
  //   ros::Duration(0.1).sleep();

  // }


  return 0;
}
