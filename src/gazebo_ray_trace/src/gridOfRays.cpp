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
#include <string.h>



int main(int argc, char **argv){

  ros::init(argc, argv, "ray_trace_grid");

  std::ofstream myfile;


  PlotRayUtils plt;


  tf::Transform t, R;
  t.setOrigin(tf::Vector3(.9,0,.3));
  t.setRotation(tf::createQuaternionFromRPY(0,0,0));




  double x, y, angle;
  angle = 0;
  while(angle <= 1.57){
    std::cout << "Angle: " << angle << std::endl;

    std::ostringstream ss;
    ss << "XY_rotation_" << angle << ".txt"; 
    myfile.open(ss.str());
    R.setRotation(tf::createQuaternionFromRPY(0,-angle,0));
    x = 0.3;
    while( x <= 1.4){
      y = -.9;
      while( y <= .9){
	tf::Point start(x, y, 1.0);
	tf::Point end(x, y, 0.0);

	start = t * R * t.inverse() * start;
	end   = t * R * t.inverse() * end;

	// plt.plotCylinder(start, end, 0.01, 0.002);
	// plt.plotRay(start, end, false);
	// ros::Duration(0.1).sleep();

	myfile << x << ", " << y << ", " << plt.getIG(start, end, 0.01, 0.002) << std::endl;
	y += .001;
      }
      std::cout << "x, y: " << x << ", " << y << std::endl;
      x += .001;
    }
    
    // angle += .1;
    angle += 2;
    myfile.close();
  }


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
