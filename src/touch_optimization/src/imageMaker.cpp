/**
 *  Plots a grid of rays and the intersections of that ray with obstacles 
 */

#include "ros/ros.h"

// #include "calcEntropy.h"
// #include "plotRayUtils.h"
#include <custom_ray_trace/rayTracer.h>
#include <sstream>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include "particle_filter/AddObservation.h"

void makeImage(double resolution, double roll, double pitch, double yaw, int frame)
{
  std::ofstream myfile;
  // PlotRayUtils plt;
  RayTracer rayt;
  tf::Transform t, R;


  t.setOrigin(tf::Vector3(.9,0,.3)); //transform to center of part
  t.setRotation(tf::createQuaternionFromRPY(0,0,0));


  double x, y;

  std::cout << "Angle: " << pitch << std::endl;

  std::ostringstream ss;
  ss << "XY_rotation_" << roll << "_" << pitch << "_" << yaw << "_" << frame << ".txt"; 
  myfile.open(ss.str());
  R.setRotation(tf::createQuaternionFromRPY(roll,pitch,yaw));

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

      // myfile << x << ", " << y << ", " << plt.getIG(start, end, 0.01, 0.002) << std::endl;

      myfile << x << ", " << y << ", " << rayt.getIG(Ray(start, end), 0.01, 0.01) << std::endl;
      y += resolution;
    }
    std::cout << "x, y: " << x << ", " << y << std::endl;
    x += resolution;
  }
    
  myfile.close();
 
}


particle_filter::AddObservation observation(double x, double y, double z,
					    double dirx, double diry, double dirz)
{
  particle_filter::AddObservation pfilter_obs;
  pfilter_obs.request.p.x = x;
  pfilter_obs.request.p.y = y;
  pfilter_obs.request.p.z = z;

  pfilter_obs.request.dir.x = dirx;
  pfilter_obs.request.dir.y = diry;
  pfilter_obs.request.dir.z = dirz;

  return pfilter_obs;

}

void addObs(double x, double y, double z, double dirx, double diry, double dirz, ros::ServiceClient &srv_add){
  particle_filter::AddObservation pfilter_obs = observation(x, y, z, dirx, diry, dirz);
  if(!srv_add.call(pfilter_obs)){
    ROS_INFO("Failed to call add observation");
    ros::Duration(5).sleep();
  }
  
}

int main(int argc, char **argv){

  ros::init(argc, argv, "ray_trace_grid");
  ros::NodeHandle n;
  ros::ServiceClient srv_add = 
    n.serviceClient<particle_filter::AddObservation>("/particle_filter_add");

  double resolution = 0.005;
  double roll = 0;
  double pitch = 0;
  double yaw = 0;

  makeImage(resolution, roll, pitch, yaw, 1);


  // return 0;
  //Touch 1
  addObs(0.729351, 0.215845, 0.358450, 
	 0.397690, 0.865360, -0.304951, srv_add);

  makeImage(resolution, roll, pitch, yaw, 2);

  //Touch 2
  ROS_INFO("Touch 2");
  addObs(0.714676, 0.210711, 0.427065,
	 -0.000231, -0.000523, -1.000000, srv_add);

  makeImage(resolution, roll, pitch, yaw, 3);

  //Touch 3
  ROS_INFO("Touch 3");
  addObs(0.751231, 0.195257, 0.429780,
	 0.000647, 0.000393, -1.000000, srv_add);

  makeImage(resolution, roll, pitch, yaw, 4);

  //Touch 4
  ROS_INFO("Touch 4");
  addObs(0.861499, -0.071836, 0.361358,
	 0.908520, -0.408613, -0.087337, srv_add);

  makeImage(resolution, roll, pitch, yaw, 5);

  //Touch 5
  ROS_INFO("Touch 5");
  addObs(0.780144, -0.295979, 0.422726,
	 -0.000919, 0.000499, -0.999999, srv_add);

  makeImage(resolution, roll, pitch, yaw, 6);



  return 0;
}
