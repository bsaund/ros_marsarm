/**
 *  This is a rosnode that randomly chooses touch points, 
 *    performs simulated measurements, and updates the particle filter.
 *   This node is made to work with a particle filter node, a node that 
 *    publishes visualization messages, and RViz.
 */
#include <iostream>
#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include "particle_filter/PFilterInit.h"
#include "particle_filter/AddObservation.h"
#include "geometry_msgs/Point.h"
 #include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include "custom_ray_trace/plotRayUtils.h"
#include "custom_ray_trace/rayTracer.h"
#include <ros/console.h>
#include <Eigen/Dense>

#define NUM_TOUCHES 20
// /**
//  * Gets initial points for the particle filter by shooting
//  * rays at the object
//  */
// particle_filter::PFilterInit getInitialPoints(PlotRayUtils &plt)
// {
//   particle_filter::PFilterInit init_points;

//   tf::Point start1(1,1,0);
//   tf::Point end1(-1,-1,0);
//   tf::Point start2(1,1,.2);
//   tf::Point end2(-1,-1,.2);
//   tf::Point start3(1.1,0.9,0);
//   tf::Point end3(-0.9,-1.1,0);
  
//   tf::Point intersection;
//   plt.getIntersectionWithPart(start1,end1, intersection);
//   tf::pointTFToMsg(intersection,  init_points.p1);
//   plt.getIntersectionWithPart(start2,end2, intersection);
//   tf::pointTFToMsg(intersection, init_points.p2);
//   plt.getIntersectionWithPart(start3,end3, intersection);
//   tf::pointTFToMsg(intersection, init_points.p3);
//   return init_points;
// }

void fixedSelection(PlotRayUtils &plt, RayTracer &rayt, tf::Point &best_start, tf::Point &best_end)
{
  int index;
  double bestIG = 0;
  tf::Point tf_start;
  tf::Point tf_end;
  bestIG = 0;
  std::random_device rd;
  std::uniform_real_distribution<double> rand(0, 1);
  std::uniform_int_distribution<> int_rand(0, 3);
  Eigen::Vector3d start;
  Eigen::Vector3d end;
  double state[6] = {0.3, 0.3, 0.3, 0.5, 0.7, 0.5};
  Eigen::Matrix3d rotationC;
  rotationC << cos(state[5]), -sin(state[5]), 0,
               sin(state[5]), cos(state[5]), 0,
               0, 0, 1;
  Eigen::Matrix3d rotationB;
  rotationB << cos(state[4]), 0 , sin(state[4]),
               0, 1, 0,
               -sin(state[4]), 0, cos(state[4]);
  Eigen::Matrix3d rotationA;
  rotationA << 1, 0, 0 ,
               0, cos(state[3]), -sin(state[3]),
               0, sin(state[3]), cos(state[3]);
  Eigen::Matrix3d rotationM = rotationC * rotationB * rotationA;
  Eigen::Vector3d displaceV(state[0], state[1], state[2]);
  for(int i=0; i<500; i++){
    index = int_rand(rd);
    if (index == 0)
    {
      double y = rand(rd) * 0.31 - 0.35;
      double z = rand(rd) * 0.18 + 0.03;
      start << 2, y, z;
      end << -1, y, z;

    }
    else if (index == 1)
    {
      double x = rand(rd) * 1.1 + 0.1;
      double z = rand(rd) * 0.18 + 0.03;
      start << x, -1, z;
      end << x, 1, z;
    }
    else if (index == 2)
    {
      double x = rand(rd) * 1.1 + 0.1;
      double y = rand(rd) * 0.01 - 0.02;
      start << x, y, 1;
      end << x, y, -1;
    }
    else
    {
      double x = rand(rd) * 0.02 + 0.33;
      double y = rand(rd) * 0.2 - 0.35;
      start << x, y, 1;
      end << x, y, -1;
    }
    
    Eigen::Vector3d tran_start = rotationM * start + displaceV;
    Eigen::Vector3d tran_end = rotationM * end + displaceV;

    tf_start.setValue(tran_start(0, 0), tran_start(1, 0), tran_start(2, 0));
    tf_end.setValue(tran_end(0, 0), tran_end(1, 0), tran_end(2, 0));
    Ray measurement(tf_start, tf_end);
    // auto timer_begin = std::chrono::high_resolution_clock::now();
    double IG = rayt.getIG(measurement, 0.01, 0.002);
    // plt.plotRay(measurement);
    // plt.labelRay(measurement, IG);
    // auto timer_end = std::chrono::high_resolution_clock::now();
    // auto timer_dur = timer_end - timer_begin;
    // cout << "IG: " << IG << endl;
    // cout << "Elapsed time for ray: " << std::chrono::duration_cast<std::chrono::milliseconds>(timer_dur).count() << endl;
    // double IG = plt.getIG(start, end, 0.01, 0.002);
    if (IG > bestIG){
      bestIG = IG;
      best_start = tf_start;
      best_end = tf_end;
    }
  }
  // plt.plotCylinder(best_start, best_end, 0.01, 0.002, true);
  ROS_INFO("Ray is: %f, %f, %f.  %f, %f, %f", 
     best_start.getX(), best_start.getY(), best_start.getZ(),
     best_end.getX(), best_end.getY(), best_end.getZ());
  plt.plotRay(Ray(best_start, best_end));
}

/**
 * Randomly chooses vectors, gets the Information Gain for each of 
 *  those vectors, and returns the ray (start and end) with the highest information gain
 */
void randomSelection(PlotRayUtils &plt, RayTracer &rayt, tf::Point &best_start, tf::Point &best_end)
{
  // tf::Point best_start, best_end;

  double bestIG;
  bestIG = 0;
  std::random_device rd;
  std::uniform_real_distribution<double> rand(-4.0,3.0);


  for(int i=0; i<500; i++){
    tf::Point start(rand(rd), rand(rd), rand(rd));
    start = start.normalize();
    tf::Point end(rand(rd), rand(rd), rand(rd));
    end.normalized();
    Ray measurement(start, end);
    // auto timer_begin = std::chrono::high_resolution_clock::now();
    double IG = rayt.getIG(measurement, 0.01, 0.002);
    // auto timer_end = std::chrono::high_resolution_clock::now();
    // auto timer_dur = timer_end - timer_begin;
    // cout << "IG: " << IG << endl;
    // cout << "Elapsed time for ray: " << std::chrono::duration_cast<std::chrono::milliseconds>(timer_dur).count() << endl;
    // double IG = plt.getIG(start, end, 0.01, 0.002);
    if (IG > bestIG){
      bestIG = IG;
      best_start = start;
      best_end = end;
    }
  }
  //Ray measurement(best_start, best_end);
  // plt.plotCylinder(best_start, best_end, 0.01, 0.002, true);
  ROS_INFO("Ray is: %f, %f, %f.  %f, %f, %f", 
	   best_start.getX(), best_start.getY(), best_start.getZ(),
	   best_end.getX(), best_end.getY(), best_end.getZ());
  
}

// bool getIntersection(PlotRayUtils &plt, tf::Point start, tf::Point end, tf::Point &intersection){
//   bool intersectionExists = plt.getIntersectionWithPart(start, end, intersection);
//   double radius = 0.001;
//   intersection = intersection - (end-start).normalize() * radius;
//   return intersectionExists;
// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "updating_particles");
  ros::NodeHandle n;
  PlotRayUtils plt;
  RayTracer rayt;

  std::random_device rd;
  std::normal_distribution<double> randn(0.0,0.0005);

  ROS_INFO("Running...");

  ros::Publisher pub_init = 
    n.advertise<particle_filter::PFilterInit>("/particle_filter_init", 5);
  ros::ServiceClient srv_add = 
    n.serviceClient<particle_filter::AddObservation>("/particle_filter_add");


 
  ros::Duration(1).sleep();
  // pub_init.publish(getInitialPoints(plt));
 
  geometry_msgs::Point obs;
  geometry_msgs::Point dir;
  double radius = 0.001;

  int i = 0;
  //for(int i=0; i<20; i++){
  while (i < NUM_TOUCHES) {
    // ros::Duration(1).sleep();
    //tf::Point start(0.95,0,-0.15);
    //tf::Point end(0.95,2,-0.15);
    tf::Point start, end;
    // randomSelection(plt, rayt, start, end);
    fixedSelection(plt, rayt, start, end);

    Ray measurement(start, end);
    
    double distToPart;
    if(!rayt.traceRay(measurement, distToPart)){
      ROS_INFO("NO INTERSECTION, Skipping");
      continue;
    }
    tf::Point intersection(start.getX(), start.getY(), start.getZ());
    intersection = intersection + (end-start).normalize() * (distToPart - radius);
	std::cout << "Intersection at: " << intersection.getX() << "  " << intersection.getY() << "   " << intersection.getZ() << std::endl;
    tf::Point ray_dir(end.x()-start.x(),end.y()-start.y(),end.z()-start.z());
    ray_dir = ray_dir.normalize();
    obs.x=intersection.getX() + randn(rd); 
    obs.y=intersection.getY() + randn(rd); 
    obs.z=intersection.getZ() + randn(rd);
    dir.x=ray_dir.x();
    dir.y=ray_dir.y();
    dir.z=ray_dir.z();
    // obs.x=intersection.getX(); 
    // obs.y=intersection.getY(); 
    // obs.z=intersection.getZ();

    // pub_add.publish(obs);
    
    // plt.plotCylinder(start, end, 0.01, 0.002, true);
    plt.plotRay(Ray(start, end));
    // ros::Duration(1).sleep();

    particle_filter::AddObservation pfilter_obs;
    pfilter_obs.request.p = obs;
    pfilter_obs.request.dir = dir;
    if(!srv_add.call(pfilter_obs)){
      ROS_INFO("Failed to call add observation");
    }

    ros::spinOnce();
    while(!rayt.particleHandler.newParticles){
      ROS_INFO_THROTTLE(10, "Waiting for new particles...");
      ros::spinOnce();
      ros::Duration(.1).sleep();
    }
    i ++;
  }
  std::ofstream myfile;
  myfile.open("/home/shiyuan/Documents/ros_marsarm/diff.csv", std::ios::out|std::ios::app);
  myfile << "\n";
  myfile.close();
  myfile.open("/home/shiyuan/Documents/ros_marsarm/time.csv", std::ios::out|std::ios::app);
  myfile << "\n";
  myfile.close();
  myfile.open("/home/shiyuan/Documents/ros_marsarm/diff_trans.csv", std::ios::out|std::ios::app);
  myfile << "\n";
  myfile.close();
  myfile.open("/home/shiyuan/Documents/ros_marsarm/diff_rot.csv", std::ios::out|std::ios::app);
  myfile << "\n";
  myfile.close();
  myfile.open("/home/shiyuan/Documents/ros_marsarm/workspace_max.csv", std::ios::out|std::ios::app);
  myfile << "\n";
  myfile.close();
  myfile.open("/home/shiyuan/Documents/ros_marsarm/workspace_min.csv", std::ios::out|std::ios::app);
  myfile << "\n";
  myfile.close();
  ROS_INFO("Finished all action");

}
