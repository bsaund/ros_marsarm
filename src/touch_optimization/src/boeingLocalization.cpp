/**
 *  This is a rosnode that randomly chooses touch points, 
 *    performs simulated measurements, and updates the particle filter.
 *   This node is made to work with a particle filter node, a node that 
 *    publishes visualization messages, and RViz.
 */
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "particle_filter/PFilterInit.h"
#include "particle_filter/AddObservation.h"
#include "geometry_msgs/Point.h"
#include <tf/transform_broadcaster.h>
#include "gazebo_ray_trace/plotRayUtils.h"
#include <Eigen/Dense>

#define NUM_TOUCHES 20
/**
 * Gets initial points for the particle filter by shooting
 * rays at the object
 */
particle_filter::PFilterInit getInitialPoints(PlotRayUtils &plt)
{
  particle_filter::PFilterInit init_points;

  tf::Point start1(1,1,0);
  tf::Point end1(-1,-1,0);
  tf::Point start2(1,1,.2);
  tf::Point end2(-1,-1,.2);
  tf::Point start3(1.1,0.9,0);
  tf::Point end3(-0.9,-1.1,0);
  
  tf::Point intersection;
  plt.getIntersectionWithPart(start1,end1, intersection);
  tf::pointTFToMsg(intersection,  init_points.p1);
  plt.getIntersectionWithPart(start2,end2, intersection);
  tf::pointTFToMsg(intersection, init_points.p2);
  plt.getIntersectionWithPart(start3,end3, intersection);
  tf::pointTFToMsg(intersection, init_points.p3);
  return init_points;
}

/**
 * Randomly chooses vectors, gets the Information Gain for each of 
 *  those vectors, and returns the ray (start and end) with the highest information gain
 */
void randomSelection(PlotRayUtils &plt, tf::Point &best_start, tf::Point &best_end)
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
    double IG = plt.getIG(start, end, 0.01, 0.002);
    if (IG > bestIG){
      bestIG = IG;
      best_start = start;
      best_end = end;
    }
  }

  // plt.plotCylinder(best_start, best_end, 0.01, 0.002, true);
  ROS_INFO("Ray is: %f, %f, %f.  %f, %f, %f", 
	   best_start.getX(), best_start.getY(), best_start.getZ(),
	   best_end.getX(), best_end.getY(), best_end.getZ());
  
}

void fixedSelection(PlotRayUtils &plt, tf::Point &best_start, tf::Point &best_end, int index)
{
  std::random_device rd;
  std::uniform_real_distribution<double> rand(0, 1);
  Eigen::Vector3d start;
  Eigen::Vector3d end;
  if (index % 6 == 0)
  {
    double y = rand(rd) * 0.2 + 0.15;
    double z = rand(rd) * 0.12 + 0.03;
     start << 2, y, z;
     end << 0, y, z;
  }
  else if (index % 6 == 1)
  {
    double x = rand(rd) * 0.35 + 1.2;
    double z = rand(rd) * 0.12 + 0.03;
    start << x, 1, z;
    end << x, 0, z;
  }
  else if (index % 6 == 2)
  {
    double x = rand(rd) * 1.4 + 0.1;
    double y = rand(rd) * 0.04 + 0.02;
    start << x, y, 1;
    end << x, y, 0;
  }
  else if (index % 6 == 3)
  {
    double y = rand(rd) * 0.2 + 0.15;
    double z = rand(rd) * 0.12 + 0.03;
     start << 0.8, y, z;
     end << 0, y, z;
  }
  else if (index % 6 == 4)
  {
    double x = rand(rd) * 0.35 + 0.1;
    double z = rand(rd) * 0.12 + 0.03;
    start << x, 1, z;
    end << x, 0, z;
  }
  else
  {
    double x = rand(rd) * 1.4 + 0.1;
    double y = rand(rd) * 0.04 + 0.02;
    start << x, y, 1;
    end << x, y, 0;
  }
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
  Eigen::Vector3d tran_start = rotationM * start + displaceV;
  Eigen::Vector3d tran_end = rotationM * end + displaceV;
  best_start.setValue(tran_start(0, 0), tran_start(1, 0), tran_start(2, 0));
  best_end.setValue(tran_end(0, 0), tran_end(1, 0), tran_end(2, 0));
  // plt.plotCylinder(best_start, best_end, 0.01, 0.002, true);
  ROS_INFO("Ray is: %f, %f, %f.  %f, %f, %f", 
     best_start.getX(), best_start.getY(), best_start.getZ(),
     best_end.getX(), best_end.getY(), best_end.getZ());
}

bool getIntersection(PlotRayUtils &plt, tf::Point start, tf::Point end, tf::Point &intersection){
  bool intersectionExists = plt.getIntersectionWithPart(start, end, intersection);
  double radius = 0.000;
  intersection = intersection - (end-start).normalize() * radius;
  return intersectionExists;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "updating_particles");
  ros::NodeHandle n;
  PlotRayUtils plt;

  std::random_device rd;
  std::normal_distribution<double> randn(0.0,0.0000001);

  ROS_INFO("Running...");

  ros::Publisher pub_init = 
    n.advertise<particle_filter::PFilterInit>("/particle_filter_init", 5);
  ros::ServiceClient srv_add = 
    n.serviceClient<particle_filter::AddObservation>("/particle_filter_add");


 
  ros::Duration(2).sleep();
  // pub_init.publish(getInitialPoints(plt));
 
  geometry_msgs::Point obs;
  geometry_msgs::Point dir;
  // Eigen::Matrix<double, NUM_TOUCHES, 3> ray_start;
  // Eigen::Matrix<double, NUM_TOUCHES, 3> ray_end;
  // ray_start << 2, 0.3, 0.05,
  //              1.4, 1, 0.18,
  //              0.1, 0.05, 1,
  //              0.8, 0.15, 0.15,
  //              0.1, 1, 0.03,
  //              1.13, 0.35, 1,
  //              1.5, 0.02, 1,
  //              1.5, -1, 0.03,
  //              2, 0.04, 0.13,
  //              -1, 0.06, 0.03,
  //              0.85, 1, 0.1,
  //              0.4, 0.04, 1,
  //              1.55, 0.06, -1,
  //              2, 0.1, 0.14,
  //              1.12, 1, 0.08,
  //              0.51, 0.18, -1,
  //              1.35, 1, 0.07,
  //              0, 0.35, 0.03,
  //              1.54, 0.025, 1,
  //              1.20, 1, 0.03;

  // ray_end << 1, 0.3, 0.05,
  //            1.4, 0, 0.18,
  //            0.1, 0.05, 0,
  //            0, 0.15, 0.15,
  //            0.1, 0, 0.03,
  //            1.13, 0.35, 0,
  //            1.5, 0.02, 0,
  //            1.5, 1, 0.03,
  //            1, 0.04, 0.13,
  //            1, 0.06, 0.03,
  //            0.85, 0, 0.1,
  //            0.4, 0.04, 0,
  //            1.55, 0.06, 1,
  //            1, 0.1, 0.14,
  //            1.12, 0, 0.08,
  //            0.51, 0.18, 1,
  //            1.35, 0, 0.07,
  //            1, 0.35, 0.03,
  //            1.54, 0.025, 0,
  //            1.20, 0, 0.03;

  // double state[6] = {0.3, 0.3, 0.3, 0.5, 0.7, 0.5};
  // Eigen::Matrix3d rotationC;
  // rotationC << cos(state[5]), -sin(state[5]), 0,
  //              sin(state[5]), cos(state[5]), 0,
  //              0, 0, 1;
  // Eigen::Matrix3d rotationB;
  // rotationB << cos(state[4]), 0 , sin(state[4]),
  //              0, 1, 0,
  //              -sin(state[4]), 0, cos(state[4]);
  // Eigen::Matrix3d rotationA;
  // rotationA << 1, 0, 0 ,
  //              0, cos(state[3]), -sin(state[3]),
  //              0, sin(state[3]), cos(state[3]);
  // Eigen::Matrix3d rotationM = rotationC * rotationB * rotationA;
  // Eigen::Matrix<double, 3, NUM_TOUCHES> displaceM;
  // for (int ii = 0; ii < NUM_TOUCHES; ii ++) {
  //   displaceM(0, ii) = state[0];
  //   displaceM(1, ii) = state[1];
  //   displaceM(2, ii) = state[2];
  // }
  // Eigen::Matrix<double, 3, NUM_TOUCHES> tran_start = rotationM * (ray_start.transpose()) + displaceM;
  // Eigen::Matrix<double, 3, NUM_TOUCHES> tran_end = rotationM * (ray_end.transpose()) + displaceM;


  int i = 0;
  //for(int i=0; i<20; i++){
  while (i < NUM_TOUCHES) {
    ros::Duration(2).sleep();
    //tf::Point start(0.95,0,-0.15);
    //tf::Point end(0.95,2,-0.15);
    tf::Point start, end;
    randomSelection(plt, start, end);
    //fixedSelection(plt, start, end, i);
    // start.setValue(tran_start(0, i), tran_start(1, i), tran_start(2, i));
    // end.setValue(tran_end(0, i), tran_end(1, i), tran_end(2, i));
    tf::Point intersection;
    if(!getIntersection(plt, start, end, intersection)){
      ROS_INFO("NO INTERSECTION, Skipping");
      continue;
    }
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
    
    plt.plotCylinder(start, end, 0.01, 0.002, true);
    ros::Duration(1).sleep();

    particle_filter::AddObservation pfilter_obs;
    pfilter_obs.request.p = obs;
    pfilter_obs.request.dir = dir;
    if(!srv_add.call(pfilter_obs)){
      ROS_INFO("Failed to call add observation");
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
  ROS_INFO("Finished all action");

}
