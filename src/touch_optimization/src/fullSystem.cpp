/**
 *  This is a rosnode that randomly chooses touch points, 
 *    performs simulated measurements, and updates the particle filter.
 *   This node is made to work with a particle filter node, a node that 
 *    publishes visualization messages, and RViz.
 */

#include <ros/ros.h>
#include "particle_filter/PFilterInit.h"
#include "particle_filter/AddObservation.h"
#include "geometry_msgs/Point.h"
#include <tf/transform_broadcaster.h>
#include "gazebo_ray_trace/plotRayUtils.h"
# define M_PI       3.14159265358979323846  /* pi */

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


void generateRandomRay(std::mt19937 gen, tf::Pose &probePose, tf::Point &start, tf::Point &end)
{
  std::uniform_real_distribution<double> rand(-1.0,1.0);
  tf::Transform rotate;
  rotate.setRotation(tf::createQuaternionFromRPY(0,0,M_PI/4));
  tf::Pose touchBase;
  touchBase.setOrigin(tf::Vector3(0.5, -0.1, 1.2));
  touchBase.setRotation(tf::createQuaternionFromRPY(0, M_PI/2, 0));

  tf::Pose offset;
  offset.setOrigin(tf::Vector3(0,rand(gen)*.05, rand(gen)*.05));
  // offset.setOrigin(tf::Vector3(0,0,0));
  offset.setRotation(tf::createQuaternionFromRPY(0,0,0));

  probePose = rotate*offset*touchBase;
  tf::Transform probeZ;
  probeZ.setRotation(tf::createQuaternionFromRPY(0,0,0));
  probeZ.setOrigin(tf::Vector3(0,0,0.1));
  start = probePose.getOrigin();
  end = (probeZ * probePose).getOrigin();
}



/**
 * Randomly chooses vectors, gets the Information Gain for each of 
 *  those vectors, and returns the ray (start and end) with the highest information gain
 */
void randomSelection(PlotRayUtils &plt, tf::Pose &probePose)
{
  // tf::Point best_start, best_end;

  double bestIG;
  bestIG = 0;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> rand(-1.0,1.0);

  tf::Point best_start, best_end;

  for(int i=0; i<500; i++){
    // tf::Point start(rand(gen), rand(gen), rand(gen));
    // start = start.normalize();
    // tf::Point end(rand(gen), rand(gen), rand(gen));
    // end.normalized();
    tf::Point start, end;
    tf::Pose probePoseTmp;
    generateRandomRay(gen, probePoseTmp, start, end);
    double IG = plt.getIG(start, end, 0.01, 0.002);
    if (IG > bestIG){
      
      bestIG = IG;
      best_start = start;
      best_end = end;
      probePose = probePoseTmp;
    }
  }

  // plt.plotCylinder(best_start, best_end, 0.01, 0.002, true);
  ROS_INFO("Ray is: %f, %f, %f.  %f, %f, %f", 
  	   best_start.getX(), best_start.getY(), best_start.getZ(),
  	   best_end.getX(), best_end.getY(), best_end.getZ());
  
}


bool getIntersection(PlotRayUtils &plt, tf::Point start, tf::Point end, tf::Point &intersection){
  bool intersectionExists = plt.getIntersectionWithPart(start, end, intersection);
  double radius = 0.005;
  intersection = intersection - (end-start).normalize() * radius;
  return intersectionExists;
}


geometry_msgs::Pose probeAt(tf::Transform rotate, tf::Transform base, double x, double y, double z, double r, double p, double yaw){
  tf::Pose offset;
  offset.setOrigin(tf::Vector3(x,y,z));
  offset.setRotation(tf::createQuaternionFromRPY(r, p, yaw));
  geometry_msgs::Pose probe_msg;
  tf::poseTFToMsg(rotate*offset*base, probe_msg);
  // probe_pub.publish(probe_msg);
  return probe_msg;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "updating_particles");
  ros::NodeHandle n;
  PlotRayUtils plt;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> randn(0.0,0.003);

  ROS_INFO("Running...");

  ros::Publisher probe_pub = 
    n.advertise<geometry_msgs::Pose>("/probe_point", 5);
 
  ros::Duration(2).sleep();
  
 
  // geometry_msgs::Pose probe_msg; 

  // Pose rotate(0, 0, 0, 0, 0, M_PI/4);

  double dtouch = 0.05;
  // Pose touchUp(0, 0, dtouch, 0, 0, 0);
  // Pose touchDown(0, 0, -dtouch, 0, 0, 0);
  // Pose touchSide(0, dtouch, 0, 0, 0, 0);
  // Pose touchMiddle(0, dtouch/2, dtouch/2, 0, 0, 0);
  // Pose touchBase(0.5, -0.1, 1.2, 0, M_PI/2, 0);
  // Pose touch2 = rotate*(touchBase+touchUp);
  // tf::Transform rotate;
  // rotate.setRotation(tf::createQuaternionFromRPY(0,0,M_PI/4));
  // tf::Pose touchBase;
  // touchBase.setOrigin(tf::Vector3(0.5, -0.1, 1.2));
  // touchBase.setRotation(tf::createQuaternionFromRPY(0, M_PI/2, 0));
  


  // probe_pub.publish(probeAt(rotate, touchBase, 0,0,0,0,0,0));
  // ros::Duration(30.0).sleep();

  // probe_pub.publish(probeAt(rotate, touchBase, 0,0,dtouch,0,0,0));
  // ros::Duration(30.0).sleep();
  // probe_pub.publish(probeAt(rotate, touchBase, 0,0,-dtouch,0,0,0));
  // ros::Duration(30.0).sleep();
  // probe_pub.publish(probeAt(rotate, touchBase, 0,dtouch,0,0,0,0));
  // ros::Duration(30.0).sleep();
  // probe_pub.publish(probeAt(rotate, touchBase, 0,-dtouch,0,0,0,0));
  // ros::Duration(30.0).sleep();
  tf::Pose probePose;
  geometry_msgs::Pose probe_msg; 

  for(int i=0; i<5; i++){
    randomSelection(plt, probePose);
    tf::poseTFToMsg(probePose, probe_msg);
    probe_pub.publish(probe_msg);
    ros::Duration(30.0).sleep();
  }


  ROS_INFO("Finished all action");

}
