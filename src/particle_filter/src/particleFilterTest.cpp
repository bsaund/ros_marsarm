#include <ros/ros.h>
#include "particleFilter.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>

#define NUM_PARTICLES 1000

void computeInitialDistribution(particleFilter::cspace binit[2],
				tf::Point p1, tf::Point p2, tf::Point p3)
{
  tf::Vector3 d1 = p2-p1;
  tf::Vector3 d2 = p2-p3;
  tf::Vector3 cross = d1.cross(d2);

  double D= cross.dot(p1);

  binit[0][0] = cross.getY()/cross.getX();
  binit[0][1] = cross.getZ()/cross.getX();
  binit[0][2] = -D/cross.getX();
  std::cerr << "Init: " << binit[0][0] << " " << binit[0][1] << " " << binit[0][2] << std::endl;
  binit[1][0] = binit[1][1] = binit[1][2] = 0.1;

}

double SQ(double d)
{
  return d*d;
}


tf::Pose poseAt(double y, double z, particleFilter::cspace plane)
{
  double t = atan(plane[0]);
  double p = acos(plane[1]/sqrt(1 + SQ(plane[0]) + SQ(plane[1])));

  tf::Pose pose;
  pose.setOrigin(tf::Vector3(-(plane[0]*y + plane[1]*z + plane[2]), y, z));
  tf::Quaternion q;
  q.setRPY(-t, p, 0);
  pose.setRotation(q);

  
  return pose;

}


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "pfilterTest");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::PoseArray>("/particles_from_filter", 5);

  ROS_INFO("Testing particle filter");

  particleFilter pfilter(NUM_PARTICLES);
  particleFilter::cspace binit[2];

  tf::Point touch1(0, 0, 0);
  tf::Point touch2(0, 1, -1);
  tf::Point touch3(1, -1, 0);

  computeInitialDistribution(binit, touch1, touch2, touch3);
  ROS_INFO("Computed Initial Distribution");
  pfilter.setDistribution(binit);
  
  double obs[3] = {0, 1, -1};
  pfilter.addObservation(obs);


  pfilter.estimatedDistribution(binit);
  ROS_INFO("Estimated Dist: %f, %f, %f", binit[0][0], binit[0][1], binit[0][2]);

  tf::Pose pose = poseAt(0,0, binit[0]);
  tf::Vector3 point = pose.getOrigin();
  ROS_INFO("Point: %f, %f, %f", point.getX(), point.getY(), point.getZ());
  
  particleFilter::cspace particles[NUM_PARTICLES];
  pfilter.getAllParticles(particles);

  geometry_msgs::PoseArray poseArray;
  for(int i=0; i<50; i++){
    pose = poseAt(0,0,particles[i]);
    geometry_msgs::Pose pose_msg;
    tf::poseTFToMsg(pose, pose_msg);
    poseArray.poses.push_back(pose_msg);
  }
  pub.publish(poseArray);
  ros::spin();

}
