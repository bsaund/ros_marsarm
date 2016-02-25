#include <ros/ros.h>
#include "particleFilter6DOF.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include "particle_filter/PFilterInit.h"
#include "particle_filter/AddObservation.h"
#include "gazebo_ray_trace/plotRayUtils.h"
#include <math.h>

#define NUM_PARTICLES 1000

class PFilterTest
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub_init;
  ros::ServiceServer srv_add_obs;
  ros::Publisher pub_particles;
  
  distanceTransform dist_transform;
  PlotRayUtils plt;
public:
  geometry_msgs::PoseArray getParticlePoseArray();
  particleFilter pFilter_;
  PFilterTest(int n_particles, particleFilter::cspace b_init[2]);
  // void addObs(geometry_msgs::Point obs);
  bool addObs(particle_filter::AddObservation::Request &req,
	      particle_filter::AddObservation::Response &resp);
};

void computeInitialDistribution(particleFilter::cspace binit[2])
{

  binit[0][0] = 0.00;
  binit[0][1] = 0.00;
  binit[0][2] = 0.00;
  binit[0][3] = 0.00;
  binit[0][4] = 0.00;
  binit[0][5] = 0.00;
  binit[1][0] = 0.01;
  binit[1][1] = 0.01;
  binit[1][2] = 0.01;
  binit[1][3] = 0.01;
  binit[1][4] = 0.01;
  binit[1][5] = 0.01;


}

double SQ(double d)
{
  return d*d;
}


/*
 *  Converts a cspace pose to a tf::Pose
 */
tf::Pose poseAt(particleFilter::cspace particle_pose)
{
  tf::Pose tf_pose;
  tf_pose.setOrigin(tf::Vector3(particle_pose[0], 
				particle_pose[1], 
				particle_pose[2]));
  tf::Quaternion q;
  q.setRPY(particle_pose[4], particle_pose[5], particle_pose[6]);
  tf_pose.setRotation(q);
  
  return tf_pose;

}

bool PFilterTest::addObs(particle_filter::AddObservation::Request &req,
			 particle_filter::AddObservation::Response &resp)
{
  geometry_msgs::Point obs = req.p;
  ROS_INFO("Adding Observation...");
  double obs2[3] = {obs.x, obs.y, obs.z};
  double cube[3] = {.508, .508, .508};
  pFilter_.addObservation(obs2, cube, &dist_transform, 0);
  ROS_INFO("...Done adding observation");
  pub_particles.publish(getParticlePoseArray());

}



geometry_msgs::PoseArray PFilterTest::getParticlePoseArray()
{
  particleFilter::cspace particles[NUM_PARTICLES];
  pFilter_.getAllParticles(particles);
  tf::Transform trans = plt.getTrans();

  geometry_msgs::PoseArray poseArray;
  for(int i=0; i<50; i++){
    tf::Pose pose = poseAt(particles[i]);
    geometry_msgs::Pose pose_msg;
    tf::poseTFToMsg(trans*pose, pose_msg);
    poseArray.poses.push_back(pose_msg);
  }
  return poseArray;
}

PFilterTest::PFilterTest(int n_particles, particleFilter::cspace b_init[2]) :
  pFilter_(n_particles, b_init, 0.003, 0.0, 0.0, 0.0),
  dist_transform(0.1, 0.0005)
  // particleFilter (int n_particles,
  // 		  double Xstd_ob=0.0001, double Xstd_tran=0.0025,
  // 		  double Xstd_scatter=0.0001, double R=0.0005);

{
  
  // sub_init = n.subscribe("/particle_filter_init", 1, &PFilterTest::initDistribution, this);
  srv_add_obs = n.advertiseService("/particle_filter_add", &PFilterTest::addObs, this);
  pub_particles = n.advertise<geometry_msgs::PoseArray>("/particles_from_filter", 5);

  // tf::Point touch1(0, 0, 0);
  // tf::Point touch2(0, 1, -1);
  // tf::Point touch3(1, -1, 0);
  // particleFilter::cspace binit[2];
  // computeInitialDistribution(binit, touch1, touch2, touch3);
  // ROS_INFO("Computed Initial Distribution");
  // pFilter_.setDistribution(binit);

}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "pfilterTest");
  // ros::NodeHandle n;
  // ros::Publisher pub = n.advertise<geometry_msgs::PoseArray>("/particles_from_filter", 5);

  ROS_INFO("Testing particle filter");
  
  particleFilter::cspace b_Xprior[2];
  computeInitialDistribution(b_Xprior);
  PFilterTest pFilterTest(NUM_PARTICLES, b_Xprior);

  // particleFilter::cspace binit[2];

  
  // double obs[3] = {0, 1, -1};
  // pfilter.addObservation(obs);


  // pfilter.estimatedDistribution(binit);


  // tf::Pose pose = poseAt(0,0, binit[0]);
  // tf::Vector3 point = pose.getOrigin();
  // ROS_INFO("Point: %f, %f, %f", point.getX(), point.getY(), point.getZ());
  // pub.publish(getParticlePoseArray(pfilter));
  // ros::Duration(5.0).sleep();
  
  // for(int i=0; i<10; i++){
  //   double obs2[3] = {0,0,0};
  //   pFilterTest.pFilter_.addObservation(obs2);
  //   pub.publish(pFilterTest.getParticlePoseArray());
  //   ROS_INFO("Estimated Dist: %f, %f, %f", binit[0][0], binit[0][1], binit[0][2]);
  //   ros::Duration(5.0).sleep();
  // }

  ros::spin();

}
