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

void computeInitialDistribution(particleFilter::cspace binit[2], ros::NodeHandle n)
{

  std::vector<double> uncertainties;
  if(!n.getParam("/initial_uncertainties", uncertainties)){
    ROS_INFO("Failed to get param");
    uncertainties.resize(6);
  }

  std::vector<double> pFrame;
  if(!n.getParam("/particle_frame", pFrame)){
    ROS_INFO("Failed to get param particle_frame");
    pFrame.resize(6);
  }


  binit[0][0] = pFrame[0];
  binit[0][1] = pFrame[1];
  binit[0][2] = pFrame[2];
  binit[0][3] = pFrame[3];
  binit[0][4] = pFrame[4];
  binit[0][5] = pFrame[5];

  binit[1][0] = uncertainties[0];
  binit[1][1] = uncertainties[1];
  binit[1][2] = uncertainties[2];

  // binit[1][0] = 0.00;
  // binit[1][1] = 0.00;
  // binit[1][2] = 0.00;

  binit[1][3] = uncertainties[3];
  binit[1][4] = uncertainties[4];
  binit[1][5] = uncertainties[5];

  // binit[1][3] = 0;
  // binit[1][4] = 0;
  // binit[1][5] = 0;


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
  // q.setRPY(particle_pose[6], particle_pose[5], particle_pose[4]);
  // q.setEulerZYX(particle_pose[6], particle_pose[5], particle_pose[4]);
  q = tf::Quaternion(tf::Vector3(0,0,1), particle_pose[5]) * 
    tf::Quaternion(tf::Vector3(0,1,0), particle_pose[4]) * 
    tf::Quaternion(tf::Vector3(1,0,0), particle_pose[3]);
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
  pFilter_(n_particles, b_init, 0.003, 0.003, 0.01, 0.005),
  dist_transform(.1, 0.0005)
  // particleFilter (int n_particles,
  // 		  double Xstd_ob=0.0001, double Xstd_tran=0.0025,
  // 		  double Xstd_scatter=0.0001, double R=0.0005);

{
  
  // sub_init = n.subscribe("/particle_filter_init", 1, &PFilterTest::initDistribution, this);
  srv_add_obs = n.advertiseService("/particle_filter_add", &PFilterTest::addObs, this);
  pub_particles = n.advertise<geometry_msgs::PoseArray>("/particles_from_filter", 5);


}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "pfilterTest");
  ros::NodeHandle n;
  // ros::Publisher pub = n.advertise<geometry_msgs::PoseArray>("/particles_from_filter", 5);

  ROS_INFO("Testing particle filter");
  
  particleFilter::cspace b_Xprior[2];
  computeInitialDistribution(b_Xprior, n);
  PFilterTest pFilterTest(NUM_PARTICLES, b_Xprior);

  ros::spin();

}
