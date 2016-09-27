#include <ros/ros.h>
#include "particleFilter3DOF.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include "particle_filter/PFilterInit.h"
#include "particle_filter/AddObservation.h"


#define NUM_PARTICLES 1000

class PFilterTest
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub_init;
  ros::ServiceServer srv_add_obs;
  ros::Publisher pub_particles;
  
  PlotRayUtils plt;
public:
  geometry_msgs::PoseArray getParticlePoseArray();
  particleFilter pFilter_;
  PFilterTest(int n_particles);
  // void addObs(geometry_msgs::Point obs);
  bool addObs(particle_filter::AddObservation::Request &req,
	      particle_filter::AddObservation::Response &resp);

  void initDistribution(particle_filter::PFilterInit points);
};

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

bool PFilterTest::addObs(particle_filter::AddObservation::Request &req,
			 particle_filter::AddObservation::Response &resp)
{
  geometry_msgs::Point obs = req.p;
  ROS_INFO("Adding Observation...");
  double obs2[3] = {obs.x, obs.y, obs.z};
  pFilter_.addObservation(obs2);
  ROS_INFO("...Done adding observation");
  pub_particles.publish(getParticlePoseArray());

}

void PFilterTest::initDistribution(particle_filter::PFilterInit points)
{
  ROS_INFO("Starting to compute distribution");
  tf::Point touch1(points.p1.x, points.p1.y, points.p1.z);
  tf::Point touch2(points.p2.x, points.p2.y, points.p2.z);
  tf::Point touch3(points.p3.x, points.p3.y, points.p3.z);
  particleFilter::cspace binit[2];
  computeInitialDistribution(binit, touch1, touch2, touch3);
  ROS_INFO("Computed Initial Distribution");
  pFilter_.setDistribution(binit);
}


geometry_msgs::PoseArray PFilterTest::getParticlePoseArray()
{
  particleFilter::cspace particles[NUM_PARTICLES];
  pFilter_.getAllParticles(particles);
  tf::Transform trans = plt.getTrans();

  geometry_msgs::PoseArray poseArray;
  for(int i=0; i<50; i++){
    tf::Pose pose = poseAt(0,0,particles[i]);
    geometry_msgs::Pose pose_msg;
    tf::poseTFToMsg(trans*pose, pose_msg);
    poseArray.poses.push_back(pose_msg);
  }
  return poseArray;
}

PFilterTest::PFilterTest(int n_particles) :
  pFilter_(n_particles, 0.003, 0.005, 0.0, 0.0)
  // particleFilter (int n_particles,
  // 		  double Xstd_ob=0.0001, double Xstd_tran=0.0025,
  // 		  double Xstd_scatter=0.0001, double R=0.0005);

{
  
  sub_init = n.subscribe("particle_filter_init", 1, &PFilterTest::initDistribution, this);
  srv_add_obs = n.advertiseService("/particle_filter_add", &PFilterTest::addObs, this);
  pub_particles = n.advertise<geometry_msgs::PoseArray>("particles_from_filter", 5);

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
  
  PFilterTest pFilterTest(NUM_PARTICLES);

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
