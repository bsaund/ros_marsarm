#include <ros/ros.h>
#include "particleFilter.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <tf/transform_listener.h>

#include "particle_filter/PFilterInit.h"
#include "particle_filter/AddObservation.h"
#include "particle_filter/EstimateGaussian.h"

#include "transformDistribution.h"
#include "stlParser.h"
#include "relationships.h"

#include <math.h>
#include <string>
#include <array>

//#define POINT_CLOUD
#define NUM_PARTICLES 50

#ifdef POINT_CLOUD
pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr2(new pcl::PointCloud<pcl::PointXYZ>);
bool update;
boost::mutex updateModelMutex;
void visualize();
#endif

class PFilterRos
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub_init;
  ros::Subscriber sub_request_particles;
  ros::ServiceServer srv_add_obs;
  ros::ServiceServer srv_est_gaus;
  ros::Publisher pub_particles;
  tf::StampedTransform trans_;
  
  distanceTransform *dist_transform;
  // ParticleHandler pHandler;
  PartRelationships rel;

  bool getMesh(std::string filename,   vector<vec4x3> &loadedMesh);
#ifdef POINT_CLOUD
  void initPointCloud();
#endif

public:
  vector<vec4x3> mesh;
  int num_voxels[3];
  geometry_msgs::PoseArray getParticlePoseArray();
  particleFilter pFilter_;
  PFilterRos(int n_particles, cspace b_init[2]);
  // void addObs(geometry_msgs::Point obs);
  bool addObs(particle_filter::AddObservation::Request &req,
	      particle_filter::AddObservation::Response &resp);
  bool estGaus(particle_filter::EstimateGaussian::Request &req,
	      particle_filter::EstimateGaussian::Response &resp);
  void sendParticles(std_msgs::Empty);
};

/*
 *  Sets binit to the initial distribution parameters 
 *   based on ros parameters (currently
 *   restricted to a gaussian).
 */
void computeInitialDistribution(cspace binit[2], ros::NodeHandle n)
{

  std::vector<double> uncertainties;
  if(!n.getParam("initial_uncertainties", uncertainties)){
    ROS_INFO("Failed to get param: initial_uncertainties");
    uncertainties.resize(6);
  }

  std::vector<double> pFrame;
  if(!n.getParam("prior_offset", pFrame)){
    ROS_INFO("Failed to get param: prior_offset");
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
  binit[1][3] = uncertainties[3];
  binit[1][4] = uncertainties[4];
  binit[1][5] = uncertainties[5];

}


/*
 *  Converts a cspace pose to a tf::Pose
 */
tf::Pose poseAt(cspace particle_pose)
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

/*
 *  Publishes the particles in response to a request
 *   over a ros message
 */
void PFilterRos::sendParticles(std_msgs::Empty emptyMsg)
{
  pub_particles.publish(getParticlePoseArray());
}


/*
 *  Updates the particle filter by the touch observation described in req
 */
bool PFilterRos::addObs(particle_filter::AddObservation::Request &req,
			 particle_filter::AddObservation::Response &resp)
{
  geometry_msgs::Point obs = req.p;
  geometry_msgs::Point dir = req.dir; 
  std::string observedObject = req.object;

  std::string ns = ros::this_node::getNamespace();
  ns.erase(0,2);


  double obs2[2][3] = {{obs.x, obs.y, obs.z}, {dir.x, dir.y, dir.z}};

  if(ns == observedObject){
    pFilter_.addObservation(obs2, mesh, dist_transform, 0);
    ROS_INFO("Adding Direct Observation...") ;
    ROS_INFO("point: %f, %f, %f", obs.x, obs.y, obs.z);
    ROS_INFO("dir: %f, %f, %f", dir.x, dir.y, dir.z);

    ROS_INFO("...Done adding direct observation on %s", ns.c_str());
    pub_particles.publish(getParticlePoseArray());



    return true;
  } 
  
  if(rel.has(ns, observedObject)){
    vector<vec4x3> hitMesh;
    getMesh(req.mesh_resource, hitMesh);

    // FixedTransform tf(rel[observedObject]); 
    // ROS_INFO("object: %s", observedObject.c_str());
    // std::unique_ptr<RandomTransform> tf = rel[observedObject];

    // ROS_INFO("Assigned transform");
    // ROS_INFO("First val: %f", rel[observedObject]->getMean()[0]);
    // ROS_INFO("First val: %f", tf->getMean()[0]);
    ROS_INFO("Adding implicit observation for %s", ns.c_str());
    pFilter_.addObservation(obs2, hitMesh, dist_transform, *rel.of(ns, observedObject), 0);
    // pFilter_.addObservation(obs2, hitMesh, dist_transform, tf, 0);

    pub_particles.publish(getParticlePoseArray());

    if(ns=="goal_hole"){
      ROS_INFO("Estimating Gaussian");
      cspace particles_est_stat;
      cspace particles_est;
      pFilter_.estimateGaussian(particles_est, particles_est_stat, true);
    }


    return true;
  }




  ROS_INFO("No Relationship detected for %s", ns.c_str());
  pub_particles.publish(getParticlePoseArray());
  return true;
}


/*
 *  Exposes Estimate Gaussian function
 */

bool PFilterRos::estGaus(particle_filter::EstimateGaussian::Request &req,
			 particle_filter::EstimateGaussian::Response &resp)
{

  cspace x_mean, x_est_stat;
  pFilter_.estimateGaussian(x_mean, x_est_stat);

  std::vector<float> mean, cov;

  for (int k = 0; k < cdim; k++) {
    mean.push_back(x_mean[k]);
    cov.push_back(x_est_stat[k]);
  }

  resp.Mean = mean;
  resp.Cov = cov;

  resp.success = true;
  ROS_INFO("Service called!");
  return true;
}

bool PFilterRos::getMesh(std::string stlFilePath, vector<vec4x3> &loadedMesh){
  loadedMesh = importSTL(stlFilePath);
}


geometry_msgs::PoseArray PFilterRos::getParticlePoseArray()
{
  std::vector<cspace> particles;
  pFilter_.getAllParticles(particles);
  // tf::Transform trans = pHandler.getTransformToPartFrame();
  tf::Transform trans = trans_;

#ifdef POINT_CLOUD
  boost::mutex::scoped_lock updateLock(updateModelMutex);	
  basic_cloud_ptr1->points.clear();
  basic_cloud_ptr2->points.clear();
  for (int j = 0; j < pFilter_.numParticles; j++ ) {
    pcl::PointXYZ basic_point;
    basic_point.x = particles[j][0] * 2;
    basic_point.y = particles[j][1] * 2;
    basic_point.z = particles[j][2] * 2;
    basic_cloud_ptr1->points.push_back(basic_point);
    basic_point.x = particles[j][3] * 2;
    basic_point.y = particles[j][4] * 2;
    basic_point.z = particles[j][5] * 2;
    basic_cloud_ptr2->points.push_back(basic_point);
  }
  basic_cloud_ptr1->width = (int) basic_cloud_ptr1->points.size ();
  basic_cloud_ptr1->height = 1;
  basic_cloud_ptr2->width = (int) basic_cloud_ptr2->points.size ();
  basic_cloud_ptr2->height = 1;
  update = true;
  updateLock.unlock();
#endif

  cspace particles_est_stat;
  cspace particles_est;
  pFilter_.estimateGaussian(particles_est, particles_est_stat);
  geometry_msgs::PoseArray poseArray;
  for(int i=0; i<50; i++){
    tf::Pose pose = poseAt(particles[i]);
    geometry_msgs::Pose pose_msg;
    tf::poseTFToMsg(trans*pose, pose_msg);
    poseArray.poses.push_back(pose_msg);
    // ROS_INFO("Pose %d: %f, %f, %f", i, poseArray.poses[i].position.x,
    // 	     poseArray.poses[i].position.y, 
    // 	     poseArray.poses[i].position.z);

  }
  return poseArray;
}

#ifdef POINT_CLOUD
/*
 * Visualize particles
 */
void visualize()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1 = 0;
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,1,1, "sample cloud1", v1);
  viewer->addText("x y z", 15, 120, 20, 1, 1, 1, "v1 text", v1);

  int v2 = 1;
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0, 0, 0, v2);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,1,1, "sample cloud2", v2);
  viewer->addText("roll pitch yaw", 15, 120, 20, 1, 1, 1, "v2 text", v2);
  viewer->addCoordinateSystem (1.0);
  try {
    while (!viewer->wasStopped ())
      {
	boost::mutex::scoped_lock updateLock(updateModelMutex);
	if(update)
	  {
	    if(!viewer->updatePointCloud<pcl::PointXYZ>(basic_cloud_ptr1, "sample cloud1"))
	      viewer->addPointCloud<pcl::PointXYZ>(basic_cloud_ptr1, "sample cloud1", v1);
	    if(!viewer->updatePointCloud<pcl::PointXYZ>(basic_cloud_ptr2, "sample cloud2"))
	      viewer->addPointCloud<pcl::PointXYZ>(basic_cloud_ptr2, "sample cloud2", v2);
	    update = false;
	  }
	updateLock.unlock();
	viewer->spinOnce (100);
      }
  } catch(boost::thread_interrupted&)
    {
      viewer->close();
      return;
    }
}

void PFilterRos::initPointCloud(){
  std::vector<cspace> particles;
  pFilter_.getAllParticles(particles);
  boost::mutex::scoped_lock updateLock(updateModelMutex);	
  basic_cloud_ptr1->points.clear();
  basic_cloud_ptr2->points.clear();
  for (int j = 0; j < NUM_PARTICLES; j++ ) {
    pcl::PointXYZ basic_point;
    basic_point.x = particles[j][0] * 2;
    basic_point.y = particles[j][1] * 2;
    basic_point.z = particles[j][2] * 2;
    basic_cloud_ptr1->points.push_back(basic_point);
    basic_point.x = particles[j][3] * 2;
    basic_point.y = particles[j][4] * 2;
    basic_point.z = particles[j][5] * 2;
    basic_cloud_ptr2->points.push_back(basic_point);
  }
  basic_cloud_ptr1->width = (int) basic_cloud_ptr1->points.size ();
  basic_cloud_ptr1->height = 1;
  basic_cloud_ptr2->width = (int) basic_cloud_ptr2->points.size ();
  basic_cloud_ptr2->height = 1;
  update = true;
  updateLock.unlock();
}

#endif








PFilterRos::PFilterRos(int n_particles, cspace b_init[2]) :
  pFilter_(n_particles, b_init, 0.001, 0.0035, 0.0001, 0.00),
  num_voxels{200, 200, 200}//,
// particleFilter (int n_particles, cspace b_init[2], 
// 		   double Xstd_ob=0.0001 (measurement error), 
//                 double Xstd_tran=0.0025, (gausian kernel sampling std
// 		   double Xstd_scatter=0.0001, (scatter particles a little before
//                                              computing mean of distance transform)
//                 double R=0.01) (probe radius);



{

  sub_request_particles = n.subscribe("request_particles", 1, &PFilterRos::sendParticles, this);
  srv_add_obs = n.advertiseService("particle_filter_add", &PFilterRos::addObs, this);
  srv_est_gaus = n.advertiseService("particle_filter_estimate_gaussian", &PFilterRos::estGaus, this);
  pub_particles = n.advertise<geometry_msgs::PoseArray>("particles_from_filter", 5);

  // ROS_INFO("Loading Boeing Particle Filter");

  std::string stlFilePath;
  if(!n.getParam("localization_object_filepath", stlFilePath)){
    ROS_INFO("Failed to get param: localization_object_filepath");
  }
  rel.parseRelationshipsFile(n);

  getMesh(stlFilePath, mesh);


  // ROS_INFO("start create dist_transform");
  dist_transform = new distanceTransform(num_voxels);

  tf::TransformListener tf_listener_;
  std::string name;
  if(!n.getParam("localization_object", name)){
    ROS_INFO("Failed to get param: localization_object");
  }
  
  // ROS_INFO("Lookuping up transform %s", name.c_str());
  // tf_listener_.waitForTransform(name, "/my_frame", ros::Time(0), ros::Duration(10.0));
  // // ROS_INFO("Transform %s found", name.c_str());
  // tf_listener_.lookupTransform(name, "/my_frame", ros::Time(0), trans_);

  //TF Listeners were causing problems. This ignores the transform and just uses the identity
  trans_.setIdentity();
  // ROS_INFO("Transform %s lookuped up", name.c_str());

#ifdef POINT_CLOUD
  initPointCloud();
#endif

  ros::Duration(1.0).sleep(); //Not sure why this is needed, but sometimes the transofrm isn't available without it

  pub_particles.publish(getParticlePoseArray());
}



int main(int argc, char **argv)
{
#ifdef POINT_CLOUD
  update = false;
  boost::thread workerThread(visualize);
#endif
  ros::init(argc, argv, "pfilter_wrapper");
  ros::NodeHandle n;

  // ROS_INFO("Testing particle filter");
  
  cspace b_Xprior[2];	
  computeInitialDistribution(b_Xprior, n);
  PFilterRos pFilterRos(NUM_PARTICLES, b_Xprior);
  
  ros::spin();
#ifdef POINT_CLOUD
  workerThread.interrupt();
  workerThread.join();
#endif
}




