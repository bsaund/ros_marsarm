/**
 *  This is a rosnode that randomly chooses touch points, 
 *    performs simulated measurements, and updates the particle filter.
 *   This node is made to work with a particle filter node, a node that 
 *    publishes visualization messages, and RViz.
 */
#include <ros/ros.h>
#include <ros/console.h>
#include "particle_filter/AddObservation.h"
#include "custom_ray_trace/rayTracePlotter.h"
#include "custom_ray_trace/rayTracer.h"
#include "particle_filter/relationships.h"
#include "particle_filter/transformDistribution.h"
#include "simulateMeasurement.h"
#include <tf/tf.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulating_measurement");
  ros::NodeHandle n;
  RayTracePlotter plt;

  std::random_device rd;
  std::vector<double> measurement;

  if(!n.getParam("measurement", measurement)){
    ROS_INFO("Failed to get param: measurement");
    return -1;
  }

  tf::Point start(measurement[0], measurement[1], measurement[2]);
  tf::Point end(measurement[3], measurement[4], measurement[5]);
  Ray measurementRay(start,end);

  plt.deleteAll();
  plt.plotRay(measurementRay); 


  std::vector<RayTracer*> rayts = getAllRayTracers();
	
  PartRelationships rel(n);
  // CalcEntropy::ConfigDist distsToParticles;
  // std::vector<CalcEntropy::ConfigDist> distsToParticles;
  // for(int i=0; i<100; i++){
  //   cspace tfp= rel["top_datum"]->sampleTransform();
  //   // ROS_INFO("TF z: %f", tfp[2]); 
  //   tf::Quaternion q = tf::createQuaternionFromRPY(tfp[3], tfp[4], tfp[5]);
  //   tf::Transform tf(q, tf::Vector3(tfp[0], tfp[1], tfp[2]));
  //   std::vector<CalcEntropy::ConfigDist> tmp;
  //   plt.traceCylinderAllParticles(Ray(tf*start, tf*end), 0.002, tmp);
  //   distsToParticles.insert(distsToParticles.end(),
  // 			    tmp.begin(), tmp.end());
  // }

  ParticleHandler pHand;
  double ig = getIG(measurementRay, rayts, rel, 
		    pHand.getParticles(), 0.002, 0.01);

  // ROS_INFO("MeasurementSize %d", distsToParticles.size());

  std::ostringstream oss;

  // ig = plt.getIG(measurementRay, 0.01, 0.005);
  // ig = CalcEntropy::calcIG(distsToParticles, 
  // 			   0.01, plt.particleHandler.getNumSubsetParticles());
  ig = ig>0 ? ig : 0;
  oss << ig;
  plt.labelRay(start, oss.str());
  // int hitPart = simulateMeasurement(measurementRay, plt, srv_add, 0.001);

  return 0;
}
