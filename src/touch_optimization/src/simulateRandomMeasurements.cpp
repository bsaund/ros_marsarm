#include <ros/ros.h>
#include <ros/console.h>
#include "particle_filter/AddObservation.h"
#include "custom_ray_trace/rayTracePlotter.h"
#include "custom_ray_trace/rayTracer.h"
#include "particle_filter/relationships.h"
#include "particle_filter/transformDistribution.h"
#include <tf/tf.h>
#include "simulateMeasurement.h"




Ray getIntersectingRay(std::vector<RayTracer*> &rayts){
  std::random_device rd;
  std::mt19937 gen(rd());
  std::vector<double> measurement;
  Ray measurementRay;
  ros::Time start = ros::Time::now();

  bool hit = false;
  double d;
  int count = 0;
  double minD=std::numeric_limits<double>::max();

  while(!hit){
      std::uniform_real_distribution<> dis(-1,1.5);
      tf::Point start(dis(gen), dis(gen), dis(gen));
      tf::Point dir(dis(gen), dis(gen), dis(gen));
      measurementRay = Ray(start, dir.normalize() + start);
      
      int i=0;
      for(auto &rayt : rayts){
	hit = hit || rayt->traceRay(measurementRay, d);
	minD = std::min(minD, d);
	i++;
      }
      count++;
  }
  // ROS_INFO("");
  // ROS_INFO("Took %f seconds", (ros::Time::now() - start).toSec());
  // ROS_INFO("Sampled %d rays", count);
  // ROS_INFO("Hit with min dist %f", minD);
  return measurementRay;
}


Ray getBestRandomRay(std::vector<RayTracer*> &rayts, RayTracePlotter &plt, PartRelationships &rel,
		     std::vector<tf::Transform> referenceParticles,
		     bool printDebug = false){

  double bestIg = 0;
  Ray bestRay;
  
  if(printDebug){
    std::string ns = ros::this_node::getNamespace();
    ROS_INFO("Working in namespace %s", ns.c_str());
  }

  // for(int i=0; i<rayts.size(); i++){
  //   std::cout << rayts[i]->getName();
  // }
  // std::cout << "\n";

  for(int i=0; i<100; i++){
    Ray ray = getIntersectingRay(rayts);

    double ig = getIG(ray, rayts, rel, referenceParticles, 0.002, 0.01, printDebug);
    std::ostringstream oss;
    ig = ig>0 ? ig : 0;
    oss << ig;


    if(false){
    // if(printDebug){
      plt.deleteAll();
      plt.plotRay(ray, 1);
      plt.labelRay(ray.start, oss.str(), 1);
      if(std::cin.get() == 'c'){;
	break;
      }
    }

    if(bestIg < ig){
      bestIg = ig;
      bestRay = ray;
    }
  }
  
  if(printDebug){
    ROS_INFO("Best Ray: ");
    getIG(bestRay, rayts, rel, referenceParticles, 0.002, 0.01, printDebug);
  }

  return bestRay;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulating_measurement");
  bool debug = true;

  ROS_INFO("TEST 1");
  ros::NodeHandle n;
  RayTracePlotter plt;
  PartRelationships rel(n);

  std::vector<RayTracer*> rayts = getAllRayTracers();
  
  ParticleHandler pHand("goal_hole");

  ros::ServiceClient srv_add = 
    n.serviceClient<particle_filter::AddObservation>("/observation_distributor");


  // Ray mRay = getIntersectingRay(rayts);
  for(int i=0; i<10; i++){
    Ray mRay = getBestRandomRay(rayts, plt, rel, pHand.getParticles(), debug);

    plt.deleteAll();
    plt.plotRay(mRay);
    ros::Duration(.1).sleep();
    plt.plotRay(mRay); //Message is sometimes lost
    ros::Duration(.1).sleep();
    plt.plotRay(mRay); //3rd time's a charm
    ROS_INFO("Simulating...");
    simOnAllParts(mRay, rayts, srv_add, 0.001);
    ROS_INFO("finished simulating");
    ros::Duration(1).sleep();
  }
}
