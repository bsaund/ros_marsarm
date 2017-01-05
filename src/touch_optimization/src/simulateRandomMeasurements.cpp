#include <ros/ros.h>
#include <ros/console.h>
#include "particle_filter/AddObservation.h"
#include "custom_ray_trace/rayTracePlotter.h"
#include "custom_ray_trace/rayTracer.h"
#include "particle_filter/relationships.h"
#include "particle_filter/transformDistribution.h"
#include <tf/tf.h>


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
      std::uniform_real_distribution<> dis(-1,1);
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
  ROS_INFO("");
  ROS_INFO("Took %f seconds", (ros::Time::now() - start).toSec());
  ROS_INFO("Sampled %d rays", count);
  ROS_INFO("Hit with min dist %f", minD);
  return measurementRay;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulating_measurement");
  ros::NodeHandle n;
  RayTracePlotter plt;
  std::vector<std::string> pieces = {"top_datum", "right_datum", "J1_section",
				    "bottom_section", "back_datum"};
  std::vector<RayTracer*> rayts;

  // rayts.resize(2);



  for(std::string &piece:pieces){
    RayTracer* rayt = new RayTracer(piece);
    // rayt->loadMesh(piece);
    rayts.push_back(rayt);
  }

  Ray mRay = getIntersectingRay(rayts);
  plt.deleteAll();
  plt.plotRay(mRay);
  ros::Duration(1).sleep();
 
}
