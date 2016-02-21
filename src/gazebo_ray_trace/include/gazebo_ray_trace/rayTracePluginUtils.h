#ifndef __RAYTRACEPLUGINUTILS__
#define __RAYTRACEPLUGINUTILS__

#include "ros/ros.h"
#include "gazebo_ray_trace/RayTrace.h"
#include "gazebo_ray_trace/RayTraceEachParticle.h"
#include "gazebo_ray_trace/RayTraceEntropy.h"
#include "gazebo_ray_trace/RayTraceCylinder.h"
#include <math.h>
#include <gazebo/common/Plugin.hh>
#include "gazebo/physics/physics.hh"
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <boost/thread.hpp>
#include "calcEntropy.h"

using namespace gazebo;
class RayTracePluginUtils{
 private:

  ros::ServiceServer srv_;
  ros::ServiceServer srv_each_;
  ros::ServiceServer srv_cylinder_;
  ros::ServiceServer srv_condDisEntropy_;


  geometry_msgs::PoseArray particles_;

  struct RayIntersection {
    geometry_msgs::Point start;
    geometry_msgs::Point end;
    std::vector<double> dist;
  };
  std::vector<RayIntersection> rayTraceCylinderHelper(geometry_msgs::Point start_msg, 
						      geometry_msgs::Point end_msg, 
						      double err);


 public:
  ros::NodeHandle* rosnode_;
  physics::WorldPtr world_;
  void advertiseServices();
    
  double rayTrace(math::Vector3 start, math::Vector3 end, gazebo::physics::RayShapePtr ray_);
  math::Vector3 vectorTFToGazebo(const tf::Vector3 t);

  std::vector<double> rayTraceAllParticles(tf::Point start, 
					   tf::Point end);
  std::vector<double> rayTraceAllParticles(geometry_msgs::Point startm, 
					   geometry_msgs::Point endm);
  bool rayTraceEachParticle(gazebo_ray_trace::RayTraceEachParticle::Request &req,
			    gazebo_ray_trace::RayTraceEachParticle::Response &resp);
  bool rayTraceEntropy(gazebo_ray_trace::RayTraceEntropy::Request &req,
		       gazebo_ray_trace::RayTraceEntropy::Response &resp);
  bool rayTrace(gazebo_ray_trace::RayTrace::Request &req,
		gazebo_ray_trace::RayTrace::Response &resp);

  bool rayTraceCylinder(gazebo_ray_trace::RayTraceCylinder::Request &req,
			gazebo_ray_trace::RayTraceCylinder::Response &resp);

  bool rayTraceCondDisEntropy(gazebo_ray_trace::RayTraceCylinder::Request &req,
			      gazebo_ray_trace::RayTraceCylinder::Response &resp);

  void setParticles(geometry_msgs::PoseArray p);





  std::vector<tf::Vector3> getOrthogonalBasis(tf::Vector3 ray);





};


#endif
