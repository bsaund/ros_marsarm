#include <iostream>
#include <math.h>
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"

#include "ros/ros.h"
#include "gazebo_ray_trace/RayTrace.h"

namespace gazebo
{


  class CollisionMapCreator : public WorldPlugin
  {
    physics::WorldPtr world;
    // gazebo::physics::RayShapePtr ray_;
    ros::NodeHandle* rosnode_;
    ros::ServiceServer srv_;

  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      world = _world;
      // gazebo::physics::PhysicsEnginePtr engine = _world->GetPhysicsEngine(); 
      // engine->InitForThread();
      // ROS_INFO("Using physics engine %s", engine->GetType().c_str());
      // ray_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
      // 	(engine->CreateShape("ray", gazebo::physics::CollisionPtr()));
      advertiseServices();
    }

  public: void advertiseServices()
    {
      if(!ros::isInitialized()){
	ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin");
      }
      std::string robot_namespace = "gazebo_simulation";
      this->rosnode_ = new ros::NodeHandle(robot_namespace);

      //This is how static services can be advertised
      // this->srv_ = this->rosnode_->advertiseService("ray_trace", &RayTracer::rayTrace);

      //boost is needed to bind member function rayTrace
      ros::AdvertiseServiceOptions ray_trace_srv = 
	ros::AdvertiseServiceOptions::create<gazebo_ray_trace::RayTrace>
	("ray_trace",
	 boost::bind(&CollisionMapCreator::rayTrace, this, _1, _2),
	 ros::VoidPtr(), NULL);

      this->srv_ = this->rosnode_->advertiseService(ray_trace_srv);
    }

  public: bool rayTrace(gazebo_ray_trace::RayTrace::Request &req,
    	      gazebo_ray_trace::RayTrace::Response &resp)
    {
      math::Vector3 start, end;
      std::string entityName;

      double dist;
      start.x = req.start.x;
      start.y = req.start.y;
      start.z = req.start.z;

      end.x = req.end.x;
      end.y = req.end.y;
      end.z = req.end.z;

      gazebo::physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
      engine->InitForThread();
      gazebo::physics::RayShapePtr ray_ =
      	boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      							       engine->CreateShape("ray", gazebo::physics::CollisionPtr()));


      ROS_INFO("Starting Ray Trace");
      ray_->SetPoints(start, end);
      ROS_INFO("Set Ray as vector");
      ray_->Update();

      double len = ray_->GetLength();
      ROS_INFO("Length is: %f", len);
      
      ray_->GetIntersection(dist, entityName);
      ROS_INFO("Got intersection");
      resp.dist = dist;
      ROS_INFO("Traced ray");
      ROS_INFO("Traced ray and responded with distance: %f", resp.dist);
      ROS_INFO("Intersected with %s", entityName.c_str());
      return true;
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(CollisionMapCreator)
}
