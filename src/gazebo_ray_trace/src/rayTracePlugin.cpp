#include "ros/ros.h"
#include "gazebo_ray_trace/RayTrace.h"
#include <math.h>
#include <gazebo/common/Plugin.hh>
#include "gazebo/physics/physics.hh"

#include <boost/thread.hpp>



namespace gazebo
{

  class RayTracer : public WorldPlugin
  {
  private:
    boost::thread deferred_load_thread_;
    ros::NodeHandle* rosnode_;
    ros::ServiceServer srv_;

    gazebo::physics::RayShapePtr ray_;

  public:
    RayTracer() : WorldPlugin()
    {
    }

    bool dist(gazebo_ray_trace::RayTrace::Request &req,
    	      gazebo_ray_trace::RayTrace::Response &resp)
    {
      resp.dist = sqrt(
    		       pow(req.start.x - req.end.x, 2) + 
    		       pow(req.start.y - req.end.y, 2) + 
    		       pow(req.start.z - req.end.z, 2));

      ROS_INFO("Sending Response From Within Gazebo: %f", resp.dist);
      return true;
    }


    bool rayTrace(gazebo_ray_trace::RayTrace::Request &req,
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


      ray_->SetPoints(start, end);
      ray_->GetIntersection(dist, entityName);

      resp.dist = dist;
      ROS_INFO("Sending Response From Within Gazebo: %ld", (long int)resp.dist);
      return true;
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      // ros::init(argc, argv, "ray_trace_server");
      // ros::NodeHandle n;

      // ros::ServiceServer service = n.advertiseService("ray_trace", &RayTracer.rayTrace);
      // ROS_INFO("Ready to ray trace");
      // ros::spin();

      if(!ros::isInitialized()){
      	ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin");
      }
      
      // this->deferred_load_thread_ = boost::thread(
      // 						  boost::bind(&RayTracer::LoadThread, this));
      this->initStructures(_world);
      this->advertiseServices();
      
      ROS_INFO("Ready to ray trace");
	  
    }

    void advertiseServices()
    {
      std::string robot_namespace = "gazebo_simulation";
      this->rosnode_ = new ros::NodeHandle(robot_namespace);
      // this->srv_ = this->rosnode_->advertiseService("ray_trace", &RayTracer::rayTrace);
      // this->srv_ = this->rosnode_->
      // 	advertiseService("ray_trace", boost::bind(&RayTracer::rayTrace, this, _1, _2));
      ros::AdvertiseServiceOptions ray_trace_srv = 
      	ros::AdvertiseServiceOptions::create<gazebo_ray_trace::RayTrace>
      	("ray_trace",
      	 boost::bind(&RayTracer::rayTrace, this, _1, _2),
      	 ros::VoidPtr(), NULL);
      this->srv_ = this->rosnode_->advertiseService(ray_trace_srv);
    }

    void initStructures(physics::WorldPtr _world)
    {
      gazebo::physics::PhysicsEnginePtr engine = _world->GetPhysicsEngine(); 
      ray_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
	(engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    }



  };
  GZ_REGISTER_WORLD_PLUGIN(RayTracer);
}





	      

