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

    /**
     *  Performs ray tracing of a ros request in the loaded world.
     */
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

      ROS_INFO("Traced ray and responded with distance: %f", resp.dist);
      return true;
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){

      if(!ros::isInitialized()){
      	ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin");
      }
      
      this->initStructures(_world);
      this->advertiseServices();
      
      ROS_INFO("Ready to ray trace");
	  
    }

    /**
     *  Advertise the ray tracing service.
     */
    void advertiseServices()
    {
      std::string robot_namespace = "gazebo_simulation";
      this->rosnode_ = new ros::NodeHandle(robot_namespace);

      // this->srv_ = this->rosnode_->advertiseService("ray_trace", &RayTracer::rayTrace);
      ros::AdvertiseServiceOptions ray_trace_srv = 
      	ros::AdvertiseServiceOptions::create<gazebo_ray_trace::RayTrace>
      	("ray_trace",
      	 boost::bind(&RayTracer::rayTrace, this, _1, _2),
      	 ros::VoidPtr(), NULL);
      this->srv_ = this->rosnode_->advertiseService(ray_trace_srv);
    }

    /** 
     *  Creates a ray using the world's physics engine.
     */
    void initStructures(physics::WorldPtr _world)
    {
      gazebo::physics::PhysicsEnginePtr engine = _world->GetPhysicsEngine(); 
      ray_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
	(engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    }

  };
  GZ_REGISTER_WORLD_PLUGIN(RayTracer);
}





	      

