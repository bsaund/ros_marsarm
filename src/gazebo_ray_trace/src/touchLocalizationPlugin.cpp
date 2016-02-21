/*
 *  Plugin for gazebo for performing ray tracing in the world
 *   via a ros service
 */
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
#include "rayTracePluginUtils.h"


namespace gazebo
{

  class TouchLocalizationPlugin : public WorldPlugin
  {
  private:
    ros::NodeHandle* rosnode_;

    ros::Subscriber particle_sub;
    geometry_msgs::PoseArray particles_;

    RayTracePluginUtils ray_tracer_;




    struct RayIntersection {
      geometry_msgs::Point start;
      geometry_msgs::Point end;
      std::vector<double> dist;
    };
      


  public:
    TouchLocalizationPlugin() : WorldPlugin()
    {
    }

    
    void updateParticles(const geometry_msgs::PoseArray p)
    {
      particles_ = p;
      ray_tracer_.setParticles(p);
    }

    /**
     *  Load is called by Gazebo to initilize the plugin. 
     */
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      sleep(2);
      if(!ros::isInitialized()){
      	ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin");
      }
      std::string robot_namespace = "gazebo_simulation";
      rosnode_ = new ros::NodeHandle(robot_namespace);


      ROS_INFO("Setting up ray tracing");
      ray_tracer_.world_ = _world;      
      ray_tracer_.rosnode_ = rosnode_;
      
      
      advertiseServices();
      particle_sub = rosnode_->subscribe("/transform_particles", 1000, 
				   &TouchLocalizationPlugin::updateParticles, this);

      ROS_INFO("Ready to ray trace");
	  
    }

    /**
     *  Advertise the ray tracing service.
     */
    void advertiseServices()
    {
      
      rosnode_->setParam("/use_sim_time", false);

      ray_tracer_.advertiseServices();
    }

  };
  GZ_REGISTER_WORLD_PLUGIN(TouchLocalizationPlugin);
}





	      

