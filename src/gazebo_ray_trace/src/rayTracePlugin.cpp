#include "ros/ros.h"
#include "gazebo_ray_trace/RayTrace.h"
#include "gazebo_ray_trace/RayTraceEachParticle.h"
#include "gazebo_ray_trace/RayTraceEntropy.h"
#include <math.h>
#include <gazebo/common/Plugin.hh>
#include "gazebo/physics/physics.hh"
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <boost/thread.hpp>
#include "calcEntropy.h"



namespace gazebo
{

  class RayTracer : public WorldPlugin
  {
  private:
    boost::thread deferred_load_thread_;
    ros::NodeHandle* rosnode_;
    ros::ServiceServer srv_;
    ros::ServiceServer srv_each_;
    ros::ServiceServer srv_entropy_;

    ros::Subscriber particle_sub;
    geometry_msgs::PoseArray particles_;


    physics::WorldPtr world_;


  public:
    RayTracer() : WorldPlugin()
    {
    }

    void updateParticles(const geometry_msgs::PoseArray p)
    {
      // ROS_INFO("Particles updated");
      particles_ = p;

    }

    
    /**
     *  Performs ray tracing of a ros request in the loaded world.
     */
    bool rayTrace(gazebo_ray_trace::RayTrace::Request &req,
    	      gazebo_ray_trace::RayTrace::Response &resp)
    {
      math::Vector3 start, end;

      start.x = req.start.x;
      start.y = req.start.y;
      start.z = req.start.z;

      end.x = req.end.x;
      end.y = req.end.y;
      end.z = req.end.z;

      gazebo::physics::RayShapePtr ray_;
      gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine(); 
      engine->InitForThread();

      ray_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
      	(engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

      resp.dist = rayTrace(start, end, ray_);

      ROS_INFO("Traced ray and responded with distance: %f", resp.dist);
      return true;
    }

    /**
     *  Ray traces each particle and return the entropy of the distribution
     */
    bool rayTraceEntropy(gazebo_ray_trace::RayTraceEntropy::Request &req,
		    gazebo_ray_trace::RayTraceEntropy::Response &resp)
    {
      std::vector<double> dist = rayTraceAllParticles(req.start, req.end);

      resp.entropy = CalcEntropy::calcEntropy(dist);
      return true;
    }
    

    /**
     *  Ray traces each particles
     */
    bool rayTraceEachParticle(gazebo_ray_trace::RayTraceEachParticle::Request &req,
    	      gazebo_ray_trace::RayTraceEachParticle::Response &resp)
    {
      resp.dist = rayTraceAllParticles(req.start, req.end);
      ROS_INFO("Traced ray and responded with %d different distances", resp.dist.size());
      return true;
    }

    std::vector<double> rayTraceAllParticles(geometry_msgs::Point startm, 
					     geometry_msgs::Point endm)
    {

      ROS_INFO("Starting ray trace each particle");
      // ROS_INFO("Number of particle %d", particles_.poses.size());
      tf::Vector3 start, end;

      start.setX(startm.x);
      start.setY(startm.y);
      start.setZ(startm.z);

      end.setX(endm.x);
      end.setY(endm.y);
      end.setZ(endm.z);

      tf::Transform trans;

      gazebo::physics::RayShapePtr ray_;
      gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine(); 
      engine->InitForThread();

      ray_ = boost::dynamic_pointer_cast<gazebo::physics::RayShape>
      	(engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

	   
      std::vector<double> dist;
      dist.resize(particles_.poses.size());
      for(int i=0; i<particles_.poses.size(); i++){

	tf::Vector3 v = tf::Vector3(particles_.poses[i].position.x,
				    particles_.poses[i].position.y,
				    particles_.poses[i].position.z);
	trans.setOrigin(v);
	tf::Quaternion q;
	tf::quaternionMsgToTF(particles_.poses[i].orientation, q);
	trans.setRotation(q);
	trans = trans.inverse();
	



	dist[i] = rayTrace(
				     vectorTFToGazebo(trans*start), 
				     vectorTFToGazebo(trans*end), 
				     ray_);
	// ROS_INFO("Finished Ray Trace, distance of: %f", resp.dist[i]);
      }


      return dist;
    }

    math::Vector3 vectorTFToGazebo(const tf::Vector3 t)
    {
      math::Vector3 v;
      v.x = t.getX();
      v.y = t.getY();
      v.z = t.getZ();
      return v;
    }

    /**
     *  Performs ray tracing on the loaded world, returning the distace to intersection
     */
    double rayTrace(math::Vector3 start, math::Vector3 end, gazebo::physics::RayShapePtr ray_){
      double dist;
      std::string entityName;
      // ROS_INFO("Ray is from %f,%f,%f to %f,%f,%f", start.x, start.y, start.z,
      // 	       end.x, end.y, end.z);

      ray_->SetPoints(start, end);
      ray_->GetIntersection(dist, entityName);

      return dist;
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){

      if(!ros::isInitialized()){
      	ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin");
      }

      ROS_INFO("Setting up ray tracing");
      world_ = _world;      
      
      this->advertiseServices();
      this->particle_sub = this->rosnode_->subscribe("/transform_particles", 1000, 
				   &RayTracer::updateParticles, this);

      ROS_INFO("Ready to ray trace");
	  
    }

    /**
     *  Advertise the ray tracing service.
     */
    void advertiseServices()
    {
      std::string robot_namespace = "gazebo_simulation";
      this->rosnode_ = new ros::NodeHandle(robot_namespace);
      
      this->rosnode_->setParam("/use_sim_time", false);
      //This is how static services can be advertised
      // this->srv_ = this->rosnode_->advertiseService("ray_trace", &RayTracer::rayTrace);

      //boost is needed to bind member function rayTrace
      ros::AdvertiseServiceOptions ray_trace_srv = 
      	ros::AdvertiseServiceOptions::create<gazebo_ray_trace::RayTrace>
      	("ray_trace",
      	 boost::bind(&RayTracer::rayTrace, this, _1, _2),
      	 ros::VoidPtr(), NULL);

      this->srv_ = this->rosnode_->advertiseService(ray_trace_srv);


      ros::AdvertiseServiceOptions ray_trace_srv_2 = 
      	ros::AdvertiseServiceOptions::create<gazebo_ray_trace::RayTraceEachParticle>
      	("ray_trace_each_particle",
      	 boost::bind(&RayTracer::rayTraceEachParticle, this, _1, _2),
      	 ros::VoidPtr(), NULL);

      this->srv_each_ = this->rosnode_->advertiseService(ray_trace_srv_2);

      ros::AdvertiseServiceOptions ray_trace_entropy = 
      	ros::AdvertiseServiceOptions::create<gazebo_ray_trace::RayTraceEntropy>
      	("ray_trace_entropy",
      	 boost::bind(&RayTracer::rayTraceEntropy, this, _1, _2),
      	 ros::VoidPtr(), NULL);

      this->srv_entropy_ = this->rosnode_->advertiseService(ray_trace_entropy);

    }

  };
  GZ_REGISTER_WORLD_PLUGIN(RayTracer);
}





	      

