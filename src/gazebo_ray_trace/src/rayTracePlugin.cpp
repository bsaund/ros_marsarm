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
// #include "plotRayUtils.h"



namespace gazebo
{

  class RayTracer : public WorldPlugin
  {
  private:
    boost::thread deferred_load_thread_;
    ros::NodeHandle* rosnode_;
    ros::ServiceServer srv_;
    ros::ServiceServer srv_each_;
    ros::ServiceServer srv_cylinder_;

    ros::Subscriber particle_sub;
    geometry_msgs::PoseArray particles_;

    // PlotRayUtils plt;

    physics::WorldPtr world_;

    struct ConfigDist {
      double dist;
      int id;
    };

    struct RayIntersection {
      geometry_msgs::Point start;
      geometry_msgs::Point end;
      std::vector<double> dist;
    };
      


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
     *  Service for casting many rays in a cylinder to the world
     */
    bool rayTraceCylinder(gazebo_ray_trace::RayTraceCylinder::Request &req,
		    gazebo_ray_trace::RayTraceCylinder::Response &resp)
    {
      std::vector<RayIntersection> rays = rayTraceCylinderHelper(req.start, req.end, req.error_radius);

      resp.rays.resize(rays.size());
      for(int i = 0; i < rays.size(); i++){
      	resp.rays[i].start = rays[i].start;
      	resp.rays[i].end = rays[i].end;
      	resp.rays[i].dist = rays[i].dist;
      }
    }


    /**
     *  Casts many rays in a cylinder to the part and returns a list of the rays 
     *   produced with all the intersections to particles
     */
    std::vector<RayIntersection> rayTraceCylinderHelper(geometry_msgs::Point start_msg, 
					     geometry_msgs::Point end_msg, 
					     double err)
    {
      tf::Vector3 start(start_msg.x, start_msg.y, start_msg.z);
      tf::Vector3 end(end_msg.x, end_msg.y, end_msg.z);
      tf::Vector3 ray = end-start;
      std::vector<tf::Vector3> ray_orthog = getOrthogonalBasis(ray);
      std::vector<RayIntersection> rays;
      int n = 6;
      rays.resize(n);
      
      for(int i = 0; i < n; i ++){
	double theta = 2*3.1415 * i / n;
	tf::Vector3 offset = err * (ray_orthog[0]*sin(theta) + ray_orthog[1]*cos(theta));
	// tf::Vector3 start_tmp = start + offset;
	// tf::Vector3 end_tmp = end + offset;
	
	tf::pointTFToMsg(start + offset, rays[i].start);
	tf::pointTFToMsg(end + offset, rays[i].end);
	
	rays[i].dist = rayTraceAllParticles(rays[i].start, rays[i].end);
      }
      
      return rays;
    }

    /**
     *  Return vector of two Vector3s that form a basis for the space orthogonal to the ray
     */
    std::vector<tf::Vector3> getOrthogonalBasis(tf::Vector3 ray)
    {
      ROS_INFO("Orthogonal Parts of %f, %f, %f", ray.getX(), ray.getY(), ray.getZ());
      ray.normalize();
      std::vector<tf::Vector3> v;

      //Initialize vector on the most orthogonal axis
      switch(ray.closestAxis()){
      case 0:
	v.push_back(tf::Vector3(0,0,1));
	v.push_back(tf::Vector3(0,1,0));
	break;
      case 1:
	v.push_back(tf::Vector3(0,0,1));
	v.push_back(tf::Vector3(1,0,0));
	break;
      case 2:
      default:
	v.push_back(tf::Vector3(0,1,0));
	v.push_back(tf::Vector3(1,0,0));
	break;
      }

      //Recover the pure orthogonal parts
      for(int i = 0; i < 2; i++){
	v[i] = (v[i] - ray * ray.dot(v[i])).normalize();
	ROS_INFO("%f, %f, %f", v[i].getX(), v[i].getY(), v[i].getZ());
      }


      return v;
    }


    
    /**
     *  Service for Ray tracing in the loaded world.
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
     *  Service for Ray tracing each particle and 
     *   return the entropy of the distribution
     */
    bool rayTraceEntropy(gazebo_ray_trace::RayTraceEntropy::Request &req,
		    gazebo_ray_trace::RayTraceEntropy::Response &resp)
    {
      ROS_INFO("Starting ray trace ent");
      std::vector<double> dist = rayTraceAllParticles(req.start, req.end);
      
      // rayTraceCylinder(req.start, req.end, 1.0);

      ROS_INFO("Finished Ray Trace ent");
      
      resp.entropy = CalcEntropy::calcEntropy(dist);
      ROS_INFO("Calculated Entropy");
      return true;
    }

    /**
     *  Service for Ray traces each particles
     */
    bool rayTraceEachParticle(gazebo_ray_trace::RayTraceEachParticle::Request &req,
    	      gazebo_ray_trace::RayTraceEachParticle::Response &resp)
    {
      resp.dist = rayTraceAllParticles(req.start, req.end);
      ROS_INFO("Traced ray and responded with %d different distances", resp.dist.size());
      return true;
    }


    /**
     *  Ray Traces a cylinder
     */
    

    /** 
     *  Ray traces each particle.
     *   Distances are returned as an array.
     *   Order of distances in the array is the same as the order of the particles
     */
    std::vector<double> rayTraceAllParticles(geometry_msgs::Point startm, 
					     geometry_msgs::Point endm)
    {
      tf::Vector3 start, end;

      start.setX(startm.x);
      start.setY(startm.y);
      start.setZ(startm.z);

      end.setX(endm.x);
      end.setY(endm.y);
      end.setZ(endm.z);
      return rayTraceAllParticles(start, end);
    }
    std::vector<double> rayTraceAllParticles(tf::Point start, 
					     tf::Point end)
    {

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
	

	dist[i] = rayTrace(vectorTFToGazebo(trans*start), 
			   vectorTFToGazebo(trans*end), 
			   ray_);
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

      // ros::AdvertiseServiceOptions ray_trace_entropy = 
      // 	ros::AdvertiseServiceOptions::create<gazebo_ray_trace::RayTraceEntropy>
      // 	("ray_trace_entropy",
      // 	 boost::bind(&RayTracer::rayTraceEntropy, this, _1, _2),
      // 	 ros::VoidPtr(), NULL);

      // this->srv_entropy_ = this->rosnode_->advertiseService(ray_trace_entropy);

      ros::AdvertiseServiceOptions ray_trace_cylinder = 
      	ros::AdvertiseServiceOptions::create<gazebo_ray_trace::RayTraceCylinder>
      	("ray_trace_cylinder",
      	 boost::bind(&RayTracer::rayTraceCylinder, this, _1, _2),
      	 ros::VoidPtr(), NULL);

      this->srv_cylinder_ = this->rosnode_->advertiseService(ray_trace_cylinder);

    }

  };
  GZ_REGISTER_WORLD_PLUGIN(RayTracer);
}





	      

