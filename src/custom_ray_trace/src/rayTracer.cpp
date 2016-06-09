#include "rayTracer.h"
#include "stlParser.h"
#include <std_msgs/Empty.h>


/*
 *************************
 ***     Ray           ***
 *************************
 */
 
Ray::Ray(tf::Point start_, tf::Point end_)
{
  start = start_;
  end = end_;
}

tf::Vector3 Ray::getDirection()
{
  return (end-start).normalize();
}
  
Ray Ray::transform(tf::Transform trans)
{
  start = trans*start;
  end =   trans*end;
  return *this;
}

Ray Ray::getTransformed(tf::Transform trans)
{
  Ray newRay(start, end);
  newRay.transform(trans);
  return newRay;
}


/*
 **************************
 ***  Particle Handler  ***
 **************************
 */

ParticleHandler::ParticleHandler()
{
  particlesInitialized = false;
  tf_listener_.waitForTransform("/my_frame", "/particle_frame", ros::Time(0), ros::Duration(10.0));
  tf_listener_.lookupTransform("/particle_frame", "/my_frame", ros::Time(0), trans_);
  particleSub = rosnode.subscribe("/transform_particles", 1000, 
				     &ParticleHandler::setParticles, this);
  requestParticlesPub = rosnode.advertise<std_msgs::Empty>("/request_particles", 5);
}

tf::StampedTransform ParticleHandler::getTransformToPartFrame()
{
  //TODO: Update when new transform becomes available
  return trans_;
}

void ParticleHandler::setParticles(geometry_msgs::PoseArray p)
{
  particles.resize(p.poses.size());
  
  for(int i=0; i<p.poses.size(); i++){

    tf::Vector3 v = tf::Vector3(p.poses[i].position.x,
				p.poses[i].position.y,
				p.poses[i].position.z);
    particles[i].setOrigin(v);

    tf::Quaternion q;
    tf::quaternionMsgToTF(p.poses[i].orientation, q);
    particles[i].setRotation(q);
    particles[i] = particles[i].inverse();
  }

  particlesInitialized = true;
}


std::vector<tf::Transform> ParticleHandler::getParticles()
{
  if(!particlesInitialized){
    int count = 0;

    while(!particlesInitialized){
      requestParticlesPub.publish(std_msgs::Empty());
      ros::spinOnce();
      ros::Duration(.1).sleep();
      ROS_INFO_THROTTLE(10, "Requesting Particles");
      count++;
      if(count > 100){
	ROS_INFO("Never Received Particles");
	throw "Particles never received";
      }
    }
  }
  return particles;
}









/*
 ********************************************
 ***********     |Ray Tracer|    ************
 ********************************************
 */


RayTracer::RayTracer()
{
  loadMesh();
}



bool RayTracer::loadMesh(){
  std::string stlFilePath;
  if(!n_.getParam("/localization_object_filepath", stlFilePath)){
    ROS_INFO("Failed to get param");
  }

  mesh = StlParser::importSTL(stlFilePath);
  // StlParser::testFunc();
  // getSurroundingBox();
  // surroundingBox = StlParser::getSurroundingBox(mesh);
  ROS_INFO("survived");
}


/*
 * Returns true if ray intersections with part and sets the distToPart
 */
bool RayTracer::tracePartFrameRay(Ray ray, double &distToPart)
{
  double startArr[3] = {ray.start.getX(), ray.start.getY(), ray.start.getZ()};

  tf::Vector3 dir = ray.getDirection();
  double dirArr[3] = {dir.getX(), dir.getY(), dir.getZ()};

  return getIntersection(mesh, startArr, dirArr, distToPart);
}


bool RayTracer::traceRay(Ray ray, double &distToPart){
  transformRayToPartFrame(ray);
  return tracePartFrameRay(ray, distToPart);
}


/*
 *  Traces a ray (specified in world frame) on all particles
 *  Returns true if at least 1 ray intersected the part
 */
bool RayTracer::traceAllParticles(Ray ray, std::vector<double> &distToPart)
{
  transformRayToPartFrame(ray);
  std::vector<tf::Transform> particles = particleHandler.getParticles();
  distToPart.resize(particles.size());

  bool hitPart = false;
  for(int i=0; i<particles.size(); i++){
    hitPart = tracePartFrameRay(ray.getTransformed(particles[i]), distToPart[i]) || hitPart;
  }
  return hitPart;
}


void RayTracer::transformRayToPartFrame(Ray &ray)
{
  ray.transform(particleHandler.getTransformToPartFrame());
}


void RayTracer::transformRayToBaseFrame(Ray &ray)
{
  ray.transform(particleHandler.getTransformToPartFrame().inverse());
}

		

