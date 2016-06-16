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

tf::Vector3 Ray::getDirection() const
{
  return (end-start).normalize();
}
  
Ray Ray::transform(tf::Transform trans)
{
  start = trans*start;
  end =   trans*end;
  return *this;
}

Ray Ray::getTransformed(tf::Transform trans) const
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
  newParticles = true;
  tf_listener_.waitForTransform("/my_frame", "/particle_frame", ros::Time(0), ros::Duration(10.0));
  tf_listener_.lookupTransform("/particle_frame", "/my_frame", ros::Time(0), trans_);
  particleSub = rosnode.subscribe("/particles_from_filter", 1000, 
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

  int num = std::min((int)particles.size(), 50);
  subsetParticles = vector<tf::Transform>(particles);
  subsetParticles.resize(num);

  
  ROS_INFO("Subset Particles Size %d", subsetParticles.size());
  
  particlesInitialized = true;
  newParticles = true;
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

std::vector<tf::Transform> ParticleHandler::getParticleSubset()
{
  if(!particlesInitialized)
    getParticles();
  return subsetParticles;
}

int ParticleHandler::getNumParticles()
{
  if(!particlesInitialized)
    getParticles();
  return particles.size();
}

int ParticleHandler::getNumSubsetParticles()
{
  if(!particlesInitialized)
    getParticles();
  return subsetParticles.size();
}


bool ParticleHandler::theseAreNewParticles(){
  bool tmp = newParticles;
  newParticles = false;
  return tmp;
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

  mesh = stl::importSTL(stlFilePath);
  surroundingBox = stl::getSurroundingBox(mesh);
}

stl::Mesh RayTracer::getBoxAroundAllParticles(stl::Mesh mesh)
{
  stl::Mesh allMesh;
  std::vector<tf::Transform> particles = particleHandler.getParticleSubset();

  for(tf::Transform particle : particles){
    stl::combineMesh(allMesh, stl::transformMesh(mesh, particle.inverse()));
  }
  return stl::getSurroundingBox(allMesh);
}


/*
 *   Casts "ray" onto "mesh", set the distance, and returns true if intersection happened
 *    sets distance to 1000 if no intersection
 */
bool RayTracer::traceRay(const stl::Mesh &mesh, const Ray &ray, double &distToPart)
{
  array<double,3> startArr = {ray.start.getX(), ray.start.getY(), ray.start.getZ()};
  tf::Vector3 dir = ray.getDirection();
  array<double,3> dirArr = {dir.getX(), dir.getY(), dir.getZ()};
  distToPart = 1000;

  return getIntersection(mesh, startArr, dirArr, distToPart);
}


/*
 * Returns true if ray intersections with part and sets the distToPart
 */
bool RayTracer::tracePartFrameRay(const Ray &ray, double &distToPart, bool quick)
{
  distToPart = 1000;

  //Quick check to see if ray has a chance of hitting any particle
  double tmp;
  if(quick && !traceRay(surroundingBox, ray, tmp))
     return false;
  return traceRay(mesh, ray, distToPart);
}


bool RayTracer::traceRay(Ray ray, double &distToPart){
  transformRayToPartFrame(ray);
  return tracePartFrameRay(ray, distToPart);
}


/*
 *  Traces a ray (specified in world frame) on all particles
 *  Returns true if at least 1 ray intersected the part
 */
bool RayTracer::traceAllParticles(Ray ray, std::vector<double> &distToPart, bool quick)
{
  transformRayToPartFrame(ray);
  std::vector<tf::Transform> particles = particleHandler.getParticleSubset();

  //Quick check to see if ray even has a chance of hitting any particle
  if(particleHandler.theseAreNewParticles()){
    surroundingBoxAllParticles = getBoxAroundAllParticles(mesh);
  }
  
  double tmp;
  if(!traceRay(surroundingBoxAllParticles, ray, tmp))
    return false;

  distToPart.resize(particles.size());

  bool hitPart = false;
  for(int i=0; i<particles.size(); i++){
    hitPart = tracePartFrameRay(ray.getTransformed(particles[i]), distToPart[i], quick) || hitPart;
  }
  return hitPart;
}

double RayTracer::getIG(Ray ray, double radialErr, double distErr)
{
  vector<CalcEntropy::ConfigDist> distsToParticles;
  if(!traceCylinderAllParticles(ray, radialErr, distsToParticles))
     return 0;
  return CalcEntropy::calcIG(distsToParticles, distErr, particleHandler.getNumSubsetParticles());
}

/*
 *  Returns the possible distances receives from casting a cylinder of rays
 *  
 *  Returns true if at least one ray hit the part
 */
bool RayTracer::traceCylinderAllParticles(Ray ray, double radius, 
					  vector<CalcEntropy::ConfigDist> &distsToPart)
{
  std::vector<tf::Vector3> ray_orthog = getOrthogonalBasis(ray.getDirection());
  int n = 12;
  
  bool hitPart = false;

  for(int i = 0; i < n; i ++){
    double theta = 2*3.1415 * i / n;
    tf::Vector3 offset = radius * (ray_orthog[0]*sin(theta) + ray_orthog[1]*cos(theta));

    Ray cylinderRay(ray.start + offset, ray.end+offset);
    vector<double> distsTmp;
    
    hitPart = traceAllParticles(cylinderRay, distsTmp) || hitPart;

    for(int pNumber = 0; pNumber<distsTmp.size(); pNumber++){
      CalcEntropy::ConfigDist cDist;
      cDist.id = pNumber;
      cDist.dist = distsTmp[pNumber];
      distsToPart.push_back(cDist);
    }
  }
  return hitPart;
}

/**
 *  Return vector of two Vector3s that form a basis for the space orthogonal to the ray
 */
std::vector<tf::Vector3> RayTracer::getOrthogonalBasis(tf::Vector3 dir)
{
  // ROS_INFO("Orthogonal Parts of %f, %f, %f", ray.getX(), ray.getY(), ray.getZ());
  dir.normalize();
  std::vector<tf::Vector3> v;

  //Initialize vector on the most orthogonal axis
  switch(dir.closestAxis()){
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
    v[i] = (v[i] - dir * dir.dot(v[i])).normalize();
    // ROS_INFO("%f, %f, %f", v[i].getX(), v[i].getY(), v[i].getZ());
  }

  return v;
}



void RayTracer::transformRayToPartFrame(Ray &ray)
{
  ray.transform(particleHandler.getTransformToPartFrame());
}


void RayTracer::transformRayToBaseFrame(Ray &ray)
{
  ray.transform(particleHandler.getTransformToPartFrame().inverse());
}

		

