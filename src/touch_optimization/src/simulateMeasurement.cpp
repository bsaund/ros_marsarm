#include "simulateMeasurement.h"
#include <ros/console.h>
#include "particle_filter/AddObservation.h"
#include "particle_filter/relationships.h"

int simulateMeasurement(Ray measurementAction, RayTracer &rayt,
			 ros::ServiceClient &pfilterAdd, double noiseStdDev) 
{
  std::random_device rd;
  std::normal_distribution<double> randn(0.0, noiseStdDev);

  tf::Point start = measurementAction.start;
  tf::Point end = measurementAction.end;
  tf::Point intersection;

  geometry_msgs::Point obs;
  geometry_msgs::Point dir;


  double distToPart;

  if(!rayt.traceRay(measurementAction, distToPart)){
    //measurement missed part
    return -1;
  }

  intersection = start + (end-start).normalize() * (distToPart);
  // std::cout << "Intersection at: " << intersection.getX() << "  " << intersection.getY() << "   " << intersection.getZ() << std::endl;
  tf::Point ray_dir(end.x()-start.x(),end.y()-start.y(),end.z()-start.z());
  ray_dir = ray_dir.normalize();

  obs.x=intersection.getX() + randn(rd); 
  obs.y=intersection.getY() + randn(rd); 
  obs.z=intersection.getZ() + randn(rd);
  dir.x=ray_dir.x();
  dir.y=ray_dir.y();
  dir.z=ray_dir.z();

  particle_filter::AddObservation pfilter_obs;
  // std::string ns = ros::this_node::getNamespace();
  // ns.erase(0,2);

  pfilter_obs.request.p = obs;
  pfilter_obs.request.dir = dir;
  // pfilter_obs.request.object = ns;
  pfilter_obs.request.object = rayt.getName();
  pfilter_obs.request.mesh_resource = rayt.stlFilePath;

  if(!pfilterAdd.call(pfilter_obs)){
    ROS_INFO("Failed to call add observation");
  }
  return 1;
}

/*
 * Simulates the measurement ray on the part composed of the pieces defined by the 
 *  RayTracers given
 */
int simOnAllParts(Ray ray, std::vector<RayTracer*> &rayts, ros::ServiceClient &srv_add,
		   double noiseStdDev){
  RayTracer* firstPart;
  
  if(!getIntersectingRayTracer(ray, rayts, firstPart))
    return -1;
  
  return simulateMeasurement(ray, *firstPart, srv_add, noiseStdDev);
}

/*
 *  Calcs IG for a measurement Ray which hits one of the objects in rayts
 *   The object for which the IG is calculated is the namespace of the node
 */
double getIG(Ray ray, std::vector<RayTracer*> rayts, Relationships rel, 
	     double radialErr, double depthErr){
  RayTracer* hitPart;
  if(!getIntersectingRayTracer(ray, rayts, hitPart))
    return 0;

  
  std::string partName = hitPart->getName();

  // ROS_INFO("Hit part %s", partName.c_str());

  std::vector<CalcEntropy::ConfigDist> distsToParticles;
  
  if(rel.count(partName) == 0)
    return 0;

  for(int i=0; i<100; i++){
    cspace tfp = rel[partName]->sampleTransform();
    tf::Quaternion q = tf::createQuaternionFromRPY(tfp[3], tfp[4], tfp[5]);
    tf::Transform tf(q, tf::Vector3(tfp[0], tfp[1], tfp[2]));
    std::vector<CalcEntropy::ConfigDist> tmp;
    hitPart->traceCylinderAllParticles(Ray(tf*ray.start, tf*ray.end), radialErr, tmp);
    distsToParticles.insert(distsToParticles.end(),
			    tmp.begin(), tmp.end());

  }
  return CalcEntropy::calcIG(distsToParticles, depthErr, 
			     hitPart->particleHandler.getNumSubsetParticles());
}


/*
 *  Sets hitPart to be the ray tracer of the part the ray hits.
 *   Returns false if no part was hit
 */
bool getIntersectingRayTracer(Ray ray, std::vector<RayTracer*> &rayts, RayTracer* &hitPart){
  double minD = std::numeric_limits<double>::max();
  double d;

  for(auto &rayt : rayts){
    if(!rayt->traceRay(ray, d))
      continue;
    if(d > minD)
      continue;
    minD = d;
    hitPart = rayt;
  }
  return minD < std::numeric_limits<double>::max();
}


/*
 *  Returns a vector of ray tracers, one for each piece that could be intersected
 *  Warning: Poor memory management, could cause memory leaks
 */
std::vector<RayTracer*> getAllRayTracers(){
  std::vector<std::string> pieces = {"top_datum", "right_datum", "J1_section",
				    "bottom_section", "back_datum"};
  std::vector<RayTracer*> rayts;

  for(std::string &piece:pieces){
    RayTracer* rayt = new RayTracer(piece);
    rayts.push_back(rayt);
  }
  return rayts;
}
