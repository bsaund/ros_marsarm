#include "simulateMeasurement.h"
#include <ros/console.h>
#include "particle_filter/AddObservation.h"
#include "particle_filter/relationships.h"



/*
 *  Simulates a measurement with some noise and passes that measurement to the particle filter
 */
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
  ros::spinOnce();
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




double getIndirectIG(Ray ray, std::shared_ptr<TransformDistribution> rel,
		     std::vector<tf::Transform> referenceParticles,
		     RayTracer* hitPart,
		     double radialErr, double depthErr){
  std::vector<CalcEntropy::ConfigDist> distsToParticles;
  ///NEEDS WORK
  for(int i=0; i<100; i++){
    tf::Transform tf = rel->sampleTransform();
    std::vector<CalcEntropy::ConfigDist> tmp;
    hitPart->traceCylinderAllParticles(Ray(tf*ray.start, tf*ray.end), radialErr, tmp,
				       referenceParticles);
    distsToParticles.insert(distsToParticles.end(),
  			    tmp.begin(), tmp.end());
  }

  return CalcEntropy::calcIG(distsToParticles, depthErr, 
  			     hitPart->particleHandler.getNumSubsetParticles());
}




/*
 *  Calcs IG for a measurement Ray which hits one of the objects in rayts
 *   The object for which the IG is calculated is the namespace of the node
 */
double getIG(Ray ray, std::vector<RayTracer*> rayts, PartRelationships &rel, 
	     std::vector<tf::Transform> referenceParticles,
	     double radialErr, double depthErr, bool printDebug){
  std::string referencePartName = ros::this_node::getNamespace();
  referencePartName.erase(0,2);

  RayTracer* hitPart;
  RayTracer* referencePart;

  if(!getIntersectingRayTracer(ray, rayts, hitPart))
    return 0;
  std::string hitPartName = hitPart->getName();


  // for(auto rayt : rayts){
  //   ROS_INFO("Names are %s and %s", rayt->getName().c_str(), referencePartName.c_str());
  //   if(rayt->getName() == referencePartName){
  //     referencePart = rayt;
  //     break;
  //   }
  // }


  double direct_ig = hitPart->getIG(ray, radialErr, depthErr);


  // if(referencePart == NULL){
  //   ROS_INFO("No reference part for getting IG");
  //   throw "No reference part";
  // }

  


  double coupled_ig = 0;
  if(rel.has(referencePartName, hitPartName)){
    coupled_ig = getIndirectIG(ray, rel.of(referencePartName, hitPartName),
			       referenceParticles, 
			       hitPart, 
			       radialErr, depthErr);
  }


  if(printDebug){
    ROS_INFO("Hit part %s with direct ig: %f and indirect ig: %f", 
	     hitPartName.c_str(), direct_ig, coupled_ig);
  }


  return coupled_ig;
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
