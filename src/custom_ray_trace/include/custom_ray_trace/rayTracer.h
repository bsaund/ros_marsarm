#ifndef RAY_TRACER_H
#define RAY_TRACER_H

#include "rayTrace.h"
#include "ros/ros.h"
#include "stlParser.h"
#include "Triangle.h"
#include "BVH.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <array>
#include <vector>
#include "calcEntropy.h"

class Ray
{
 public:
  Ray();
  Ray(tf::Point start_, tf::Point end_);

  tf::Point start;
  tf::Point end;

  tf::Vector3 getDirection() const;
  double getLength() const;
  Ray transform(tf::Transform trans);
  Ray getTransformed(tf::Transform trans) const;
  tf::Point travelAlongFor(double dist) const;

  double length;

};

class ParticleHandler
{
 private:
  ros::NodeHandle rosnode;
  ros::Subscriber particleSub;
  ros::Publisher requestParticlesPub;
  bool particlesInitialized;


 public:
  ParticleHandler();
  ParticleHandler(std::string name);
  void connectToParticles(std::string name);
  tf::TransformListener tf_listener_;
  tf::StampedTransform trans_;
  std::vector<tf::Transform> particles;
  std::vector<tf::Transform> subsetParticles;
  bool newParticles;  

  tf::StampedTransform getTransformToPartFrame();  
  void setParticles(geometry_msgs::PoseArray p);
  std::vector<tf::Transform> getParticles();
  std::vector<tf::Transform> getParticleSubset();
  int getNumParticles();
  int getNumSubsetParticles();
  bool theseAreNewParticles();

};




class RayTracer
{
 private:
  ros::NodeHandle n_;
  stl::Mesh mesh;
  BVH *bvh;
  // stl::Mesh surroundingBox;
  // stl::Mesh surroundingBoxAllParticles;

  
 public:

  RayTracer();
  RayTracer(std::string pieceName);
  ~RayTracer();
  bool loadMesh();
  bool loadMesh(std::string pieceName);
  void generateBVH();
  bool bvhIntersection(BVHRay &ray, IntersectionInfo &I);
  int getIntersection(array<double,3> pstart, array<double,3> dir, double &distToPart);
  bool tracePartFrameRay(const Ray &ray, double &distToPart);
  bool traceRay(Ray ray, double &distToPart);
  bool traceRay(Ray ray, tf::Point &intersection);
  bool traceRay(const stl::Mesh &mesh, const Ray &ray, double &distToPart);
  bool traceAllParticles(Ray ray, std::vector<double> &distToPart);

  double getIG(Ray ray, double radialErr, double distErr);
  double getIG(std::vector<Ray> rays, double radialErr, double distErr);
  bool traceCylinderAllParticles(Ray ray, double radius, vector<CalcEntropy::ConfigDist> &dists);
  std::vector<tf::Vector3> getOrthogonalBasis(tf::Vector3 dir);
  void getCylinderRays(Ray ray, double radius, std::vector<Ray> &rays);

  // stl::Mesh getBoxAroundAllParticles(stl::Mesh mesh);

  void transformRayToPartFrame(Ray &ray);
  ParticleHandler particleHandler;
  std::string stlFilePath;
};


#endif
