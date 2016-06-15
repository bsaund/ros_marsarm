#ifndef RAY_TRACER_H
#define RAY_TRACER_H

#include "rayTrace.h"
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include "calcEntropy.h"

class Ray
{
 public:
  Ray(tf::Point start_, tf::Point end_);

  tf::Point start;
  tf::Point end;

  tf::Vector3 getDirection() const;
  Ray transform(tf::Transform trans);
  Ray getTransformed(tf::Transform trans) const;
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
  tf::TransformListener tf_listener_;
  tf::StampedTransform trans_;
  std::vector<tf::Transform> particles;
  std::vector<tf::Transform> subsetParticles;

  tf::StampedTransform getTransformToPartFrame();  
  void setParticles(geometry_msgs::PoseArray p);
  std::vector<tf::Transform> getParticles();
  std::vector<tf::Transform> getParticleSubset();
  int getNumParticles();
  int getNumSubsetParticles();
};




class RayTracer
{
 private:
  ros::NodeHandle n_;
  stl::Mesh mesh;
  stl::Mesh surroundingBox;
  stl::Mesh surroundingBoxAllParticles;
  
  ParticleHandler particleHandler;

 public:
  RayTracer();
  bool loadMesh();
  bool tracePartFrameRay(const Ray &ray, double &distToPart, bool quick=false);
  bool traceRay(Ray ray, double &distToPart);
  bool traceRay(const stl::Mesh &mesh, const Ray &ray, double &distToPart);
  bool traceAllParticles(Ray ray, std::vector<double> &distToPart, bool quick=true);

  double getIG(Ray ray, double radialErr, double distErr);
  bool traceCylinderAllParticles(Ray ray, double radius, vector<CalcEntropy::ConfigDist> &dists);
  std::vector<tf::Vector3> getOrthogonalBasis(tf::Vector3 dir);

  void getBoxAroundAllParticles();

  void transformRayToPartFrame(Ray &ray);
  void transformRayToBaseFrame(Ray &ray);

};


#endif
