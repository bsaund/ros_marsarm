#ifndef RAY_TRACER_H
#define RAY_TRACER_H

#include "rayTrace.h"
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>


class Ray
{
 public:
  Ray(tf::Point start_, tf::Point end_);

  tf::Point start;
  tf::Point end;

  tf::Vector3 getDirection();
  Ray transform(tf::Transform trans);
  Ray getTransformed(tf::Transform trans);
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

  tf::StampedTransform getTransformToPartFrame();  
  void setParticles(geometry_msgs::PoseArray p);
  std::vector<tf::Transform> getParticles();
};




class RayTracer
{
 private:
  ros::NodeHandle n_;
  vector<vec4x3> mesh;
  vector<vec4x3> surroundingBox;
  ParticleHandler particleHandler;

 public:
  RayTracer();
  bool loadMesh();
  bool tracePartFrameRay(Ray ray, double &distToPart);
  bool traceRay(Ray ray, double &distToPart);
  bool traceAllParticles(Ray ray, std::vector<double> &distToPart);
  void transformRayToPartFrame(Ray &ray);
  void transformRayToBaseFrame(Ray &ray);

};


#endif
