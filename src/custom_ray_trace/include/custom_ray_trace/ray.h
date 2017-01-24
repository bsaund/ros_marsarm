#ifndef CUSTOM_RAY_H
#define CUSTOM_RAY_H

#include <tf/tf.h>

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


#endif
