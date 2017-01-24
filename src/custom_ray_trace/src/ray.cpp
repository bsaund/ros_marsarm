#include "ray.h"

/*
 *************************
 ***     Ray           ***
 *************************
 */

Ray::Ray(){
}
 
Ray::Ray(tf::Point start_, tf::Point end_)
{
  start = start_;
  end = end_;
  length = (end-start).length();
}

tf::Vector3 Ray::getDirection() const
{
  return (end-start).normalize();
}

double Ray::getLength() const
{
  return length;
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
