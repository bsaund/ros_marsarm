#ifndef RANDOM_TRANSFORM_H
#define RANDOM_TRANSFORM_H

#include <random>
#include "PfDefinitions.h"


class RandomTransform
{
 public:
  virtual cspace sampleTransform() = 0;
};




class FixedTransform : public RandomTransform
{
 public:
  FixedTransform(double x_, double y_, double z_, 
		 double r_, double p_, double ya_){
    x = x_; y = y_; z = z_;
    r = r_; p = p_; ya = ya_;
  }
  
  cspace sampleTransform(){
    cspace tr;
    tr[0] = x; tr[1] = y; tr[2] = z;
    tr[3] = r; tr[4] = p; tr[5] = ya;
    return tr;
  }
  
 private:
  double x, y, z, r, p, ya;
};

#endif
