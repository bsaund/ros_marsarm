#ifndef RANDOM_TRANSFORM_H
#define RANDOM_TRANSFORM_H

#include <random>
#include "PfDefinitions.h"
#include "tf/tf.h"

class RandomTransform
{
 public:
  virtual cspace sampleTransform() = 0;
  virtual cspace getMean() = 0;
  virtual cspace getVariance() = 0;
};




class FixedTransform : public RandomTransform
{
 public:
  FixedTransform(cspace transform){    tr = transform;  }
  cspace sampleTransform(){    return tr;  }
  cspace getMean(){    return tr;  }
  cspace getVariance(){    return cspace{};  }
  
 private:
  cspace tr;
};

class UniformRandomTransform : public RandomTransform
{
 public:
  UniformRandomTransform(cspace meanTF, cspace rangeTF){    
    *dist = std::uniform_real_distribution<double>(-1, 1);
    mean = meanTF;
    range = rangeTF;
  }

  cspace sampleTransform(){    
    cspace sampled;
    for(int i=0; i<cdim; i++){
      sampled[i] = mean[i] + (*dist)(rd)*range[i];
    }
    return getMean();  
  }


  cspace getMean(){    return mean;  }
  cspace getVariance(){    return range;  }
  
 private:
  cspace mean, range;
  random_device rd;
  std::uniform_real_distribution<double> *dist; 
};



/* class GaussianTransform : public RandomTransform */
/* { */
/* } */

#endif
