#ifndef RANDOM_TRANSFORM_H
#define RANDOM_TRANSFORM_H

#include <random>
#include "PfDefinitions.h"
#include "tf/tf.h"

class TransformDistribution
{
 public:
  virtual cspace sampleTransform() = 0;
  virtual cspace getMean() = 0;
  virtual cspace getVariance() = 0;
};




class FixedTfDist : public TransformDistribution
{
 public:
  FixedTfDist(cspace transform){    tr = transform;  }
  cspace sampleTransform(){    return tr;  }
  cspace getMean(){    return tr;  }
  cspace getVariance(){    return cspace{};  }
  
 private:
  cspace tr;
};

class UniformTfDist : public TransformDistribution
{
 public:
  UniformTfDist(cspace meanTF, cspace rangeTF){    
    dist = std::uniform_real_distribution<double>(-1, 1);
    mean = meanTF;
    range = rangeTF;
  }

  cspace sampleTransform(){    
    cspace sampled;
    for(int i=0; i<cdim; i++){
      sampled[i] = mean[i] + dist(rd)*range[i];
    }
    return sampled;  
  }

  cspace getMean(){    return mean;  }
  cspace getVariance(){    return range;  }
  
 private:
  cspace mean, range;
  random_device rd;
  std::uniform_real_distribution<double> dist; 
};



class GaussianTfDist: public TransformDistribution 
{ 
  public:
  GaussianTfDist(cspace meanTF, cspace rangeTF){
    for(int i=0; i<cdim; i++){
      dist[i] = std::normal_distribution<double>(meanTF[i], rangeTF[i]);
    }
    mean = meanTF;
    range = rangeTF;
  }

  cspace sampleTransform(){    
    //cout << "gaussian sampled!";
    cspace sampled;
    for(int i=0; i<cdim; i++){
      sampled[i] = dist[i](rd);
    }
    return sampled;  
  }

  cspace getMean(){    return mean;  }
  cspace getVariance(){    return range;  }
  
 private:
  cspace mean, range;
  random_device rd;
  std::normal_distribution<double> dist[cdim]; 
};

#endif
