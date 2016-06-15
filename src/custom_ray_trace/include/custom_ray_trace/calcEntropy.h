#ifndef CALC_ENTROPY_H
#define CALC_ENTROPY_H
#include <vector>


namespace CalcEntropy{
  struct ConfigDist {
    double dist;
    int id;
  };

  struct Particle{
    int id;
    double probability;
  };
  
  struct BinWithParticles{
    double probability;
    std::vector<Particle> particles;
  };

  struct ProcessedBin{
    int id;
    double probability;
  };
  
  struct ParticlesWithBin{
    double probability;
    std::vector<ProcessedBin> bin;
  };

  struct ProcessedHistogram{
    std::vector<BinWithParticles> bin;
    std::vector<ParticlesWithBin> particle;
  };
  




  double calcCondDisEntropy(std::vector<ConfigDist> p, double binSize, int numParticles);
  double calcIG(std::vector<ConfigDist> p, double binSize, int numConfigs);

}

#endif


