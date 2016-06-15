#ifndef CALC_ENTROPY_H
#define CALC_ENTROPY_H
#include <vector>
#include <map>


namespace CalcEntropy{
  struct ConfigDist {
    double dist;
    int id;
  };

  /* struct Particle{ */
  /*   int id; */
  /*   double probability; */
  /* }; */
  
  struct BinWithParticles{
    double probability;
    //map from Particle ID to probability of that particle in this bin
    std::map<int, double> particles;
  };

  /* struct ProcessedBin{ */
  /*   int id; */
  /*   double probability; */
  /* }; */
  
  struct ParticlesWithBin{
    double probability;
    //map from binId to weighted bins this particle is in
    std::map<int, double> bin;
  };

  struct ProcessedHistogram{
    std::vector<BinWithParticles> bin;
    std::vector<ParticlesWithBin> particle;
  };
  




  double calcCondDisEntropy(std::vector<ConfigDist> p, double binSize, int numParticles);
  double calcIG(std::vector<ConfigDist> p, double binSize, int numConfigs);

}

#endif


