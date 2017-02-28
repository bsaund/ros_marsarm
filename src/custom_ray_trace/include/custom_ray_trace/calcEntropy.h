#ifndef CALC_ENTROPY_H
#define CALC_ENTROPY_H
#include <vector>
#include <map>


namespace CalcEntropy{
  typedef std::vector<int> BinId;
  
  struct ConfigDist {
    double dist;
    int id;
  };

  struct BinWithParticles{
    double binProbability;
    double binMin;
    double binMax;
    //map from Particle ID to probability of that particle in this bin
    std::map<int, double> particles;
  };

  struct ParticlesWithBin{
    double probability;
    //map from binId to weighted bins this particle is in
    std::map<BinId, double> bin;
  };

  struct ProcessedHistogram{
    std::map<BinId, BinWithParticles> bin;
    std::vector<ParticlesWithBin> particle;
  };
  

  ProcessedHistogram processMeasurements(std::vector<ConfigDist> p, double binSize, int numConfigs);
  ProcessedHistogram combineHist(const ProcessedHistogram &hist1, 
				 const ProcessedHistogram &hist2);
  double calcCondDisEntropy(const ProcessedHistogram &procHist);
  double calcIG(const std::vector<ConfigDist> &distances, double binSize, int numParticles);
  double calcIG(const ProcessedHistogram &procHist, int numParticles);
  double calcIndirectIG(std::vector<ConfigDist> &directDistances, 
			std::vector<ConfigDist> &indirectDistances, 
			double binSize, int numParticles);

}

#endif


