#include <calcEntropy.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <ctime>

struct Bin {
  // std::vector<CalcEntropy::ConfigDist> element;
  std::vector<int> particleIds;
};

/*
 *  Get the min and max of a sorted ConfigDist vector
 */
static void getConfigDistMinAndMax(const std::vector<CalcEntropy::ConfigDist> &measurements,
				   double &min, double &max){
  min = measurements[0].dist;
  
  int m = measurements.size()-1;
  while(m>0 && measurements[m].dist > 999){
    m--;
  }
  max = measurements[m].dist;
}

/*
 * Given a sorted vector of config dists, returns the histogram of configurations
 *   based on "dist"
 */
static void histogram(const std::vector<CalcEntropy::ConfigDist> &measurements, 
		      double binSize, std::vector<Bin> &hist){
  double min, max;

  getConfigDistMinAndMax(measurements, min, max);
  
  double binValue = min + binSize;
  int binNum = 0;

  Bin b;
  hist.push_back(b);

  for(CalcEntropy::ConfigDist m:measurements){
    if(m.dist > binValue){
      binNum++;
      Bin b;
      hist.push_back(b);
      while(m.dist > binValue){
  	binValue += binSize;
      }
    }
    hist[binNum].particleIds.push_back(m.id);
  }
}

static void processBin(Bin &unproc, CalcEntropy::BinWithParticles &procBin){
    std::sort(unproc.particleIds.begin(), unproc.particleIds.end());
    int numInBin = unproc.particleIds.size();    
    
    if(unproc.particleIds.size() == 0)
      return;

    int count = 0;
    int id = unproc.particleIds[0];;
    for(int i=0; i<unproc.particleIds.size(); i++){
      if(unproc.particleIds[i] == id){
	count++;
      } else {
	CalcEntropy::Particle particle;
	particle.id = id;
	particle.probability = (double)count / numInBin;
	procBin.particles.push_back(particle);
	count = 1;
	id = unproc.particleIds[i];
      }
    }
    CalcEntropy::Particle particle;
    particle.id = id;
    particle.probability = (double)count / numInBin;
    procBin.particles.push_back(particle);
}

static void processHistogram(std::vector<Bin> &unproc, 
			     CalcEntropy::ProcessedHistogram &proc){
  proc.bin.resize(unproc.size());

  for(int bin=0; bin<unproc.size(); bin++){
    processBin(unproc[bin], proc.bin[bin]);

  }


  for(int bin=0; bin<unproc.size(); bin++){
    std::cout << "Bin " << bin << ": " << std::endl;
    for(int p:unproc[bin].particleIds){
      std::cout << p << ", ";
    }
    std::cout << std::endl;
    for(CalcEntropy::Particle p:proc.bin[bin].particles){
      std::cout << "(" << p.id << ", " << p.probability << "), ";
    }
    std::cout << std::endl;
  }
}


static double calcEntropyOfBin(Bin bin){
  double entropy = 0;
  double numPointsInBin = bin.particleIds.size();
  std::sort(bin.particleIds.begin(), bin.particleIds.end());

  int unique_particleIds_count = 0;
  int unique_particleIds = -1;
  for(int i=0; i<bin.particleIds.size(); i++){
    if(unique_particleIds == bin.particleIds[i]){
      unique_particleIds_count++;
    } else {
      if(unique_particleIds_count > 0){
  	 double p = (double)unique_particleIds_count / numPointsInBin;
  	 entropy -= p * log(p);
      }
      unique_particleIds_count = 1;
      unique_particleIds = bin.particleIds[i];
    }
  }
  if(unique_particleIds_count > 0){
    double p = (double)unique_particleIds_count / numPointsInBin;
    entropy -= p * log(p);
  }

  
  return entropy;
}


/**
 * Ordering function for ConfigDist sort.
 *  
 */
bool distOrdering(const CalcEntropy::ConfigDist &left, const CalcEntropy::ConfigDist &right) {
  return left.dist < right.dist;
}


namespace CalcEntropy{


  /*
   *  Calculates conditional discrete entropy of histogram of distance
   */
  double calcCondDisEntropy(std::vector<ConfigDist> p, double binSize, int numParticles)
  {
    std::sort(p.begin(), p.end(), &distOrdering);
    // ParticleBinHistogram hist;
    // hist.particle.resize(numParticles);

    std::vector<Bin> hist;
    histogram(p, binSize, hist);


    //TEST CODE
    CalcEntropy::ProcessedHistogram procHist;
    processHistogram(hist, procHist);
    //END TEST CODE


    double totalPoints = p.size();
    double entropy = 0;


    for(int binId = 0; binId < hist.size(); binId++){
      // std::cout << "Bin " << binId << ": ";
      // for(int j=0; j<hist[binId].particleIds.size(); j++){
      // 	std::cout << hist[binId].particleIds[j] << ", ";
      // }
      // std::cout << std::endl;

      Bin bin = hist[binId];

      double p_bin = bin.particleIds.size()/totalPoints;
      entropy += p_bin * calcEntropyOfBin(bin);
    //   // std::cout << std::endl; 
    //   // std::cout << "Entropy: " << calcEntropyOfBin(bin);
    //   // std::cout << " Probability: " << p_bin << std::endl;
    }

    // std::cout << "Entropy: " << entropy << std::endl;
    return entropy;
  }
  
  double calcIG(std::vector<ConfigDist> distances, double binSize, int numParticles)
  {
    // int start_s = clock();
    double H_Y_given_X = calcCondDisEntropy(distances, binSize, numParticles);
    double p = 1.0 / (double)numParticles;
    double H_Y = -log(p); // note this is - sum(p_i * log(p_i)) * n

    // std::cout << "IG: " <<  H_Y - H_Y_given_X << std::endl;
    // int afterIG_s = clock();
    // std::cout << "IG time: " << (afterIG_s - start_s)/double(CLOCKS_PER_SEC) << std::endl;

    return H_Y - H_Y_given_X;
  }

}
