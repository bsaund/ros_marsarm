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
 *   ignoring the out of bounds measurements that miss the part
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
  int binNum = -1;

  getConfigDistMinAndMax(measurements, min, max);
  double binValue = min;

  for(CalcEntropy::ConfigDist m:measurements){
    if(m.dist >= binValue){
      binNum++;
      hist.push_back(Bin());
      while(m.dist >= binValue){
  	binValue += binSize;
      }
    }
    hist[binNum].particleIds.push_back(m.id);
  }
}

static void processBin(const Bin &unproc, CalcEntropy::BinWithParticles &procBin){
    double singleProb = (double)1 / unproc.particleIds.size();    

    for(int particleId:unproc.particleIds){
      procBin.particles[particleId] += singleProb;
    }
}

static CalcEntropy::BinId idOf(int id){
  CalcEntropy::BinId b = {id};
  return b;
}

/*
 *   Turns a vector of Bins (vectors of particles ids) into 
 *    a procesed histogram. It condenses the lists of particles
 *    into smaller lists of (particle, probability) for each bin.
 *    it also assigned a probability to each bin based on the 
 *    fraction of particles in the bin
 */
static void processBins(const std::vector<Bin> &unproc, 
		       CalcEntropy::ProcessedHistogram &proc){
  // proc.bin.resize(unproc.size());

  int totalData = 0;
  for(int bin=0; bin<unproc.size(); bin++){
    processBin(unproc[bin], proc.bin[idOf(bin)]);
    totalData += unproc[bin].particleIds.size();
  }

  for(int bin=0; bin<unproc.size(); bin++){
    proc.bin[idOf(bin)].probability = (double)unproc[bin].particleIds.size() / totalData;
  }
}

/*
 *  
 */
static void processParticles(std::vector<Bin> unproc, 
			     CalcEntropy::ProcessedHistogram &proc,
			     int numRepeated){
  double p = 1.0/numRepeated;
  for(int bin = 0; bin<unproc.size(); bin++){
    for(int particleId:unproc[bin].particleIds){
      proc.particle[particleId].bin[idOf(bin)] += p;
    }
  }
}

static void printParticles(CalcEntropy::ProcessedHistogram &proc){
  std::cout << std::endl << 
    "=========== PARTICLES ===========" << std::endl << std::endl;
  for(int pId=0; pId<proc.particle.size(); pId++){
    std::cout << pId << ": ";
    for(const auto &b : proc.particle[pId].bin){
      std::cout << "(<" ;
      for(int binNum : b.first){
	std::cout << binNum << ", ";
      }
      std::cout << "\b\b>, " << b.second << "), ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

static void printBins(CalcEntropy::ProcessedHistogram &proc){
  std::cout << std::endl << 
    "============= BINS ===============" << std::endl << std::endl;
  for(const auto &b : proc.bin){
    std::cout << "<" ;
    for(int binNum : b.first){
      std::cout << binNum << ", ";
    }
    std::cout << "\b\b>: p=" << b.second.probability << "  " << std::endl;
    std::cout << "    ";
    for(const auto &p : b.second.particles){
      std::cout << "(" << p.first << ", " << p.second << "), ";
    }
    std::cout <<std::endl;
  }
  std::cout << std::endl;
}

/*
 * Processed a vector of Bins (vectors of particle ids) into
 *  vector of ProcessedBins (probability of the bin with a vector 
 *  weighted unique particles) and vector of ProcessedParticles 
 *  (particle probabilities vector of weighted list of bins the particle 
 *  might be in)
 *  NOTE: proc.particle must be the correct size when given to this function
 */
static void processHistogram(std::vector<Bin> &unproc, 
			     CalcEntropy::ProcessedHistogram &proc,
			     int numRepeated){
  processBins(unproc, proc);
  processParticles(unproc, proc, numRepeated);


  //Print Bins
  // for(int bin=0; bin<unproc.size(); bin++){
  //   std::cout << "Bin " << bin << " Probability:  " << proc.bin[bin].probability << std::endl;
  //   for(int p:unproc[bin].particleIds){
  //     std::cout << p << ", ";
  //   }
  //   std::cout << std::endl;

  //   for(const auto &p : proc.bin[bin].particles){
  //     std::cout << "(" << p.first << ", " << p.second << "), ";
  //   }
  //   std::cout << std::endl << std::endl;
  // }
  printParticles(proc);
  printBins(proc);

  
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

    std::vector<Bin> hist;
    histogram(p, binSize, hist);

    CalcEntropy::ProcessedHistogram procHist;
    procHist.particle.resize(numParticles);
    processHistogram(hist, procHist, p.size()/numParticles);

    double entropy = 0;
    
    for(const auto &b : procHist.bin){
      for(const auto &p : b.second.particles){
	double particleProb = p.second;
	entropy -= b.second.probability * particleProb * log(particleProb);
      }
    }

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
