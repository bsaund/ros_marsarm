#include <calcEntropy.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <ctime>
#include <cmath>

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
  // min = std::floor(min*100)/100; //used to force first bin at a round number
  // std::cout << "Min: " << min <<std::endl;
  
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
    proc.bin[idOf(bin)].binProbability = (double)unproc[bin].particleIds.size() / totalData;
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
  //Assume equal particle probabilities;
  double particleProb = 1.0/proc.particle.size();
  for(auto &p : proc.particle){
    p.probability = particleProb;
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

static void printBins(const CalcEntropy::ProcessedHistogram &proc){
  double totalBinP = 0;
  std::cout << std::endl << 
    "============= BINS ===============" << std::endl << std::endl;
  for(const auto &b : proc.bin){
    std::cout << "<" ;
    for(int binNum : b.first){
      std::cout << binNum << ", ";
    }
    std::cout << "\b\b>: p=" << b.second.binProbability << "  " << std::endl;
    std::cout << "    ";
    totalBinP += b.second.binProbability;
    double pProb = 0;
    for(const auto &p : b.second.particles){
      std::cout << "(" << p.first << ", " << p.second << "), ";
      pProb += p.second;
    }
    if(std::abs(pProb - 1) > 0.00001){
      std::cout << "!!!!!!!!!!!!!!  Probability doesnt sum to 1" << std::endl;
      std::cout << "Sum particle prob in a bin: " << pProb << std::endl;
    }
    std::cout <<std::endl;
  }
  std::cout << std::endl;
  
  if(std::abs(totalBinP -1) > 0.00001){
    std::cout << std::endl << "!!!!!!!! Probability doesnt sum to 1" << std::endl;
    std::cout << "Sum Bin Probability: " << totalBinP << std::endl << std::endl;
  }
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

  // printParticles(proc);
  // printBins(proc);

  //TEST CODE!!!!
  // CalcEntropy::ProcessedHistogram comb;
  // comb = CalcEntropy::combineHist(proc, proc);
  // printParticles(comb);
  // printBins(comb);
}


/**
 * Ordering function for ConfigDist sort.
 *  
 */
bool distOrdering(const CalcEntropy::ConfigDist &left, const CalcEntropy::ConfigDist &right) {
  return left.dist < right.dist;
}
namespace CalcEntropy{
  ProcessedHistogram 
  processMeasurements(std::vector<ConfigDist> p, double binSize, int numParticles){
    std::sort(p.begin(), p.end(), &distOrdering);

    std::vector<Bin> hist;
    histogram(p, binSize, hist);

    CalcEntropy::ProcessedHistogram procHist;
    procHist.particle.resize(numParticles);
    processHistogram(hist, procHist, p.size()/numParticles);
    // printParticles(procHist);
    return procHist;
  }


  /*
   *  Calculates conditional discrete entropy of histogram of distance
   */
  double calcCondDisEntropy(const CalcEntropy::ProcessedHistogram &procHist){
    double entropy = 0;
    
    for(const auto &b : procHist.bin){
      for(const auto &p : b.second.particles){
	double particleProb = p.second;
	entropy -= b.second.binProbability * particleProb * log2(particleProb);
      }
    }
    return entropy;
  }

  
  double calcIG(const std::vector<ConfigDist> &distances, double binSize, int numParticles)
  {
    return calcIG(processMeasurements(distances, binSize, numParticles), numParticles);
  }

  double calcIG(const ProcessedHistogram &procHist, int numParticles){
    double H_Y_given_X = calcCondDisEntropy(procHist);
    double H_Y = -log2(1.0/(double)numParticles);
    // std::cout << "H_Y: " << H_Y << "    H_Y_given_X: " << H_Y_given_X << std::endl;
    // printBins(procHist);
    return H_Y - H_Y_given_X;
  }

  ProcessedHistogram combineHist(const ProcessedHistogram &hist1, 
				 const ProcessedHistogram &hist2){
    ProcessedHistogram comb;
    comb.particle.resize(hist1.particle.size());

    // std::cout <<" Combining Hists " << std::endl;

    //For every particle add probability of observation <x_1, x_2>
    for(int pId = 0; pId < hist1.particle.size(); pId++){
      comb.particle[pId].probability = hist1.particle[pId].probability;

      for(const auto &binPair_1 : hist1.particle[pId].bin){
	for(const auto &binPair_2 : hist2.particle[pId].bin){
	  BinId b_1 = binPair_1.first;
	  BinId b_2 = binPair_2.first;
    	  BinId newBinId = b_1;
    	  newBinId.insert(newBinId.end(), b_2.begin(), b_2.end());

	  double prob_1 = hist1.bin.at(b_1).particles.at(pId);
	  double prob_2 = hist2.bin.at(b_2).particles.at(pId);
	  comb.bin[newBinId].particles[pId] = prob_1*prob_2;
	  
	  comb.bin[newBinId].binProbability += hist1.particle[pId].probability * 
	    binPair_1.second * binPair_2.second;
	  comb.particle[pId].bin[newBinId] = binPair_1.second * binPair_2.second;
	}
      }
    }
    
    //normalize particle probabilities in each bin:
    for(auto &b : comb.bin){
      double totalProb = 0;
      for(auto &p : b.second.particles){
	totalProb += p.second;
      }
      for(auto &p : b.second.particles){
	p.second = p.second / totalProb;
      }
    }

    // printBins(comb);
    // printParticles(comb);

    return comb;
  }


}
