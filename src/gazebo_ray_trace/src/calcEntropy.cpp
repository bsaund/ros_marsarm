#include <calcEntropy.h>
#include <iostream>
#include <algorithm>
#include <math.h>

/*
 * Given a sorted vector of doubles, returns the histogram 
 * of the data in n evenly spaced bins
 */
static std::vector<double> histogram(std::vector<double> dist, int nbins, double* binSize){
  std::vector<double> hist;
  hist.resize(nbins);
  double min = dist[0];
  double max = dist[dist.size()-1];
  *binSize = (max-min)/nbins;
  double binNum = 0;
  double binValue = min + *binSize;

  if(max == min){
    hist.resize(0);
    return hist;
  }

  int i = 0;
  while(i < dist.size()){
    if(dist[i] < binValue){
      hist[binNum]++;
      i++;
    } else {
      binNum++;
      binValue += *binSize;
    }
  }
  return hist;
}

namespace CalcEntropy{

  /*
   * Calculates entropy of element of dist distribution
   *  Uses a histogram then evalutes the entropy
   *  of the discrete elements
   */
  double calcEntropy(std::vector<double> dist){
    std::sort(dist.begin(), dist.end());
    double binSize;
    std::vector<double> hist = histogram(dist, dist.size()/5, &binSize);
    if(hist.size() == 0){
      return 0;
    }
    
    double entropy = 0;
    for(int i=0; i < hist.size(); i++){
      double f = hist[i] / dist.size();
      if(f > 0){
	entropy += -f * log(f/binSize);
      }
      // std::cout << hist[i] << std::endl;
    }

   




    return entropy;
  }

}
