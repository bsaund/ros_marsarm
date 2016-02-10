#include <iostream>
#include <gazebo_ray_trace/calcEntropy.h>
#include <vector>

int main()
{

  std::vector<double> bla;
  std::vector<double> dist;
  for(int i=0; i< 50; i++){
    bla.push_back(i + 0.01);
  }
  
  std::cout<< CalcEntropy::calcEntropy(bla);

  std::cout << "Hello World" << std::endl;
  return 0;
  
}
