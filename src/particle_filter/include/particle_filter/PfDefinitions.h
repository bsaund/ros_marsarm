#ifndef PFDEFINITIONS_H
#define PFDEFINITIONS_H
#include <array>
#include <vector>

const int cdim = 6;


typedef std::array<std::array<float, 3>, 4> vec4x3;
typedef std::array<double,cdim> cspace; // configuration space of the particles
typedef std::vector<cspace> Particles; 



#endif
