#ifndef STL_PARSER_CUSTOM_RAY_H
#define STL_PARSER_CUSTOM_RAY_H
#include <array>
#include "plucker.h"

using namespace std;
typedef array<array<float, 3>, 4> vec4x3;
typedef vector<vec4x3> stlMesh;

struct pluckerMesh{
  stlMesh stl;
  vector<pluckerTriangle> plucker;
};


namespace StlParser{
  /*
   * Import binary STL file to arrays.
   * Input: filename: STL file name
   * Output: Triangle mesh vector.
   */
  pluckerMesh importSTL(string filename);

  pluckerMesh getSurroundingBox(pluckerMesh fullMesh);

  pluckerMesh meshToPluckerMesh(stlMesh);

}

#endif // STL_PARSER_H
