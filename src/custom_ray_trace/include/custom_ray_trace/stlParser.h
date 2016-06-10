#ifndef STL_PARSER_CUSTOM_RAY_H
#define STL_PARSER_CUSTOM_RAY_H
#include <array>


using namespace std;
typedef array<array<float, 3>, 4> vec4x3;
typedef vector<vec4x3> stlMesh;



namespace StlParser{
  /*
   * Import binary STL file to arrays.
   * Input: filename: STL file name
   * Output: Triangle mesh vector.
   */
  stlMesh importSTL(string filename);

  stlMesh getSurroundingBox(stlMesh fullMesh);

}

#endif // STL_PARSER_H
