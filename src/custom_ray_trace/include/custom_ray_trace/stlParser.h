#ifndef STL_PARSER_CUSTOM_RAY_H
#define STL_PARSER_CUSTOM_RAY_H
#include <array>
#include <vector>

#include "Triangle.h"

using namespace std;



namespace stl{
  typedef array<array<double, 3>, 4> vec4x3;
  typedef vector<Object*> Mesh;

  /*
   * Import binary STL file to arrays.
   * Input: filename: STL file name
   * Output: Triangle mesh vector.
   */
  Mesh importSTL(string filename);

  // Mesh getSurroundingBox(Mesh fullMesh);
  
  // void combineMesh(Mesh &mesh1, const Mesh &mesh2){
  //   mesh1.insert(mesh1.end(), mesh2.begin(), mesh2.end());
  // }

  // Mesh transformMesh(Mesh mesh, tf::Transform t);

}

#endif // STL_PARSER_H
