#ifndef STL_PARSER_H
#define STL_PARSER_H
#include <array>

using namespace std;
typedef array<array<float, 3>, 4> vec4x3;

/*
* Import binary STL file to arrays.
* Input: filename: STL file name
* Output: Triangle mesh vector.
*/
vector<vec4x3> importSTL(string filename);

#endif // STL_PARSER_H