#ifndef STL_PARSER_H
#define STL_PARSER_H

using namespace std;
typedef float vec4x3[4][3];

/*
* Import binary STL file to arrays.
* Input: filename: STL file name
* Output: Triangle mesh vector.
*/
vector<vec4x3> importSTL(string filename);

#endif // STL_PARSER_H