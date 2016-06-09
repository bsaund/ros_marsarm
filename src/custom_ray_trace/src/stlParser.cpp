#include <cstring>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <Eigen/Dense>
#include <array>
#include "stlParser.h"


using namespace std;

// typedef array<array<float, 3>, 4> vec4x3;

/*
* Import binary STL file to arrays.
* Input: filename: STL file name
* Output: Triangle mesh vector.
*/
vector<vec4x3> StlParser::importSTL(string filename)
{
	ifstream stlFile;
	char title[80];
	unsigned int num_triangles;
	short attribute;
	stlFile.open(filename, ios::binary | ios::in);
	if (!stlFile) {
	  cerr << "Cant open " << filename << endl;
	}
	stlFile.read((char *)&title, 80 * sizeof(char));
	stlFile.read((char *)&num_triangles, sizeof(num_triangles));
	cout << title << endl;
	cout << num_triangles << endl;
	vector<vec4x3> mesh(num_triangles);
	Eigen::Vector3f normal_vec, edge1, edge2;
	Eigen::Vector3f vet[4];

	for (unsigned int i = 0; i < num_triangles; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			stlFile.read((char *)&mesh[i][j][0], sizeof(float));
			stlFile.read((char *)&mesh[i][j][1], sizeof(float));
			stlFile.read((char *)&mesh[i][j][2], sizeof(float));
			mesh[i][j][0] *= 0.0254f;
			mesh[i][j][1] *= 0.0254f;
			mesh[i][j][2] *= 0.0254f;
			vet[j] << mesh[i][j][0], mesh[i][j][1], mesh[i][j][2];
		}
		edge1 = vet[2] - vet[1];
		edge2 = vet[3] - vet[1];
		vet[0] = edge1.cross(edge2) / edge1.cross(edge2).norm();
		mesh[i][0][0] = vet[0][0];
		mesh[i][0][1] = vet[0][1];
		mesh[i][0][2] = vet[0][2];
		stlFile.read((char *)&attribute, sizeof(short));
	}
	return mesh;
}

void setMeshPoint(float* point, float* value)
{
  for(int i=0; i<3; i++){
    point[i] = value[i];
  }
}

void setMeshFace(vec4x3 &face, float boundX[2], 
		 float boundY[2], float boundZ[2],
		 vector<vector<int>> faceBool)
{
  for(int i=0; i<3; i++){
    float point[3] = {boundX[faceBool[i][0]], 
		      boundY[faceBool[i][1]], 
		      boundZ[faceBool[i][2]]};

    setMeshPoint(&face[i+1][0], point);
  }
}

vector<vector<int>> generateBinarySet(vector<int> start)
{
  vector<vector<int>> v;
  int i=start[0];
  int j=start[1];
  int k=start[2];

  while(i < 2){
    while(j < 2){
      while(k < 2){
	vector<int> point = {i,j,k};
	v.push_back(point);
	k++;
      }
      k = 0;
      j++;
    }
    j = 0;
    i++;
  }
	

  // for(int i=start[0]; i<2; i++){
  //   for(int j=start[1]; j<2; j++){
  //     for(int k=start[2]; k<2; k++){
  // 	vector<int> point = {i,j,k};
  // 	v.push_back(point);
  //     }
  //   }
  // }
  return v;
}

/*
 *  Returns true if exactly one element of v2 is exactly one larger than 
 *  that same element in v1. All other elements are equal.
 */
bool oneLarger(vector<int> v1, vector<int> v2){
  int absDiff = 0;
  for(int i=0; i<3; i++){
    absDiff += abs(v2[i] - v1[i]);
  }
  return absDiff == 1;
}

vector<vector<vector<int>>> getFaceIndices()
{
  /*
  vector<int> base = {0, 0, 0};
  vector<vector<int>> basePoints = generateBinarySet(base);
  vector<vector<vector<int>>> faceIndices;


  for(vector<int> mainCorner: basePoints){

    for(vector<int> c1: generateBinarySet(mainCorner)){
      cout << endl;
      cout << endl;

      cout << c1[0] << ", " << c1[1] << ", " << c1[2] << endl;
      cout << endl;
      
      for(vector<int> c2: generateBinarySet(c1)){
	cout << c2[0] << ", " << c2[1] << ", " << c2[2] << endl;

	// cout << oneLarger(mainCorner, c1) << endl;
	// if(oneLarger(mainCorner, c1) &&
	//    oneLarger(mainCorner, c2)){
	//   cout << mainCorner[0] << ", " << mainCorner[1] << ", " << mainCorner[2] << endl;
	//   cout << c1[0] << ", " << c1[1] << ", " << c1[2] << endl;
	//   cout << c2[0] << ", " << c2[1] << ", " << c2[2] << endl;
	//   cout << endl;
	// }
	
	if(oneLarger(mainCorner, c1) &&
	   oneLarger(mainCorner, c2) &&
	   c1 != c2){
	  // vector<vector<int>> face {mainCorner, c1, c2};
	  // cout << mainCorner[0] << ", " << mainCorner[1] << ", " << mainCorner[2] << endl;	  
	  for(int val: mainCorner)
	    cout << val << ", ";
	  for(int val: c1)
	    cout << val << ", ";
	  for(int val: c2)
	    cout << val << ", ";
	  cout << endl;
	  
	  vector<vector<int>> face;
	  face.push_back(mainCorner);
	  face.push_back(c1);
	  face.push_back(c2);
	  faceIndices.push_back(face);
	}
      }
    }
  }

  */
  vector<vector<vector<int>>> faceIndices;
  faceIndices = {{{0,0,0}, {0,0,1}, {0,1,0}},
		 {{0,0,0}, {0,0,1}, {1,0,0}},
		 {{0,0,0}, {0,1,0}, {1,0,0}}, 
		 {{0,1,1}, {0,0,1}, {0,1,0}},
		 {{0,1,1}, {0,0,1}, {1,1,1}},
		 {{0,1,1}, {0,1,0}, {1,1,1}},
		 {{1,0,1}, {0,0,1}, {1,0,0}},
		 {{1,0,1}, {0,0,1}, {1,1,1}},
		 {{1,0,1}, {1,0,0}, {1,1,1}},
		 {{1,1,0}, {0,1,0}, {1,0,0}},
		 {{1,1,0}, {0,1,0}, {1,1,1}},
		 {{1,1,0}, {1,0,0}, {1,1,1}}};

  return faceIndices;
}

/*
 * Returns a mesh of the smallest axis-aligned box 
 * that surrounds the full mesh
 */
vector<vec4x3> StlParser::getSurroundingBox(vector<vec4x3> fullMesh){
  vector<vec4x3> surroundingBox(12);

  float boundX[2] = {fullMesh[0][1][0], fullMesh[0][1][0]};
  float boundY[2] = {fullMesh[0][1][1], fullMesh[0][1][1]};
  float boundZ[2] = {fullMesh[0][1][2], fullMesh[0][1][2]};


  for(int i=0; i<fullMesh.size(); i++){
    for(int j=1; j<4; j++){
      boundX[0] = std::min(boundX[0], fullMesh[i][j][0]);
      boundX[1] = std::max(boundX[1], fullMesh[i][j][0]);
      boundY[0] = std::min(boundY[0], fullMesh[i][j][1]);
      boundY[1] = std::max(boundY[1], fullMesh[i][j][1]);
      boundZ[0] = std::min(boundZ[0], fullMesh[i][j][2]);
      boundZ[1] = std::max(boundZ[1], fullMesh[i][j][2]);
    }
  }



  int faceIndex = -1;

  vector<vector<vector<int>>> faceIndBool = getFaceIndices();


  for(vector<vector<int>> faceBool: faceIndBool){
    setMeshFace(surroundingBox[faceIndex++], boundX, boundY, boundZ,
		faceBool);
  }

  
  // int n = 0;
  // for(vec4x3 tri: surroundingBox){
  //   cout << n++ << endl;
  //   cout << endl;
  //   for(array<float,3> point: tri){
  //     cout << endl;
  //     for(float coord: point){
  // 	cout << coord << ", ";
  //     }
  //   }

  // }

  return surroundingBox;

}
