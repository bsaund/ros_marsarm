#include <cstring>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <Eigen/Dense>
#include <array>
#include "stlParser.h"


using namespace std;

// typedef array<array<float, 3>, 4> stl::vec4x3;

/*
 * Import binary STL file to arrays.
 * Input: filename: STL file name
 * Output: Triangle mesh vector.
 */
stl::Mesh stl::importSTL(string filename)
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
  stl::Mesh mesh(num_triangles);
  Eigen::Vector3f normal_vec, edge1, edge2;
  Eigen::Vector3f vet[4];

  for (unsigned int i = 0; i < num_triangles; i++){
    for (int j = 0; j < 4; j++) {
      float tmp;
      stlFile.read((char *)&tmp, sizeof(float));
      mesh[i][j][0] = (double)tmp;
      stlFile.read((char *)&tmp, sizeof(float));
      mesh[i][j][1] = (double)tmp;
      stlFile.read((char *)&tmp, sizeof(float));
      mesh[i][j][2] = tmp;
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

void setMeshPoint(array<double,3> &point, double* value)
{
  for(int i=0; i<3; i++){
    point[i] = value[i];
  }
}

void setMeshFace(stl::vec4x3 &face, double boundX[2], 
		 double boundY[2], double boundZ[2],
		 vector<vector<int>> faceBool)
{
  for(int i=0; i<3; i++){
    double point[3] = {boundX[faceBool[i][0]], 
		       boundY[faceBool[i][1]], 
		       boundZ[faceBool[i][2]]};

    setMeshPoint(face[i+1], point);
    // for(int j=0; j<3; j++){
    //   face[i+1][j] = point[j];
    // }
    // cout << point[0] << ", " << point[1] << ", " << point[2] << endl; 
  }
}


vector<vector<vector<int>>> getFaceIndices()
{
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
stl::Mesh stl::getSurroundingBox(stl::Mesh fullMesh){

  stl::Mesh surroundingBox(12);
  


  double boundX[2] = {fullMesh[0][1][0], fullMesh[0][1][0]};
  double boundY[2] = {fullMesh[0][1][1], fullMesh[0][1][1]};
  double boundZ[2] = {fullMesh[0][1][2], fullMesh[0][1][2]};

  

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

  int faceIndex = 0;

  vector<vector<vector<int>>> faceIndBool = getFaceIndices();


  for(vector<vector<int>> faceBool: faceIndBool){
    setMeshFace(surroundingBox[faceIndex++], boundX, boundY, boundZ,
    		faceBool);
  }

  
  // int n = 0;
  // for(stl::vec4x3 tri: surroundingBox){

  //   cout << endl;
  //   cout << n++ << endl;
  //   for(array<double,3> point: tri){
  //     cout << endl;

  //     for(double coord: point){
  // 	cout << coord << ", ";
  //     }
  //   }

  // }

  return surroundingBox;
}


stl::Mesh stl::transformMesh(stl::Mesh mesh, tf::Transform t){
  for(stl::vec4x3 face:mesh){
    for(int i=1; i<4; i++){
      
    }
  }
}




