#include <cstring>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <Eigen/Dense>

using namespace std;
typedef float vec4x3[4][3];

/*
* Import binary STL file to arrays.
* Input: filename: STL file name
* Output: Triangle mesh vector.
*/
vector<vec4x3> importSTL(string filename)
{
	ifstream stlFile;
	char title[80];
	unsigned int num_triangles;
	short attribute;
	stlFile.open(filename, ios::binary | ios::in);
	if (!stlFile) {
		cerr << "Cant open " << endl;
	}
	stlFile.read((char *)&title, 80 * sizeof(char));
	stlFile.read((char *)&num_triangles, sizeof(num_triangles));
	cout << title << endl;
	cout << num_triangles << endl;
	vec4x3 a = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 },{ 0,0,0 } };
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