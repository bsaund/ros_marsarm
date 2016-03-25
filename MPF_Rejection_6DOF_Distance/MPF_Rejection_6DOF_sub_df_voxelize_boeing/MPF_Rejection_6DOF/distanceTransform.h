#ifndef DISTANCE_TRANSFORM_H
#define DISTANCE_TRANSFORM_H
#include <vector>

using namespace std;
typedef float vec4x3[4][3];
class distanceTransform
{
public:
	double voxel_size; //voxel size
	double world_range[3][2]; //work space range
	int num_voxels[3]; // voxel number along each dimension
	vector<vector<vector<double>>> *dist_transform;
	vector<vector<vector<double>>> *obstacle_map;
	distanceTransform(int n_voxels[3]);
	void build();
	void voxelizeSTL(vector<vec4x3> &mesh, double World_Range[3][2]);
protected:
	
	void distanceTransform_1D(vector<vector<vector<double>>> *dist_transform_1D, int range[3], 
		                      vector<vector<vector<double>>> *cost_fun, int dir, int idx1, int idx2);
};


#endif // DISTANCE_TRANSFORM_H