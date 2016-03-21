#include <limits>
#include <vector>
#include <Eigen/Dense>
#include "distanceTransform.h"
#include "tribox.h"

#define max3(a,b,c) ((a>b?a:b)>c?(a>b?a:b):c)
#define max2(a,b) (a>b?a:b)
#define min3(a,b,c) ((a<b?a:b)<c?(a<b?a:b):c)
#define min2(a,b) (a<b?a:b)

using namespace std;
typedef float vec4x3[4][3];

/*distanceTransform::distanceTransform(double World_Range[3][2], double Grid_Res) :voxel_size(Grid_Res) {
memcpy(world_range, World_Range, 6 * sizeof(double));
}*/

distanceTransform::distanceTransform(int n_voxels[3]) {
	for (int i = 0; i < 3; i++) {
		num_voxels[i] = n_voxels[i];
	}
	const double LARGE_NUM = 10000000;
	dist_transform = new vector<vector<vector<double>>>(num_voxels[0], vector<vector<double>>(num_voxels[1], vector<double>(num_voxels[2])));
	obstacle_map = new vector<vector<vector<double>>>(num_voxels[0], vector<vector<double>>(num_voxels[1], vector<double>(num_voxels[2], LARGE_NUM)));
}
void distanceTransform::voxelizeSTL(vector<vec4x3> &mesh, double World_Range[3][2])
{
	const double LARGE_NUM = 10000000;
	const double MIN_VOXEL_RES = 0.000001;
	voxel_size = (World_Range[0][1] - World_Range[0][0]) / num_voxels[0];
	voxel_size = voxel_size - fmod(voxel_size, MIN_VOXEL_RES) + MIN_VOXEL_RES;
	/*cout << "DT Voxel Size: " << voxel_size << endl;*/
	memcpy(world_range, World_Range, 6 * sizeof(double));
	for (int i = 0; i < 3; i++) {
		//cout << "before: " << world_range[i][0] << "  " << world_range[i][1] << endl;
		world_range[i][0] = world_range[i][0] - fmod(world_range[i][0], MIN_VOXEL_RES);
		world_range[i][1] = world_range[i][0] + num_voxels[i] * voxel_size;
		//cout << "now: " << world_range[i][0] << "  " << world_range[i][1] << endl;
	}
	for (int i = 0; i < num_voxels[0]; i++) {
		for (int j = 0; j < num_voxels[1]; j++) {
			for (int k = 0; k < num_voxels[2]; k++) {
				(*obstacle_map)[i][j][k] = LARGE_NUM;
			}
		}
	}
	int num_mesh = int(mesh.size());
	double bbox[3][2];
	double box_halfsize[3];
	double triverts[3][3];
	box_halfsize[0] = voxel_size / 2;
	box_halfsize[1] = voxel_size / 2;
	box_halfsize[2] = voxel_size / 2;
	double ix, iy, iz = 0;
	double xstart, ystart, zstart, xend, yend, zend = 0;
	Eigen::Vector3d point_a, point_b, point_c, norm;
	double voxel_center[3];
	string key, value = "";
	for (int i = 0; i < num_mesh; i++)
	{
		triverts[0][0] = mesh[i][1][0];
		triverts[0][1] = mesh[i][1][1];
		triverts[0][2] = mesh[i][1][2];
		triverts[1][0] = mesh[i][2][0];
		triverts[1][1] = mesh[i][2][1];
		triverts[1][2] = mesh[i][2][2];
		triverts[2][0] = mesh[i][3][0];
		triverts[2][1] = mesh[i][3][1];
		triverts[2][2] = mesh[i][3][2];
		//norm = (point_b - point_a).cross(point_c - point_a);
		//norm /= norm.norm();
		norm << mesh[i][0][0], mesh[i][0][1], mesh[i][0][2];
		for (int j = 0; j < 3; j++)
		{
			double temp_val = min3(mesh[i][1][j], mesh[i][2][j], mesh[i][3][j]);
			//cout << min3(1.000, 1.000, -1.000) << endl << endl;
			bbox[j][0] = temp_val - voxel_size;
			temp_val = max3(mesh[i][1][j], mesh[i][2][j], mesh[i][3][j]);
			bbox[j][1] = temp_val + voxel_size;
		}
		if ((bbox[0][0] >world_range[0][1]) || (bbox[1][0] > world_range[1][1]) || (bbox[2][0] > world_range[2][1])
			|| (bbox[0][1] < world_range[0][0]) || (bbox[1][1] < world_range[1][0]) || (bbox[2][1] < world_range[2][0]))
			continue;
		xstart = max2(bbox[0][0], world_range[0][0]);
		xstart = xstart - fmod(xstart - world_range[0][0], voxel_size) + voxel_size / 2;
		ystart = max2(bbox[1][0], world_range[1][0]);
		ystart = ystart - fmod(ystart - world_range[1][0], voxel_size) + voxel_size / 2;
		zstart = max2(bbox[2][0], world_range[2][0]);
		zstart = zstart - fmod(zstart - world_range[2][0], voxel_size) + voxel_size / 2;
		xend = min2(bbox[0][1], world_range[0][1]);
		xend = xend - fmod(xend - world_range[0][1], voxel_size) - voxel_size / 2;
		yend = min2(bbox[1][1], world_range[1][1]);
		yend = yend - fmod(yend - world_range[1][1], voxel_size) - voxel_size / 2;
		zend = min2(bbox[2][1], world_range[2][1]);
		zend = zend - fmod(zend - world_range[2][1], voxel_size) - voxel_size / 2;
		ix = xstart;
		for (ix = xstart; ix <= xend; ix += voxel_size)
		{
			for (iy = ystart; iy <= yend; iy += voxel_size)
			{
				for (iz = zstart; iz <= zend; iz += voxel_size)
				{
					voxel_center[0] = ix;
					voxel_center[1] = iy;
					voxel_center[2] = iz;
					if (triBoxOverlap(voxel_center, box_halfsize, triverts) == 1) {
						(*obstacle_map)[int(floor((ix - world_range[0][0]) / voxel_size))]
							[int(floor((iy - world_range[1][0]) / voxel_size))]
						[int(floor((iz - world_range[2][0]) / voxel_size))] = 0;
						/*if (iy < 0.2) {
						cout << ix << "   " << iy << "   " << iz << " norm " << mesh[i][0][0] << "   " << mesh[i][0][1] << "   " << mesh[i][0][2]
						<< endl;
						}*/
					}
				}
			}
		}
	}
}
void distanceTransform::distanceTransform_1D(vector<vector<vector<double>>> *dist_transform_1D, int range[3],
	vector<vector<vector<double>>> *cost_fun, int dir, int idx1, int idx2)
{
	int k = 0;
	int *envelope = new int[range[dir]];
	envelope[0] = 0;
	double *bound = new double[range[dir] + 1];
	bound[0] = -std::numeric_limits<double>::infinity();
	bound[1] = std::numeric_limits<double>::infinity();
	double s;
	if (dir == 0) {
		for (int q = 1; q < range[dir]; q++) {
			s = (((*cost_fun)[q][idx1][idx2] + double(q * q)) - ((*cost_fun)[envelope[k]][idx1][idx2] + double(envelope[k] * envelope[k]))) / double(2 * q - 2 * envelope[k]);
			while (s <= bound[k])
			{
				k = k - 1;
				s = (((*cost_fun)[q][idx1][idx2] + double(q * q)) - ((*cost_fun)[envelope[k]][idx1][idx2] + double(envelope[k] * envelope[k]))) / double(2 * q - 2 * envelope[k]);
			}
			k = k + 1;
			envelope[k] = q;
			bound[k] = s;
			bound[k + 1] = std::numeric_limits<double>::infinity();
		}

		k = 0;
		for (int q = 0; q < range[dir]; q++) {
			while (bound[k + 1] < q) {
				k = k + 1;
			}
			(*dist_transform_1D)[q][idx1][idx2] = double((q - envelope[k]) * (q - envelope[k])) + (*cost_fun)[envelope[k]][idx1][idx2];
		}
	}
	else if (dir == 1) {
		for (int q = 1; q < range[dir]; q++) {
			s = (((*cost_fun)[idx1][q][idx2] + double(q * q)) - ((*cost_fun)[idx1][envelope[k]][idx2] + double(envelope[k] * envelope[k]))) / double(2 * q - 2 * envelope[k]);
			while (s <= bound[k])
			{
				k = k - 1;
				s = (((*cost_fun)[idx1][q][idx2] + double(q * q)) - ((*cost_fun)[idx1][envelope[k]][idx2] + double(envelope[k] * envelope[k]))) / double(2 * q - 2 * envelope[k]);
			}
			k = k + 1;
			envelope[k] = q;
			bound[k] = s;
			bound[k + 1] = std::numeric_limits<double>::infinity();
		}

		k = 0;
		for (int q = 0; q < range[dir]; q++) {
			while (bound[k + 1] < q) {
				k = k + 1;
			}
			(*dist_transform_1D)[idx1][q][idx2] = double((q - envelope[k]) * (q - envelope[k])) + (*cost_fun)[idx1][envelope[k]][idx2];
		}
	}
	else {
		for (int q = 1; q < range[dir]; q++) {
			double t = ((*cost_fun)[idx1][idx2][q] + double(q * q)) - ((*cost_fun)[idx1][idx2][envelope[k]]);
			s = (((*cost_fun)[idx1][idx2][q] + double(q * q)) - ((*cost_fun)[idx1][idx2][envelope[k]] + double(envelope[k] * envelope[k]))) / double(2 * q - 2 * envelope[k]);
			while (s <= bound[k])
			{
				k = k - 1;
				s = (((*cost_fun)[idx1][idx2][q] + double(q * q)) - ((*cost_fun)[idx1][idx2][envelope[k]] + double(envelope[k] * envelope[k]))) / double(2 * q - 2 * envelope[k]);
			}
			k = k + 1;
			envelope[k] = q;
			bound[k] = s;
			bound[k + 1] = std::numeric_limits<double>::infinity();
		}

		k = 0;
		for (int q = 0; q < range[dir]; q++) {
			while (bound[k + 1] < q) {
				k = k + 1;
			}
			(*dist_transform_1D)[idx1][idx2][q] = double((q - envelope[k]) * (q - envelope[k])) + (*cost_fun)[idx1][idx2][envelope[k]];
		}
	}
	delete[] envelope;
	delete[] bound;
}
void distanceTransform::build()
{
	for (int i = 0; i < num_voxels[0]; i++) {
		for (int j = 0; j < num_voxels[1]; j++) {
			distanceTransform_1D(dist_transform, num_voxels, obstacle_map, 2, i, j);
		}
		for (int k = 0; k < num_voxels[2]; k++) {
			distanceTransform_1D(obstacle_map, num_voxels, dist_transform, 1, i, k);
		}
	}

	for (int j = 0; j < num_voxels[1]; j++) {
		for (int k = 0; k < num_voxels[2]; k++) {
			distanceTransform_1D(dist_transform, num_voxels, obstacle_map, 0, j, k);
			for (int i = 0; i < num_voxels[0]; i++) {
				(*dist_transform)[i][j][k] = sqrt((*dist_transform)[i][j][k]) * voxel_size;
			}
		}
	}
}