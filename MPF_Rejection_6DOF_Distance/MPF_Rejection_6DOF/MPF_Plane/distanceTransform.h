#ifndef DISTANCE_TRANSFORM_H
#define DISTANCE_TRANSFORM_H
#include <limits>

class distanceTransform
{
public:
	int voxel_size; //voxel size
	double world_range[3][2]; //work space range
	double ***dist_transform;
	double ***obstacle_map;
	distanceTransform(int voxel_nums[3]);
	void build(double cube_para[3], double World_Range[3][2]);
protected:
	int voxel_nums[3]; // voxel number along each dimension
	void distanceTransform_1D(double ***dist_transform_1D, int range[3], double ***cost_fun, int dir, int idx1, int idx2);
};
/*distanceTransform::distanceTransform(double World_Range[3][2], double Grid_Res) :voxel_size(Grid_Res) {
	memcpy(world_range, World_Range, 6 * sizeof(double));
}*/
distanceTransform::distanceTransform(int voxel_nums[3]) {
	memcpy(this->voxel_nums, voxel_nums, 3 * sizeof(int));
	dist_transform = new double **[voxel_nums[0]];
	obstacle_map = new double **[voxel_nums[0]];
	for (int i = 0; i < voxel_nums[0]; i++) {
		dist_transform[i] = new double *[voxel_nums[1]];
		obstacle_map[i] = new double *[voxel_nums[1]];
		for (int j = 0; j < voxel_nums[1]; j++) {
			dist_transform[i][j] = new double[voxel_nums[2]];
			obstacle_map[i][j] = new double[voxel_nums[2]];
		}
	}
}
void distanceTransform::distanceTransform_1D(double ***dist_transform_1D, int range[3], double ***cost_fun, int dir, int idx1, int idx2)
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
			s = ((cost_fun[q][idx1][idx2] + double(q * q)) - (cost_fun[envelope[k]][idx1][idx2] + double(envelope[k] * envelope[k]))) / double(2 * q - 2 * envelope[k]);
			while (s <= bound[k])
			{
				k = k - 1;
				s = ((cost_fun[q][idx1][idx2] + double(q * q)) - (cost_fun[envelope[k]][idx1][idx2] + double(envelope[k] * envelope[k]))) / double(2 * q - 2 * envelope[k]);
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
			dist_transform_1D[q][idx1][idx2] = double((q - envelope[k]) * (q - envelope[k])) + cost_fun[envelope[k]][idx1][idx2];
		}
	}
	else if (dir == 1) {
		for (int q = 1; q < range[dir]; q++) {
			s = ((cost_fun[idx1][q][idx2] + double(q * q)) - (cost_fun[idx1][envelope[k]][idx2] + double(envelope[k] * envelope[k]))) / double(2 * q - 2 * envelope[k]);
			while (s <= bound[k])
			{
				k = k - 1;
				s = ((cost_fun[idx1][q][idx2] + double(q * q)) - (cost_fun[idx1][envelope[k]][idx2] + double(envelope[k] * envelope[k]))) / double(2 * q - 2 * envelope[k]);
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
			dist_transform_1D[idx1][q][idx2] = double((q - envelope[k]) * (q - envelope[k])) + cost_fun[idx1][envelope[k]][idx2];
		}
	}
	else {
		for (int q = 1; q < range[dir]; q++) {
			double t = (cost_fun[idx1][idx2][q] + double(q * q)) - (cost_fun[idx1][idx2][envelope[k]]);
			s = ((cost_fun[idx1][idx2][q] + double(q * q)) - (cost_fun[idx1][idx2][envelope[k]] + double(envelope[k] * envelope[k]))) / double(2 * q - 2 * envelope[k]);
			while (s <= bound[k])
			{
				k = k - 1;
				s = ((cost_fun[idx1][idx2][q] + double(q * q)) - (cost_fun[idx1][idx2][envelope[k]] + double(envelope[k] * envelope[k]))) / double(2 * q - 2 * envelope[k]);
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
			dist_transform_1D[idx1][idx2][q] = double((q - envelope[k]) * (q - envelope[k])) + cost_fun[idx1][idx2][envelope[k]];
		}
	}
	delete[] envelope;
	delete[] bound;
}
void distanceTransform::build(double cube_para[3], double World_Range[3][2])
{
	const double LARGE_NUM = 10000000;
	double x, y, z;
	memcpy(world_range, World_Range, 6 * sizeof(double));
	for (int i = 0; i < voxel_nums[0]; i++) {
		for (int j = 0; j < voxel_nums[1]; j++) {
			for (int k = 0; k < voxel_nums[2]; k++) {
				x = double(i)*voxel_size + world_range[0][0];
				y = double(j)*voxel_size + world_range[1][0];
				z = double(k)*voxel_size + world_range[2][0];
				if (abs(x) <= cube_para[0] / 2 && abs(y) <= cube_para[1] / 2 && abs(z) <= cube_para[2] / 2) {
					obstacle_map[i][j][k] = 0;
				}
				else
					obstacle_map[i][j][k] = LARGE_NUM;
			}
		}
	}
	for (int i = 0; i < voxel_nums[0]; i++) {
		for (int j = 0; j < voxel_nums[1]; j++) {
			distanceTransform_1D(dist_transform, voxel_nums, obstacle_map, 2, i, j);
		}
		for (int k = 0; k < voxel_nums[2]; k++) {
			distanceTransform_1D(obstacle_map, voxel_nums, dist_transform, 1, i, k);
		}
	}

	for (int j = 0; j < voxel_nums[1]; j++) {
		for (int k = 0; k < voxel_nums[2]; k++) {
			distanceTransform_1D(dist_transform, voxel_nums, obstacle_map, 0, j, k);
			for (int i = 0; i < voxel_nums[0]; i++) {
				dist_transform[i][j][k] = sqrt(dist_transform[i][j][k]) * voxel_size;
			}
		}
	}
}

#endif // DISTANCE_TRANSFORM_H