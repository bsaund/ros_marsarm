#ifndef DISTANCE_TRANSFORM_H
#define DISTANCE_TRANSFORM_H
#include <limits>
//using namespace std;

class distanceTransform
{
public:
	double grid_res;
	double world_range[3][2];
	int grid_nums[3];
	double ***dist_transform;
	double ***obstacle_map;
	distanceTransform(double world_range[3][2], double grid_res);
	void build(double cube_para[3]);
protected:
	void distanceTransform_1D(double ***dist_transform_1D, int range[3], double ***cost_fun, int dir, int idx1, int idx2);
};
distanceTransform::distanceTransform(double World_Range[3][2], double Grid_Res) :grid_res(Grid_Res) {
	memcpy(world_range, World_Range, 6 * sizeof(double));
}
void distanceTransform::distanceTransform_1D(double ***dist_transform_1D, int range[3], double ***cost_fun, int dir, int idx1, int idx2)
{
	int k = 0;
	int *envelope = new int[range[dir]];
	envelope[0] = 0;
	double *bound = new double[range[dir]];
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
	
}
void distanceTransform::build(double cube_para[3])
{
	double LARGE_NUM = 10000000;
	for (int i = 0; i < 3; i++) {
		grid_nums[i] = ceil((world_range[i][1] - world_range[i][0]) / grid_res);
	}
	dist_transform = new double **[grid_nums[0]];
	obstacle_map = new double **[grid_nums[0]];
	double ***temp = new double **[grid_nums[0]];
	for (int i = 0; i < grid_nums[0]; i++) {
		dist_transform[i] = new double *[grid_nums[1]];
		obstacle_map[i] = new double *[grid_nums[1]];
		temp[i] = new double *[grid_nums[1]];
		for (int j = 0; j < grid_nums[1]; j++) {
			dist_transform[i][j] = new double[grid_nums[2]];
			obstacle_map[i][j] = new double[grid_nums[2]];
			temp[i][j] = new double[grid_nums[2]];
		}
	}
	double x, y, z;
	for (int i = 0; i < grid_nums[0]; i++) {
		for (int j = 0; j < grid_nums[1]; j++) {
			for (int k = 0; k < grid_nums[2]; k++) {
				x = double(i)*grid_res + world_range[0][0];
				y = double(j)*grid_res + world_range[1][0];
				z = double(k)*grid_res + world_range[2][0];
				if (abs(x) <= cube_para[0] / 2 && abs(y) <= cube_para[1] / 2 && abs(z) <= cube_para[2] / 2) {
					obstacle_map[i][j][k] = 0;
				}
				else
					obstacle_map[i][j][k] = LARGE_NUM;
			}
		}
	}
	for (int i = 0; i < grid_nums[0]; i++) {
		for (int j = 0; j < grid_nums[1]; j++) {
			distanceTransform_1D(dist_transform, grid_nums, obstacle_map, 2, i, j);
		}
		for (int k = 0; k < grid_nums[2]; k++) {
			distanceTransform_1D(temp, grid_nums, dist_transform, 1, i, k);
		}
	}
	//memcpy(dist_transform, temp, grid_nums[0] * grid_nums[1] * grid_nums[2] * sizeof(double));
	for (int j = 0; j < grid_nums[1]; j++) {
		for (int k = 0; k < grid_nums[2]; k++) {
			distanceTransform_1D(dist_transform, grid_nums, temp, 0, j, k);
			for (int i = 0; i < grid_nums[0]; i++) {
				dist_transform[i][j][k] = sqrt(dist_transform[i][j][k]) * grid_res;
			}
		}
	}
	//for (int k = 0; k < grid_nums[2]; k++) {
	//	for (int i = 0; i < grid_nums[0]; i++) {
	//		for (int j = 0; j < grid_nums[1]; j++) {
	//			cout << dist_transform[i][j][k] << "  ";
	//		}
	//		cout << endl;
	//	}
	//	cout << endl;
	//}
}

#endif // DISTANCE_TRANSFORM_H