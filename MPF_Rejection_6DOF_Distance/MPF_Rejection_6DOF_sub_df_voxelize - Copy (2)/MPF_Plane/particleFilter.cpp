#include <string.h>
#include <iostream>
#include <random>
#include <cmath>
#include <chrono>
#include <fstream>
#include <unordered_map>
#include <Eigen/Dense>
#include "distanceTransform.h"
#include "particleFilter.h"
#include "matrix.h"
#include "tribox.h"
using namespace std;

# define Pi          3.141592653589793238462643383279502884L

#define SQ(x) ((x)*(x))
#define bzero(b,len) (memset((b), '\0', (len)), (void) 0)
#define max(a,b,c) ((a>b?a:b)>c?(a>b?a:b):c)
#define min(a,b,c) ((a<b?a:b)<c?(a<b?a:b):c)
typedef double vector3[4][3];
typedef unordered_map<string, string> hashmap;
void importSTL(vector<vector3> &mesh);
void voxelizeSTL(vector<vector3> &mesh, hashmap &boundary_voxel, double voxel_size, double R, double Xstd_ob, double cube_center, double cube_size);
/*
 * Initialize particleFilter class
 * Input: 
 */
particleFilter::particleFilter(int n_particles, cspace b_init[2],
	double Xstd_ob, double Xstd_tran,
	double Xstd_scatter, double R)
	: numParticles(n_particles), Xstd_ob(Xstd_ob), Xstd_tran(Xstd_tran),
	Xstd_scatter(Xstd_scatter), R(R), firstObs(true)
{
	memcpy(b_Xprior, b_init, 2 * sizeof(cspace));
	memcpy(b_Xpre, b_Xprior, 2 * sizeof(cspace));

	particles = new cspace[numParticles];
	bzero(particles, numParticles*sizeof(cspace));
	particles0 = new cspace[numParticles];
	bzero(particles0, numParticles*sizeof(cspace));

	createParticles(particles0, b_Xprior, numParticles);

	particles_1 = new cspace[numParticles];
	W = new double[numParticles];
}

void particleFilter::addObservation(double obs[3], double cube[3], distanceTransform *dist_transform, int idx_obs)
{
	std::default_random_engine generator;
	normal_distribution<double> dist2(0, Xstd_scatter);

	if (!firstObs) {
		bzero(b_Xpre, 2 * sizeof(cspace));
		for (int k = 0; k < cdim; k++) {
			for (int j = 0; j < numParticles; j++) {
				particles0[j][k] += dist2(generator);
				b_Xpre[0][k] += particles0[j][k];
			}
			b_Xpre[0][k] /= numParticles;
			for (int j = 0; j < numParticles; j++) {
				b_Xpre[1][k] += SQ(particles0[j][k] - b_Xpre[0][k]);
			}
			b_Xpre[1][k] = sqrt(b_Xpre[1][k] / numParticles);
		}
	}
	bool iffar = updateParticles(particles_1, particles0, particles, obs, cube, idx_obs, dist_transform, numParticles, R, Xstd_ob, Xstd_tran);
	if (firstObs)
	{
		firstObs = false;
		memcpy(particles0, particles, numParticles*sizeof(cspace));
		memcpy(particles_1, particles, numParticles*sizeof(cspace));
	}
	else if (iffar == true)
	{
		memcpy(particles0, particles_1, numParticles*sizeof(cspace));
	}
	//calcWeight(W, numParticles, Xstd_tran, particles0, particles);
	memcpy(particles_1, particles0, numParticles*sizeof(cspace));
	//resampleParticles(particles0, particles, W, numParticles);
	memcpy(particles0, particles, numParticles*sizeof(cspace));

	for (int k = 0; k < cdim; k++) {
		particles_est[k] = 0;
		for (int j = 0; j < numParticles; j++) {
			particles_est[k] += particles0[j][k];
		}
		particles_est[k] /= numParticles;
	}
	particles_est_stat[1] = 0;
	for (int j = 0; j < numParticles; j++)
	{
		for (int k = 0; k < cdim; k++) {
			particles_est_stat[1] += SQ(particles0[j][k] - particles_est[k]);
		}
	}
	particles_est_stat[1] = sqrt(particles_est_stat[1] / numParticles);

	if (particles_est_stat[1] < 0.005 && (abs(particles_est[0] - b_Xpre[0][0])>0.001 ||
		abs(particles_est[1] - b_Xpre[0][1])>0.001 ||
		abs(particles_est[2] - b_Xpre[0][2])>0.001 ||
		abs(particles_est[3] - b_Xpre[0][3])>0.001 ||
		abs(particles_est[4] - b_Xpre[0][4]) > 0.001 ||
		abs(particles_est[5] - b_Xpre[0][5]) > 0.001))
		Xstd_scatter = 0.01;
	else
		Xstd_scatter = 0.0001;
}

void particleFilter::createParticles(cspace *particles, cspace b_Xprior[2],
	int n_particles)
{
	random_device rd;
	mt19937 e2(rd());
	normal_distribution<double> dist(0, 1);
	int cdim = sizeof(cspace) / sizeof(double);
	for (int i = 0; i < n_particles; i++)
	{
		for (int j = 0; j < cdim; j++)
		{
			particles[i][j] = b_Xprior[0][j] + b_Xprior[1][j] * (dist(e2));
		}
	}
};

bool particleFilter::updateParticles(cspace *particles_1, cspace *particles0, cspace *particles, double cur_M[3], 
		double cube[3], int idx_Measure, distanceTransform *dist_transform, int n_particles,
		double R, double Xstd_ob, double Xstd_tran)
{
	random_device rd;
	mt19937 e2(rd());
	normal_distribution<double> dist(0, 1);
	uniform_real_distribution<double> distribution(0, n_particles);
	int cdim = sizeof(cspace) / sizeof(double);
	int i = 0;
	int count = 0;
	bool iffar = false;
	cspace *b_X = particles0;
	int dir = idx_Measure % 3;
	int idx = 0;
	double tempState[6];
	double D;
	double cur_inv_M[3];
	int num_Mean = 1000;
	double **measure_workspace = new double*[num_Mean];
	double var_measure[3] = { 0, 0, 0 };
	cspace meanConfig = { 0, 0, 0, 0, 0, 0 };
	for (int t = 0; t < num_Mean; t++) {
		measure_workspace[t] = new double[3];
		int index = int(floor(distribution(e2)));
		//memcpy(sampleConfig[t], b_X[index], sizeof(cspace));
		for (int m = 0; m < cdim; m++) {
			meanConfig[m] += b_X[index][m] / num_Mean;
		}
		inverseTransform(cur_M, b_X[index], measure_workspace[t]);
	}
	// inverse-transform using sampled configuration
	inverseTransform(cur_M, meanConfig, cur_inv_M);
	for (int t = 0; t < num_Mean; t++) {
		var_measure[0] += SQ(measure_workspace[t][0] - cur_inv_M[0]);
		var_measure[1] += SQ(measure_workspace[t][1] - cur_inv_M[1]);
		var_measure[2] += SQ(measure_workspace[t][2] - cur_inv_M[2]);
	}
	var_measure[0] /= num_Mean;
	var_measure[1] /= num_Mean;
	var_measure[2] /= num_Mean;
	cout << "Touch Var: " << sqrt(var_measure[0]) << "  " << sqrt(var_measure[1]) << "  " << sqrt(var_measure[2]) << endl;
	double world_range[3][2];
	cout << cur_inv_M[0] << "    " << cur_inv_M[1] << "    " << cur_inv_M[2] << endl;
	for (int t = 0; t < 3; t++) {
		world_range[t][0] = cur_inv_M[t] - 0.05;
		world_range[t][1] = cur_inv_M[t] + 0.05;
		//cout << world_range[t][0] << " to " << world_range[t][1] << endl;
	}
	dist_transform->build(cube, world_range);
	// sample particles
	while (i < n_particles)
	{
		//if ((count >= 10000000 || (i > 0 && count / i > 5000)) && iffar == false)
		//{
		//	iffar = true;
		//	b_X = particles_1;
		//	//count = 0;
		//	i = 0;
		//}
		idx = int(floor(distribution(e2)));
		for (int j = 0; j < cdim; j++)
		{
			tempState[j] = b_X[idx][j] + Xstd_tran * dist(e2);
		}
		//double tempState[6] = { 2.11, 1.388, 0.818, Pi / 6 + Pi / 400, Pi / 12 + Pi / 420, Pi / 18 - Pi / 380 };
		inverseTransform(cur_M, tempState, cur_inv_M);
		
		// reject particles ourside of distance transform
		if (cur_inv_M[0] > world_range[0][1] || cur_inv_M[0] < world_range[0][0] ||
			cur_inv_M[1] > world_range[1][1] || cur_inv_M[1] < world_range[1][0] ||
			cur_inv_M[2] > world_range[2][1] || cur_inv_M[2] < world_range[2][0]) {
			continue;
		}
		//cout << cur_inv_M[0] << "    " << cur_inv_M[1] << "    " << cur_inv_M[2] << endl << endl;
 		//D = cur_inv_M[dir] - cube[dir] / 2 - R;
		//cout << D << endl;
		//cout << int(floor((cur_inv_M[0] - dist_transform->world_range[0][0]) / dist_transform->voxel_size)) << "    " << int(floor((cur_inv_M[1] - dist_transform->world_range[1][0]) / dist_transform->voxel_size)) << "     " << int(floor((cur_inv_M[2] - dist_transform->world_range[2][0]) / dist_transform->voxel_size)) << endl;
		D = (*dist_transform->dist_transform)[int(floor((cur_inv_M[0] - dist_transform->world_range[0][0]) / dist_transform->voxel_size))]
										     [int(floor((cur_inv_M[1] - dist_transform->world_range[1][0]) / dist_transform->voxel_size))]
										     [int(floor((cur_inv_M[2] - dist_transform->world_range[2][0]) / dist_transform->voxel_size))] - R;
		//cout << D << endl << endl;
		if (D >= -Xstd_ob && D <= Xstd_ob)
		{
			for (int j = 0; j < cdim; j++)
			{
				particles[i][j] = tempState[j];
			}
			//cout << D << "       " << cur_inv_M[dir] - cube[dir] / 2 - R << endl;
			i += 1;
		}
		count += 1;
		
	}
	//cout << count << endl;
	return iffar;
};

//void particleFilter::calcWeight(double *W, int n_particles, double Xstd_tran,
//	cspace *particles0, cspace *particles)
//{
//	double A = 1.0 / (sqrt(2 * Pi) * Xstd_tran);
//	double B = -0.5 / SQ(Xstd_tran);
//	double sum = 0;
//	for (int k = 0; k < n_particles; k++) {
//		for (int m = 0; m < n_particles; m++) {
//			W[k] += A*exp(B*(SQ(particles0[m][0] - particles[k][0]) + SQ(particles0[m][1] - particles[k][1]) +
//				SQ(particles0[m][2] - particles[k][2])));
//		}
//		sum += W[k];
//	}
//	for (int k = 0; k < n_particles; k++) {
//		W[k] /= sum;
//	}
//};

//void particleFilter::resampleParticles(cspace *particles0, cspace *particles, double *W,
//	int n_particles)
//{
//	double *Cum_sum = new double[n_particles];
//	Cum_sum[0] = W[0];
//	std::default_random_engine generator;
//	std::uniform_real_distribution<double> rd(0, 1);
//	for (int i = 1; i < n_particles; i++)
//	{
//		Cum_sum[i] = Cum_sum[i - 1] + W[i];
//
//	}
//	double t;
//	for (int i = 0; i < n_particles; i++)
//	{
//		t = rd(generator);
//		for (int j = 0; j < n_particles; j++)
//		{
//			if (j == 0 && t <= Cum_sum[0])
//			{
//				particles0[i][0] = particles[0][0];
//				particles0[i][1] = particles[0][1];
//				particles0[i][2] = particles[0][2];
//			}
//			else if (Cum_sum[j - 1] < t && t <= Cum_sum[j])
//			{
//				particles0[i][0] = particles[j][0];
//				particles0[i][1] = particles[j][1];
//				particles0[i][2] = particles[j][2];
//			}
//		}
//	}
//}

int main()
{
	float boxcenter[3] = { 2.1,2,2 };
	float boxhalfsize[3] = { 1,1,1 };
	float triverts[3][3] = { {-1,-1,-1},{1,1,1},{0,0,0} };
	cout << triBoxOverlap(boxcenter, boxhalfsize, triverts) << endl;
	vector<vector3> mesh(12);
	hashmap boundary_voxel;
	importSTL(mesh);
	int numParticles = 500; // number of particles
	double Xstd_ob = 0.0001;
	double Xstd_tran = 0.0025;
	double Xstd_scatter = 0.0001;
	double voxel_size = 0.0005; // voxel size for distance transform.
	double range = 0.1; //size of the distance transform
	double R = 0.01; // radius of the touch probe

	double cube_para[3] = { 6, 4, 2 }; // cube size: 6m x 4m x 2m with center at the origin.
	//double range[3][2] = { {-3.5, 3.5}, {-2.5, 2.5}, {-1.5, 1.5} };
	particleFilter::cspace X_true = { 2.12, 1.388, 0.818, Pi / 6 + Pi / 400, Pi / 12 + Pi / 420, Pi / 18 - Pi / 380 }; // true state of configuration
	cout << X_true[0] << ' ' << X_true[1] << ' ' << X_true[2] << ' ' 
		 << X_true[3] << ' ' << X_true[4] << ' ' << X_true[5] << endl;
	particleFilter::cspace b_Xprior[2] = { { 2.1, 1.4, 0.8, Pi / 6, Pi / 12, Pi / 18 },
										   { 0.01, 0.01, 0.01, Pi / 360, Pi / 360, Pi / 360 } }; // our prior belief

	particleFilter pfilter(numParticles, b_Xprior, Xstd_ob, Xstd_tran, Xstd_scatter, R);
	distanceTransform *dist_transform = new distanceTransform(range, voxel_size);
	//dist_transform->build(cube_para);

	int N_Measure = 60; // total number of measurements
	double M_std = 0.0001; // measurement error
	double M[3]; // measurement
	double tempM[3];
	particleFilter::cspace particles_est;
	double particles_est_stat[2];
	double rotationM[3][3];

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(-1, 1);
	normal_distribution<double> dist(0, M_std);
	for (int i = 0; i < N_Measure; i++) {
		// generate measurement in fixed frame, then transform it to particle frame.
		if (i % 3 == 0)
		{
			M[0] = cube_para[0] / 2 + R + dist(generator);
			M[1] = distribution(generator) * 1.8 + dist(generator);
			M[2] = distribution(generator) * 0.9 + dist(generator);
		}
		else if (i % 3 == 1)
		{
			M[0] = distribution(generator) * 2.8 + dist(generator);
			M[1] = cube_para[1] / 2 + R + dist(generator);
			M[2] = distribution(generator) * 0.9 + dist(generator);
		}
		else
		{
			M[0] = distribution(generator) * 2.8 + dist(generator);
			M[1] = distribution(generator) * 1.8 + dist(generator);
			M[2] = cube_para[2] / 2 + R + dist(generator);
		}
		rotationMatrix(X_true, rotationM);
		multiplyM(rotationM, M, tempM);
		double transition[3] = { X_true[0], X_true[1], X_true[2] };
		addM(tempM, transition, M);

		cout << "Observation " << i << " : touch at " << M[0] << " " << M[1] << " " << M[2] << endl;
		auto tstart = chrono::high_resolution_clock::now();
		pfilter.addObservation(M, cube_para, dist_transform, i); // update particles
		//pfilter.addObservation(M, cube_para, i);
		pfilter.estimatedDistribution(particles_est, particles_est_stat);
		auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - tstart);
		particles_est_stat[0] = 0;
		for (int k = 0; k < particleFilter::cdim; k++) {
			particles_est_stat[0] += SQ(particles_est[k] - X_true[k]);
		}
		particles_est_stat[0] = sqrt(particles_est_stat[0]);
		cout << "est: ";
		for (int k = 0; k < particleFilter::cdim; k++) {
			cout << particles_est[k] << ' ';
		}
		cout << endl;
		cout << "Diff: " << particles_est_stat[0] << endl;
		cout << "Var: " <<  particles_est_stat[1] << endl;
		cout << "Time: " << diff.count() << " milliseconds." << endl << endl;
	}
	delete (dist_transform);
}
void inverseTransform(double measure[3], particleFilter::cspace src, double dest[3])
{
	double rotation[3][3];
	double tempRot[3][3];
	double invRot[3][3];
	double tempM[3];
	double rotationC[3][3] = { { cos(src[5]), -sin(src[5]), 0 },
	{ sin(src[5]), cos(src[5]), 0 },
	{ 0, 0, 1 } };
	double rotationB[3][3] = { { cos(src[4]), 0 , sin(src[4]) },
	{ 0, 1, 0 },
	{ -sin(src[4]), 0, cos(src[4]) } };
	double rotationA[3][3] = { { 1, 0, 0 },
	{ 0, cos(src[3]), -sin(src[3]) },
	{ 0, sin(src[3]), cos(src[3]) } };
	multiplyM(rotationC, rotationB, tempRot);
	multiplyM(tempRot, rotationA, rotation);
	inverseMatrix(rotation, invRot);
	double transition[3] = { src[0], src[1], src[2] };
	subtractM(measure, transition, tempM);
	multiplyM(invRot, tempM, dest);

}

void importSTL(vector<vector3> &mesh)
{
	fstream stlFile;
	stlFile.open("cube.txt", ios::in);
	if (!stlFile)
		cerr << "Cant open " << endl;
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			stlFile >> mesh[i][j][0] >> mesh[i][j][1] >> mesh[i][j][2];
			cout << mesh[i][j][0] << "  " << mesh[i][j][1] << "  " << mesh[i][j][2] << endl;
		}
	}
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			cout << mesh[i][j][0] << "  " << mesh[i][j][1] << "  " << mesh[i][j][2] << endl;
		}
	}
	cout << mesh.size() << endl;
	cout << min(1,3,2) << endl;
}
void voxelizeSTL(vector<vector3> &mesh, distanceTransform *dist_transform, double voxel_size, double Xstd_ob,
				 double cube_center, double cube_size)
{
	int num_mesh = mesh.size();
	//double bbox[3][2];
	Eigen::Matrix<double, 3, 2> bbox;
	double ix, iy, iz = 0;
	double xstart, ystart, zstart, xend, yend, zend = 0;
	Eigen::Vector3d voxel_center, point_a, point_b, point_c, norm;
	double dist;
	string key, value = "";
	for (int i = 0; i < num_mesh; i++)
	{
		point_a(0) = mesh[i][1][0];
		point_a(1) = mesh[i][1][1];
		point_a(2) = mesh[i][1][2];
		point_b(0) = mesh[i][2][0];
		point_b(1) = mesh[i][2][1];
		point_b(2) = mesh[i][2][2];
		point_c(0) = mesh[i][3][0];
		point_c(1) = mesh[i][3][1];
		point_c(2)  = mesh[i][3][2];
		//norm = (point_b - point_a).cross(point_c - point_a);
		//norm /= norm.norm();
		norm << mesh[i][0][0], mesh[i][0][1], mesh[i][0][2];
		for (int j = 0; j < 3; j++)
		{
			double temp_val = min(mesh[i][1][j], mesh[i][2][j], mesh[i][3][j]) - 2*R;
			bbox(j,0) = temp_val - fmod(temp_val, voxel_size) - voxel_size/2;
			temp_val = max(mesh[i][1][j], mesh[i][2][j], mesh[i][3][j]) + 2*R;
			bbox(j,1) = temp_val - fmod(temp_val, voxel_size) + 3*voxel_size/2;
		}
		xstart = bbox[0][0];
		ystart = bbox[1][0];
		zstart = bbox[2][0];
		xend = bbox[0][1];
		yend = bbox[1][1];
		zend = bbox[2][1];
		ix = xstart;
		while (ix <= xend)
		{
			iy = ystart;
			while (iy <= yend)
			{
				iz = zstart;
				while (iz <= zend)
				{
					voxel_center << ix, iy, iz;
					dist = norm.dot(voxel_center - point_a);
					if (dist >= R - Xstd_ob && dist <= R + Xstd_ob)
					{
						key = to_string(ix - voxel_size / 2) + " " + to_string(iy - voxel_size / 2) + " " + to_string(iz - voxel_size / 2);
						value = boundary_voxel[key];
						if (value == "-1")
						{
							iz += voxel_size;
							continue;
						}
						else if (value != "")
							boundary_voxel[key] = value + " " + to_string(i);
						else
							boundary_voxel[key] = to_string(i);
					}
					else if (dist < R - Xstd_ob)
					{
							
					}
					iz += voxel_size;
				}
				iy += voxel_size;
			}
			ix += voxel_size;
		}
	}
}
//void voxelizeSTL(vector<vector3> &mesh, hashmap &boundary_voxel, double voxel_size, double R, double Xstd_ob,
//	double cube_center, double cube_size)
//{
//	int num_mesh = mesh.size();
//	//double bbox[3][2];
//	Eigen::Matrix<double, 3, 2> bbox;
//	double ix, iy, iz = 0;
//	double xstart, ystart, zstart, xend, yend, zend = 0;
//	Eigen::Vector3d voxel_center, point_a, point_b, point_c, norm;
//	double dist;
//	string key, value = "";
//	for (int i = 0; i < num_mesh; i++)
//	{
//		point_a(0) = mesh[i][1][0];
//		point_a(1) = mesh[i][1][1];
//		point_a(2) = mesh[i][1][2];
//		point_b(0) = mesh[i][2][0];
//		point_b(1) = mesh[i][2][1];
//		point_b(2) = mesh[i][2][2];
//		point_c(0) = mesh[i][3][0];
//		point_c(1) = mesh[i][3][1];
//		point_c(2) = mesh[i][3][2];
//		//norm = (point_b - point_a).cross(point_c - point_a);
//		//norm /= norm.norm();
//		norm << mesh[i][0][0], mesh[i][0][1], mesh[i][0][2];
//		for (int j = 0; j < 3; j++)
//		{
//			double temp_val = min(mesh[i][1][j], mesh[i][2][j], mesh[i][3][j]) - 2 * R;
//			bbox(j, 0) = temp_val - fmod(temp_val, voxel_size) - voxel_size / 2;
//			temp_val = max(mesh[i][1][j], mesh[i][2][j], mesh[i][3][j]) + 2 * R;
//			bbox(j, 1) = temp_val - fmod(temp_val, voxel_size) + 3 * voxel_size / 2;
//		}
//		xstart = bbox[0][0];
//		ystart = bbox[1][0];
//		zstart = bbox[2][0];
//		xend = bbox[0][1];
//		yend = bbox[1][1];
//		zend = bbox[2][1];
//		ix = xstart;
//		while (ix <= xend)
//		{
//			iy = ystart;
//			while (iy <= yend)
//			{
//				iz = zstart;
//				while (iz <= zend)
//				{
//					voxel_center << ix, iy, iz;
//					dist = norm.dot(voxel_center - point_a);
//					if (dist >= R - Xstd_ob && dist <= R + Xstd_ob)
//					{
//						key = to_string(ix - voxel_size / 2) + " " + to_string(iy - voxel_size / 2) + " " + to_string(iz - voxel_size / 2);
//						value = boundary_voxel[key];
//						if (value == "-1")
//						{
//							iz += voxel_size;
//							continue;
//						}
//						else if (value != "")
//							boundary_voxel[key] = value + " " + to_string(i);
//						else
//							boundary_voxel[key] = to_string(i);
//					}
//					else if (dist < R - Xstd_ob)
//					{
//
//					}
//					iz += voxel_size;
//				}
//				iy += voxel_size;
//			}
//			ix += voxel_size;
//		}
//	}
//}