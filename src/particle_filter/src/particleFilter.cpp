#include <string.h>
#include <iostream>
#include <random>
#include <chrono>
#include <fstream>
#include <unordered_map>
#include <Eigen/Dense>
#include <unordered_set>
#include <array>
#include "tribox.h"
#include "raytri.h"
#include "distanceTransformNew.h"
#include "particleFilter.h"
#include "matrix.h"
#include "stlParser.h"

using namespace std;

# define Pi          3.141592653589793238462643383279502884L

#define SQ(x) ((x)*(x))
#define bzero(b,len) (memset((b), '\0', (len)), (void) 0)
#define max3(a,b,c) ((a>b?a:b)>c?(a>b?a:b):c)
#define max2(a,b) (a>b?a:b)
#define min3(a,b,c) ((a<b?a:b)<c?(a<b?a:b):c)
#define min2(a,b) (a<b?a:b)
typedef array<array<float, 3>, 4> vec4x3;
typedef unordered_map<string, string> hashmap;
#define epsilon 0.0001
#define ARM_LENGTH 0.2

//vector<vec4x3> importSTL(string filename);

/*
 * particleFilter class Construction
 * Input: n_particles: number of particles
 *        b_init[2]: prior belief
 *        Xstd_ob: observation error
 *        Xstd_tran: standard deviation of gaussian kernel when sampling
 *        Xstd_scatter: scatter param before sampling the mean of dist_trans
 *        R: radius of the touch probe
 * output: none
 */
particleFilter::particleFilter(int n_particles, cspace b_init[2],
							   double Xstd_ob, double Xstd_tran,
							   double Xstd_scatter, double R)
	: numParticles(n_particles), Xstd_ob(Xstd_ob), Xstd_tran(Xstd_tran),
	Xstd_scatter(Xstd_scatter), R(R), firstObs(true)
{
	memcpy(b_Xprior, b_init, 2 * sizeof(cspace));
	//memcpy(b_Xpre, b_Xprior, 2 * sizeof(cspace));

	particles = new cspace[numParticles];
	bzero(particles, numParticles*sizeof(cspace));
	particles0 = new cspace[numParticles];
	bzero(particles0, numParticles*sizeof(cspace));

	createParticles(particles0, b_Xprior, numParticles);

	particles_1 = new cspace[numParticles];
	//W = new double[numParticles];
}
void particleFilter::getAllParticles(cspace *particles_dest)
{
  for(int i=0; i<numParticles; i++){
    for(int j=0; j<cdim; j++){
      particles_dest[i][j] = particles0[i][j];
    }
  }
}
/*
 * Add new observation and call updateParticles() to update the particles
 * Input: obs: observation
 *        mesh: object mesh arrays
 *        dist_transform: distance transform class instance
 *        idx_obs: not used
 * output: none
 */
void particleFilter::addObservation(double obs[2][3], vector<vec4x3> &mesh, distanceTransform *dist_transform, int idx_obs)
{
	std::default_random_engine generator;
	normal_distribution<double> dist2(0, Xstd_scatter);

	if (!firstObs) {
		//bzero(b_Xpre, 2 * sizeof(cspace));
		for (int k = 0; k < cdim; k++) {
			for (int j = 0; j < numParticles; j++) {
				particles0[j][k] += dist2(generator);
				//b_Xpre[0][k] += particles0[j][k];
			}
			/*b_Xpre[0][k] /= numParticles;
			for (int j = 0; j < numParticles; j++) {
				b_Xpre[1][k] += SQ(particles0[j][k] - b_Xpre[0][k]);
			}
			b_Xpre[1][k] = sqrt(b_Xpre[1][k] / numParticles);*/
		}
	}
	bool iffar = updateParticles(particles_1, particles0, particles, obs, mesh, idx_obs, dist_transform, numParticles, R, Xstd_ob, Xstd_tran);
	if (firstObs)
	{
		firstObs = false;
		memcpy(particles0, particles, numParticles*sizeof(cspace));
		memcpy(particles_1, particles, numParticles*sizeof(cspace));
	}
	/*else if (iffar == true)
	{
		memcpy(particles0, particles_1, numParticles*sizeof(cspace));
	}*/
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

	/*if (particles_est_stat[1] < 0.005 && (abs(particles_est[0] - b_Xpre[0][0])>0.001 ||
		abs(particles_est[1] - b_Xpre[0][1])>0.001 ||
		abs(particles_est[2] - b_Xpre[0][2])>0.001 ||
		abs(particles_est[3] - b_Xpre[0][3])>0.001 ||
		abs(particles_est[4] - b_Xpre[0][4]) > 0.001 ||
		abs(particles_est[5] - b_Xpre[0][5]) > 0.001))
		Xstd_scatter = 0.01;
	else
		Xstd_scatter = 0.0001;*/
}

/*
 * Create initial particles at start
 * Input: particles
 *        b_Xprior: prior belief
 *        n_partcles: number of particles
 * output: none
 */
void particleFilter::createParticles(cspace *particles_dest, cspace b_Xprior[2],
	int n_particles)
{
	random_device rd;
	normal_distribution<double> dist(0, 1);
	int cdim = sizeof(cspace) / sizeof(double);
	for (int i = 0; i < n_particles; i++)
	{
		for (int j = 0; j < cdim; j++)
		{
			particles_dest[i][j] = b_Xprior[0][j] + b_Xprior[1][j] * (dist(rd));
		}
	}
};

/*
 * Update particles (Build distance transform and sampling)
 * Input: particles_1: estimated particles before previous ones (not used here)
 *        particles0: previous estimated particles
 *        particles: current particles
 *        cur_M: current observation
 *        mesh: object mesh arrays
 *        dist_transform: distance transform class instance
 *        n_particles: number of particles
 *        R: radius of the touch probe
 *        Xstd_ob: observation error
 *        Xstd_tran: gaussian kernel standard deviation when sampling
 * output: return whether previous estimate is bad (not used here)
 */
bool particleFilter::updateParticles(cspace *particles_1, cspace *particles0, cspace *particles, double cur_M[2][3], 
		vector<vec4x3> &mesh, int idx_Measure, distanceTransform *dist_transform, int n_particles,
		double R, double Xstd_ob, double Xstd_tran)
{
	random_device rd;
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
	//double D2;
	double cur_inv_M[2][3];
	int num_Mean = 1000;
	double **measure_workspace = new double*[num_Mean];
	double var_measure[3] = { 0, 0, 0 };
	cspace meanConfig = { 0, 0, 0, 0, 0, 0 };
	double unsigned_dist_check = R + Xstd_ob;
	double voxel_size;
	double distTransSize;
	double mean_inv_M[3];
	double safe_point[2][3];
	for (int t = 0; t < num_Mean; t++) {
		measure_workspace[t] = new double[3];
		int index = int(floor(distribution(rd)));
		//memcpy(sampleConfig[t], b_X[index], sizeof(cspace));
		for (int m = 0; m < cdim; m++) {
			meanConfig[m] += b_X[index][m] / num_Mean;
		}
		inverseTransform(cur_M[0], b_X[index], measure_workspace[t]);
	}
	// inverse-transform using sampled configuration
	inverseTransform(cur_M[0], meanConfig, mean_inv_M);
	for (int t = 0; t < num_Mean; t++) {
		var_measure[0] += SQ(measure_workspace[t][0] - mean_inv_M[0]);
		var_measure[1] += SQ(measure_workspace[t][1] - mean_inv_M[1]);
		var_measure[2] += SQ(measure_workspace[t][2] - mean_inv_M[2]);
	}
	var_measure[0] /= num_Mean;
	var_measure[1] /= num_Mean;
	var_measure[2] /= num_Mean;
	distTransSize = 4 * max3(sqrt(var_measure[0]), sqrt(var_measure[1]), sqrt(var_measure[2]));
	distTransSize = 150 * 0.001;
	cout << "Touch Std: " << sqrt(var_measure[0]) << "  " << sqrt(var_measure[1]) << "  " << sqrt(var_measure[2]) << endl;
	double world_range[3][2];
	cout << "Current Inv_touch: " << mean_inv_M[0] << "    " << mean_inv_M[1] << "    " << mean_inv_M[2] << endl;
	for (int t = 0; t < 3; t++) {
		world_range[t][0] = mean_inv_M[t] - distTransSize;
		world_range[t][1] = mean_inv_M[t] + distTransSize;
		/*cout << world_range[t][0] << " to " << world_range[t][1] << endl;*/
	}
	voxel_size = distTransSize / 150;
	cout << "Voxel Size: " << voxel_size << endl;
	dist_transform->voxelizeSTL(mesh, world_range);
	dist_transform->build();
	double cube[3] = { 6,4,2 };
	Eigen::Vector3d gradient;
	Eigen::Vector3d touch_dir;
	// sample particles
	//touch_dir << cur_M[1][0], cur_M[1][1], cur_M[1][2];
	while (i < n_particles)
	{
		//if ((count >= 10000000 || (i > 0 && count / i > 5000)) && iffar == false)
		//{
		//	iffar = true;
		//	b_X = particles_1;
		//	//count = 0;
		//	i = 0;
		//}
		idx = int(floor(distribution(rd)));
		for (int j = 0; j < cdim; j++)
		{
			tempState[j] = b_X[idx][j] + Xstd_tran * dist(rd);
		}
		inverseTransform(cur_M, tempState, cur_inv_M);
		touch_dir << cur_inv_M[1][0], cur_inv_M[1][1], cur_inv_M[1][2];
		// reject particles ourside of distance transform
		if (cur_inv_M[0][0] > dist_transform->world_range[0][1] || cur_inv_M[0][0] < dist_transform->world_range[0][0] ||
			cur_inv_M[0][1] > dist_transform->world_range[1][1] || cur_inv_M[0][1] < dist_transform->world_range[1][0] ||
			cur_inv_M[0][2] > dist_transform->world_range[2][1] || cur_inv_M[0][2] < dist_transform->world_range[2][0]) {
			continue;
		}
		int xind = int(floor((cur_inv_M[0][0] - dist_transform->world_range[0][0]) / dist_transform->voxel_size));
		int yind = int(floor((cur_inv_M[0][1] - dist_transform->world_range[1][0]) / dist_transform->voxel_size));
		int zind = int(floor((cur_inv_M[0][2] - dist_transform->world_range[2][0]) / dist_transform->voxel_size));
		D = (*dist_transform->dist_transform)[xind][yind][zind];
		
		double dist_adjacent[3] = { 0, 0, 0 };
		if (D <= unsigned_dist_check)
		{
			if (xind < (dist_transform->num_voxels[0] - 1) && yind < (dist_transform->num_voxels[1] - 1) && zind < (dist_transform->num_voxels[2] - 1))
			{
				dist_adjacent[0] = (*dist_transform->dist_transform)[xind + 1][yind][zind];
				dist_adjacent[1] = (*dist_transform->dist_transform)[xind][yind + 1][zind];
				dist_adjacent[2] = (*dist_transform->dist_transform)[xind][yind][zind + 1];
				//gradient /= gradient.norm();
			}
			else
				continue;
			gradient[0] = dist_adjacent[0] - D;
			gradient[1] = dist_adjacent[1] - D;
			gradient[2] = dist_adjacent[2] - D;
			if (checkInObject(mesh, cur_inv_M[0]) == 1 && D != 0)
			{
				if (gradient.dot(touch_dir) <= epsilon)
					continue;
				D = -D - R;
			}
			else if (D == 0) 
			{
				double tmp[3] = { cur_inv_M[0][0] + dist_transform->voxel_size, cur_inv_M[0][1], cur_inv_M[0][2] };
				if (checkInObject(mesh, tmp) == 1)
					gradient[0] = -gradient[0];
				tmp[0] -= dist_transform->voxel_size;
				tmp[1] += dist_transform->voxel_size;
				if (checkInObject(mesh, tmp) == 1)
					gradient[1] = -gradient[1];
				tmp[1] -= dist_transform->voxel_size;
				tmp[2] += dist_transform->voxel_size;
				if (checkInObject(mesh, tmp) == 1)
					gradient[2] = -gradient[2];
				if (gradient.dot(touch_dir) >= -epsilon)
					continue;
				D = - R;
			}
			else
			{
				if (gradient.dot(touch_dir) >= -epsilon)
					continue;
				D = D - R;
			}	
		}
		else
			continue;
		/*if (abs(D2 - D) > voxel_size)
		{
			cout << D2 + R<< endl;
			cout << D + R<< endl << endl;
			cout << "AT " << cur_inv_M[0] << "  " << cur_inv_M[1] << "  " << cur_inv_M[2] << endl << endl;;
		}*/
		if (D >= -Xstd_ob && D <= Xstd_ob)
		{
			
			safe_point[1][0] = cur_M[1][0];
			safe_point[1][1] = cur_M[1][1];
			safe_point[1][2] = cur_M[1][2];
			safe_point[0][0] = cur_M[0][0] - cur_M[1][0] * ARM_LENGTH;
			safe_point[0][1] = cur_M[0][1] - cur_M[1][1] * ARM_LENGTH;
			safe_point[0][2] = cur_M[0][2] - cur_M[1][2] * ARM_LENGTH;
			if (checkObstacles(mesh, tempState, safe_point , D + R) == 1)
				continue;
			for (int j = 0; j < cdim; j++)
			{
				particles[i][j] = tempState[j];
			}
			double d = testResult(mesh, particles[i], cur_M, R);
			//if (d > 0.01)
			//	cout << cur_inv_M[0][0] << "  " << cur_inv_M[0][1] << "  " << cur_inv_M[0][2] << "   " << d << "   " << D << //"   " << gradient << "   " << gradient.dot(touch_dir) << 
			//	     "   " << dist_adjacent[0] << "   " << dist_adjacent[1] << "   " << dist_adjacent[2] << "   " << particles[i][2] << endl;
			i += 1;
		}
		count += 1;
		
	}
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
	hashmap boundary_voxel;
	vector<vec4x3> mesh = importSTL("boeing_part_binary.stl");
	int numParticles = 500; // number of particles
	double Xstd_ob = 0.001;
	double Xstd_tran = 0.0035;
	double Xstd_scatter = 0.0001;
	//double voxel_size = 0.0005; // voxel size for distance transform.
	int num_voxels[3] = { 300,300,300 };
	//double range = 0.1; //size of the distance transform
	double R = 0.001; // radius of the touch probe

	double cube_para[3] = { 6, 4, 2 }; // cube size: 6m x 4m x 2m with center at the origin.
	//double range[3][2] = { {-3.5, 3.5}, {-2.5, 2.5}, {-1.5, 1.5} };
	particleFilter::cspace X_true = { 2.12, 1.388, 0.818, Pi / 6 + Pi / 400, Pi / 12 + Pi / 220, Pi / 18 - Pi / 180 }; // true state of configuration
	//particleFilter::cspace X_true = { 0, 0, 0.818, 0, 0, 0 }; // true state of configuration
	cout << "True state: " << X_true[0] << ' ' << X_true[1] << ' ' << X_true[2] << ' ' 
		 << X_true[3] << ' ' << X_true[4] << ' ' << X_true[5] << endl;
	particleFilter::cspace b_Xprior[2] = { { 2.11, 1.4, 0.81, Pi / 6, Pi / 12, Pi / 18 },
										   { 0.03, 0.03, 0.03, Pi / 180, Pi / 180, Pi / 180 } }; // our prior belief
	//particleFilter::cspace b_Xprior[2] = { { 0, 0, 0.81, 0, 0, 0 },
	//									 { 0.001, 0.001, 0.001, Pi / 3600, Pi / 3600, Pi / 3600 } }; // our prior belief

	particleFilter pfilter(numParticles, b_Xprior, Xstd_ob, Xstd_tran, Xstd_scatter, R);
	distanceTransform *dist_transform = new distanceTransform(num_voxels);
	//dist_transform->build(cube_para);

	int N_Measure = 60; // total number of measurements
	double M_std = 0.000; // measurement error
	double M[2][3]; // measurement
	particleFilter::cspace particles_est;
	double particles_est_stat[2];

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0, 1);

	double pstart[3];
	normal_distribution<double> dist(0, M_std);
	for (int i = 0; i < N_Measure; i++) {
		// generate measurement in fixed frame, then transform it to particle frame.
		//1.117218  0.043219  0.204427
		if (i % 3 == 0)
		{
			M[1][0] = -1; M[1][1] = 0; M[1][2] = 0;
			pstart[0] = 2;
			pstart[1] = distribution(generator) * 0.07 + 0.15 + dist(generator);
			pstart[2] = distribution(generator) * 0.1 + 0.05 + dist(generator);
			if (getIntersection(mesh, pstart, M[1], M[0]) == 0)
				cout << "err" << endl;
			M[0][0] += R + dist(generator);	
		}
		else if (i % 3 == 1)
		{
			M[1][0] = 0; M[1][1] = -1; M[1][2] = 0;
			pstart[0] = distribution(generator) * 0.15 + 1.38 + dist(generator);
			pstart[1] = 1;
			pstart[2] = distribution(generator) * 0.1 + 0.05 + dist(generator);
			if (getIntersection(mesh, pstart, M[1], M[0]) == 0)
				cout << "err" << endl;
			M[0][1] += R + dist(generator);
		}
		else
		{
			M[1][0] = 0; M[1][1] = 0; M[1][2] = -1;
			pstart[0] = distribution(generator) * 1 + 0.2 + dist(generator);
			pstart[1] = distribution(generator) * 0.03 + 0.02 + dist(generator);
			pstart[2] = 1;
			if (getIntersection(mesh, pstart, M[1], M[0]) == 0)
				cout << "err" << endl;
			M[0][2] += R + dist(generator);
			
		}
		Transform(M, X_true, M);
		//rotationMatrix(X_true, rotationM);
		//multiplyM(rotationM, M[0], tempM);
		//double transition[3] = { X_true[0], X_true[1], X_true[2] };
		//addM(tempM, transition, M[0]);
		///*multiplyM(rotationM, M[1], tempM);
		//M[1][0] = tempM[0];
		//M[1][1] = tempM[1];
		//M[1][2] = tempM[2];*/

		cout << "Observation " << i << " : touch at " << M[0][0] << " " << M[0][1] << " " << M[0][2] << endl;
		cout << "Theoretic distance: " << testResult(mesh, X_true, M, R) << endl;
		auto tstart = chrono::high_resolution_clock::now();
		pfilter.addObservation(M, mesh, dist_transform, i); // update particles
		//pfilter.addObservation(M, cube_para, i);
		pfilter.estimatedDistribution(particles_est, particles_est_stat);
		auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - tstart);
		particles_est_stat[0] = 0;
		for (int k = 0; k < particleFilter::cdim; k++) {
			particles_est_stat[0] += SQ(particles_est[k] - X_true[k]);
		}
		particles_est_stat[0] /= particleFilter::cdim;
		particles_est_stat[0] = sqrt(particles_est_stat[0]);
		cout << "est: ";
		for (int k = 0; k < particleFilter::cdim; k++) {
			cout << particles_est[k] << ' ';
		}
		cout << endl;
		cout << "Real distance: " << testResult(mesh, particles_est, M, R) << endl;
		cout << "Diff: " << particles_est_stat[0] << endl;
		cout << "Var: " <<  particles_est_stat[1] << endl;
		cout << "Time: " << diff.count() << " milliseconds." << endl << endl;
	}
	delete (dist_transform);
}

/*
* Transform the touch point from particle frame
*/
void Transform(double measure[2][3], particleFilter::cspace src, double dest[2][3])
{
	double rotation[3][3];
	double tempM[3];
	rotationMatrix(src, rotation);
	multiplyM(rotation, measure[0], tempM);
	multiplyM(rotation, measure[1], dest[1]);
	double transition[3] = { src[0], src[1], src[2] };
	addM(tempM, transition, dest[0]);
}
/*
* Inverse transform the touch point to particle frame using sampled configuration
*/
void inverseTransform(double measure[3], particleFilter::cspace src, double dest[3])
{
	double rotation[3][3];
	double invRot[3][3];
	double tempM[3];
	rotationMatrix(src, rotation);
	inverseMatrix(rotation, invRot);
	double transition[3] = { src[0], src[1], src[2] };
	subtractM(measure, transition, tempM);
	multiplyM(invRot, tempM, dest);
}
void inverseTransform(double measure[2][3], particleFilter::cspace src, double dest[2][3])
{
	double rotation[3][3];
	double invRot[3][3];
	double tempM[3];
	rotationMatrix(src, rotation);
	inverseMatrix(rotation, invRot);
	double transition[3] = { src[0], src[1], src[2] };
	subtractM(measure[0], transition, tempM);
	multiplyM(invRot, tempM, dest[0]);
	multiplyM(invRot, measure[1], dest[1]);
}


/*
 * Check if the center of a voxel is within the object
 * defined by mesh arrays.
 * Input: mesh arrays
 *        voxel center
 * Output: 1 if inside
 *         0 if outside
 */
int checkInObject(vector<vec4x3> &mesh, double voxel_center[3])
{
	int countIntersections = 0;
	int num_mesh = int(mesh.size());
	double dir[3] = { 1,0,0 };
	double vert0[3], vert1[3], vert2[3];
	double *t = new double; 
	double *u = new double; 
	double *v = new double;
	std::unordered_set<double> hashset;
	for (int i = 0; i < num_mesh; i++)
	{
		vert0[0] = mesh[i][1][0];
		vert0[1] = mesh[i][1][1];
		vert0[2] = mesh[i][1][2];
		vert1[0] = mesh[i][2][0];
		vert1[1] = mesh[i][2][1];
		vert1[2] = mesh[i][2][2];
		vert2[0] = mesh[i][3][0];
		vert2[1] = mesh[i][3][1];
		vert2[2] = mesh[i][3][2];
		if (intersect_triangle(voxel_center, dir, vert0, vert1, vert2, t, u, v) == 1)
		{
			if (hashset.find(*t) == hashset.end())
			{
				hashset.insert(*t);
				countIntersections++;
			}
		}
			
	}
	if (countIntersections % 2 == 0)
	{
		return 0;
	}
	delete t, u, v;
	return 1;
}

/*
 * Find the intersection point between a ray and meshes
 * Input: mesh: mesh arrays
 * 	      pstart: start point
 * 	      dir: ray direction
 * 	      intersection: intersection point
 * Output: 1 if intersect
 *         0 if not
 */
int getIntersection(vector<vec4x3> &mesh, double pstart[3], double dir[3], double intersection[3])
{
	int num_mesh = int(mesh.size());
	double vert0[3], vert1[3], vert2[3];
	double *t = new double;
	double *u = new double;
	double *v = new double;
	double tMin = 100000;
	for (int i = 0; i < num_mesh; i++)
	{
		vert0[0] = mesh[i][1][0];
		vert0[1] = mesh[i][1][1];
		vert0[2] = mesh[i][1][2];
		vert1[0] = mesh[i][2][0];
		vert1[1] = mesh[i][2][1];
		vert1[2] = mesh[i][2][2];
		vert2[0] = mesh[i][3][0];
		vert2[1] = mesh[i][3][1];
		vert2[2] = mesh[i][3][2];
		if (intersect_triangle(pstart, dir, vert0, vert1, vert2, t, u, v) == 1 && *t < tMin)
		{
			tMin = *t;
		}

	}
	delete t, u, v;
	if (tMin == 100000)
		return 0;
	for (int i = 0; i < 3; i++)
	{
		intersection[i] = pstart[i] + dir[i] * tMin;
	}
	
	return 1;
}

/*
 * Calculate the distance between touch probe and object using 
 * mean estimated configuration after each update
 * Input: mesh: mesh arrays
 *        config: estimated mean configuration
 *        touch: touch point in particle frame
 *        R: radius of the touch probe
 * Output: distance
 */
double testResult(vector<vec4x3> &mesh, double config[6], double touch[2][3], double R)
{
	double inv_touch[2][3];
	inverseTransform(touch, config, inv_touch);
	int num_mesh = int(mesh.size());
	double vert0[3], vert1[3], vert2[3];
	double *t = new double;
	double *u = new double;
	double *v = new double;
	double tMin = 100000;
	for (int i = 0; i < num_mesh; i++)
	{
		vert0[0] = mesh[i][1][0];
		vert0[1] = mesh[i][1][1];
		vert0[2] = mesh[i][1][2];
		vert1[0] = mesh[i][2][0];
		vert1[1] = mesh[i][2][1];
		vert1[2] = mesh[i][2][2];
		vert2[0] = mesh[i][3][0];
		vert2[1] = mesh[i][3][1];
		vert2[2] = mesh[i][3][2];
		if (intersect_triangle(inv_touch[0], inv_touch[1], vert0, vert1, vert2, t, u, v) == 1 && *t < tMin)
		{
			tMin = *t;
		}
	}
	delete t, u, v;
	if (tMin == 100000)
		return 0;

	return tMin - R;
}

/*
 * Raytrace checker. Check obstacle along the ray
 * Input: mesh: mesh arrays
 *        config: estimated mean configuration
 *        start: safepoint of the joint
 *        dist: distance between center of touch probe and object
 * Output: 1 if obstacle exists
 */
int checkObstacles(vector<vec4x3> &mesh, double config[6], double start[2][3], double dist)
{
	double inv_start[2][3];
	inverseTransform(start, config, inv_start);
	int num_mesh = int(mesh.size());
	double vert0[3], vert1[3], vert2[3]; 
	double *t = new double;
	double *u = new double;
	double *v = new double;
	double tMin = 100000;
	Eigen::Vector3d normal_dir;
	Eigen::Vector3d ray_length;
	double length;
	for (int i = 0; i < num_mesh; i++)
	{
		vert0[0] = mesh[i][1][0];
		vert0[1] = mesh[i][1][1];
		vert0[2] = mesh[i][1][2];
		vert1[0] = mesh[i][2][0];
		vert1[1] = mesh[i][2][1];
		vert1[2] = mesh[i][2][2];
		vert2[0] = mesh[i][3][0];
		vert2[1] = mesh[i][3][1];
		vert2[2] = mesh[i][3][2];
		if (intersect_triangle(inv_start[0], inv_start[1], vert0, vert1, vert2, t, u, v) == 1 && *t < tMin)
		{
			tMin = *t;
			normal_dir << mesh[i][0][0], mesh[i][0][1], mesh[i][0][2];
			length = ARM_LENGTH - tMin;
			ray_length << length * inv_start[1][0], length * inv_start[1][1], length * inv_start[1][2];
		}
	}
	delete t, u, v;
	if (tMin >= ARM_LENGTH)
		return 0;
	else if (dist < 0)
	{
		double inter_dist = normal_dir.dot(ray_length);
		cout << "inter_dist: " << inter_dist << endl;
		if (inter_dist >= dist - epsilon && inter_dist <= dist + epsilon)
			return 0;
	}
		
	return 1;
}
//void voxelizeSTL(vector<vec4x3> &mesh, hashmap &boundary_voxel, double voxel_size, double R, double Xstd_ob,
//	double cube_center, double cube_size)
//{
//	int num_mesh = mesh.size();
//	//double bbox[3][2];
//	Eigen::Matrix<double, 3, 2> bbox;
//	double ix, iy, iz = 0;
//	double xstart, ystart, zstart, xend, yend, zend = 0;
//	Eigen::vec4x3d voxel_center, point_a, point_b, point_c, norm;
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
