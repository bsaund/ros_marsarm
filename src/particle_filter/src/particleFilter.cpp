#include <string.h>
#include <iostream>
#include <random>
#include <chrono>
#include <fstream>
#include <Eigen/Dense>
#include <unordered_set>
#include <unordered_map>
#include <array>
#include <chrono>
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
#define epsilon 0.0001
#define ARM_LENGTH 0.2
#define N_MIN 50
#define DISPLACE_INTERVAL 0.015
#define SAMPLE_RATE 0.50
#define MAX_ITERATION 500000
#define COV_MULTIPLIER 10.0

int total_time = 0;
int converge_count = 0;
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
	: numParticles(n_particles), maxNumParticles(n_particles), Xstd_ob(Xstd_ob),
	  Xstd_tran(Xstd_tran), Xstd_scatter(Xstd_scatter), R(R), firstObs(true)
{
	memcpy(b_Xprior, b_init, 2 * sizeof(cspace));
	//memcpy(b_Xpre, b_Xprior, 2 * sizeof(cspace));
	particles = new cspace[numParticles];
	bzero(particles, numParticles*sizeof(cspace));
	particles0 = new cspace[numParticles];
	bzero(particles0, numParticles*sizeof(cspace));
	particles_1 = new cspace[numParticles];

	createParticles(particles0, b_Xprior, numParticles);
	Eigen::MatrixXd mat = Eigen::Map<Eigen::MatrixXd>((double *)particles0, cdim, numParticles);
	Eigen::MatrixXd mat_centered = mat.colwise() - mat.rowwise().mean();
	cov_mat = (mat_centered * mat_centered.adjoint()) / double(mat.cols());
	cout << cov_mat << endl;
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
}

/*
 * Add new observation and call updateParticles() to update the particles
 * Input: obs: observation
 *        mesh: object mesh arrays
 *        dist_transform: distance transform class instance
 *        miss: if it is a miss touch
 * output: none
 */
void particleFilter::addObservation(double obs[2][3], vector<vec4x3> &mesh, distanceTransform *dist_transform, bool miss)
{
	auto timer_begin = std::chrono::high_resolution_clock::now();
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
	bool iffar = updateParticles(particles_1, particles0, particles, obs, miss, mesh, dist_transform, R, Xstd_ob, Xstd_tran);
	if (firstObs)
	{
		firstObs = false;
	}
	/*else if (iffar == true)
	{
		memcpy(particles0, particles_1, numParticles*sizeof(cspace));
	}*/
	//calcWeight(W, numParticles, Xstd_tran, particles0, particles);
	memcpy(particles_1, particles0, numParticles*sizeof(cspace));
	//resampleParticles(particles0, particles, W, numParticles);
	memcpy(particles0, particles, numParticles*sizeof(cspace));
	Eigen::MatrixXd mat = Eigen::Map<Eigen::MatrixXd>((double *)particles0, cdim, numParticles);
	Eigen::MatrixXd mat_centered = mat.colwise() - mat.rowwise().mean();
	cov_mat = (mat_centered * mat_centered.adjoint()) / double(mat.cols());
	auto timer_end = std::chrono::high_resolution_clock::now();
	auto timer_dur = timer_end - timer_begin;
	cout << cov_mat << endl;
	cout << "Estimated Mean: ";
	for (int k = 0; k < cdim; k++) {
		particles_mean[k] = 0;
		for (int j = 0; j < numParticles; j++) {
			particles_mean[k] += particles0[j][k];
		}
		particles_mean[k] /= numParticles;
		cout << particles_mean[k] << "  ";
	}
	cout << endl;
	cout << "Estimated Std: ";
	for (int k = 0; k < cdim; k++)
	{
		particles_est_stat[k] = 0;
		for (int j = 0; j < numParticles; j++) {
			particles_est_stat[k] += SQ(particles0[j][k] - particles_mean[k]);
		}
		particles_est_stat[k] = sqrt(particles_est_stat[k] / numParticles);
		cout << particles_est_stat[k] << "  ";
	}
	cout << endl;
	cout << "Estimate diff: ";
	double est_diff = sqrt(SQ(particles_mean[0] - 0.3) + SQ(particles_mean[1] - 0.3) + SQ(particles_mean[2] - 0.3)
			 		    + SQ(particles_mean[3] - 0.5) + SQ(particles_mean[4] - 0.7) + SQ(particles_mean[5] - 0.5));
	cout << est_diff << endl;
	if (est_diff >= 0.005) {
		converge_count ++;
	}
	cout << "Converge count: " << converge_count << endl;
	cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(timer_dur).count() << endl << endl;
	total_time += std::chrono::duration_cast<std::chrono::milliseconds>(timer_dur).count();
	cout << "Total time: " << total_time << endl;
	cout << "Average time: " << total_time / 20.0 << endl;
	//Eigen::MatrixXd centered = mat.rowwise() - mat.colwise().mean();
	//Eigen::MatrixXd cov = (centered.adjoint() * centered) / double(mat.rows() - 1);
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
 * Update particles (Build distance transform and sampling)
 * Input: particles_1: estimated particles before previous ones (not used here)
 *        particles0: previous estimated particles
 *        particles: current particles
 *        cur_M: current observation
 *        mesh: object mesh arrays
 *        dist_transform: distance transform class instance
 *        R: radius of the touch probe
 *        Xstd_ob: observation error
 *        Xstd_tran: gaussian kernel standard deviation when sampling
 * output: return whether previous estimate is bad (not used here)
 */
bool particleFilter::updateParticles(cspace *particles_1, cspace *particles0, cspace *particles, double cur_M[2][3], 
		bool miss, vector<vec4x3> &mesh, distanceTransform *dist_transform, double R, double Xstd_ob, double Xstd_tran)
{
	std::unordered_set<string> bins;
	std::random_device rd;
	std::normal_distribution<double> dist(0, 1);
	std::uniform_real_distribution<double> distribution(0, numParticles);
	int cdim = sizeof(cspace) / sizeof(double);
	int i = 0;
	int count = 0;
	int count2 = 0;
	int count3 = 0;
	bool iffar = false;
	cspace *b_X = particles0;
	int idx = 0;
	double tempState[6];
	double D;
	//double D2;
	double cur_inv_M[2][3];
	int num_Mean = SAMPLE_RATE * numParticles;
	double **measure_workspace = new double*[num_Mean];
	double var_measure[3] = { 0, 0, 0 };
	cspace meanConfig = { 0, 0, 0, 0, 0, 0 };
	double unsigned_dist_check = R + Xstd_ob;
	double voxel_size;
	double distTransSize;
	double mean_inv_M[3];
	double safe_point[2][3];
	//Eigen::Vector3d gradient;
	Eigen::Vector3d touch_dir;
	int num_bins = 0;
	if (!miss) {
		for (int t = 0; t < num_Mean; t++) {
			measure_workspace[t] = new double[3];
			int index = int(floor(distribution(rd)));
			//memcpy(sampleConfig[t], b_X[index], sizeof(cspace));
			for (int m = 0; m < cdim; m++) {
				meanConfig[m] += b_X[index][m];
			}
			inverseTransform(cur_M[0], b_X[index], measure_workspace[t]);
		}
		for (int m = 0; m < cdim; m++) {
			meanConfig[m] /= num_Mean;
		}
		// inverse-transform using sampled configuration
		inverseTransform(cur_M[0], meanConfig, mean_inv_M);
		for (int t = 0; t < num_Mean; t++) {
			var_measure[0] += SQ(measure_workspace[t][0] - mean_inv_M[0]);
			var_measure[1] += SQ(measure_workspace[t][1] - mean_inv_M[1]);
			var_measure[2] += SQ(measure_workspace[t][2] - mean_inv_M[2]);
			delete [] measure_workspace[t];
		}
		delete [] measure_workspace;
		var_measure[0] /= num_Mean;
		var_measure[1] /= num_Mean;
		var_measure[2] /= num_Mean;
		distTransSize = 4 * max3(sqrt(var_measure[0]), sqrt(var_measure[1]), sqrt(var_measure[2]));
		distTransSize = 100 * 0.0005;
		cout << "Touch Std: " << sqrt(var_measure[0]) << "  " << sqrt(var_measure[1]) << "  " << sqrt(var_measure[2]) << endl;
		double world_range[3][2];
		cout << "Current Inv_touch: " << mean_inv_M[0] << "    " << mean_inv_M[1] << "    " << mean_inv_M[2] << endl;
		for (int t = 0; t < 3; t++) {
			world_range[t][0] = mean_inv_M[t] - distTransSize;
			world_range[t][1] = mean_inv_M[t] + distTransSize;
			/*cout << world_range[t][0] << " to " << world_range[t][1] << endl;*/
		}
		voxel_size = distTransSize / 100;
		cout << "Voxel Size: " << voxel_size << endl;
		
		dist_transform->voxelizeSTL(mesh, world_range);
		dist_transform->build();
		
		double coeff = pow(numParticles, -0.2)/1.2155;
		Eigen::MatrixXd H_cov = coeff * cov_mat;
		//cout << " H  : " << H_cov << endl;
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(H_cov);
		Eigen::MatrixXd rot = eigenSolver.eigenvectors(); 
		Eigen::VectorXd scl = eigenSolver.eigenvalues();

		for (int j = 0; j < cdim; j++) {
		  scl(j, 0) = sqrt(scl(j, 0));
		}
		Eigen::VectorXd samples(cdim, 1);
		Eigen::VectorXd rot_sample(cdim, 1);
		//cout << "Sampled Co_std_deviation: " << scl << endl;
		// sample particles
		//touch_dir << cur_M[1][0], cur_M[1][1], cur_M[1][2];
		while (i < numParticles && i < maxNumParticles)
		{
			if(count > MAX_ITERATION || count2 > 6000 || count3 > 3000)
			{
				H_cov = COV_MULTIPLIER * H_cov;
				Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(H_cov);
				rot = eigenSolver.eigenvectors(); 
				scl = eigenSolver.eigenvalues();

				for (int j = 0; j < cdim; j++) {
				  scl(j, 0) = sqrt(scl(j, 0));
				}
				count = 0;
				//cout << "Oops, increase cov_mat" << endl << endl;
				//cout << " H  : " << H_cov << endl;
				//cout << "Sampled Co_std_deviation: " << scl << endl;
			}
			
			count += 1;
			idx = int(floor(distribution(rd)));
			for (int j = 0; j < cdim; j++)
			{
				samples(j, 0) = scl(j, 0) * dist(rd);
			}
			rot_sample = rot*samples;
			//Eigen::VectorXd returnVal = meanVec + rot*samples;
			for (int j = 0; j < cdim; j++)
			{
				tempState[j] = b_X[idx][j] + rot_sample(j, 0);
			}
			inverseTransform(cur_M, tempState, cur_inv_M);
			touch_dir << cur_inv_M[1][0], cur_inv_M[1][1], cur_inv_M[1][2];
			// reject particles ourside of distance transform
			if (cur_inv_M[0][0] >= dist_transform->world_range[0][1] || cur_inv_M[0][0] <= dist_transform->world_range[0][0] ||
				cur_inv_M[0][1] >= dist_transform->world_range[1][1] || cur_inv_M[0][1] <= dist_transform->world_range[1][0] ||
				cur_inv_M[0][2] >= dist_transform->world_range[2][1] || cur_inv_M[0][2] <= dist_transform->world_range[2][0]) {
				continue;
			}
			int xind = int(floor((cur_inv_M[0][0] - dist_transform->world_range[0][0]) / dist_transform->voxel_size));
			int yind = int(floor((cur_inv_M[0][1] - dist_transform->world_range[1][0]) / dist_transform->voxel_size));
			int zind = int(floor((cur_inv_M[0][2] - dist_transform->world_range[2][0]) / dist_transform->voxel_size));
			D = (*dist_transform->dist_transform)[xind][yind][zind];
			// if (xind >= (dist_transform->num_voxels[0] - 1) || yind >= (dist_transform->num_voxels[1] - 1) || zind >= (dist_transform->num_voxels[2] - 1))
			// 	continue;
				
			double dist_adjacent[3] = { 0, 0, 0 };
			if (D <= unsigned_dist_check)
			{
				// if (xind < (dist_transform->num_voxels[0] - 1) && yind < (dist_transform->num_voxels[1] - 1) && zind < (dist_transform->num_voxels[2] - 1))
				// {
				// 	dist_adjacent[0] = (*dist_transform->dist_transform)[xind + 1][yind][zind];
				// 	dist_adjacent[1] = (*dist_transform->dist_transform)[xind][yind + 1][zind];
				// 	dist_adjacent[2] = (*dist_transform->dist_transform)[xind][yind][zind + 1];
				// 	//gradient /= gradient.norm();
				// }
				// else
				// 	continue;
				// gradient[0] = dist_adjacent[0] - D;
				// gradient[1] = dist_adjacent[1] - D;
				// gradient[2] = dist_adjacent[2] - D;
				count2 ++;
				if (checkIntersections(mesh, cur_inv_M[0], cur_inv_M[1], ARM_LENGTH, D))
					continue;
				D -= R;
				// if (checkInObject(mesh, cur_inv_M[0]) == 1 && D != 0)
				// {
				// 	// if (gradient.dot(touch_dir) <= epsilon)
				// 	// 	continue;
				// 	D = -D - R;
				// }
				// else if (D == 0) 
				// {
				// 	// double tmp[3] = { cur_inv_M[0][0] + dist_transform->voxel_size, cur_inv_M[0][1], cur_inv_M[0][2] };
				// 	// if (checkInObject(mesh, tmp) == 1)
				// 	// 	gradient[0] = -gradient[0];
				// 	// tmp[0] -= dist_transform->voxel_size;
				// 	// tmp[1] += dist_transform->voxel_size;
				// 	// if (checkInObject(mesh, tmp) == 1)
				// 	// 	gradient[1] = -gradient[1];
				// 	// tmp[1] -= dist_transform->voxel_size;
				// 	// tmp[2] += dist_transform->voxel_size;
				// 	// if (checkInObject(mesh, tmp) == 1)
				// 	// 	gradient[2] = -gradient[2];
				// 	// if (gradient.dot(touch_dir) >= -epsilon)
				// 	// 	continue;
				// 	D = - R;
				// }
				// else
				// {
				// 	// if (gradient.dot(touch_dir) >= -epsilon)
				// 	// 	continue;
				// 	D = D - R;
				// }	
			}
			else
				continue;
			if (D >= -Xstd_ob && D <= Xstd_ob)
			{
				
				// safe_point[1][0] = cur_M[1][0];
				// safe_point[1][1] = cur_M[1][1];
				// safe_point[1][2] = cur_M[1][2];
				// safe_point[0][0] = cur_M[0][0] - cur_M[1][0] * ARM_LENGTH;
				// safe_point[0][1] = cur_M[0][1] - cur_M[1][1] * ARM_LENGTH;
				// safe_point[0][2] = cur_M[0][2] - cur_M[1][2] * ARM_LENGTH;
				// count3 ++;
				// if (checkObstacles(mesh, tempState, safe_point , D + R) == 1)
				// 	continue;
				for (int j = 0; j < cdim; j++)
				{
					particles[i][j] = tempState[j];
				}
				if (checkEmptyBin(&bins, particles[i]) == 1) {
					num_bins++;
					if (i >= N_MIN) {
						//int numBins = bins.size();
						numParticles = min2(maxNumParticles, max2((num_bins - 1) * 2, N_MIN));
					}
				}
				//double d = testResult(mesh, particles[i], cur_M, R);
				//if (d > 0.01)
				//	cout << cur_inv_M[0][0] << "  " << cur_inv_M[0][1] << "  " << cur_inv_M[0][2] << "   " << d << "   " << D << //"   " << gradient << "   " << gradient.dot(touch_dir) << 
				//	     "   " << dist_adjacent[0] << "   " << dist_adjacent[1] << "   " << dist_adjacent[2] << "   " << particles[i][2] << endl;
				i += 1;
			}			
		}
		cout << "Number of total iterations: " << count << endl;
		cout << "Number of iterations after unsigned_dist_check: " << count2 << endl;
		cout << "Number of iterations before safepoint check: " << count3 << endl;
		cout << "Number of occupied bins: " << num_bins << endl;
	}
	else {
		// cast multiple rays to check intersections
		double touch_mnt;
		while (i < numParticles)
		{
			idx = int(floor(distribution(rd)));
			for (int j = 0; j < cdim; j++)
			{
				tempState[j] = b_X[idx][j] + Xstd_tran * dist(rd);
			}
			// inverseTransform(cur_M[0], tempState, cur_inv_M[0]);
			// inverseTransform(cur_M[1], tempState, cur_inv_M[1]);
			touch_dir << cur_M[0][0] - cur_M[1][0],
						 cur_M[0][1] - cur_M[1][1],
						 cur_M[0][2] - cur_M[1][2];
			touch_mnt = touch_dir.norm();
			touch_dir = touch_dir / touch_mnt;
			// reject particles ourside of distance transform
			
			safe_point[1][0] = touch_dir[0];
			safe_point[1][1] = touch_dir[1];
			safe_point[1][2] = touch_dir[2];
			safe_point[0][0] = cur_M[1][0] - touch_dir[0] * ARM_LENGTH;
			safe_point[0][1] = cur_M[1][1] - touch_dir[1] * ARM_LENGTH;
			safe_point[0][2] = cur_M[1][2] - touch_dir[2] * ARM_LENGTH;
			if (checkObstacles(mesh, tempState, safe_point, touch_mnt + ARM_LENGTH, 0) == 1)
				continue;
			for (int j = 0; j < cdim; j++)
			{
				particles[i][j] = tempState[j];
			}
			//double d = testResult(mesh, particles[i], cur_M, R);
			//if (d > 0.01)
			//	cout << cur_inv_M[0][0] << "  " << cur_inv_M[0][1] << "  " << cur_inv_M[0][2] << "   " << d << "   " << D << //"   " << gradient << "   " << gradient.dot(touch_dir) << 
			//	     "   " << dist_adjacent[0] << "   " << dist_adjacent[1] << "   " << dist_adjacent[2] << "   " << particles[i][2] << endl;
			i += 1;
			std::cout << "Miss!" << endl;
		}
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
	vector<vec4x3> mesh = importSTL("boeing_part_binary.stl");
	int numParticles = 500; // number of particles
	double Xstd_ob = 0.001;
	double Xstd_tran = 0.0035;
	double Xstd_scatter = 0.0001;
	//double voxel_size = 0.0005; // voxel size for distance transform.
	int num_voxels[3] = { 200,200,200 };
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
	particleFilter::cspace particles_est_stat;
	double particle_est_diff;

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
		particle_est_diff = 0;
		for (int k = 0; k < particleFilter::cdim; k++) {
			particle_est_diff += SQ(particles_est[k] - X_true[k]);
		}
		particle_est_diff /= particleFilter::cdim;
		particle_est_diff = sqrt(particle_est_diff);
		cout << "est: ";
		for (int k = 0; k < particleFilter::cdim; k++) {
			cout << particles_est[k] << ' ';
		}
		cout << endl;
		cout << "Real distance: " << testResult(mesh, particles_est, M, R) << endl;
		cout << "Diff: " << particle_est_diff << endl;
		cout << "Var: ";
		for (int k = 0; k < particleFilter::cdim; k++) {
			cout << particles_est_stat[k] << ' ';
		}
		cout << endl;
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
 * Check if the configuration falls into an empty bin
 * Input: set: Set to store non-empty bins
 *        config: Sampled particle
 * Output: 1 if empty
 *         0 if not, and then add the bin to set
 */
int checkEmptyBin(std::unordered_set<string> *set, particleFilter::cspace config)
{
	string s = "";
	for (int i = 0; i < particleFilter::cdim; i++) {
		s += floor(config[i] / DISPLACE_INTERVAL);
		s += ":";
	}
	if (set->find(s) == set->end()) {
		set->insert(s);
		return 1;
	}
	return 0;
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
	return checkObstacles(mesh, config, start, ARM_LENGTH, dist);
}
int checkObstacles(vector<vec4x3> &mesh, double config[6], double start[2][3], double check_length, double dist)
{
	double inv_start[2][3];
	int countIntersections = 0;
	inverseTransform(start, config, inv_start);
	int num_mesh = int(mesh.size());
	double vert0[3], vert1[3], vert2[3]; 
	double *t = new double;
	double *u = new double;
	double *v = new double;
	double tMin = 100000;
	Eigen::Vector3d normal_dir;
	Eigen::Vector3d ray_length;
	double inside_length;
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
		if (intersect_triangle(inv_start[0], inv_start[1], vert0, vert1, vert2, t, u, v) == 1)
		{
			if (*t < tMin)
			{
				tMin = *t;
				normal_dir << mesh[i][0][0], mesh[i][0][1], mesh[i][0][2];
				inside_length = check_length - tMin;
				ray_length << inside_length * inv_start[1][0], inside_length * inv_start[1][1], inside_length * inv_start[1][2];
			}
			if (hashset.find(*t) == hashset.end())
			{
				hashset.insert(*t);
				countIntersections++;
			}
		}
	}
	delete t, u, v;
	if (countIntersections % 2 == 1)
	{
		return 1;
	}
	if (tMin >= check_length)
		return 0;
	else if (dist < 0)
	{
		double inter_dist = normal_dir.dot(ray_length);
		//cout << "inter_dist: " << inter_dist << endl;
		if (inter_dist >= dist - epsilon && inter_dist <= dist + epsilon)
			return 0;
	}
		
	return 1;
}

int checkIntersections(vector<vec4x3> &mesh, double voxel_center[3], double dir[3], double check_length, double &dist)
{
	int countIntersections = 0;
	int countIntRod = 0;
	int num_mesh = int(mesh.size());
	double vert0[3], vert1[3], vert2[3];
	double *t = new double; 
	double *u = new double; 
	double *v = new double;
	double tMax = 0;
	double ray_dir[3] = {-dir[0], -dir[1], -dir[2]};
	Eigen::Vector3d normal_dir;
	Eigen::Vector3d ray_length;
	double inside_length;
	std::unordered_set<double> hashset;
	//std::unordered_map<double, int> hashmap;
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
		if (intersect_triangle(voxel_center, ray_dir, vert0, vert1, vert2, t, u, v) == 1)
		{
			if (hashset.find(*t) == hashset.end())
			{
				if (*t < check_length && *t > tMax)
				{
					countIntRod++;
					tMax = *t;
					normal_dir << mesh[i][0][0], mesh[i][0][1], mesh[i][0][2];
				}
				else if (*t < check_length)
					countIntRod++;
				hashset.insert(*t);
				countIntersections++;
			}
		}
			
	}
	delete t, u, v;
	if (countIntersections % 2 == 0)
	{
		if (tMax > 0)
			return 1;
		return 0;
	}
	else {
		dist = -dist;
		if (tMax > 0 && countIntRod % 2 == 1) {
			ray_length << tMax * dir[0], tMax * dir[1], tMax * dir[2];
			double inter_dist = normal_dir.dot(ray_length);
			if (inter_dist >= dist - epsilon && inter_dist <= dist + epsilon)
				return 0;
		}
		return 1;
	}
}