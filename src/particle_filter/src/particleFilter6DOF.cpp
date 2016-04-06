#include <string.h>
#include <iostream>
#include <random>
#include <cmath>
#include <chrono>

#include "particleFilter6DOF.h"
#include "matrix6DOF.h"
using namespace std;

# define Pi          3.141592653589793238462643383279502884L

#define SQ(x) ((x)*(x))
#define bzero(b,len) (memset((b), '\0', (len)), (void) 0)

particleFilter::particleFilter(int n_particles, cspace b_init[2],
			       double Xstd_ob, double Xstd_tran,
			       double Xstd_scatter, double R)
  : N(n_particles), Xstd_ob(Xstd_ob), Xstd_tran(Xstd_tran),
    Xstd_scatter(Xstd_scatter), R(R), firstObs(true)
{
  memcpy(b_Xprior, b_init, 2 * sizeof(cspace));
  memcpy(b_Xpre, b_Xprior, 2 * sizeof(cspace));

  X = new cspace[N];
  bzero(X, N*sizeof(cspace));
  X0 = new cspace[N];
  bzero(X0, N*sizeof(cspace));

  createParticles(X0, b_Xprior, N);

  X_1 = new cspace[N];
  W = new double[N];
}

void particleFilter::getAllParticles(cspace *particles)
{
  for(int i=0; i<N; i++){
    for(int j=0; j<cdim; j++){
      particles[i][j] = X[i][j];
    }
  }
}

/**
 *  Adds an observation and updates the particles accordingly
 *   obs: the contact point of the observation
 *   cube: the x, y, z dimensions of the cube (prism) being touched
 * 
 */
void particleFilter::addObservation(double obs[3], double cube[3], distanceTransform *dist_transform, int idx_obs)
{
  std::default_random_engine generator;
  normal_distribution<double> dist2(0, Xstd_scatter);

  if (!firstObs) {
    bzero(b_Xpre, 2 * sizeof(cspace));
    for (int k = 0; k < cdim; k++) {
      for (int j = 0; j < N; j++) {
	X0[j][k] += dist2(generator);
	b_Xpre[0][k] += X0[j][k];
      }
      b_Xpre[0][k] /= N;
      for (int j = 0; j < N; j++) {
	b_Xpre[1][k] += SQ(X0[j][k] - b_Xpre[0][k]);
      }
      b_Xpre[1][k] = sqrt(b_Xpre[1][k] / N);
    }
  }
  bool iffar = updateParticles(X_1, X0, X, obs, cube, idx_obs, dist_transform, N, R, Xstd_ob, Xstd_tran);
  if (firstObs)
    {
      firstObs = false;
      memcpy(X0, X, N*sizeof(cspace));
      memcpy(X_1, X, N*sizeof(cspace));
    }
  else if (iffar == true)
    {
      memcpy(X0, X_1, N*sizeof(cspace));
    }
  //calcWeight(W, N, Xstd_tran, X0, X);
  memcpy(X_1, X0, N*sizeof(cspace));
  //resampleParticles(X0, X, W, N);
  memcpy(X0, X, N*sizeof(cspace));

  for (int k = 0; k < cdim; k++) {
    X_est[k] = 0;
    for (int j = 0; j < N; j++) {
      X_est[k] += X0[j][k];
    }
    X_est[k] /= N;
  }
  X_est_stat[1] = 0;
  for (int j = 0; j < N; j++)
    {
      for (int k = 0; k < cdim; k++) {
	X_est_stat[1] += SQ(X0[j][k] - X_est[k]);
      }
    }
  X_est_stat[1] = sqrt(X_est_stat[1] / N);

  if (X_est_stat[1] < 0.005 && (abs(X_est[0] - b_Xpre[0][0])>0.001 ||
				abs(X_est[1] - b_Xpre[0][1])>0.001 ||
				abs(X_est[2] - b_Xpre[0][2])>0.001 ||
				abs(X_est[3] - b_Xpre[0][3])>0.001 ||
				abs(X_est[4] - b_Xpre[0][4]) > 0.001 ||
				abs(X_est[5] - b_Xpre[0][5]) > 0.001))
    Xstd_scatter = 0.01;
  else
    Xstd_scatter = 0.0001;
}

void particleFilter::createParticles(cspace *X, cspace b_Xprior[2],
				     int n_particles)
{
  random_device rd;
  mt19937 e2(rd());
  normal_distribution<double> dist(0, 1);
  int cdim = sizeof(cspace) / sizeof(double);
  for (int i = 0; i < n_particles; i++){
    for (int j = 0; j < cdim; j++){
      X[i][j] = b_Xprior[0][j] + b_Xprior[1][j] * (dist(e2));
    }
  }
}

bool particleFilter::updateParticles(cspace *X_1, cspace *X0, cspace *X, double cur_M[3], 
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
  cspace *b_X = X0;
  int dir = idx_Measure % 3;
  int idx = 0;
  double tempState[6];
  double D;
  double rotation[3][3];
  double tempRot[3][3];
  double invRot[3][3];
  double cur_inv_M[3];
  double tempM[3];
  int num_Mean = 1000;
  //cspace *sampleConfig = new cspace[num_Mean];
  cspace meanConfig = { 0, 0, 0, 0, 0, 0 };
  for (int t = 0; t < num_Mean; t++) {
    int index = int(floor(distribution(e2)));
    //memcpy(sampleConfig[t], b_X[index], sizeof(cspace));
    for (int m = 0; m < cdim; m++) {
      meanConfig[m] += b_X[index][m] / num_Mean;
    }
  }
  cout << "Measurement: " << cur_M[0]<< ", " << cur_M[1] << ", " << cur_M[2] << endl;
  cout << "MeanConfig:" << meanConfig[0] << ", " << meanConfig[1] << ", " << meanConfig[2] << ", " << meanConfig[3] << ", " << meanConfig[4] << ", " << meanConfig[5] << endl;
  // inverse-transform using sampled configuration
  double rotationC[3][3] = { { cos(meanConfig[cdim - 1]), -sin(meanConfig[cdim - 1]), 0 },
			     { sin(meanConfig[cdim - 1]), cos(meanConfig[cdim - 1]), 0 },
			     { 0, 0, 1 } };
  double rotationB[3][3] = { { cos(meanConfig[cdim - 2]), 0 , sin(meanConfig[cdim - 2]) },
			     { 0, 1, 0 },
			     { -sin(meanConfig[cdim - 2]), 0, cos(meanConfig[cdim - 2]) } };
  double rotationA[3][3] = { { 1, 0, 0 },
			     { 0, cos(meanConfig[cdim - 3]), -sin(meanConfig[cdim - 3]) },
			     { 0, sin(meanConfig[cdim - 3]), cos(meanConfig[cdim - 3]) } };
  multiplyM(rotationC, rotationB, tempRot);
  multiplyM(tempRot, rotationA, rotation);
  inverseMatrix(rotation, invRot);
  double transition[3] = { meanConfig[0], meanConfig[1], meanConfig[2] };
  subtractM(cur_M, transition, tempM);
  multiplyM(invRot, tempM, cur_inv_M);

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
      //	b_X = X_1;
      //	//count = 0;
      //	i = 0;
      //}
      idx = floor(distribution(e2));
      for (int j = 0; j < cdim; j++)
	{
	  tempState[j] = b_X[idx][j] + Xstd_tran * dist(e2);
	}
      //double tempState[6] = { 2.11, 1.388, 0.818, Pi / 6 + Pi / 400, Pi / 12 + Pi / 420, Pi / 18 - Pi / 380 };
      double rotationC[3][3] = { { cos(tempState[cdim - 1]), -sin(tempState[cdim - 1]), 0 },
				 { sin(tempState[cdim - 1]), cos(tempState[cdim - 1]), 0 },
				 { 0, 0, 1 } };
      double rotationB[3][3] = { { cos(tempState[cdim - 2]), 0 , sin(tempState[cdim - 2]) },
				 { 0, 1, 0 },
				 { -sin(tempState[cdim - 2]), 0, cos(tempState[cdim - 2]) } };
      double rotationA[3][3] = { { 1, 0, 0 },
				 { 0, cos(tempState[cdim - 3]), -sin(tempState[cdim - 3]) },
				 { 0, sin(tempState[cdim - 3]), cos(tempState[cdim - 3]) } };
      multiplyM(rotationC, rotationB, tempRot);
      multiplyM(tempRot, rotationA, rotation);
      inverseMatrix(rotation, invRot);
      double transition[3] = { tempState[0], tempState[1], tempState[2] };
      subtractM(cur_M, transition, tempM);
      multiplyM(invRot, tempM, cur_inv_M);
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
      D = dist_transform->dist_transform[int(floor((cur_inv_M[0] - dist_transform->world_range[0][0]) / dist_transform->voxel_size))]
	[int(floor((cur_inv_M[1] - dist_transform->world_range[1][0]) / dist_transform->voxel_size))]
	[int(floor((cur_inv_M[2] - dist_transform->world_range[2][0]) / dist_transform->voxel_size))] - R;
      //cout << D << endl << endl;
      if (D >= -Xstd_ob && D <= Xstd_ob){
	if(i<10){
	  cout << D << endl;
	}
	for (int j = 0; j < cdim; j++){
	  X[i][j] = tempState[j];
	}
	//cout << D << "       " << cur_inv_M[dir] - cube[dir] / 2 - R << endl;
	i += 1;
      }
      count += 1;
		
    }
  cout << "count: " << count << endl;
  return iffar;
}

//void particleFilter::calcWeight(double *W, int n_particles, double Xstd_tran,
//	cspace *X0, cspace *X)
//{
//	double A = 1.0 / (sqrt(2 * Pi) * Xstd_tran);
//	double B = -0.5 / SQ(Xstd_tran);
//	double sum = 0;
//	for (int k = 0; k < n_particles; k++) {
//		for (int m = 0; m < n_particles; m++) {
//			W[k] += A*exp(B*(SQ(X0[m][0] - X[k][0]) + SQ(X0[m][1] - X[k][1]) +
//				SQ(X0[m][2] - X[k][2])));
//		}
//		sum += W[k];
//	}
//	for (int k = 0; k < n_particles; k++) {
//		W[k] /= sum;
//	}
//};

//void particleFilter::resampleParticles(cspace *X0, cspace *X, double *W,
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
//				X0[i][0] = X[0][0];
//				X0[i][1] = X[0][1];
//				X0[i][2] = X[0][2];
//			}
//			else if (Cum_sum[j - 1] < t && t <= Cum_sum[j])
//			{
//				X0[i][0] = X[j][0];
//				X0[i][1] = X[j][1];
//				X0[i][2] = X[j][2];
//			}
//		}
//	}
//}


#if 0
int main()
{
  int N = 500; // number of particles
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

  particleFilter pfilter(N, b_Xprior, Xstd_ob, Xstd_tran, Xstd_scatter, R);
  distanceTransform *dist_transform = new distanceTransform(range, voxel_size);
  //dist_transform->build(cube_para);

  int N_Measure = 60; // total number of measurements
  double M_std = 0.0001; // measurement error
  double M[3]; // measurement
  double tempM[3];
  particleFilter::cspace X_est;
  double X_est_stat[2];
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
    pfilter.estimatedDistribution(X_est, X_est_stat);
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - tstart);
    X_est_stat[0] = 0;
    for (int k = 0; k < particleFilter::cdim; k++) {
      X_est_stat[0] += SQ(X_est[k] - X_true[k]);
    }
    X_est_stat[0] = sqrt(X_est_stat[0]);
    cout << "est: ";
    for (int k = 0; k < particleFilter::cdim; k++) {
      cout << X_est[k] << ' ';
    }
    cout << endl;
    cout << "Diff: " << X_est_stat[0] << endl;
    cout << "Var: " <<  X_est_stat[1] << endl;
    cout << "Time: " << diff.count() << " milliseconds." << endl << endl;
  }
}

#endif
