#include <ros/ros.h>
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

#define COMBINE_RAYCASTING
#define ADAPTIVE_NUMBER
#define ADAPTIVE_BANDWIDTH

# define Pi          3.141592653589793238462643383279502884L

#define SQ(x) ((x)*(x))
#define bzero(b,len) (memset((b), '\0', (len)), (void) 0)
#define max3(a,b,c) ((a>b?a:b)>c?(a>b?a:b):c)
#define max2(a,b) (a>b?a:b)
#define min3(a,b,c) ((a<b?a:b)<c?(a<b?a:b):c)
#define min2(a,b) (a<b?a:b)
typedef array<array<float, 3>, 4> vec4x3;
#define epsilon 0.0001
#define ARM_LENGTH 0.8
#define N_MIN 50
#define DISPLACE_INTERVAL 0.015
#define SAMPLE_RATE 0.50
#define MAX_ITERATION 100000
#define COV_MULTIPLIER 5.0
#define MIN_STD 1.0e-6

int total_time = 0;
int converge_count = 0;


std::random_device ParticleDistribution::rd;

cspace ParticleDistribution::sampleFrom(){

  std::normal_distribution<double> dist(0, 1);
  std::uniform_real_distribution<double> distribution(0, this->size());

  Eigen::VectorXd samples(cdim, 1);
  Eigen::VectorXd rot_sample(cdim, 1);

  cspace sampled;
  int idx = int(floor(distribution(rd)));

  for (int j = 0; j < cdim; j++) {
    samples(j, 0) = scl(j, 0) * dist(rd);
  }


  rot_sample = rot*samples;
  for (int j = 0; j < cdim; j++) {
    /* TODO: use quaternions instead of euler angles */
    sampled[j] = this->at(idx).at(j) + rot_sample(j, 0);
  }
  return sampled;
}


/*
 *  Update the adaptive bandwidth for the particle distribution
 *   This should be called before sampling
 */
void ParticleDistribution::updateBandwidth(){

  Eigen::MatrixXd mat = Eigen::Map<Eigen::MatrixXd>((double *)this->data(), cdim, this->size());
  Eigen::MatrixXd mat_centered = mat.colwise() - mat.rowwise().mean();
  Eigen::MatrixXd cov_mat = (mat_centered * mat_centered.adjoint()) / double(max2(mat.cols()-1,1));

  double coeff = pow(this->size(), -0.2) * 0.87055/1.2155/1.2155;
  Eigen::MatrixXd H_cov = coeff * cov_mat;
  // cout << "H_cov: " << H_cov << endl;

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(H_cov);
  rot = eigenSolver.eigenvectors(); 
  scl = eigenSolver.eigenvalues();

  for (int j = 0; j < cdim; j++) {
    scl(j, 0) = sqrt(scl(j, 0));
  }
    
}



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
    Xstd_tran(Xstd_tran), Xstd_scatter(Xstd_scatter), R(R)
{
  b_Xprior[0] = b_init[0];
  b_Xprior[1] = b_init[1];
  particles.resize(numParticles);
  particlesPrev.resize(numParticles);


  createParticles(particlesPrev, b_Xprior, numParticles);
  particlesPrev.updateBandwidth();
}

void particleFilter::getAllParticles(std::vector<cspace> &particles_dest)
{
  particles_dest = particlesPrev;
}

/*
 * Create initial particles at start
 * Input: particles
 *        b_Xprior: prior belief
 *        n_partcles: number of particles
 * output: none
 */
void particleFilter::createParticles(Particles &particles_dest, cspace b_Xprior[2],
				     int n_particles)
{
  random_device rd;
  normal_distribution<double> dist(0, 1);
  for (int i = 0; i < n_particles; i++) {
    for (int j = 0; j < cdim; j++) {
      particles_dest[i][j] = b_Xprior[0][j] + b_Xprior[1][j] * (dist(rd));
    }
  }
}


void particleFilter::addObservation(double obs[2][3], vector<vec4x3> &mesh, distanceTransform *dist_transform, bool miss)
{
  FixedTransform tf(0,0,0,0,0,0);
  addObservation(obs, mesh, dist_transform, tf, miss);
}


/*
 * Add new observation and call updateParticles() to update the particles
 * Input: obs: observation
 *        mesh: object mesh arrays
 *        dist_transform: distance transform class instance
 *        miss: if it is a miss touch
 * output: none
 */
void particleFilter::addObservation(double obs[2][3], vector<vec4x3> &mesh, distanceTransform *dist_transform, RandomTransform &tf, bool miss)
{
  numObs++;

  auto timer_begin = std::chrono::high_resolution_clock::now();

  bool iffar = updateParticles(obs, mesh, dist_transform, tf, miss);
  
  particlesPrev = particles;
  particlesPrev.updateBandwidth();

  auto timer_end = std::chrono::high_resolution_clock::now();
  auto timer_dur = timer_end - timer_begin;

  cspace particles_mean, tmp2;
  estimateGaussian(particles_mean, tmp2);
  cout << "Estimate diff: ";
  double est_diff = sqrt(SQ(particles_mean[0] - 0.3) + SQ(particles_mean[1] - 0.3) + 
			 SQ(particles_mean[2] - 0.3) + SQ(particles_mean[3] - 0.5) + 
			 SQ(particles_mean[4] - 0.7) + SQ(particles_mean[5] - 0.5));
  cout << est_diff << endl;
  if (est_diff >= 0.005) {
    converge_count ++;
  }
  cout << "Converge count: " << converge_count << endl;
  cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(timer_dur).count() << endl;
  total_time += std::chrono::duration_cast<std::chrono::milliseconds>(timer_dur).count();
  cout << "Total time: " << total_time << endl;
  cout << "Average time: " << total_time / 20.0 << endl << endl;

}

void particleFilter::estimateGaussian(cspace &x_mean, cspace &x_est_stat, bool record) const {

  ofstream outputData;
  if(record){
    cout << "Saving data to file\n";
    outputData.open("/home/bsaund/MeanCov.txt", std::ios_base::app);
    outputData << numObs << "\t";
  }

  cout << "Estimated Mean: ";
  for (int k = 0; k < cdim; k++) {
    x_mean[k] = 0;
    for (int j = 0; j < numParticles; j++) {
      x_mean[k] += particlesPrev[j][k];
    }
    x_mean[k] /= numParticles;
    cout << x_mean[k] << "  ";
    if(record)
      outputData << x_mean[k] << "\t";
  }
  cout << endl;
  cout << "Estimated Std: ";
  for (int k = 0; k < cdim; k++) {
    x_est_stat[k] = 0;
    for (int j = 0; j < numParticles; j++) {
      x_est_stat[k] += SQ(particlesPrev[j][k] - x_mean[k]);
    }
    x_est_stat[k] = sqrt(x_est_stat[k] / numParticles);
    cout << x_est_stat[k] << "  ";
    if(record)
      outputData << x_est_stat[k] << "\t";
  }
  cout << endl;
  if(record){
    outputData << "\n";
    outputData.close();
  }
}


/*
 *  Used primarily to print output with logrithmic frequency
 */
int isPowerOfTwo (unsigned int x){
  return ((x != 0) && ((x & (~x + 1)) == x));
}



/*
 * Build a local distance field around the point (and direction) cur_M
 *  This uses the mean and variance of the particles to determine the location
 *  and size of the distance field
 */
void particleFilter::buildDistTransformAroundPoint(const double cur_M[2][3], vector<vec4x3> &mesh,
						   distanceTransform *dist_transform){
  int num_Mean = SAMPLE_RATE * numParticles;
  std::vector<std::array<double,3>> measure_workspace;
  measure_workspace.resize(num_Mean);
  std::random_device rd;
  std::uniform_real_distribution<double> distribution(0, numParticles);
  double var_measure[3] = { 0, 0, 0 };
  cspace meanConfig = { 0, 0, 0, 0, 0, 0 };
  double mean_inv_M[3];
  Particles b_X = particlesPrev;
  double distTransSize;


  for (int t = 0; t < num_Mean; t++) {
    int index = int(floor(distribution(rd)));
    for (int m = 0; m < cdim; m++) {
      meanConfig[m] += b_X[index][m];
    }
    inverseTransform(cur_M[0], b_X[index], measure_workspace[t].data());
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
  }
  var_measure[0] /= num_Mean;
  var_measure[1] /= num_Mean;
  var_measure[2] /= num_Mean;
  distTransSize = max2(2 * max3(sqrt(var_measure[0]), sqrt(var_measure[1]), sqrt(var_measure[2])), 20 * Xstd_ob);
  // distTransSize = 100 * 0.0005;
  cout << "Touch Std: " << sqrt(var_measure[0]) << "  " << sqrt(var_measure[1]) << "  " << sqrt(var_measure[2]) << endl;
  double world_range[3][2];
  cout << "Current Inv_touch: " << mean_inv_M[0] << "    " << mean_inv_M[1] << "    " << mean_inv_M[2] << endl;
  for (int t = 0; t < 3; t++) {
    world_range[t][0] = mean_inv_M[t] - distTransSize;
    world_range[t][1] = mean_inv_M[t] + distTransSize;
    /*cout << world_range[t][0] << " to " << world_range[t][1] << endl;*/
  }
		
  dist_transform->voxelizeSTL(mesh, world_range);
  dist_transform->build();
  
}



bool insideBounds(double point[3], double bounds[3][2]){
  for(int i=0; i<3; i++){
    if(point[i] <= bounds[i][0])
      return false;
    if(point[i] >= bounds[i][1])
      return false;
  }
  return true;
}


/*
 * Update particles (Build distance transform and sampling)
 * Input: particlesPrev: previous estimated particles
 *        particles: current particles
 *        cur_M: current observation
 *        mesh: object mesh arrays
 *        dist_transform: distance transform class instance
 *        R: radius of the touch probe
 *        Xstd_ob: observation error
 *        Xstd_tran: gaussian kernel standard deviation when sampling
 * output: return whether previous estimate is bad (not used here)
 */
bool particleFilter::updateParticles(const double cur_M[2][3], vector<vec4x3> &mesh, distanceTransform *dist_transform, bool miss)
{
  FixedTransform tf(0,0,0,0,0,0);
  return updateParticles(cur_M, mesh, dist_transform, tf, miss);
}


bool particleFilter::updateParticles(const double measurementWorldFr[2][3], vector<vec4x3> &mesh, distanceTransform *dist_transform, RandomTransform &tf, bool miss){
  std::unordered_set<string> bins;
  std::random_device rd;
  std::normal_distribution<double> dist(0, 1);
  std::uniform_real_distribution<double> distribution(0, numParticles);
  int i = 0;
  int count = 0;
  int count2 = 0;
  int count3 = 0;
  bool iffar = false;
  Particles b_X = particlesPrev;
  int idx = 0;
  cspace tempState;
  double D;
  double measurementPartFr[2][3];

  double unsigned_dist_check = R + Xstd_ob;
  double signed_dist_check = 2 * Xstd_ob;



  double safe_point[2][3];
  //Eigen::Vector3d gradient;
  Eigen::Vector3d touch_dir;
  int num_bins = 0;
  int count_bar = 0;
  double trans_M[2][3];
  Transform(measurementWorldFr, tf.sampleTransform(), trans_M);
  

  if (!miss) {
    buildDistTransformAroundPoint(trans_M, mesh, dist_transform);
    cout << "Finish building DT !!" << endl;
    cout << " trans_M: " << trans_M[0][0];
    cout << ", " << trans_M[0][1] << ", " << trans_M[0][2] << "\n";


    /*   Begin Rejection Sampling */
    while (i < numParticles && i < maxNumParticles) {

      tempState = particlesPrev.sampleFrom();

      cspace stf = tf.sampleTransform();
      Transform(measurementWorldFr, stf, trans_M);

      // inverseTransform(measurementWorldFr, tf.sampleTransform(), measurementPartFr);
      inverseTransform(measurementWorldFr, tempState, measurementPartFr);
      Transform(measurementPartFr, stf, measurementPartFr);

      // cout << "sampledtf" << stf[0] << ", " << stf[1] << ", " << 
      // 	stf[2] << ", " << stf[3] << ", " << stf[4] << ", " << stf[5] << "\n";
      // cout << "curM" << measurementWorldFr[0][0] << ", " << measurementWorldFr[0][1] << ", " << measurementWorldFr[0][2] << "\n";

      // cout << "curinvM" << measurementPartFr[0][0] << ", " << measurementPartFr[0][1] << ", " << measurementPartFr[0][2] << "\n";

      count += 1;
      // if(isPowerOfTwo(count) && count > 1000){
      // 	cout << "Sampled " << count << " particles\t";
      // 	cout << "count2: " << count2 << "\t count3: " << count3 << "\n";

      // 	cout << " trans_M: " << trans_M[0][0];
      // 	cout << ", " << trans_M[0][1] << ", " << trans_M[0][2] << "\n";

      // 	cout << " measurementPartFr: " << measurementPartFr[0][0];
      // 	cout << ", " << measurementPartFr[0][1] << ", " << measurementPartFr[0][2] << "\n";
      // 	cout << " sampledState: " << tempState[0];
      // 	cout << ", " << tempState[1] << ", " << tempState[2];
      // 	cout << ", " << tempState[3] << ", " << tempState[4];
      // 	cout << ", " << tempState[5] << "\n";
      // 	cout << "World Range: [";
      // 	cout << "(" << dist_transform->world_range[0][0];
      // 	cout << ", " << dist_transform->world_range[0][1] << ")";
      // 	cout << ", (" << dist_transform->world_range[1][0];
      // 	cout << ", " << dist_transform->world_range[1][1] << ")";
      // 	cout << ", (" << dist_transform->world_range[2][0];
      // 	cout << ", " << dist_transform->world_range[2][1] << ")]\n";
      // }

      touch_dir << measurementPartFr[1][0], measurementPartFr[1][1], measurementPartFr[1][2];
      // reject particles ourside of distance transform
      if (!insideBounds(measurementPartFr[0], dist_transform->world_range)) {
	continue;
      }
			
      int xind = int(floor((measurementPartFr[0][0] - dist_transform->world_range[0][0]) / 
			   dist_transform->voxel_size));
      int yind = int(floor((measurementPartFr[0][1] - dist_transform->world_range[1][0]) / 
			   dist_transform->voxel_size));
      int zind = int(floor((measurementPartFr[0][2] - dist_transform->world_range[2][0]) / 
			   dist_transform->voxel_size));
      D = (*dist_transform->dist_transform)[xind][yind][zind];

      double dist_adjacent[3] = { 0, 0, 0 };


      // if(isPowerOfTwo(count) && count > 1000){
      // 	cout << "Sampled " << count << " particles\t";
      // 	cout << "count2: " << count2 << "\t count3: " << count3 << "\n";
      // 	cout << "D: " << D << " measurement World: " << trans_M[0][0];
      // 	cout << ", " << trans_M[0][1] << ", " << trans_M[0][2] << "\n";
      // }
      if (D <= unsigned_dist_check) {
	count2 ++;
#ifdef COMBINE_RAYCASTING
	if (checkIntersections(mesh, measurementPartFr[0], measurementPartFr[1], ARM_LENGTH, D)) {
	  count_bar ++;
	  if (count_bar > 1000){
	    std::cout<< "!!!!!!!! BREAKING DUE TO INTERSECTIONS !!!!!" << "\n";
	    break;
	  }
	  continue;
	}
	count_bar = 0;
	D -= R;
#else
	if (checkInObject(mesh, measurementPartFr[0]) == 1 && D != 0) {
	  // if (gradient.dot(touch_dir) <= epsilon)
	  // 	continue;
	  D = -D - R;
	}
	else if (D == 0) {
	  D = - R;
	}
	else {
	  // if (gradient.dot(touch_dir) >= -epsilon)
	  // 	continue;
	  D = D - R;
	}
#endif
      }
      else
	continue;
      if (D >= -signed_dist_check && D <= signed_dist_check) {
#ifndef COMBINE_RAYCASTING	
	safe_point[1][0] = trans_M[1][0];
	safe_point[1][1] = trans_M[1][1];
	safe_point[1][2] = trans_M[1][2];
	safe_point[0][0] = trans_M[0][0] - trans_M[1][0] * ARM_LENGTH;
	safe_point[0][1] = trans_M[0][1] - trans_M[1][1] * ARM_LENGTH;
	safe_point[0][2] = trans_M[0][2] - trans_M[1][2] * ARM_LENGTH;
	count3 ++;
	if (checkObstacles(mesh, tempState, safe_point , D + R) == 1) {
	  count_bar ++;
	  if (count_bar > 1000)
	    break;
	  continue;
	}
	count_bar = 0;
#endif
	particles[i] = tempState;

#ifdef ADAPTIVE_NUMBER
	if (checkEmptyBin(&bins, particles[i]) == 1) {
	  num_bins++;
	  // if (i >= N_MIN) {
	  //int numBins = bins.size();
	  numParticles = min2(maxNumParticles, max2((num_bins - 1) * 2, N_MIN));
	  // }
	}
#endif
	i++;;
      }			
    }
    cout << "Number of total iterations: " << count << endl;
    cout << "Number of iterations after unsigned_dist_check: " << count2 << endl;
    cout << "Number of iterations before safepoint check: " << count3 << endl;
    cout << "Number of occupied bins: " << num_bins << endl;
    cout << "Number of particles: " << numParticles << endl;
    particles.resize(i);
  }
  else {
    // ------------------- MISSED TOUCH ------
    // cast multiple rays to check intersections
    double touch_mnt;
    while (i < numParticles) {
      idx = int(floor(distribution(rd)));
      for (int j = 0; j < cdim; j++) {
	tempState[j] = b_X[idx][j] + Xstd_tran * dist(rd);
      }
      // inverseTransform(measurementWorldFr[0], tempState, measurementPartFr[0]);
      // inverseTransform(measurementWorldFr[1], tempState, measurementPartFr[1]);
      touch_dir << trans_M[0][0] - trans_M[1][0],
	trans_M[0][1] - trans_M[1][1],
	trans_M[0][2] - trans_M[1][2];
      touch_mnt = touch_dir.norm();
      touch_dir = touch_dir / touch_mnt;
      // reject particles ourside of distance transform
			
      safe_point[1][0] = touch_dir[0];
      safe_point[1][1] = touch_dir[1];
      safe_point[1][2] = touch_dir[2];
      safe_point[0][0] = trans_M[1][0] - touch_dir[0] * ARM_LENGTH;
      safe_point[0][1] = trans_M[1][1] - touch_dir[1] * ARM_LENGTH;
      safe_point[0][2] = trans_M[1][2] - touch_dir[2] * ARM_LENGTH;
      if (checkObstacles(mesh, tempState, safe_point, touch_mnt + ARM_LENGTH, 0) == 1)
	continue;
      for (int j = 0; j < cdim; j++) {
	particles[i][j] = tempState[j];
      }
      //double d = testResult(mesh, particles[i], trans_M, R);
      //if (d > 0.01)
      //	cout << measurementPartFr[0][0] << "  " << measurementPartFr[0][1] << "  " << measurementPartFr[0][2] << "   " << d << "   " << D << //"   " << gradient << "   " << gradient.dot(touch_dir) << 
      //	     "   " << dist_adjacent[0] << "   " << dist_adjacent[1] << "   " << dist_adjacent[2] << "   " << particles[i][2] << endl;
      i += 1;
      std::cout << "Miss!" << endl;
    }
  }

  return iffar;
};



























//--------------------!!!!!!!!!!!!!!!---------------
//--------------------      MAIN     ---------------
//--------------------!!!!!!!!!!!!!!!---------------
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
  double R = 0.00; // radius of the touch probe

  double cube_para[3] = { 6, 4, 2 }; // cube size: 6m x 4m x 2m with center at the origin.
  //double range[3][2] = { {-3.5, 3.5}, {-2.5, 2.5}, {-1.5, 1.5} };
  cspace X_true = { 2.12, 1.388, 0.818, Pi / 6 + Pi / 400, Pi / 12 + Pi / 220, Pi / 18 - Pi / 180 }; // true state of configuration
  //particleFilter::cspace X_true = { 0, 0, 0.818, 0, 0, 0 }; // true state of configuration
  cout << "True state: " << X_true[0] << ' ' << X_true[1] << ' ' << X_true[2] << ' ' 
       << X_true[3] << ' ' << X_true[4] << ' ' << X_true[5] << endl;
  cspace b_Xprior[2] = { { 2.11, 1.4, 0.81, Pi / 6, Pi / 12, Pi / 18 },
					 { 0.03, 0.03, 0.03, Pi / 180, Pi / 180, Pi / 180 } }; // our prior belief
  //particleFilter::cspace b_Xprior[2] = { { 0, 0, 0.81, 0, 0, 0 },
  //									 { 0.001, 0.001, 0.001, Pi / 3600, Pi / 3600, Pi / 3600 } }; // our prior belief

  particleFilter pfilter(numParticles, b_Xprior, Xstd_ob, Xstd_tran, Xstd_scatter, R);
  distanceTransform *dist_transform = new distanceTransform(num_voxels);
  //dist_transform->build(cube_para);

  int N_Measure = 60; // total number of measurements
  double M_std = 0.000; // measurement error
  double M[2][3]; // measurement
  cspace particles_est;
  cspace particles_est_stat;
  double particle_est_diff;

  std::random_device generator;
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
    pfilter.estimateGaussian(particles_est, particles_est_stat);
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - tstart);
    particle_est_diff = 0;
    for (int k = 0; k < cdim; k++) {
      particle_est_diff += SQ(particles_est[k] - X_true[k]);
    }
    particle_est_diff /= cdim;
    particle_est_diff = sqrt(particle_est_diff);
    cout << "est: ";
    for (int k = 0; k < cdim; k++) {
      cout << particles_est[k] << ' ';
    }
    cout << endl;
    cout << "Real distance: " << testResult(mesh, particles_est, M, R) << endl;
    cout << "Diff: " << particle_est_diff << endl;
    cout << "Var: ";
    for (int k = 0; k < cdim; k++) {
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
void Transform(const double measure[2][3], cspace src, double dest[2][3])
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
void inverseTransform(const double measure[3], cspace src, double dest[3])
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
void inverseTransform(const double measure[2][3], cspace src, double dest[2][3])
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
double testResult(vector<vec4x3> &mesh, cspace config, double touch[2][3], double R)
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
int checkEmptyBin(std::unordered_set<string> *set, cspace config)
{
  string s = "";
  for (int i = 0; i < cdim; i++) {
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
int checkObstacles(vector<vec4x3> &mesh, cspace config, double start[2][3], double dist)
{
  return checkObstacles(mesh, config, start, ARM_LENGTH, dist);
}
int checkObstacles(vector<vec4x3> &mesh, cspace config, double start[2][3], double check_length, double dist)
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
  for (int i = 0; i < num_mesh; i++) {
    vert0[0] = mesh[i][1][0];
    vert0[1] = mesh[i][1][1];
    vert0[2] = mesh[i][1][2];
    vert1[0] = mesh[i][2][0];
    vert1[1] = mesh[i][2][1];
    vert1[2] = mesh[i][2][2];
    vert2[0] = mesh[i][3][0];
    vert2[1] = mesh[i][3][1];
    vert2[2] = mesh[i][3][2];
    if (intersect_triangle(voxel_center, ray_dir, vert0, vert1, vert2, t, u, v) == 1) {
      if (hashset.find(*t) == hashset.end()) {
	if (*t < check_length && *t > tMax) {
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
  if (countIntersections % 2 == 0) {
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
