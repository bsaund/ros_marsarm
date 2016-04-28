#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H
#include <vector>
#include <array>
#include <cstring>
#include <unordered_set>
#include "distanceTransformNew.h"
using namespace std;
typedef array<array<float, 3>, 4> vec4x3;

class particleFilter
{
 public:
  static const int cdim = 6;
  typedef double cspace[cdim]; // configuration space of the particles
  int numParticles; // number of particles
  int maxNumParticles;

  particleFilter (int n_particles, cspace b_init[2], 
				double Xstd_ob=0.0001, double Xstd_tran=0.0025,
				double Xstd_scatter=0.0001, double R=0.01);

  void addObservation (double obs[2][3], vector<vec4x3> &mesh, distanceTransform *dist_transform, bool miss = false);
  //void addObservation (double obs[3], double cube[3], int idx_obs);
  void estimatedDistribution (cspace x_est, double x_est_stat [2]) {
    memcpy(x_est, particles_est, sizeof(cspace));
	x_est_stat[0] = particles_est_stat[0];
	x_est_stat[1] = particles_est_stat[1];
  }
  void getAllParticles(cspace *particles_dest);

 protected:
  // Parameters of filter
  
  double Xstd_ob; // observation measurement error
  double Xstd_tran;
  double Xstd_scatter; // default scattering of particles
  double R; // probe radius
  bool firstObs;

  // internal variables
  cspace b_Xprior[2]; // Initial distribution (mean and variance)
  //cspace b_Xpre[2];   // Previous (estimated) distribution (mean and variance)
  cspace *particles;  // Current set of particles
  cspace *particles0; // Previous set of particles
  cspace *particles_1; // Previous previous set of particles
  cspace particles_est; // Estimated distribution
  double particles_est_stat[2];
  //double *W;

  // Local functions
  void createParticles(cspace *particles, cspace b_Xprior[2], int n_particles);
  bool updateParticles(cspace *particles_1, cspace *particles0, cspace *particles, double cur_M[2][3],
			bool miss, vector<vec4x3> &mesh, distanceTransform *dist_transform,
			double R, double Xstd_ob, double Xstd_tran);
  //void calcWeight(double *W, int n_particles, double Xstd_tran, 
		//   cspace *particles0, cspace *particles);
  //void resampleParticles(cspace *particles0, cspace *particles, double *W, int n_particles);
};
void Transform(double measure[2][3], particleFilter::cspace src, double dest[2][3]);
void inverseTransform(double measure[3], particleFilter::cspace src, double dest[3]);
void inverseTransform(double measure[2][3], particleFilter::cspace src, double dest[2][3]);
int checkInObject(vector<vec4x3> &mesh, double voxel_center[3]);
int getIntersection(vector<vec4x3> &mesh, double pstart[3], double dir[3], double intersection[3]);
double testResult(vector<vec4x3> &mesh, double config[6], double touch[2][3], double R);
int checkObstacles(vector<vec4x3> &mesh, double config[6], double touch[2][3], double dist);
int checkObstacles(vector<vec4x3> &mesh, double config[6], double start[2][3], double check_length, double dist);
int checkEmptyBin(std::unordered_set<string> *set, particleFilter::cspace config);
#endif // PARTICLE_FILTER_H

