#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H
#include <vector>
#include <array>
#include <cstring>
#include <unordered_set>
#include <Eigen/Dense>
#include "distanceTransformNew.h"
using namespace std;
typedef array<array<float, 3>, 4> vec4x3;

class particleFilter
{
 public:
  static const int cdim = 6;
  typedef std::array<double,cdim> cspace; // configuration space of the particles
  typedef std::vector<cspace> Particles;
  int numParticles; // number of particles
  int maxNumParticles;
  int numObs = 0;

  particleFilter (int n_particles, cspace b_init[2], 
				double Xstd_ob=0.0001, double Xstd_tran=0.0025,
				double Xstd_scatter=0.0001, double R=0.01);

  void addObservation (double obs[2][3], vector<vec4x3> &mesh, distanceTransform *dist_transform, bool miss = false);
  //void addObservation (double obs[3], double cube[3], int idx_obs);
  void estimateGaussian(cspace &x_mean, cspace &x_est_stat, bool record = false);
  void getAllParticles(Particles &particles_dest);



 protected:
  // Parameters of filter
  
  double Xstd_ob; // observation measurement error
  double Xstd_tran;
  double Xstd_scatter; // default scattering of particles
  double R; // probe radius

  // internal variables
  cspace b_Xprior[2]; // Initial distribution (mean and variance)
  //cspace b_Xpre[2];   // Previous (estimated) distribution (mean and variance)
  Particles particles;  // Current set of particles
  Particles particlesPrev; // Previous set of particles
  Particles particles_1; // Previous previous set of particles
  Eigen::MatrixXd cov_mat;

  // Local functions
  void createParticles(Particles &particles, cspace b_Xprior[2], int n_particles);

  void buildDistTransformAroundPoint(double cur_M[2][3], vector<vec4x3> &mesh, 
				     distanceTransform *dist_transform);

  bool updateParticles(double cur_M[2][3], vector<vec4x3> &mesh, distanceTransform *dist_transform, bool miss);

};
void Transform(double measure[2][3], particleFilter::cspace src, double dest[2][3]);
void inverseTransform(double measure[3], particleFilter::cspace src, double dest[3]);
void inverseTransform(double measure[2][3], particleFilter::cspace src, double dest[2][3]);
int checkInObject(vector<vec4x3> &mesh, double voxel_center[3]);
int getIntersection(vector<vec4x3> &mesh, double pstart[3], double dir[3], double intersection[3]);
double testResult(vector<vec4x3> &mesh, particleFilter::cspace config, double touch[2][3], double R);
int checkObstacles(vector<vec4x3> &mesh, particleFilter::cspace config, double touch[2][3], double dist);
int checkObstacles(vector<vec4x3> &mesh, particleFilter::cspace config, double start[2][3], double check_length, double dist);
int checkIntersections(vector<vec4x3> &mesh, double voxel_center[3], double dir[3], double check_length, double &dist);
int checkEmptyBin(std::unordered_set<string> *set, particleFilter::cspace config);
#endif // PARTICLE_FILTER_H

