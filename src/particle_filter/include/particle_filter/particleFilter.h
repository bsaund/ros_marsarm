#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H
#include <vector>
#include <array>
#include <cstring>
#include <unordered_set>
#include <Eigen/Dense>
#include "distanceTransformNew.h"
#include "PfDefinitions.h"
#include "randomTransform.h"
using namespace std;


class ParticleDistribution : public Particles
{
 public:
  cspace sampleFrom();
  void updateBandwidth();
 private:
  Eigen::MatrixXd rot;
  Eigen::VectorXd scl;
  static std::random_device rd;
};


class particleFilter
{
 public:
  /* static const int cdim = 6; */


  int numParticles; // number of particles
  int maxNumParticles;
  int numObs = 0;

  particleFilter (int n_particles, cspace b_init[2], 
				double Xstd_ob=0.0001, double Xstd_tran=0.0025,
				double Xstd_scatter=0.0001, double R=0.01);

  void addObservation (double obs[2][3], vector<vec4x3> &mesh, distanceTransform *dist_transform, bool miss = false);
  void addObservation (double obs[2][3], vector<vec4x3> &mesh, 
		       distanceTransform *dist_transform, 
		       RandomTransform &tf, bool miss = false);
  //void addObservation (double obs[3], double cube[3], int idx_obs);
  void estimateGaussian(cspace &x_mean, cspace &x_est_stat, bool record = false) const;
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
  ParticleDistribution particles;  // Current set of particles
  ParticleDistribution particlesPrev; // Previous set of particles
  /* Eigen::MatrixXd cov_mat; */

  // Local functions
  void createParticles(Particles &particles, cspace b_Xprior[2], int n_particles);

  void buildDistTransformAroundPoint(const double cur_M[2][3], vector<vec4x3> &mesh, 
				     distanceTransform *dist_transform);

  bool updateParticles(const double cur_M[2][3], vector<vec4x3> &mesh, distanceTransform *dist_transform, bool miss);

  bool updateParticles(const double cur_M[2][3], vector<vec4x3> &mesh, 
		       distanceTransform *dist_transform, 
		       RandomTransform &tf, bool miss);

};
void Transform(const double measure[2][3], cspace src, double dest[2][3]);
void inverseTransform(const double measure[3], cspace src, double dest[3]);
void inverseTransform(const double measure[2][3], cspace src, double dest[2][3]);
int checkInObject(vector<vec4x3> &mesh, double voxel_center[3]);
int getIntersection(vector<vec4x3> &mesh, double pstart[3], double dir[3], double intersection[3]);
double testResult(vector<vec4x3> &mesh, cspace config, double touch[2][3], double R);
int checkObstacles(vector<vec4x3> &mesh, cspace config, double touch[2][3], double dist);
int checkObstacles(vector<vec4x3> &mesh, cspace config, double start[2][3], double check_length, double dist);
int checkIntersections(vector<vec4x3> &mesh, double voxel_center[3], double dir[3], double check_length, double &dist);
int checkEmptyBin(std::unordered_set<string> *set, cspace config);
#endif // PARTICLE_FILTER_H

