#ifndef PARTICLE_FILTER_6DOF_H
#define PARTICLE_FILTER_6DOF_H

class particleFilter
{
 public:
  static const int cdim = 6;
  typedef double cspace[cdim]; // configuration space of the particles

  particleFilter (int n_particles, cspace b_init[2], 
		  double Xstd_ob=0.0001, double Xstd_tran=0.0025,
		  double Xstd_scatter=0.0001, double R=0.01);

  void addObservation (double obs[3], double cube[3], distanceTransform *dist_transform, int idx_obs);
  //void addObservation (double obs[3], double cube[3], int idx_obs);
  void estimatedDistribution (cspace x_est, double x_est_stat [2]) {
    memcpy(x_est, X_est, sizeof(cspace));
	x_est_stat[0] = X_est_stat[0];
	x_est_stat[1] = X_est_stat[1];
  }

 protected:
  // Parameters of filter
  int N; // number of particles
  double Xstd_ob; // observation measurement error
  double Xstd_tran;
  double Xstd_scatter; // default scattering of particles
  double R; // probe radius
  bool firstObs;

  // internal variables
  cspace b_Xprior[2]; // Initial distribution (mean and variance)
  cspace b_Xpre[2];   // Previous (estimated) distribution (mean and variance)
  cspace *X;  // Current set of particles
  cspace *X0; // Previous set of particles
  cspace *X_1; // Previous previous set of particles
  cspace X_est; // Estimated distribution
  double X_est_stat[2];
  double *W;

  // Local functions
  void createParticles(cspace *X, cspace b_Xprior[2], int n_particles);
  bool updateParticles(cspace *X_1, cspace *X0, cspace *X, double cur_M[3],
			double cube[3], int idx_Measure, distanceTransform *dist_transform,
			int n_particles, double R, double Xstd_ob, double Xstd_tran);
  void calcWeight(double *W, int n_particles, double Xstd_tran, 
		   cspace *X0, cspace *X);
  void resampleParticles(cspace *X0, cspace *X, double *W, int n_particles);
};

#endif // PARTICLE_FILTER_H
