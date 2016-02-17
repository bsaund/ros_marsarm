#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

class particleFilter
{
 public:
  static const int cdim = 3;
  typedef double cspace[cdim]; // configuration space of the particles

  particleFilter (int n_particles, cspace b_init[2], 
		  double Xstd_ob=0.0001, double Xstd_tran=0.0025,
		  double Xstd_scatter=0.0001, double R=0.01);

  void addObservation (double obs[3]);
  void estimatedDistribution (cspace x_est[2]) { 
    memcpy(x_est, X_est, 2*sizeof(cspace));
  }

 protected:
  // Parameters of filter
  int N; // number of particles
  double Xstd_ob; // observation measurement error
  double Xstd_tran;
  double Xstd_scatter; // default scattering of particles
  double R; // ???
  bool firstObs;

  // internal variables
  cspace b_Xprior[2]; // Initial distribution (mean and variance)
  cspace b_Xpre[2];   // Previous (estimated) distribution (mean and variance)
  cspace *X;  // Current set of particles
  cspace *X0; // Previous set of particles
  cspace X_est[2]; // Estimated distribution
  cspace *Xpre_weight;
  double *W;

  // Local functions
  void create_particles(cspace *X, cspace b_Xprior[2], int n_particles);
  bool update_particles(cspace *X, cspace b_Xprior[2], cspace b_Xpre[2],
			double Xstd_ob, double R, double cur_M[3],
			int n_particles);
  void calc_weight(double *W, int n_particles, double Xstd_tran, 
		   cspace *X0, cspace *X);
  void resample_particles(cspace *X0, cspace *X, double *W, int n_particles);
};

#endif // PARTICLE_FILTER_H
